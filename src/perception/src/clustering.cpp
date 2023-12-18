#include <ros/ros.h>
#include <vector>
#include <stdlib.h>
#include <math.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl_ros/point_cloud.h>
#include <laser_geometry/laser_geometry.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point32.h>
#include <custom_msgs/waypoints.h>

#include <pcl/ml/kmeans.h>
#include <pcl/search/kdtree.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>

// #include <message_filters/subscriber.h>
// #include <message_filters/time_synchronizer.h>
// #include <message_filters/synchronizer.h>
// #include <message_filters/sync_policies/approximate_time.h>

class LaserScan2PCD{
    private:
        ros::NodeHandle nh;
        ros::Subscriber sub;
        ros::Publisher pub;
        ros::Publisher pub_cluster;
        ros::Publisher pub_waypoint;
        ros::Publisher pub_waypoint_geo;
        ros::Publisher pub_front;

        float roi_side = 0.7;  // first = 0.4
        float roi_front = 0.4; // first = 0.3

        std::string target_frame = "laser";
        sensor_msgs::LaserScan scan;
        sensor_msgs::PointCloud2 scan_converted;
        sensor_msgs::PointCloud2 msg_inliers;
        sensor_msgs::PointCloud2 msg_clustered;
        sensor_msgs::PointCloud2 msg_waypoint;
        sensor_msgs::PointCloud2 msg_frontpoints;
        custom_msgs::waypoints   msg_waypoint_geo;
    
        pcl::PCLPointCloud2::Ptr temp_pc2;
        pcl::PointCloud<pcl::PointXYZI>::Ptr temp;
        pcl::search::KdTree<pcl::PointXYZI>::Ptr tree;
        pcl::PointCloud<pcl::PointXYZI>::Ptr lane_points_front;
        pcl::PointCloud<pcl::PointXYZI>::Ptr lane_points_left;
        pcl::PointCloud<pcl::PointXYZI>::Ptr lane_points_right;
        pcl::PointCloud<pcl::PointXYZI>::Ptr waypoints;
        std::vector<geometry_msgs::Point32> waypoints_geo;

    public:
        LaserScan2PCD(double rate);
        ~LaserScan2PCD();
        void subScan(const sensor_msgs::LaserScanConstPtr &msg);
        void pubPCD(const sensor_msgs::PointCloud2ConstPtr &msg );
};

LaserScan2PCD::LaserScan2PCD(double rate){
    sub = nh.subscribe<sensor_msgs::LaserScan>("/scan_filtered",100,&LaserScan2PCD::subScan,this);
    pub = nh.advertise<sensor_msgs::PointCloud2>("/Scan_to_PointCloud",100);
    pub_cluster = nh.advertise<sensor_msgs::PointCloud2>("/Scan_to_PointCloud/clustered",100);
    pub_waypoint = nh.advertise<sensor_msgs::PointCloud2>("/Scan_to_PointCloud/waypoints",100);
    pub_waypoint_geo = nh.advertise<custom_msgs::waypoints>("/Scan_to_PointCloud/waypoints_geo",100);
    pub_front = nh.advertise<sensor_msgs::PointCloud2>("/Scan_to_PointCloud/front",100);

    temp_pc2.reset(new pcl::PCLPointCloud2());
    temp.reset(new pcl::PointCloud<pcl::PointXYZI>);
    lane_points_front.reset(new pcl::PointCloud<pcl::PointXYZI>);
    lane_points_left.reset(new pcl::PointCloud<pcl::PointXYZI>);
    lane_points_right.reset(new pcl::PointCloud<pcl::PointXYZI>);
    tree.reset(new pcl::search::KdTree<pcl::PointXYZI>);
}

LaserScan2PCD::~LaserScan2PCD(){}

void LaserScan2PCD::subScan(const sensor_msgs::LaserScanConstPtr &msg){
    static laser_geometry::LaserProjection projector;
    sensor_msgs::PointCloud2 pc2_dst;
    projector.projectLaser(*msg, scan_converted,-1,laser_geometry::channel_option::Intensity | laser_geometry::channel_option::Distance);
    scan_converted.header.frame_id = target_frame;

    pcl_conversions::toPCL(scan_converted,*temp_pc2);
    pcl::fromPCLPointCloud2(*temp_pc2,*temp);
    pcl::toROSMsg(*temp,msg_inliers);
    msg_inliers.header.frame_id = target_frame;
    pub.publish(msg_inliers);


    /*      Preprocess 1 
        * PassThrough filter
        * xfilter : outlier removal 
        * yfilter : lane line 
    */
    pcl::PassThrough<pcl::PointXYZI> lane_right,lane_left,front;
    lane_right.setInputCloud(temp);
    lane_right.setFilterFieldName("x");
    lane_right.setFilterLimits(0,roi_side);
    lane_right.filter(*lane_points_right);
    
    lane_left.setInputCloud(temp);
    lane_left.setFilterFieldName("x");
    lane_left.setFilterLimits(-roi_side,0);
    lane_left.filter(*lane_points_left);

    front.setInputCloud(temp);
    front.setFilterFieldName("y");
    front.setFilterLimits(0.0,roi_front);
    front.filter(*lane_points_front);
    front.setInputCloud(lane_points_front);
    front.setFilterFieldName("x");
    front.setFilterLimits(-roi_side/3,roi_side/3);
    front.filter(*lane_points_front);

    pcl::toROSMsg(*lane_points_front,msg_frontpoints);
    msg_frontpoints.header.frame_id = target_frame;
    pub_front.publish(msg_frontpoints);

    /*      Preprocess 2 
        * Euclidean clustering for traffic cone detection  
        * tba
        * tba
    */
    int j=0;
    temp.reset(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
    std::vector<pcl::PointIndices> cluster_indices;

    ec.setClusterTolerance(0.1);
    ec.setMinClusterSize(2);
    ec.setMaxClusterSize(10);
    tree->setInputCloud(lane_points_right);
    ec.setSearchMethod(tree);
    ec.setInputCloud(lane_points_right);
    ec.extract(cluster_indices);
    for(const auto& cluster: cluster_indices){
        for(const auto& idx: cluster.indices){
            pcl::PointXYZI temp_point;
            (*lane_points_right)[idx].intensity = j;
            temp->push_back((*lane_points_right)[idx]);
        }
        j++;
    }

    tree->setInputCloud(lane_points_right);
    ec.setSearchMethod(tree);
    ec.setInputCloud(lane_points_left);
    cluster_indices.clear();
    ec.extract(cluster_indices);
    for(const auto& cluster: cluster_indices){
        for(const auto& idx: cluster.indices){
            pcl::PointXYZI temp_point;
            (*lane_points_left)[idx].intensity = j;
            temp->push_back((*lane_points_left)[idx]);
            }
        j++;
    }


    pcl::toROSMsg(*temp,msg_clustered);
    msg_clustered.header.frame_id = target_frame;
    pub_cluster.publish(msg_clustered);

    /*      Preprocess 3 
        * Get waypoints from clustered points  
        * tba
        * tba
    */
    waypoints.reset(new pcl::PointCloud<pcl::PointXYZI>);
    for(size_t i=0;i!=(*lane_points_right).size();i++){
        for(size_t j=0;j!=(*lane_points_left).size();j++){
            if (fabs((*lane_points_right)[i].y - (*lane_points_left)[j].y) <= 0.1)
            {
                pcl::PointXYZI temp_p;
                temp_p.x = ((*lane_points_right)[i].x + (*lane_points_left)[j].x) / 2;
                temp_p.y = ((*lane_points_right)[i].y + (*lane_points_left)[j].y) / 2;
                temp_p.z = 0;
                temp_p.intensity = 10.0;
                waypoints->push_back(temp_p);
            }
        }
    }
    
    /*      Preprocess 4
        * Kmeans clustering   
        * derivates centroid of waypoints
        * tba
    */
    pcl::Kmeans km((*waypoints).size(),3);
    km.setClusterSize(5);
    for(size_t i=0;i!=(*waypoints).size();i++){
        std::vector<float> temp_point(3);
        temp_point[0] = (*waypoints)[i].x;
        temp_point[1] = (*waypoints)[i].y;
        temp_point[2] = 0;
        km.addDataPoint(temp_point);
    }
    km.kMeans();
    pcl::Kmeans::Centroids centroids = km.get_centroids();
    temp.reset(new pcl::PointCloud<pcl::PointXYZI>);
    for(size_t i=0;i!=centroids.size();i++){
       
        
        pcl::PointXYZI temp_pc;
        temp_pc.x = centroids[i][0];
        temp_pc.y = centroids[i][1];
        temp_pc.z = 0;
        float dist_ = sqrt(pow(temp_pc.x,2)+pow(temp_pc.y,2)); 
        if(dist_ >= 0.2 && dist_ <= 0.45){
            temp->push_back(temp_pc);
        }
        // temp->push_back(temp_pc);

        geometry_msgs::Point32 temp_gp;
        temp_gp.x = centroids[i][0];
        temp_gp.y = centroids[i][1];
        temp_gp.z = 0;

        float dist_of_centroid = sqrt(pow(temp_gp.x,2)+pow(temp_gp.y,2)); 
        if(dist_of_centroid >= 0.2 && dist_of_centroid <= 0.45){
            msg_waypoint_geo.points.push_back(temp_gp);
        }
    }    
    pcl::toROSMsg(*temp,msg_waypoint);
    msg_waypoint.header.frame_id = target_frame;
    pub_waypoint.publish(msg_waypoint);
    pub_waypoint_geo.publish(msg_waypoint_geo);
    msg_waypoint_geo.points.clear();
}


void LaserScan2PCD::pubPCD(const sensor_msgs::PointCloud2ConstPtr &msg){}


int main(int argc, char** argv){
    ros::init(argc,argv,"LaserScan_Converter");

    LaserScan2PCD conversion_node(10);

    ros::Rate r(10);

    while(ros::ok()){
        ros::spinOnce();
    }

}