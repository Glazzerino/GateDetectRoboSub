#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include "pcl_ros/transforms.h"
#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/filters/filter.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl/visualization/point_cloud_handlers.h>
#include <pcl/filters/voxel_grid.h>
#include <vector>
#include <pcl/filters/passthrough.h>

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

// pcl::visualization::CloudViewer test_viewer("Point Cloud");

boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(
    new pcl::visualization::PCLVisualizer ("Visualizer")
);

pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);

pcl::PCLPointCloud2* cloud_filtered = new pcl::PCLPointCloud2;
pcl::PCLPointCloud2Ptr cloud_filtered_ptr (cloud_filtered);

pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);


std::vector<pcl::PointIndices> cluster_indices;

//Eucledian clustering objclusterert
pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);

pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);

pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> clusterer;

float test_input = 0;

//Stores individual clusters as point clouds
std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> segments;

void callback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {

    viewer->removeAllShapes();
    viewer->removeAllPointClouds();
    segments.clear();
    segments.reserve(10);
    cluster_indices.clear();

    pcl_conversions::toPCL(*cloud_msg, *cloud);

    clusterer.setClusterTolerance (0.02); // 2cm
    clusterer.setMinClusterSize (100);
    clusterer.setMaxClusterSize (25000);

    pcl::VoxelGrid<pcl::PCLPointCloud2> voxel_grid;
    voxel_grid.setInputCloud(cloudPtr);
    voxel_grid.setLeafSize(0.01,0.01, 0.01);
    voxel_grid.filter(*cloud_filtered_ptr);

    std::vector<int> indices;
    pcl::fromPCLPointCloud2(*cloud_filtered_ptr, *temp_cloud);

    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud(temp_cloud);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(-test_input, FLT_MAX);
    pass.filter(*filtered_cloud);

    pcl::removeNaNFromPointCloud(*filtered_cloud, *filtered_cloud, indices);

    clusterer.setSearchMethod (tree);
    clusterer.setInputCloud (filtered_cloud);
    clusterer.extract (cluster_indices);


    // int j = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it) {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr individual_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);

        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
            individual_cluster->push_back((*filtered_cloud)[*pit]);
        individual_cluster->width = individual_cluster->size();
        individual_cluster->height = 1;
        individual_cluster->is_dense = true;
        segments.push_back(individual_cluster);
    }
    int counter = 0;
    for (pcl::PointCloud<pcl::PointXYZRGB>::Ptr seg : segments) {

        ++counter;
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> color_handler(
            seg,
            rand() % 255,
            rand() % 255,
            rand() % 255);
        std::string id_name = std::to_string(counter);

        viewer->addPointCloud<pcl::PointXYZRGB> (seg,color_handler ,id_name);
         viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, id_name);
    }
    viewer->spinOnce(100);
}

int main(int argc, char** argv) {

    std::cout << "Enter max height: ";
    std::cin >> test_input;
    viewer->setBackgroundColor(0,0,0);
    segments.reserve(10);

    ros::init(argc, argv, "point_cloud_processing");
    ros::NodeHandle handler;
    ros::Subscriber subscriber = handler.subscribe("frontr200/camera/depth_registered/points", 10, callback);

    ros::spin();
    // while(!viewer->wasStopped()) {
    //     ros::spinOnce();
    //     viewer->spinOnce(20, true);
    // }
}
