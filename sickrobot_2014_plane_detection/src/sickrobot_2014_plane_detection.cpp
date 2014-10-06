#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "pcl_conversions/pcl_conversions.h"
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/segmentation/organized_multi_plane_segmentation.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>

ros::Publisher pub;

void detect_plane(const sensor_msgs::PointCloud2::ConstPtr& msg){
    ROS_INFO("Detecting Planes");
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg (*msg, *cloud);
    ROS_INFO("Msg converted");

    // Create the normal estimation class, and pass the input dataset to it
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud (cloud);

    // Create an empty kdtree representation, and pass it to the normal estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
    ne.setSearchMethod (tree);

    // Output datasets
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);

    // Use all neighbors in a sphere of radius 3cm
    ne.setRadiusSearch (0.003);

    // Compute the features
    ne.compute (*cloud_normals);
    ROS_INFO("Normales Computed");

    pcl::OrganizedMultiPlaneSegmentation< pcl::PointXYZ, pcl::Normal, pcl::Label > mps;
    mps.setMinInliers (1000);
    mps.setAngularThreshold (0.017453 * 2.0); // 2 degrees
    mps.setDistanceThreshold (0.02); // 2cm
    mps.setInputNormals (cloud_normals);
    mps.setInputCloud (cloud);

    std::vector< pcl::PlanarRegion< pcl::PointXYZ >, Eigen::aligned_allocator<pcl::PlanarRegion<pcl::PointXYZ> > > regions;

    //mps.segmentAndRefine(regions);
    mps.segment(regions);

    ROS_INFO("Segmentation complete: %d", regions.size ());

    for (size_t i = 0; i < regions.size (); i++)
    {
        Eigen::Vector3f centroid = regions[i].getCentroid ();
        Eigen::Vector4f model = regions[i].getCoefficients ();
        pcl::PointCloud<pcl::PointXYZ> boundary_cloud;
        boundary_cloud.points = regions[i].getContour ();
        ROS_INFO ("Centroid: (%f, %f, %f)\n  Coefficients: (%f, %f, %f, %f)\n Inliers: %d\n",
                  centroid[0], centroid[1], centroid[2],
                  model[0], model[1], model[2], model[3],
                  boundary_cloud.points.size ());
    }
}

void detect_plane_sacsegmentation(const sensor_msgs::PointCloud2::ConstPtr& msg){
    ROS_INFO("Detecting Planes");

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg (*msg, *cloud);

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.01);

    seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients);

    ROS_INFO("Indices_size: %d", inliers->indices.size ());

    if (inliers->indices.size () == 0)
    {
        PCL_ERROR ("Could not estimate a planar model for the given dataset.");
        return;
    }

    std::cerr << "Model coefficients: " << coefficients->values[0] << " "
              << coefficients->values[1] << " "
              << coefficients->values[2] << " "
              << coefficients->values[3] << std::endl;


    pcl::PointCloud<pcl::PointXYZ>::Ptr final (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud<pcl::PointXYZ>(*cloud, inliers->indices, *final);
    sensor_msgs::PointCloud2 new_msg;
    pcl::toROSMsg(*final, new_msg);
    pub.publish(new_msg);
}

int main(int argc, char** argv)
{

    ros::init(argc, argv, "plane_detector");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe<sensor_msgs::PointCloud2>("/camera/depth/points", 1000, detect_plane);
    pub = n.advertise<sensor_msgs::PointCloud2>("/plane", 1000);
    ROS_INFO("Start");

    ros::spin();

    return 0;
}
