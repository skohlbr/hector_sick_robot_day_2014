#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "geometry_msgs/PolygonStamped.h"

#include "opencv/cv.h"
#include "opencv/highgui.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include <cv_bridge/cv_bridge.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/search/kdtree.h>

#include <pcl/common/common.h>

#include <pcl/surface/convex_hull.h>
#include <pcl/filters/project_inliers.h>

#include <pcl/segmentation/extract_clusters.h>

#include <pcl/filters/voxel_grid.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <image_geometry/pinhole_camera_model.h>

#include <message_filters/subscriber.h>

#include <message_filters/sync_policies/approximate_time.h>


typedef pcl::PointXYZ PointT;

ros::Publisher plane_pub;
ros::Publisher poly_pub;


sensor_msgs::CameraInfoConstPtr last_info;


void filterDepth(boost::shared_ptr< pcl::PointCloud<PointT> >& cloud){
    pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);

    pcl::PassThrough<PointT> no_ground_pass;

    no_ground_pass.setInputCloud (cloud);
    no_ground_pass.setFilterFieldName ("z");
    no_ground_pass.setFilterLimits (0.1, 1.4); //TODO tune with wall distance

    no_ground_pass.filter (*cloud_filtered);

    cloud = cloud_filtered;
}

void filterHeight(boost::shared_ptr< pcl::PointCloud<PointT> >& cloud){
    pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);

    pcl::PassThrough<PointT> no_ground_pass;
    no_ground_pass.setFilterLimitsNegative (true);

    no_ground_pass.setInputCloud (cloud);
    no_ground_pass.setFilterFieldName ("y");
    no_ground_pass.setFilterLimits (0, 1);

    no_ground_pass.filter (*cloud_filtered);

    cloud = cloud_filtered;
}


bool segmentDigitPlane(boost::shared_ptr< pcl::PointCloud<PointT> >& cloud, boost::shared_ptr< pcl::PointCloud<PointT> >& cloud_plane, pcl::ModelCoefficients::Ptr& coefficients ,pcl::PointCloud<pcl::PointXYZ>::Ptr& convex_hull){
    ROS_INFO("Start segmentation");
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PERPENDICULAR_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.03);

    seg.setAxis(Eigen::Vector3f(0.0,0.0,1.0)); //Parallel to view point
    seg.setEpsAngle( 0.2 ); //With 20 degree difference

    seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients);
    ROS_INFO("Segmentation done");
    ROS_INFO("Model coefficients: %f %f %f %f", coefficients->values[0], coefficients->values[1], coefficients->values[2], coefficients->values[3]);

    pcl::ExtractIndices<PointT> eifilter (true); // Initializing with true will allow us to extract the removed indices


    pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);

    eifilter.setInputCloud (cloud);
    eifilter.setIndices (inliers);
    eifilter.filter (*cloud_filtered);

    ROS_INFO("Filtering done");


    //----------------- Filter out spurious stuff in the environment (Assume table is largest cluster) -----------

    //Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (cloud_filtered);

    std::vector<pcl::PointIndices> cluster_indices;

    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (0.02); // 2cm
    ec.setMinClusterSize (100);
    ec.setMaxClusterSize (250000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud_filtered);
    ec.extract (cluster_indices);
    ROS_INFO("Extraction done: %d", cluster_indices.size());


    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);

    if  (cluster_indices.size() > 0){
        //for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
        //  cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*
        pcl::PointIndices& indices = cluster_indices[0];

        size_t size = indices.indices.size();

        for (size_t i = 0; i < size; ++i){
            cloud_cluster->points.push_back(cloud_filtered->points[indices.indices[i]]);
        }
        cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

    }else{
        return false;
    }

    cloud_plane = cloud_cluster;

    return true;
}

std::vector<Eigen::Vector3f> getPlaneRectangle(boost::shared_ptr< pcl::PointCloud<PointT> >& cloud, pcl::ModelCoefficients::Ptr& coefficients)
{
    std::vector<Eigen::Vector3f> table_top_bbx;

    // Project points onto the table plane
    pcl::ProjectInliers<PointT> proj;
    proj.setModelType(pcl::SACMODEL_PLANE);
    pcl::PointCloud<PointT> projected_cloud;
    proj.setInputCloud(cloud);
    proj.setModelCoefficients(coefficients);
    proj.filter(projected_cloud);

    // store the table top plane parameters
    Eigen::Vector3f plane_normal;
    plane_normal.x() = coefficients->values[0];
    plane_normal.y() = coefficients->values[1];
    plane_normal.z() = coefficients->values[2];
    // compute an orthogonal normal to the plane normal
    Eigen::Vector3f v = plane_normal.unitOrthogonal();
    // take the cross product of the two normals to get
    // a thirds normal, on the plane
    Eigen::Vector3f u = plane_normal.cross(v);

    // project the 3D point onto a 2D plane
    std::vector<cv::Point2f> points;
    // choose a point on the plane
    Eigen::Vector3f p0(projected_cloud.points[0].x,
                       projected_cloud.points[0].y,
                       projected_cloud.points[0].z);
    for(unsigned int ii=0; ii<projected_cloud.points.size(); ii++)
    {
        Eigen::Vector3f p3d(projected_cloud.points[ii].x,
                            projected_cloud.points[ii].y,
                            projected_cloud.points[ii].z);

        // subtract all 3D points with a point in the plane
        // this will move the origin of the 3D coordinate system
        // onto the plane
        p3d = p3d - p0;

        cv::Point2f p2d;
        p2d.x = p3d.dot(u);
        p2d.y = p3d.dot(v);
        points.push_back(p2d);
    }

    cv::Mat points_mat(points);
    cv::RotatedRect rrect = cv::minAreaRect(points_mat);
    cv::Point2f rrPts[4];
    rrect.points(rrPts);

    //store the table top bounding points in a vector
    for(unsigned int ii=0; ii<4; ii++)
    {
        Eigen::Vector3f pbbx(rrPts[ii].x*u + rrPts[ii].y*v + p0);
        table_top_bbx.push_back(pbbx);
    }
    Eigen::Vector3f center(rrect.center.x*u + rrect.center.y*v + p0);
    table_top_bbx.push_back(center);

    return table_top_bbx;
}

sensor_msgs::PointCloud2 plane_msg;

void point_cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg){

    ROS_INFO("Processing point_cloud");
    boost::shared_ptr< pcl::PointCloud<PointT> > cloud(new pcl::PointCloud<PointT>);
    pcl::fromROSMsg(*msg, *cloud);
    filterDepth(cloud);
    filterHeight(cloud);
    sensor_msgs::PointCloud2 filterd_msg;
    pcl::toROSMsg(*cloud, filterd_msg);
    plane_pub.publish(filterd_msg);

    boost::shared_ptr< pcl::PointCloud<PointT> > cloud_plane(new pcl::PointCloud<PointT>);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointCloud<pcl::PointXYZ>::Ptr convex_hull (new pcl::PointCloud<pcl::PointXYZ>);
    if(!segmentDigitPlane(cloud, cloud_plane, coefficients, convex_hull)){
        ROS_ERROR("Segmentation failed");
        return;
    }
    const std::vector<Eigen::Vector3f> plane_rect = getPlaneRectangle(cloud_plane, coefficients);

    geometry_msgs::PolygonStamped plane_poly;
    plane_poly.header.frame_id = msg->header.frame_id;
    plane_poly.header.stamp = ros::Time::now();

    image_geometry::PinholeCameraModel cam_model;
    //cam_model.fromCameraInfo(last_info);

    for (size_t i= 0 ; i < 4; ++i){
        geometry_msgs::Point32 tmp;
        tmp.x = plane_rect[i].x();
        tmp.y = plane_rect[i].y();
        tmp.z = plane_rect[i].z();
        ROS_INFO("PolyPoint %d: %f %f %f", i, tmp.x, tmp.y, tmp.z);
        plane_poly.polygon.points.push_back(tmp);
    }

    poly_pub.publish(plane_poly);

}

sensor_msgs::ImageConstPtr last_image;

void imageCallback(const sensor_msgs::ImageConstPtr& msg){
    last_image = msg;
}


void infoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg){
    //last_info = msg;
}


void callback(const sensor_msgs::Image::ConstPtr &image_msg,
              const sensor_msgs::CameraInfo::ConstPtr& info_msg,
              const sensor_msgs::PointCloud2::ConstPtr& pc_msg){
    ROS_INFO("Callback");
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        if (sensor_msgs::image_encodings::isColor(image_msg->encoding))
            cv_ptr = cv_bridge::toCvShare(image_msg, sensor_msgs::image_encodings::BGR8);
        else
            cv_ptr = cv_bridge::toCvShare(image_msg, sensor_msgs::image_encodings::MONO8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat img = cv_ptr->image;

    boost::shared_ptr< pcl::PointCloud<PointT> > cloud(new pcl::PointCloud<PointT>);
    pcl::fromROSMsg(*pc_msg, *cloud);
    filterDepth(cloud);
    filterHeight(cloud);
    sensor_msgs::PointCloud2 filterd_msg;
    pcl::toROSMsg(*cloud, filterd_msg);
    plane_pub.publish(filterd_msg);

    boost::shared_ptr< pcl::PointCloud<PointT> > cloud_plane(new pcl::PointCloud<PointT>);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointCloud<pcl::PointXYZ>::Ptr convex_hull (new pcl::PointCloud<pcl::PointXYZ>);
    if(!segmentDigitPlane(cloud, cloud_plane, coefficients, convex_hull)){
        ROS_ERROR("Segmentation failed");
        return;
    }
    const std::vector<Eigen::Vector3f> plane_rect = getPlaneRectangle(cloud_plane, coefficients);

    geometry_msgs::PolygonStamped plane_poly;
    plane_poly.header.frame_id = pc_msg->header.frame_id;
    plane_poly.header.stamp = ros::Time::now();

    image_geometry::PinholeCameraModel cam_model;
    cam_model.fromCameraInfo(*info_msg);

    std::vector<cv::Point> img_poly;
    ROS_INFO("IMG: %d, %d", img.cols, img.rows);

    for (size_t i= 0 ; i < 4; ++i){
        geometry_msgs::Point32 tmp;
        tmp.x = plane_rect[i].x();
        tmp.y = plane_rect[i].y();
        tmp.z = plane_rect[i].z();
        ROS_INFO("PolyPoint %d: %f %f %f", i, tmp.x, tmp.y, tmp.z);
        plane_poly.polygon.points.push_back(tmp);
        cv::Point3d cvP(tmp.x, tmp.y, tmp.z);
        cv::Point2d imgPoint = cam_model.project3dToPixel(cvP);
        cv::Point point((int) imgPoint.x, (int) imgPoint.y);
        img_poly.push_back(point);
        ROS_INFO("ImgPoint: %f, %f", imgPoint.x, imgPoint.y);

    }

    cv::Mat mask = cv::Mat::zeros(img.rows, img.cols, img.type());
    cv::fillConvexPoly(mask, img_poly, cv::Scalar(255, 255, 255));

    cv::Mat dst = cv::Mat::zeros(img.rows, img.cols, img.type());

    img.copyTo(dst, mask);

    poly_pub.publish(plane_poly);

    //cv::polylines(img, img_poly, true, cv::Scalar(0,255,0), 3);

    cv::imshow("view", dst);
    cv::waitKey(3);
}

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::PointCloud2> ApproximateTimeSyncPolicy;

int main(int argc, char** argv)
{

    ros::init(argc, argv, "plane_detector");
    ros::NodeHandle nh_;

    //ros::Subscriber pc_sub = nh_.subscribe<sensor_msgs::PointCloud2>("/camera/depth_registered/points", 10, point_cloudCallback);


    image_transport::ImageTransport it_(nh_);
    //image_transport::Subscriber sub = it_.subscribe("/camera/rgb/image_color", 1, imageCallback);
    //image_transport::Subscriber info_sub = it_.subscribe("/camera/rgb/camera_info", 1, infoCallback);

    poly_pub = nh_.advertise<geometry_msgs::PolygonStamped>("/digit_plane_poly",1);
    plane_pub = nh_.advertise<sensor_msgs::PointCloud2>("/plane", 1);

    //ros::Subscriber pc_sub = nh_.subscribe<sensor_msgs::PointCloud2>("/camera/depth/points", 10, point_cloudCallback);


    image_transport::SubscriberFilter image_sub;
    message_filters::Subscriber<sensor_msgs::CameraInfo> info_sub;
    message_filters::Subscriber<sensor_msgs::PointCloud2> pc_sub;
    message_filters::Synchronizer<ApproximateTimeSyncPolicy> sync_(ApproximateTimeSyncPolicy(5), image_sub, info_sub, pc_sub);

    image_sub.subscribe(it_,"/camera/rgb/image_color", 1);
    info_sub.subscribe(nh_, "/camera/rgb/camera_info", 1);
    pc_sub.subscribe(nh_, "/camera/depth_registered/points", 1);

    //    sync_.connectInput(image_sub, info_sub, pc_sub);
    //   sync_.registerCallback(callback);
    sync_.registerCallback(boost::bind(&callback, _1, _2, _3));

    cv::namedWindow("view");


    ros::spin();

    cv::destroyAllWindows();

    return 0;
}
