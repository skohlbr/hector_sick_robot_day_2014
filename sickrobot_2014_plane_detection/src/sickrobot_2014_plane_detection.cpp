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

#include <tf/transform_listener.h>

#include <float.h>

#include <hector_digit_detection_msgs/Image2Digit.h>

#include <hector_worldmodel_msgs/PosePercept.h>

#include <sstream>

#include <tf_conversions/tf_eigen.h>


typedef pcl::PointXYZ PointT;

ros::Publisher image_pub;
ros::Publisher plane_pub;
ros::Publisher poly_pub;
ros::Publisher percept_publisher_;
tf::TransformListener* listener;
ros::ServiceClient digit_service;

double sign_width = 0.6;
double sign_height = 0.84;
double sign_width_tolerance = 0.2;
double sign_height_tolerance = 0.2;
double height_cutoff_min = 0.5;
double height_cutoff_max = 1.5;


Eigen::Affine3d to_base_link_;

sensor_msgs::PointCloud2 plane_msg;


struct RectanglePointData{
    Eigen::Vector3f point_base_link;
    cv::Point2d point_image;
};

enum corners {BOTTOM_LEFT, UPPER_LEFT, UPPER_RIGHT, BOTTOM_RIGHT};

void filterDepth(boost::shared_ptr< pcl::PointCloud<PointT> >& cloud){
    pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);

    pcl::PassThrough<PointT> no_ground_pass;

    no_ground_pass.setInputCloud (cloud);
    no_ground_pass.setFilterFieldName ("z");
    //no_ground_pass.setFilterLimits (wall_distance - 1.0, wall_distance + 1.0);

    no_ground_pass.filter (*cloud_filtered);

    cloud = cloud_filtered;
}

void filterVoxel(boost::shared_ptr< pcl::PointCloud<PointT> >& cloud, float voxel_size){
    pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);

    pcl::VoxelGrid<PointT> voxel_grid;

    voxel_grid.setInputCloud (cloud);
    voxel_grid.setLeafSize(voxel_size, voxel_size, voxel_size);
    voxel_grid.filter (*cloud_filtered);

    cloud = cloud_filtered;
}

void filterHeight(boost::shared_ptr< pcl::PointCloud<PointT> >& cloud, const std::string field_name, float min, float max){
    pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);

    pcl::PassThrough<PointT> no_ground_pass;

    no_ground_pass.setInputCloud (cloud);
    no_ground_pass.setFilterFieldName (field_name);
    no_ground_pass.setFilterLimits (min , max);

    no_ground_pass.filter (*cloud_filtered);

    cloud = cloud_filtered;
}

void publishPolygonMsg(const std::vector<Eigen::Vector3f>& plane_rect, const std_msgs::Header& header)
{
  if (poly_pub.getNumSubscribers() > 0){
    geometry_msgs::PolygonStamped plane_poly;
    plane_poly.header = header;
    plane_poly.header.frame_id = "base_link";

    for (size_t i= 0 ; i < 4; ++i){
      geometry_msgs::Point32 tmp;
      tmp.x = plane_rect[i].x();
      tmp.y = plane_rect[i].y();
      tmp.z = plane_rect[i].z();

      plane_poly.polygon.points.push_back(tmp);

    }
    poly_pub.publish(plane_poly);
  }
}

bool checkGeometryProperties(const std::vector<Eigen::Vector3f>& plane_rect,
                             const image_geometry::PinholeCameraModel& cam_model,
                             const tf::Transform& to_cam_transform,
                             std::vector<RectanglePointData>& plane_rect_data_ordered)
{
  std::vector<RectanglePointData> plane_rect_data_unordered;
  plane_rect_data_unordered.resize(4);
  plane_rect_data_ordered.resize(4);

  float max_x = -1000, min_x = 1000;
  float max_y = -1000, min_y = 1000;

  for (size_t i= 0 ; i < 4; ++i){

      plane_rect_data_unordered[i].point_base_link = plane_rect[i];

      Eigen::Vector3d point_depth_frame (to_base_link_.inverse() * plane_rect[i].cast<double>());

      tf::Point v3(point_depth_frame.x(), point_depth_frame.y(), point_depth_frame.z());
      v3 = to_cam_transform * v3;


      cv::Point3d cvP(v3.x(), v3.y(), v3.z());
      cv::Point2d imgPoint = cam_model.project3dToPixel(cvP);
      plane_rect_data_unordered[i].point_image = imgPoint;

      if(max_x < imgPoint.x){
          max_x = imgPoint.x;
      }
      if(min_x > imgPoint.x){
          min_x = imgPoint.x;
      }
      if(max_y < imgPoint.y){
          max_y = imgPoint.y;
      }
      if(min_y > imgPoint.y){
          min_y = imgPoint.y;
      }
  }

  float avg_x = ((max_x - min_x)/2 + min_x);
  float avg_y = ((max_y - min_y)/2 + min_y);
  ROS_DEBUG("max_x: %f", max_x);
  ROS_DEBUG("min_x: %f", min_x);
  ROS_DEBUG("max_y: %f", max_y);
  ROS_DEBUG("min_y: %f", min_y);
  ROS_DEBUG("avg_x: %f", avg_x);
  ROS_DEBUG("avg_y: %f", avg_y);

  for (size_t i= 0 ; i < 4; ++i){
    //cv::Point2f point = src[i];
    if(plane_rect_data_unordered[i].point_image.x <= avg_x){
      if(plane_rect_data_unordered[i].point_image.y <= avg_y){
        // Bottom left
        plane_rect_data_ordered[BOTTOM_LEFT] = plane_rect_data_unordered[i];
      }else{
        // Upper left
        plane_rect_data_ordered[UPPER_LEFT] = plane_rect_data_unordered[i];
      }
    }else {
      if(plane_rect_data_unordered[i].point_image.y <= avg_y){
        // Bottom right
        plane_rect_data_ordered[BOTTOM_RIGHT] = plane_rect_data_unordered[i];
      } else{
        // Upper right
        plane_rect_data_ordered[UPPER_RIGHT] = plane_rect_data_unordered[i];
      }
    }
  }

  double height_left  = (plane_rect_data_ordered[BOTTOM_LEFT].point_base_link - plane_rect_data_ordered[UPPER_LEFT].point_base_link).norm();
  double height_right = (plane_rect_data_ordered[BOTTOM_RIGHT].point_base_link - plane_rect_data_ordered[UPPER_RIGHT].point_base_link).norm();
  double width_top    = (plane_rect_data_ordered[UPPER_LEFT].point_base_link - plane_rect_data_ordered[UPPER_RIGHT].point_base_link).norm();
  double width_bottom = (plane_rect_data_ordered[BOTTOM_LEFT].point_base_link - plane_rect_data_ordered[BOTTOM_RIGHT].point_base_link).norm();

  if ( (height_left > (sign_height + sign_height_tolerance) ) || (height_left < (sign_height - sign_height_tolerance) )){
    ROS_DEBUG("Left height out of threshold: %f", height_left);
    return false;
  }
  if ( (height_right > (sign_height + sign_height_tolerance) ) || (height_right < (sign_height - sign_height_tolerance) )){
    ROS_DEBUG("Right height out of threshold: %f", height_right);
    return false;
  }
  if ( (width_top > (sign_width + sign_width_tolerance) ) || (width_top < (sign_width - sign_width_tolerance) )){
    ROS_DEBUG("Top width out of threshold: %f", width_top);
    return false;
  }
  if ( (width_bottom > (sign_width + sign_width_tolerance) ) || (width_bottom < (sign_width - sign_width_tolerance) )){
    ROS_DEBUG("Bottom width out of threshold: %f", width_bottom);
    return false;
  }

  return true;
}

void getPerspectiveTransformedImage(std::vector<RectanglePointData>& plane_rect_data_ordered,
                               cv::Mat& source_img,
                               cv::Mat& target_img)
{
  cv::Point2f src[4];
  cv::Point2f dst[4];


  // Order BOTTOM_LEFT, UPPER_LEFT, UPPER_RIGHT, BOTTOM_RIGHT
  for (size_t i = 0; i < 4; ++i){
    src[i] = plane_rect_data_ordered[i].point_image;
  }

  double aspect_ratio = sign_width / sign_height;
  double base_width = 40.0;
  double height = base_width / aspect_ratio;
  //int x_max = static_cast<int>(base_width);
  //int y_max = static_cast<int>(height);

  float x_max = base_width;
  float y_max = height;

  dst[BOTTOM_LEFT]  = cv::Point2f(0.0f,0.0f);
  dst[UPPER_LEFT]   = cv::Point2f(0.0f,y_max);
  dst[UPPER_RIGHT]  = cv::Point2f(x_max, y_max);
  dst[BOTTOM_RIGHT] = cv::Point2f(x_max,0.0f);

  cv::Mat M = cv::getPerspectiveTransform(src, dst);
  cv::warpPerspective(source_img, target_img, M, cv::Size(x_max, y_max));

}


bool segmentDigitPlane(boost::shared_ptr< pcl::PointCloud<PointT> >& cloud, boost::shared_ptr< pcl::PointCloud<PointT> >& cloud_plane, pcl::ModelCoefficients::Ptr& coefficients ,pcl::PointCloud<pcl::PointXYZ>::Ptr& convex_hull){

    if(cloud->size() < 100){
        return false;
    }

    ROS_DEBUG("Start segmentation");
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PARALLEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.04);

    seg.setAxis(Eigen::Vector3f(0.0, 0.0, 1.0)); //Parallel to view point
    seg.setEpsAngle( 10.0 * M_PI / 180 ); //With 20 degree difference

    seg.setInputCloud (cloud);
    seg.segment (*inliers, *coefficients);
    ROS_DEBUG("Segmentation done");
    ROS_DEBUG("Model coefficients: %f %f %f %f", coefficients->values[0], coefficients->values[1], coefficients->values[2], coefficients->values[3]);

    pcl::ExtractIndices<PointT> eifilter (true); // Initializing with true will allow us to extract the removed indices


    pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);

    eifilter.setInputCloud (cloud);
    eifilter.setIndices (inliers);
    eifilter.filter (*cloud_filtered);

    ROS_DEBUG("Filtering done");

    if(inliers->indices.size() < 100){
        return false;
    }

    cloud_plane = cloud_filtered;
    return true;


    //----------------- Filter out spurious stuff in the environment (Assume table is largest cluster) -----------



/*
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);

    if  (cluster_indices.size() > 0){
        pcl::PointIndices& indices = cluster_indices[0];

        size_t size = indices.indices.size();

        ROS_DEBUG("Indices size: %d", size);

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
    */
}

void getCandidateClusters(boost::shared_ptr< pcl::PointCloud<PointT> >& cloud, std::vector<pcl::PointIndices>& cluster_indices)
{
  //Creating the KdTree object for the search method of the extraction
  if (!cloud->empty()){
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (cloud);

    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (0.05);
    ec.setMinClusterSize (800);
    ec.setMaxClusterSize (2400);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    ec.extract (cluster_indices);
    ROS_DEBUG("Extraction done: %d", cluster_indices.size());
  }
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

void callback(const sensor_msgs::Image::ConstPtr &image_msg,
              const sensor_msgs::CameraInfo::ConstPtr& info_msg,
              const sensor_msgs::PointCloud2::ConstPtr& pc_msg){
  //ROS_INFO("Callback");
  //ROS_INFO("image: %s", image_msg->header.frame_id.c_str());
  //ROS_INFO("info: %s", info_msg->header.frame_id.c_str());
  //ROS_INFO("pc: %s", pc_msg->header.frame_id.c_str());
  ros::WallTime start_proc_time = ros::WallTime::now();

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
  //ROS_INFO("Original: %d", cloud->size());

  filterVoxel(cloud, 0.02);


  //filterDepth(cloud);
  //ROS_INFO("Depth: %d", cloud->size());
  //filterHeight(cloud);
  //ROS_INFO("Height: %d", cloud->size());

  tf::StampedTransform trans_to_base_link;
  try{
    listener->lookupTransform("base_link", pc_msg->header.frame_id,
                              ros::Time(0), trans_to_base_link);
  }
  catch (tf::TransformException &ex) {
    ROS_ERROR("Lookup Transform failed: %s",ex.what());
    return;
  }

  tf::transformTFToEigen(trans_to_base_link, to_base_link_);

  tf::StampedTransform transform_to_rgb_frame;
  try{
    listener->lookupTransform(pc_msg->header.frame_id, image_msg->header.frame_id,
                              ros::Time(0), transform_to_rgb_frame);
  }
  catch (tf::TransformException &ex) {
    ROS_ERROR("Lookup Transform failed: %s",ex.what());
    return;
  }


  // Transform to base_link
  boost::shared_ptr< pcl::PointCloud<PointT> > cloud_tmp(new pcl::PointCloud<PointT>);
  pcl::transformPointCloud(*cloud, *cloud_tmp, to_base_link_);
  cloud = cloud_tmp;

  // Filter height in base_link frame
  filterHeight(cloud, "z", height_cutoff_min, height_cutoff_max);

  // Publish filtered cloud to ROS for debugging
  if (plane_pub.getNumSubscribers() > 0){
    sensor_msgs::PointCloud2 filtered_msg;
    pcl::toROSMsg(*cloud, filtered_msg);
    filtered_msg.header.frame_id = "base_link";
    plane_pub.publish(filtered_msg);
  }

  boost::shared_ptr< pcl::PointCloud<PointT> > cloud_plane(new pcl::PointCloud<PointT>);
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointCloud<pcl::PointXYZ>::Ptr convex_hull (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);


  std::vector<pcl::PointIndices> cluster_indices;

  getCandidateClusters(cloud, cluster_indices);

  int max_clusters = std::min(5, (int)cluster_indices.size());

  for (int i = 0; i < max_clusters; ++i){

    cloud_cluster->clear();

    pcl::PointIndices& indices = cluster_indices[i];

    size_t size = indices.indices.size();

    ROS_DEBUG("Indices size: %d", size);

    for (size_t i = 0; i < size; ++i){
      cloud_cluster->points.push_back(cloud->points[indices.indices[i]]);
    }

    cloud_cluster->width = cloud_cluster->points.size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    if(!segmentDigitPlane(cloud_cluster, cloud_plane, coefficients, convex_hull)){
      ROS_DEBUG("Segmentation failed");
      break;
    }

    const std::vector<Eigen::Vector3f> plane_rect_base_link = getPlaneRectangle(cloud_plane, coefficients);

    publishPolygonMsg(plane_rect_base_link, pc_msg->header);

    image_geometry::PinholeCameraModel cam_model;
    cam_model.fromCameraInfo(*info_msg);


    std::vector<RectanglePointData> plane_rect_data_ordered;
    bool is_digit_sign = checkGeometryProperties(plane_rect_base_link, cam_model, transform_to_rgb_frame, plane_rect_data_ordered);

    // Only process image data if we actually have detected a digit
    if (is_digit_sign){

      cv_bridge::CvImage out_msg;
      out_msg.header   = image_msg->header;
      out_msg.encoding = image_msg->encoding;

      getPerspectiveTransformedImage(plane_rect_data_ordered, img, out_msg.image);

      hector_digit_detection_msgs::Image2Digit image2digit;

      out_msg.toImageMsg(image2digit.request.data);

      if (image_pub.getNumSubscribers() > 0){
        sensor_msgs::Image new_msg;
        out_msg.toImageMsg(new_msg);
        image_pub.publish(new_msg);
      }

      ROS_INFO("Plane detect total proc time: %f milliseconds", (ros::WallTime::now() - start_proc_time).toSec()*1000.0);

      if(digit_service.call(image2digit)){
        if(image2digit.response.digit >= 0){
          ROS_INFO("Number found: %d", image2digit.response.digit);
          hector_worldmodel_msgs::PosePercept percept;
          percept.header.stamp = pc_msg->header.stamp;
          percept.header.frame_id = "base_link";
          percept.info.class_id = "unload_fiducial";
          percept.info.class_support = 1.0;

          //percept.info.object_support = 0.01;
          std::stringstream sstr;
          sstr << image2digit.response.digit;
          sstr >> percept.info.name;

          Eigen::Vector3f centroid_coords(
                (plane_rect_data_ordered[0].point_base_link +
                 plane_rect_data_ordered[1].point_base_link +
                 plane_rect_data_ordered[2].point_base_link +
                 plane_rect_data_ordered[3].point_base_link) * 0.25 );

          percept.pose.pose.position.x = centroid_coords.x();
          percept.pose.pose.position.y = centroid_coords.y();
          percept.pose.pose.position.z = centroid_coords.z();

          percept.pose.pose.orientation = tf::createQuaternionMsgFromYaw(atan2(centroid_coords.y(), centroid_coords.x()));
          percept_publisher_.publish(percept);
        }else{
          ROS_INFO("No Number found");
        }
      }else{
        ROS_ERROR("Service call failed");
      }
    }
  }
}

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::PointCloud2> ApproximateTimeSyncPolicy;

int main(int argc, char** argv)
{

  ros::init(argc, argv, "plane_detection");
  ros::NodeHandle nh_("~");

  nh_.getParam("sign_width", sign_width);
  nh_.getParam("sign_height", sign_height);
  nh_.getParam("width_tolerance", sign_width_tolerance);
  nh_.getParam("height_tolerance", sign_height_tolerance);
  nh_.getParam("height_cutoff_min", height_cutoff_min);
  nh_.getParam("height_cutoff_max", height_cutoff_max);

  image_transport::ImageTransport it_(nh_);


  tf::TransformListener l;
  listener = &l;

  image_pub = nh_.advertise<sensor_msgs::Image>("/plane_image",1);
  poly_pub = nh_.advertise<geometry_msgs::PolygonStamped>("/digit_plane_poly",1);
  plane_pub = nh_.advertise<sensor_msgs::PointCloud2>("/plane", 1);
  percept_publisher_ = nh_.advertise<hector_worldmodel_msgs::PosePercept>("/worldmodel/pose_percept", 1);

  image_transport::SubscriberFilter image_sub;
  message_filters::Subscriber<sensor_msgs::CameraInfo> info_sub;
  message_filters::Subscriber<sensor_msgs::PointCloud2> pc_sub;
  message_filters::Synchronizer<ApproximateTimeSyncPolicy> sync_(ApproximateTimeSyncPolicy(5), image_sub, info_sub, pc_sub);

  image_sub.subscribe(it_,"/camera/rgb/image_raw", 1);
  info_sub.subscribe(nh_, "/camera/rgb/camera_info", 1);
  pc_sub.subscribe(nh_, "/camera/depth/points", 1);
  sync_.registerCallback(boost::bind(&callback, _1, _2, _3));

  digit_service = nh_.serviceClient<hector_digit_detection_msgs::Image2Digit>("/image2digit");

  ros::spin();

  return 0;
}
