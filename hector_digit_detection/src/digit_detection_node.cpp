#include <ros/ros.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/ml/ml.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>
#include <hector_worldmodel_msgs/ImagePercept.h>
#include <sensor_msgs/image_encodings.h>

#include <string>
#include <sstream>
#include <vector>

using cv::CHAIN_APPROX_SIMPLE;
using cv::COLOR_BGR2GRAY;
using cv::imread;
using cv::KNearest;
using cv::Mat;
using cv::namedWindow;
using cv::Point;
using cv::Rect;
using cv::RETR_LIST;
using cv::Size;
using cv::Scalar;
using cv::waitKey;

using std::string;
using std::stringstream;
using std::vector;

ros::Publisher digit_publisher_;
bool debug_;

CvMat* trainData;
CvMat* trainClasses;
void setup_classifier();

string img_path;

KNearest * knearest = NULL;

const int train_samples = 1;
const int classes = 10;
const int sizex = 20;
const int sizey = 30;
const int ImageSize = sizex * sizey;
char pathToImages[] = "images";

void imageCallback(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::CameraInfoConstPtr& info_msg);
void learnFromImages(CvMat* trainData, CvMat* trainClasses);
void preProcessImage(Mat *inImage, Mat *outImage, int sizex, int sizey);
void runSelfTest(KNearest& knn2);
int analyseImage(Mat &image, KNearest &knearest);


int main(int argc, char **argv)
{
  debug_ = true;
  ros::init(argc, argv, "digit_detection");
  ros::NodeHandle nh("~");

  nh.getParam("img_path", img_path);

  digit_publisher_ = nh.advertise<hector_worldmodel_msgs::ImagePercept>("digit_percept", 10);



  setup_classifier();
  std::cout << "END SETUP CLASSIFIER" << std::endl;

  image_transport::ImageTransport it(nh);
  //image_transport::CameraSubscriber sub_camera_ = it.subscribeCamera("camera", 3, imageCallback);
  image_transport::CameraSubscriber sub_camera_ = it.subscribeCamera("/camera/rgb/image_raw", 3, imageCallback);

  //image_transport::Subscriber sub = it.subscribe("/camera/rgb/image_raw", 1, imageCallback);

  ros::spin();

  if(knearest)
  {
    delete knearest;
    knearest = NULL;
  }

  if (debug_){
    namedWindow("Circles");
    namedWindow("Lines");
  }

  //cv::namedWindow("Template");
  //templ = cv::imread("template.jpg");

  if (debug_){
    cv::destroyWindow("Circles");
    cv::destroyWindow("Lines");
  }

  //cv::destroyWindow("Template");

  return 0;
}


void imageCallback(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::CameraInfoConstPtr& info_msg)
{
  cv_bridge::CvImageConstPtr cv_ptr;
  try
  {
    if (sensor_msgs::image_encodings::isColor(msg->encoding))
      cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
    else
      cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  ROS_DEBUG("Image received");
  Mat image_big, image, image_gray;
  image_big = cv_ptr->image.clone();
  // cv::resize(image_big, image, Size(sizex, sizey));

  // image_circles = cv_ptr->image.clone();
  // image_lines = cv_ptr->image.clone();
  // image_template = cv_ptr->image.clone();
  // cv::cvtColor( cv_ptr->image, image_gray, CV_BGR2GRAY ); // Convert image to gray
  // cv::GaussianBlur( image_gray, image_gray, cv::Size(9, 9), 2, 2 ); // Reduce the noise so we avoid false circle detection

  ROS_INFO("BEGIN analyseImage");
  int digit = analyseImage(image_big, *knearest);
  ROS_INFO("END analyseImage");

  std::stringstream sstr;
  sstr << digit;

  if (0 <= digit && digit <= 9)
  {
    // extract results
    hector_worldmodel_msgs::ImagePercept percept;
    percept.header = msg->header;
    percept.camera_info = *info_msg;
    percept.info.class_id = "digit_detection";
    percept.info.class_support = 1.0;

    percept.info.object_support = 1.0;


    percept.info.name = sstr.str().c_str();

    percept.x      = 0;
    percept.y      = 0;
    percept.width  = 0;
    percept.height = 0;
    digit_publisher_.publish(percept);
  }
  //analyseImage(knearest);

}

void setup_classifier()
{
  CvMat* trainData = cvCreateMat(classes * train_samples,ImageSize, CV_32FC1);
  CvMat* trainClasses = cvCreateMat(classes * train_samples, 1, CV_32FC1);

  namedWindow("single", CV_WINDOW_AUTOSIZE);
  namedWindow("all", CV_WINDOW_AUTOSIZE);
  learnFromImages(trainData, trainClasses);
  if(knearest)
  {
    delete knearest;
    knearest = NULL;
  }
  knearest = new KNearest(trainData, trainClasses);
  runSelfTest(*knearest);
  ROS_INFO("Setup classifier correctly.");
}

void learnFromImages(CvMat* trainData, CvMat* trainClasses)
{
  Mat img;
  for (int i = 0; i < classes; i++)
  {
    stringstream sstrFile;
    sstrFile << img_path << "/" << i << ".png";
    img = imread(sstrFile.str().c_str(), 1);
    if (!img.data)
    {
      ROS_INFO("File %s not found\n", sstrFile.str().c_str());
      exit(1);
    }
    Mat outfile;
    preProcessImage(&img, &outfile, sizex, sizey);
    for (int n = 0; n < ImageSize; n++)
    {
      trainData->data.fl[i * ImageSize + n] = outfile.data[n];
    }
    trainClasses->data.fl[i] = i;
  }
}

void preProcessImage(Mat *inImage,Mat *outImage, int sizex, int sizey)
{
  Mat grayImage,blurredImage,thresholdImage,contourImage,regionOfInterest;
  vector<vector<Point> > contours;
  if(inImage->empty())
  {
    ROS_INFO("preProcessImage: Sorry, no data!");
    exit(1);
  }
  else if(inImage->channels() > 1)
    cvtColor(*inImage, grayImage , COLOR_BGR2GRAY);
  else
    grayImage = *inImage;


  GaussianBlur(grayImage, blurredImage, Size(5, 5), 2, 2);
  adaptiveThreshold(blurredImage, thresholdImage, 255, 1, 1, 11, 2);

  thresholdImage.copyTo(contourImage);

  findContours(contourImage, contours, RETR_LIST, CHAIN_APPROX_SIMPLE);

  int idx = 0;
  size_t area = 0;
  for (size_t i = 0; i < contours.size(); i++)
  {
    if (area < contours[i].size() )
    {
      idx = i;
      area = contours[i].size();
    }
  }

  Rect rec = boundingRect(contours[idx]);
  regionOfInterest = thresholdImage(rec);
  resize(regionOfInterest,*outImage, Size(sizex, sizey));
}

void runSelfTest(KNearest& knn2)
{
  Mat img;
  CvMat* sample2 = cvCreateMat(1, ImageSize, CV_32FC1);
  // SelfTest
  int z = 0;
  while (z++ < 10)
  {
    int iSecret = rand() % 10;
    stringstream sstrFile;
    sstrFile << img_path << "/" << iSecret << ".png";
    img = imread(sstrFile.str().c_str(), 1);

    Mat stagedImage;
    preProcessImage(&img, &stagedImage, sizex, sizey);
    for (int n = 0; n < ImageSize; n++)
    {
      sample2->data.fl[n] = stagedImage.data[n];
    }
    float detectedClass = knn2.find_nearest(sample2, 1);

    if (iSecret != (int) ((detectedClass)))
    {
      ROS_INFO("Falsch. Ist %d aber geraten ist %d .", iSecret, (int) ((detectedClass)));
      exit(1);
    }
    // imshow("single", img);
    // waitKey(0);
  }  ROS_INFO("END imageCallback");

}

int analyseImage(Mat & image, KNearest & knearest)
{
  int imgSize = image.size().height * image.size().width;

  ROS_INFO("READY 2 EXECUTE cvCreateMat");
  CvMat* sample2 = cvCreateMat(1, ImageSize, CV_32FC1);
  Mat gray, blur, thresh;
  vector < vector<Point> > contours;

  ROS_INFO("READY 2 EXECUTE cvtColor");
  if(image.empty())
  {
    ROS_INFO("analyseImage: Sorry, no data!");
    exit(1);
  }
  else if(image.channels() > 1)
    cvtColor(image, gray, COLOR_BGR2GRAY);
  else
    gray = image;
  GaussianBlur(gray, blur, Size(5, 5), 2, 2);
  adaptiveThreshold(blur, thresh, 255, 1, 1, 11, 2);
  findContours(thresh, contours, RETR_LIST, CHAIN_APPROX_SIMPLE);

  ROS_INFO("READY 2 EXECUTE preProcessImage");

  float result;
  Mat stagedImage;
  preProcessImage(&thresh, &stagedImage, sizex, sizey);
  for (int n = 0; n < ImageSize; n++)
  {
    sample2->data.fl[n] = stagedImage.data[n];
  }

  ROS_INFO("READY 2 EXECUTE find_nearest");

  result = knearest.find_nearest(sample2, 1);

  ROS_INFO("RESULT = %f", result);


  // rectangle(image, Point(rec.x, rec.y),
  //          Point(rec.x + rec.width, rec.y + rec.height),
  //            Scalar(0, 0, 255), 2);
  // imshow("all", image);
  // cout << result << "\n";
  // imshow("single", stagedImage);
  // waitKey(0);

  float result2;
  for (size_t i = 0; i < contours.size(); i++)
  {
    vector < Point > cnt = contours[i];
    if (contourArea(cnt) > 50)
    {
      Rect rec = boundingRect(cnt);
      if (rec.height > 28)
      {
        Mat roi = image(rec);
        Mat stagedImage;
        preProcessImage(&roi, &stagedImage, sizex, sizey);
        for (int n = 0; n < ImageSize; n++)
        {
          sample2->data.fl[n] = stagedImage.data[n];
        }
        result2 = knearest.find_nearest(sample2, 1);
        rectangle(image, Point(rec.x, rec.y),
                  Point(rec.x + rec.width, rec.y + rec.height),
                  Scalar(0, 0, 255), 2);

        imshow("all", image);
        std::cout << "RES2 = " << result2 << "\n";


        imshow("single", stagedImage);
        waitKey(0);
      }

    }

  }

  return (int)((result));
}
