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
using cv::waitKey;

using std::string;
using std::stringstream;
using std::vector;

ros::Publisher digit_publisher_;
bool debug_;

CvMat* trainData;
CvMat* trainClasses;
void setupClassifier();

string img_path;

KNearest * knearest = NULL;

void imageCallback(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::CameraInfoConstPtr& info_msg);
void learnFromImages(CvMat* trainData, CvMat* trainClasses);
void preProcessImage(Mat *inImage, Mat *outImage, int sizex, int sizey);
void runSelfTest(KNearest& knn2);

int main(int argc, char **argv)
{
    debug_ = true;
    ros::init(argc, argv, "digit_detection");
    ros::NodeHandle nh("~");

    nh.getParam("img_path", img_path);

    digit_publisher_ = nh.advertise<hector_worldmodel_msgs::ImagePercept>("digit_percept", 10);

    image_transport::ImageTransport it(nh);
    image_transport::CameraSubscriber sub_camera_ = it.subscribeCamera("camera", 3, imageCallback);

    setupClassifier();
    std::cout << "END SETUP CLASSIFIER" << std::endl;


    //image_transport::Subscriber sub = it.subscribe("/camera/rgb/image_raw", 1, imageCallback);

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
    ros::spin();

    if (debug_){
      cv::destroyWindow("Circles");
      cv::destroyWindow("Lines");
    }

    //cv::destroyWindow("Template");
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
    cv::Mat image, image_gray, image_circles, image_lines, image_template;
    image = cv_ptr->image.clone();
    image_circles = cv_ptr->image.clone();
    image_lines = cv_ptr->image.clone();
    image_template = cv_ptr->image.clone();
    // cv::cvtColor( cv_ptr->image, image_gray, CV_BGR2GRAY ); // Convert image to gray
    // cv::GaussianBlur( image_gray, image_gray, cv::Size(9, 9), 2, 2 ); // Reduce the noise so we avoid false circle detection

    //analyseImage(knearest);

}

const int train_samples = 1;
const int classes = 10;
const int sizex = 20;
const int sizey = 30;
const int ImageSize = sizex * sizey;
char pathToImages[] = "images";

void setupClassifier()
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
    cvtColor(*inImage,grayImage , COLOR_BGR2GRAY);

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
    }
}