#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <vector>

cv::Mat templ;
bool debug_;

void matchCircles(cv::Mat &image_gray, std::vector<cv::Vec3f> &circles){
    cv::HoughCircles( image_gray, circles, CV_HOUGH_GRADIENT, 1, 10); // Apply the Hough Transform to find the circles
}

void drawFoundCircles(cv::Mat &src, std::vector<cv::Vec3f> &circles){
    ROS_DEBUG("Found Circles: %d", circles.size());
    for( size_t i = 0; i < circles.size(); i++ )
    {
        cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
        int radius = cvRound(circles[i][2]);
        // circle center
        cv::circle( src, center, 3, cv::Scalar(0,255,0), -1, 8, 0 );
        // circle outline
        cv::circle( src, center, radius, cv::Scalar(0,0,255), 3, 8, 0 );
    }
}

void matchLines(cv::Mat &image_gray, std::vector<cv::Vec2f> &lines){
    cv::Mat edges;
    cv::Canny(image_gray,edges, 50, 100);

    cv::HoughLines(edges, lines, 1, CV_PI/180, 65);
}

struct lineIsInvalid {
    bool operator()(const cv::Vec2f& line) const {
        float theta = line[1];
        return ((theta>CV_PI/180*10 && theta<CV_PI/180*80) || (theta>CV_PI/180*100 && theta<CV_PI/180*170) || (theta>CV_PI/180*190 && theta<CV_PI/180*260) ||(theta>CV_PI/180*280 && theta<CV_PI/180*350));
    }
};

void filterLines(std::vector<cv::Vec2f> &lines){
    lines.erase(std::remove_if( lines.begin(), lines.end(), lineIsInvalid()), lines.end());
}

void matchLinesStandart(cv::Mat &image_gray, std::vector<cv::Vec2f> &lines){
    cv::HoughLines(image_gray, lines, 1, CV_PI, 300, 0, 0 );
}
void matchLinesProbabilistic(cv::Mat &image_gray, std::vector<cv::Vec4i> &lines){
    cv::HoughLinesP(image_gray, lines, 1, CV_PI/180, 500, 550, 1 );
}

void drawFoundLines(cv::Mat &src, std::vector<cv::Vec2f> &lines){
    ROS_DEBUG("Found Lines: %d", lines.size());
    for( size_t i = 0; i < lines.size(); i++ )
    {
        float rho = lines[i][0], theta = lines[i][1];

        // if( theta>CV_PI/180*170 || theta<CV_PI/180*10){
        cv::Point pt1, pt2;
        double a = cos(theta), b = sin(theta);
        double x0 = a*rho, y0 = b*rho;
        pt1.x = cvRound(x0 + 1000*(-b));
        pt1.y = cvRound(y0 + 1000*(a));
        pt2.x = cvRound(x0 - 1000*(-b));
        pt2.y = cvRound(y0 - 1000*(a));
        cv::line( src, pt1, pt2, cv::Scalar(0,0,255), 1, CV_AA);
        //}
    }
}

void drawFoundLines(cv::Mat &src, std::vector<cv::Vec4i> &lines){
    ROS_DEBUG("Found Lines: %d", lines.size());
    for( size_t i = 0; i < lines.size(); i++ )
    {
        cv::Vec4i l = lines[i];
        cv::line( src, cv::Point(l[0], l[1]), cv::Point(l[2], l[3]), cv::Scalar(0,0,255), 3, CV_AA);
    }
}

void matchTemplate(cv::Mat &img, cv::Mat &result, int match_method){
    // Create the result matrix
    int result_cols =  img.cols - templ.cols + 1;
    int result_rows = img.rows - templ.rows + 1;

    result.create( result_cols, result_rows, CV_32FC1 );

    /// Do the Matching and Normalize
    cv::matchTemplate( img, templ, result, match_method );
    cv::normalize( result, result, 0, 1, cv::NORM_MINMAX, -1, cv::Mat() );

    /// Localizing the best match with minMaxLoc
    double minVal; double maxVal; cv::Point minLoc; cv::Point maxLoc;
    cv::Point matchLoc;

    cv::minMaxLoc( result, &minVal, &maxVal, &minLoc, &maxLoc, cv::Mat() );

    if( match_method  == CV_TM_SQDIFF || match_method == CV_TM_SQDIFF_NORMED )
    { matchLoc = minLoc; }
    else
    { matchLoc = maxLoc; }

    /// Show me what you got
    cv::rectangle( img, matchLoc, cv::Point( matchLoc.x + templ.cols , matchLoc.y + templ.rows ), cv::Scalar::all(0), 2, 8, 0 );
    cv::rectangle( result, matchLoc, cv::Point( matchLoc.x + templ.cols , matchLoc.y + templ.rows ), cv::Scalar::all(0), 2, 8, 0 );
}


void imageCallback(const sensor_msgs::ImageConstPtr& msg)
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
    cv::cvtColor( cv_ptr->image, image_gray, CV_BGR2GRAY ); // Convert image to gray
    cv::GaussianBlur( image_gray, image_gray, cv::Size(9, 9), 2, 2 ); // Reduce the noise so we avoid false circle detection

    std::vector<cv::Vec3f> circles;
    matchCircles(image_gray, circles);

    std::vector<cv::Vec2f> lines;
    matchLines(image_gray, lines);
    filterLines(lines);


    /*
     std::vector<cv::Vec4i> linesProb;
    matchLinesProbabilistic(image_gray, linesProb);
    drawFoundLines(image_lines, linesProb);
*/
    //matchTemplate(image, image_template, 1);

    if (debug_){
      drawFoundCircles(image_circles, circles);
      drawFoundLines(image_lines, lines);

      cv::imshow("Circles", image_circles);
      cv::imshow("Lines", image_lines);
      //cv::imshow("Template", image_template);
      cv::waitKey(3);
    }

}

int main(int argc, char **argv)
{
    debug_ = false;

    ros::init(argc, argv, "target_detection");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("/camera/rgb/image_raw", 1, imageCallback);

    if (debug_){
      cv::namedWindow("Circles");
      cv::namedWindow("Lines");
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
