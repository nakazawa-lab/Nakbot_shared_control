#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>

using namespace cv;

class TemplateMatchingNode
{
private:
    ros::NodeHandle nh_;    

    image_transport::ImageTransport it_;
    image_transport::Subscriber sub_;
    
    const char* template_img_path_ = "/home/${USERNAME}/Pictures/template.png";
    Mat template_img_;
    int match_method_ = CV_TM_SQDIFF;

public:
    TemplateMatchingNode()
    : it_(nh_)
    {
         sub_ = it_.subscribe("/camera/rgb/image_color", 1, &TemplateMatchingNode::camera_callback, this);
         template_img_ = cv::imread(template_img_path_);
    }

    ~TemplateMatchingNode()
    {
    }


    void camera_callback(const sensor_msgs::ImageConstPtr& img_msg)
    {
        Mat img;
        try
        {
            img = cv_bridge::toCvShare(img_msg, "bgr8")->image;
            waitKey(30);
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_ERROR("Could not convert from '%s' to 'bgr8'.", img_msg->encoding.c_str());
        }

        // テンプレートマッチング
        // https://docs.opencv.org/3.4/de/da9/tutorial_template_matching.html
        Mat result;
        int result_cols =  img.cols - template_img_.cols + 1;
        int result_rows = img.rows - template_img_.rows + 1;
        result.create( result_rows, result_cols, CV_32FC1 );

        matchTemplate(img, template_img_, result, match_method_);
        bool method_accepts_mask = (TM_SQDIFF == match_method_ || match_method_ == TM_CCORR_NORMED);
        matchTemplate(img, template_img_, result, match_method_);
        normalize(result, result, 0, 1, NORM_MINMAX, -1, Mat());
        double minVal;
        double maxVal;
        Point minLoc;
        Point maxLoc;
        Point matchLoc;
        minMaxLoc(result, &minVal, &maxVal, &minLoc, &maxLoc, Mat());
        matchLoc = minLoc;
        rectangle(   img, matchLoc, Point(matchLoc.x + template_img_.cols , matchLoc.y + template_img_.rows ), Scalar::all(0), 2, 8, 0 );
        rectangle(result, matchLoc, Point(matchLoc.x + template_img_.cols, matchLoc.y + template_img_.rows), Scalar::all(0), 2, 8, 0);
        imshow("subscribed img", img);
        imshow("result_window", result);
        waitKey(30);
    }
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "matching_node");
    TemplateMatchingNode node = TemplateMatchingNode(); 
    ros::spin();

    return 0;
}