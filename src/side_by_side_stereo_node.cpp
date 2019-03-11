// (MIT License)
//
// Copyright 2019 David B. Curtis
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//
////////////////////////////////////////////////////////////////////////////////
//
// Subscribes to an image stream of side-by-side stereo where each image
// message consists of a left and right image concatenated to form a single
// double-wide image.  This node splits the incoming image down the middle
// and republishes each half as stereo/left and stereo/right images.
//
// This is a modified version of public domain code posted by PeteBlackerThe3rd
// in response to my question on ROS Answers:
// https://answers.ros.org/question/315298/splitting-side-by-side-video-into-stereoleft-stereoright/
//
// -- dbc

#include <ros/ros.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>


// If non-zero, outputWidth and outputHeight set the size of the output images.
// If zero, the outputWidth is set to 1/2 the width of the input image, and
// outputHeight is the same as the height of the input image.
int outputWidth, outputHeight;

// Input image subscriber.
ros::Subscriber imageSub;

// Left and right image publishers.
image_transport::Publisher leftImagePublisher, rightImagePublisher;

// Left and right camera info publishers and messages.
ros::Publisher leftCameraInfoPublisher, rightCameraInfoPublisher;
sensor_msgs::CameraInfo leftCameraInfoMsg, rightCameraInfoMsg;

// Camera info managers.
camera_info_manager::CameraInfoManager *left_cinfo_;
camera_info_manager::CameraInfoManager *right_cinfo_;


// Image capture callback.
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    // Get double camera image.
    cv_bridge::CvImagePtr cvImg = cv_bridge::toCvCopy(msg, "rgb8");
    cv::Mat image = cvImg->image;

    // If there are any subscribers to either output topic then publish images
    // on them.
    if (leftImagePublisher.getNumSubscribers() > 0 ||
        rightImagePublisher.getNumSubscribers() > 0)
    {
        // Define the relevant rectangles to crop.
        cv::Rect leftROI, rightROI;
        leftROI.y = rightROI.y = 0;
        leftROI.width = rightROI.width = image.cols / 2;
        leftROI.height = rightROI.height = image.rows;
        leftROI.x = 0;
        rightROI.x = image.cols / 2;

        // Crop images.
        cv::Mat leftImage = cv::Mat(image, leftROI);
        cv::Mat rightImage = cv::Mat(image, rightROI);

        // Apply scaling, if specified.
        bool use_scaled;
        cv::Mat leftScaled, rightScaled;
        if (use_scaled = (outputWidth > 0 && outputHeight > 0))
        {
            cv::Size sz = cv::Size(outputWidth, outputHeight);
            cv::resize(leftImage, leftScaled, sz);
            cv::resize(rightImage, rightScaled, sz);
        }

        // Publish.
        cv_bridge::CvImage cvImage;
        sensor_msgs::ImagePtr img;
        cvImage.encoding = "rgb8";
        if (leftImagePublisher.getNumSubscribers() > 0
            || leftCameraInfoPublisher.getNumSubscribers() > 0)
        {
            cvImage.image = use_scaled ? leftScaled : leftImage;
            img = cvImage.toImageMsg();
            leftImagePublisher.publish(img);
            leftCameraInfoMsg.header.stamp = img->header.stamp;
            leftCameraInfoPublisher.publish(leftCameraInfoMsg);
        }
        if (rightImagePublisher.getNumSubscribers() > 0
            || rightCameraInfoPublisher.getNumSubscribers() > 0)
        {
            cvImage.image = use_scaled ? rightScaled : rightImage;
            img = cvImage.toImageMsg();
            rightImagePublisher.publish(img);
            rightCameraInfoMsg.header.stamp = img->header.stamp;
            rightCameraInfoPublisher.publish(rightCameraInfoMsg);
        }
    }
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "sxs_stereo");
    ros::NodeHandle nh("sxs_stereo");
    ros::NodeHandle nh_left(nh, "left");
    ros::NodeHandle nh_right(nh, "right");

    image_transport::ImageTransport it(nh);

    // Allocate and initialize camera info managers.
    left_cinfo_ =
        new camera_info_manager::CameraInfoManager(nh_left);
    right_cinfo_ =
        new camera_info_manager::CameraInfoManager(nh_right);
    left_cinfo_->loadCameraInfo("");
    right_cinfo_->loadCameraInfo("");

    // Pre-fill camera_info messages.
    leftCameraInfoMsg = left_cinfo_->getCameraInfo();
    rightCameraInfoMsg = right_cinfo_->getCameraInfo();

    // Load node settings.
    std::string inputImageTopic, leftOutputImageTopic, rightOutputImageTopic,
        leftCameraInfoTopic, rightCameraInfoTopic;
    nh.param("input_image_topic", inputImageTopic, 
        std::string("input_image_topic_not_set"));
    ROS_INFO("input topic to stereo splitter=%s\n", inputImageTopic.c_str());
    nh.param("left_output_image_topic", leftOutputImageTopic,
        std::string("/sxs_stereo/left/image_raw"));
    nh.param("right_output_image_topic", rightOutputImageTopic,
        std::string("/sxs_stereo/right/image_raw"));
    nh.param("left_camera_info_topic", leftCameraInfoTopic,
        std::string("/sxs_stereo/left/camera_info"));
    nh.param("right_camera_info_topic", rightCameraInfoTopic,
        std::string("/sxs_stereo/right/camera_info"));
    nh.param("output_width", outputWidth, 0);  // 0 -> use 1/2 input width.
    nh.param("output_height", outputHeight, 0);  // 0 -> use input height.


    // Register publishers and subscriber.
    imageSub = nh.subscribe(inputImageTopic.c_str(), 2, &imageCallback);
    leftImagePublisher = it.advertise(leftOutputImageTopic.c_str(), 1);
    rightImagePublisher = it.advertise(rightOutputImageTopic.c_str(), 1);
    leftCameraInfoPublisher = nh_left.advertise<sensor_msgs::CameraInfo>
        (leftCameraInfoTopic.c_str(), 1, true);
    rightCameraInfoPublisher = nh_right.advertise<sensor_msgs::CameraInfo>
        (rightCameraInfoTopic.c_str(), 1, true);

    // Run node until cancelled.
    ros::spin();

    // De-allocate CameraInfoManagers.
    left_cinfo_ -> ~CameraInfoManager();
    right_cinfo_ -> ~CameraInfoManager();
}
