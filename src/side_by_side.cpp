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


// If non-zero, outputWidth and outputHeight set the size of the output images.
// If zero, the outputWidth is set to 1/2 the width of the input image, and
// outputHeight is the same as the height of the input image.
int outputWidth, outputHeight;

// Left and right image publishers.
image_transport::Publisher leftImagePublisher;
image_transport::Publisher rightImagePublisher;

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
        if (bool use_scaled = (outputWidth > 0 && outputHeight > 0))
        {
            cv::Mat leftScaled, rightScaled;
            cv::Size sz = cv::Size(outputWidth, outputHeight);
            cv::resize(leftImage, leftScaled, sz);
            cv::resize(rightImage, rightScaled, sz);

        // Publish.
        cv_bridge::CvImage cvImage;
        cvImage.encoding = "bgr8";
        if (leftImagePublisher.getNumSubscribers() > 0)
        {
            cvImage.image = use_scaled ? leftScaled : leftImage;
            leftImagePublisher.publish(cvImage.toImageMsg());
        }
        if (rightImagePublisher.getNumSubscribers() > 0)
        {
            cvImage.image = use_scaled ? rightScaled : rightImage;
            rightImagePublisher.publish(cvImage.toImageMsg());
        }
    }
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "side_by_side_stereo_splitter_node");
    ros::NodeHandle nh("~");

    image_transport::ImageTransport it(nh);

    // load node settings
    std::string inputInputTopic, leftOutputImageTopic, rightOutputImageTopic;
    nh.param("input_image_topic", inputImageTopic, std::string("not_set"));
    nh.param("left_output_image_topic", leftOutputImageTopic,
        std::string("/stereo/left/image_raw"));
    nh.param("right_output_image_topic", rightOutputImageTopic,
        std::string("/stereo/right/image_raw"));
    nh.param("output_width", outputWidth, 0);
    nh.param("output_height", outputHeight, 0);

    // register publishers and subscriber
    ros::Subscriber imageSub = nh.subscribe(
        inputImageTopic.c_str(), 2, &imageCallback);
    leftImagePublisher = it.advertise(leftOutputImageTopic.c_str(), 1);
    rightImagePublisher = it.advertise(rightOutputImageTopic.c_str(), 1);

    // run node until cancelled
    ros::spin();
}
