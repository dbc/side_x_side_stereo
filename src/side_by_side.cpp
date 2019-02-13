// (MIT License)
//
// Copyright 2019 David B. Curtis
//
// Permission is hereby granted, free of charge, to any person obtaining a
// copy of this software and associated documentation files (the "Software"),
// to deal in the Software without restriction, including without limitation
// the rights to use, copy, modify, merge, publish, distribute, sublicense,
// and/or sell copies of the Software, and to permit persons to whom the
// Software is furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included
// in all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
// OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
// DEALINGS IN THE SOFTWARE.
//
///////////////////////////////////////////////////////////////////////////
//
// Subscribes to an image stream of side-by-side stereo 
// where each image message consists of a left and right
// image concatenated to form a single double-wide image. 
// This node splits the incoming image down the middle
// and republishes each half as stereo/left and stereo/right
// images.
//
// Modified version of public domain code posted by 
// PeteBlackerThe3rd in response to my question on ROS Answers:
// https://answers.ros.org/question/315298/splitting-side-by-side-video-into-stereoleft-stereoright/
// -- dbc

#include <ros/ros.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>

// global controlling which half of the image will be re-published
std::string side;

// size of the output image
int outputWidth, outputHeight;

image_transport::Publisher rePublisher;

// Image capture callback
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    // get double camera image
    cv_bridge::CvImagePtr cvImg = cv_bridge::toCvCopy(msg, "rgb8");
    cv::Mat image = cvImg->image;

    // if there are any subscribers to the topic then publish image on it
    if (rePublisher.getNumSubscribers() > 0)
    {
        // define the relevant rectangle to crop
        cv::Rect ROI;
        ROI.y = 0;
        ROI.width = image.cols / 2;
        ROI.height = image.rows;
        if (side == "left")
            ROI.x = 0;
        else
            ROI.x = image.cols / 2;

        // crop image and publish
        cv::Mat croppedImage = cv::Mat(image, ROI);

        cv::Mat scaledImage;
        cv::resize(croppedImage,
                   scaledImage,
                   cv::Size(outputWidth, outputHeight) );

        cv_bridge::CvImage cvImage;
        cvImage.image = scaledImage;
        cvImage.encoding = "bgr8";
        rePublisher.publish(cvImage.toImageMsg());
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "half_image_publisher_node");
    ros::NodeHandle nh("~");

    image_transport::ImageTransport it(nh);

    // load node settings
    std::string inputTopic, outputTopic;
    nh.param("image_side", side, std::string("left"));
    nh.param("input_topic", inputTopic, std::string("not_set"));
    nh.param("output_topic", outputTopic, std::string("not_set"));
    nh.param("width", outputWidth, 640);
    nh.param("height", outputHeight, 480);

    // register publisher and subscriber
    ros::Subscriber imageSub = nh.subscribe(inputTopic.c_str(), 2, &imageCallback);

    rePublisher = it.advertise(outputTopic.c_str(), 1);

    // run node until cancelled
    ros::spin();
}
