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
