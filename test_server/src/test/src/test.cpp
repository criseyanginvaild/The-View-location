#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/String.h>
#include <iostream>
#include <opencv2/opencv.hpp>

ros::Publisher pub;

// Global variables to hold the latest image
cv::Mat currentImage;
bool imageUpdated = false;

// Callback function for mouse events
void onMouse(int event, int x, int y, int flags, void* userdata)
{
    if (event == cv::EVENT_LBUTTONDOWN && !currentImage.empty())
    {
        // Create PointStamped message
        geometry_msgs::PointStamped point_msg;
        point_msg.header.stamp = ros::Time::now();
        point_msg.header.frame_id = "camera_frame";  // Set the appropriate frame_id
        point_msg.point.x = x;
        point_msg.point.y = y;
        point_msg.point.z = 0;  // Since this is a 2D point, z is set to 0
        pub.publish(point_msg);
    }
}

// Callback function to handle incoming images
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        currentImage = cv_bridge::toCvCopy(msg, "bgr8")->image;
        imageUpdated = true;
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_listener");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);

    // Publisher for the clicked point's info
    pub = nh.advertise<geometry_msgs::PointStamped>("clicked_point", 1000);

    // Subscribe to input video feed and publish output
    image_transport::Subscriber sub = it.subscribe("camera/transcolor_image", 1, imageCallback);
    // OpenCV Window setup
    cv::namedWindow("View Image", cv::WINDOW_AUTOSIZE);
    cv::setMouseCallback("View Image", onMouse, NULL);

    ros::Rate loop_rate(5); // Loop at 5 Hz
    while (ros::ok())
    {
        if (imageUpdated)
        {
            cv::imshow("View Image", currentImage);
            cv::waitKey(30);
            imageUpdated = false;
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    cv::destroyWindow("View Image");
    return 0;
}

