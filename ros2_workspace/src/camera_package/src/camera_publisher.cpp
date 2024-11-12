#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <memory>
#include <cstring>
#include <iostream>
#include <vector>

class StereoCameraNode
{
public:
    StereoCameraNode()
        : node(std::make_shared<rclcpp::Node>("stereo_camera_node"))
    {
        // Open the camera using OpenCV VideoCapture
        if (!cap.open(0)) // Open default camera (0 for the first camera)
        {
            RCLCPP_ERROR(node->get_logger(), "Failed to open camera.");
            rclcpp::shutdown();
            return;
        }

        // Set the desired camera properties (optional, but can help if you need specific resolution)
        cap.set(cv::CAP_PROP_FRAME_WIDTH, 1280);  // Set width to 1280
        cap.set(cv::CAP_PROP_FRAME_HEIGHT, 480); // Set height to 720
        //cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));

        // Create publishers for left and right images
        pub_left = node->create_publisher<sensor_msgs::msg::Image>("stereo/left/image_raw", 2);
        pub_right = node->create_publisher<sensor_msgs::msg::Image>("stereo/right/image_raw", 2);

        // Timer to publish images
        timer_ = node->create_wall_timer(
            std::chrono::milliseconds(33), // ~20 FPS
            [this]() { publishImages(); });
    }

    void spin()
    {
        rclcpp::spin(node);
    }

private:
    void publishImages()
    {
        cv::Mat frame;
        if (cap.read(frame)) // Capture a frame from the camera
        {
            // Split frame into left and right images
            cv::Mat left_image = frame(cv::Rect(0, 0, frame.cols / 2, frame.rows));
            cv::Mat right_image = frame(cv::Rect(frame.cols / 2, 0, frame.cols / 2, frame.rows));

            // Publish left and right images
            publishImage(left_image, pub_left, "left_camera_frame");
            publishImage(right_image, pub_right, "right_camera_frame");
        }
        else
        {
            RCLCPP_WARN(node->get_logger(), "Failed to capture frame.");
        }
    }

    void publishImage(const cv::Mat &image, const rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr &raw_pub, const std::string &frame_id)
    {
        auto image_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image).toImageMsg();
        image_msg->header.stamp = node->get_clock()->now();
        image_msg->header.frame_id = frame_id;
        raw_pub->publish(*image_msg);
    }

    std::shared_ptr<rclcpp::Node> node;
    cv::VideoCapture cap; // OpenCV VideoCapture for handling camera input
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_left;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_right;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    StereoCameraNode stereo_camera_node;
    stereo_camera_node.spin();
    rclcpp::shutdown();
    return 0;
}
