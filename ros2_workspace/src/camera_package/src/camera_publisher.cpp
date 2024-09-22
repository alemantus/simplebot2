#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/videodev2.h>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <memory>

class StereoCameraNode
{
public:
    StereoCameraNode()
        : node_(std::make_shared<rclcpp::Node>("stereo_camera_node"))
    {
        // Open video device
        openDevice("/dev/video0", cap_fd);

        // Create publishers for raw and compressed images
        pub_left_ = node_->create_publisher<sensor_msgs::msg::Image>("stereo/left/image_raw", 10);
        pub_right_ = node_->create_publisher<sensor_msgs::msg::Image>("stereo/right/image_raw", 10);
        pub_left_compressed_ = node_->create_publisher<sensor_msgs::msg::CompressedImage>("stereo/left/image_raw/compressed", 10);
        pub_right_compressed_ = node_->create_publisher<sensor_msgs::msg::CompressedImage>("stereo/right/image_raw/compressed", 10);

        // Timer to publish images
        timer_ = node_->create_wall_timer(
            std::chrono::milliseconds(200), // ~30 FPS
            [this]() { publishImages(); });
    }

    void spin() {
        rclcpp::spin(node_);
    }

private:
    void openDevice(const std::string &device_path, int &fd)
    {
        fd = open(device_path.c_str(), O_RDWR | O_NONBLOCK, 0);
        if (fd < 0) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to open %s", device_path.c_str());
            rclcpp::shutdown();
        }

        // Set format
        struct v4l2_format fmt;
        memset(&fmt, 0, sizeof(fmt));
        fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
        fmt.fmt.pix.width = 1280; // Width for HD720
        fmt.fmt.pix.height = 720;
        if (ioctl(fd, VIDIOC_S_FMT, &fmt) < 0) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to set format for %s", device_path.c_str());
            close(fd);
            rclcpp::shutdown();
        }
    }

    void publishImages()
    {
        cv::Mat frame;

        // Capture frame from video0
        if (captureFrame(cap_fd, frame)) {
            // Split the frame into left and right images
            cv::Mat left_image = frame(cv::Rect(0, 0, frame.cols / 2, frame.rows));
            cv::Mat right_image = frame(cv::Rect(frame.cols / 2, 0, frame.cols / 2, frame.rows));

            // Publish left image
            publishImage(left_image, pub_left_, pub_left_compressed_, "camera_left_frame");
            // Publish right image
            publishImage(right_image, pub_right_, pub_right_compressed_, "camera_right_frame");
        }
    }

    void publishImage(const cv::Mat& image, 
                      const rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr& raw_pub, 
                      const rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr& compressed_pub, 
                      const std::string& frame_id)
    {
        // Publish raw image
        auto image_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image).toImageMsg();
        image_msg->header.stamp = node_->get_clock()->now();
        image_msg->header.frame_id = frame_id;
        raw_pub->publish(*image_msg);

        // Publish compressed image
        sensor_msgs::msg::CompressedImage compressed_image_msg;
        cv_bridge::CvImage(image_msg->header, "bgr8", image).toCompressedImageMsg(compressed_image_msg);
        compressed_pub->publish(compressed_image_msg);
    }

    bool captureFrame(int fd, cv::Mat &frame)
    {
        // Placeholder for actual frame capturing logic
        frame = cv::Mat(720, 2560, CV_8UC3); // Example dimensions
        // Actual frame capturing code goes here
        return true;
    }

    // ROS node
    std::shared_ptr<rclcpp::Node> node_;

    // File descriptor for the video device
    int cap_fd;

    // Image publishers
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_left_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_right_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr pub_left_compressed_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr pub_right_compressed_;

    // Timer to control the frame capture rate
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    StereoCameraNode stereo_camera_node; // Create the node object
    stereo_camera_node.spin(); // Spin the node
    rclcpp::shutdown();
    return 0;
}
