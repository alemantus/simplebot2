#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>

class StereoImageCompressor : public rclcpp::Node
{
public:
    StereoImageCompressor()
        : Node("stereo_image_compressor")
    {
        // Subscribe to the left and right image topics
        left_image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "stereo/left/image_raw", 10,
            std::bind(&StereoImageCompressor::leftImageCallback, this, std::placeholders::_1));

        right_image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "stereo/right/image_raw", 10,
            std::bind(&StereoImageCompressor::rightImageCallback, this, std::placeholders::_1));

        // Publishers for the compressed images
        compressed_left_image_pub_ = this->create_publisher<sensor_msgs::msg::CompressedImage>(
            "stereo/left/compressed_image", 10);
        compressed_right_image_pub_ = this->create_publisher<sensor_msgs::msg::CompressedImage>(
            "stereo/right/compressed_image", 10);
    }

private:
    void leftImageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        processAndPublish(msg, compressed_left_image_pub_);
    }

    void rightImageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        processAndPublish(msg, compressed_right_image_pub_);
    }

    void processAndPublish(const sensor_msgs::msg::Image::SharedPtr &image_msg,
                           const rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr &publisher)
    {
        try
        {
            // Convert the image to OpenCV format
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(image_msg, "bgr8");

            // Compress the image
            std::vector<unsigned char> compressed_data;
            std::vector<int> compression_params = {cv::IMWRITE_JPEG_QUALITY, 90};
            cv::imencode(".jpg", cv_ptr->image, compressed_data, compression_params);

            // Publish the compressed image
            auto compressed_msg = sensor_msgs::msg::CompressedImage();
            compressed_msg.header = image_msg->header;
            compressed_msg.format = "jpeg";
            compressed_msg.data = compressed_data;

            publisher->publish(compressed_msg);
        }
        catch (const cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "CV Bridge error: %s", e.what());
        }
        catch (const cv::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "OpenCV error: %s", e.what());
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr left_image_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr right_image_sub_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_left_image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_right_image_pub_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StereoImageCompressor>());
    rclcpp::shutdown();
    return 0;
}
