#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "usb_cam_custom/action/change_image_mode.hpp"

class ImageModeActionServer : public rclcpp::Node {
public:
    using ChangeImageMode = usb_cam_custom::action::ChangeImageMode;
    using GoalHandleChangeImageMode = rclcpp_action::ServerGoalHandle<ChangeImageMode>;

    ImageModeActionServer()
        : Node("image_mode_action_server"),
          current_mode_(false) {  // Default to color
        // Create the action server
        action_server_ = rclcpp_action::create_server<ChangeImageMode>(
            this,
            "change_image_mode",
            std::bind(&ImageModeActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&ImageModeActionServer::handle_cancel, this, std::placeholders::_1),
            std::bind(&ImageModeActionServer::handle_accepted, this, std::placeholders::_1)
        );

        // Publisher for the output image
        image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("output_image", 10);

        // Subscriber to input image
        image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/image_raw", 10,
            std::bind(&ImageModeActionServer::image_callback, this, std::placeholders::_1)
        );

        RCLCPP_INFO(this->get_logger(), "Image Mode Action Server is up and running.");
    }

private:
    rclcpp_action::Server<ChangeImageMode>::SharedPtr action_server_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
    cv::Mat last_image_;
    bool current_mode_;  // Grayscale mode: true, Color mode: false

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const ChangeImageMode::Goal> goal) {
        RCLCPP_INFO(this->get_logger(), "Received request: grayscale=%d", goal->grayscale);
        (void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleChangeImageMode> goal_handle) {
        RCLCPP_INFO(this->get_logger(), "Cancel request received.");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleChangeImageMode> goal_handle) {
        std::thread([this, goal_handle]() {
            execute(goal_handle);
        }).detach();
    }

    void execute(const std::shared_ptr<GoalHandleChangeImageMode> goal_handle) {
        RCLCPP_INFO(this->get_logger(), "Processing request...");

        auto feedback = std::make_shared<ChangeImageMode::Feedback>();
        auto result = std::make_shared<ChangeImageMode::Result>();

        // Update mode based on the request
        current_mode_ = goal_handle->get_goal()->grayscale;

        // Simulate progress
        for (int i = 0; i <= 5; ++i) {
            if (goal_handle->is_canceling()) {
                result->success = false;
                result->message = "Goal canceled.";
                goal_handle->canceled(result);
                RCLCPP_INFO(this->get_logger(), "Goal canceled.");
                return;
            }

            feedback->progress = i * 20.0f;  // Progress percentage
            goal_handle->publish_feedback(feedback);
            RCLCPP_INFO(this->get_logger(), "Progress: %.1f%%", feedback->progress);
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }

        result->success = true;
        result->message = std::string("Image mode changed to ") + (current_mode_ ? "grayscale" : "color");
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "%s", result->message.c_str());
    }

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        try {
            // Convert ROS image to OpenCV format
            last_image_ = cv_bridge::toCvCopy(msg, "bgr8")->image;

            // Process the image based on the current mode
            cv::Mat processed_image;
            if (current_mode_) {
                cv::cvtColor(last_image_, processed_image, cv::COLOR_BGR2GRAY);
                sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(msg->header, "mono8", processed_image).toImageMsg();      
                image_publisher_->publish(*msg);
                // cv::imshow("Camera", processed_image);
                // cv::waitKey(10);
            } else {
                sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(msg->header, "bgr8", last_image_).toImageMsg();      
                image_publisher_->publish(*msg);
                // cv::imshow("Camera", last_image_);
                // cv::waitKey(10);
            }
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Image processing failed: %s", e.what());
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImageModeActionServer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
