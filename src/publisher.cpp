/*
 * Copyright 2023 Abraruddin Syed
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS," WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES, OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF, OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 */


#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include "beginner_tutorials/srv/modify_string.hpp"
#include <chrono>
#include <cstddef>
#include <functional>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <memory>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <string>


using namespace std::chrono_literals;

/**
 * @brief A minimal ROS 2 publisher node that publishes custom messages and provides a service to modify strings.
 */
class MinimalPublisher : public rclcpp::Node {
 public:
    /**
     * @brief Constructor for the MinimalPublisher class.
     */
    MinimalPublisher()
        : Node("minimal_publisher") {
        message.data = "Custom Message ";
        RCLCPP_DEBUG_STREAM(this->get_logger(),
         "Unmodified Ros message is"+message.data);

        /*Retrieve the publish_frequency parameter from the
         parameter server in milliseconds*/ 
        this->declare_parameter<int>("publish_frequency_ms", 500);
              RCLCPP_INFO_STREAM(
          this->get_logger(), "current frequency "<<
         this->get_parameter("publish_frequency_ms").as_int());

        int publish_frequency_ms_ = this->get_parameter(
            "publish_frequency_ms").as_int();

        if (publish_frequency_ms_ == 500) {
      RCLCPP_WARN_STREAM(
          this->get_logger(),
          "Using default frequency "
              << publish_frequency_ms_);
    }
            if (publish_frequency_ms_ >= 1000) {
      RCLCPP_FATAL_STREAM(
          this->get_logger(),
          "Publshing slow, change frequency ");
    }
                if (publish_frequency_ms_ < 50) {
      RCLCPP_ERROR_STREAM(
          this->get_logger(),
          "publishing too fast , some values may skip ");
    }
    // Default publish frequency: 500 milliseconds
        publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
        tf_broadcaster_ =
            std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(publish_frequency_ms_),
            std::bind(&MinimalPublisher::timer_callback, this));
        // Create a service server to handle requests
        service_ = this->create_service<beginner_tutorials::srv::ModifyString>(
            "modify_string",
             std::bind(&MinimalPublisher::modifyString, this,
                 std::placeholders::_1, std::placeholders::_2));
    }

 private:
    /**
     * @brief Timer callback function, publishing custom messages periodically.
     */
    void timer_callback() {
        RCLCPP_INFO(this->get_logger(),
                     "Publishing: '%s'", message.data.c_str());
        publisher_->publish(message);
        this->broadcastTF();
    }


        void broadcastTF() {
        geometry_msgs::msg::TransformStamped transformStamped;
        transformStamped.header.frame_id = "world";
        transformStamped.child_frame_id = "talk";
        transformStamped.header.stamp = this->now();

        // Set non-zero translation
        transformStamped.transform.translation.x = 1.0;
        transformStamped.transform.translation.y = 2.0;
        transformStamped.transform.translation.z = 3.0;

        // Set non-zero rotation (quaternion)
        transformStamped.transform.rotation.x = 0.0;
        transformStamped.transform.rotation.y = 0.0;
        transformStamped.transform.rotation.z = 0.707;
        transformStamped.transform.rotation.w = 0.707;

        tf_broadcaster_->sendTransform(transformStamped);
    }

    /**
     * @brief Service callback function to modify strings based on service requests.
     *
     * @param request The service request containing the input string.
     * @param response The service response containing the modified output string.
     */
    void modifyString(const std::shared_ptr<beginner_tutorials
                    ::srv::ModifyString::Request> request,
                      std::shared_ptr<beginner_tutorials::srv
                      ::ModifyString::Response> response) {
        response->output_string = "Received input: " + request->input_string;
        RCLCPP_INFO(this->get_logger(),
         "Service request received: '%s'. Response: '%s'",
                     request->input_string.c_str(),
                     response->output_string.c_str());
                     message.data = request->input_string;
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Service<beginner_tutorials::srv::ModifyString>::SharedPtr service_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;

    std_msgs::msg::String message;
};

/**
 * @brief The main function to create and run the MinimalPublisher node.
 *
 * @param argc Number of command-line arguments.
 * @param argv Array of command-line argument strings.
 * @return Exit status.
 */
int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<MinimalPublisher>());
    rclcpp::shutdown();
    return 0;
}
