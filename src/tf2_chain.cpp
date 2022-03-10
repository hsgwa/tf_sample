// Copyright 2021 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/exceptions.h>

#include <memory>
#include <string>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/image.hpp>


using std::placeholders::_1;
using namespace std::chrono_literals;

class Broadcaster : public rclcpp::Node
{
public:
  Broadcaster(std::string node_name, std::string frame_id, std::string child_frame_id)
  : Node(node_name), frame_id_(frame_id), child_frame_id_(child_frame_id)
  {
    tf_broadcaster_ =
      std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    timer_ = create_wall_timer(
      1s, std::bind(&Broadcaster::on_timer, this)
    );
  }

private:
  void on_timer()
  {
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = now();
    t.header.frame_id = frame_id_;
    t.child_frame_id = child_frame_id_;
    tf_broadcaster_->sendTransform(t);
    // std::cerr << "sendTransform:" << t.header.stamp.sec << t.header.stamp.nanosec << " " << t.header.frame_id << " " << t.child_frame_id << std::endl;
  }

  std::string frame_id_;
  std::string child_frame_id_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
};

class MessageSource : public rclcpp::Node
{
public:
  MessageSource(std::string node_name, std::string topic_name)
  : Node(node_name)
  {
    pub_ = create_publisher<sensor_msgs::msg::Image>(topic_name, 1);
    timer_ = create_wall_timer(
      1s, [&]() {
        auto msg = std::make_unique<sensor_msgs::msg::Image>();
        pub_->publish(std::move(msg));
      });
  }

private:
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

class NormalToNormal : public rclcpp::Node
{
public:
  NormalToNormal(
    std::string node_name,
    std::string sub_topic_name,
    std::string pub_topic_name)
  : Node(node_name)
  {
    pub_ = create_publisher<sensor_msgs::msg::Image>(pub_topic_name, 1);
    sub_ = create_subscription<sensor_msgs::msg::Image>(
      sub_topic_name,
      1,
      [&](sensor_msgs::msg::Image::UniquePtr msg) {
        pub_->publish(std::move(msg));
      });
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
};

class NormalToTransform : public rclcpp::Node
{
public:
  NormalToTransform(
    std::string node_name,
    std::string sub_topic_name,
    std::string pub_frame_id,
    std::string pub_child_frame_id
  )
  : Node(node_name),
    pub_frame_id_(pub_frame_id),
    pub_child_frame_id_(pub_child_frame_id)
  {
    br_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    sub_ = create_subscription<sensor_msgs::msg::Image>(
      sub_topic_name,
      1,
      [&](sensor_msgs::msg::Image::UniquePtr msg) {
        auto msg_ = std::make_unique<geometry_msgs::msg::TransformStamped>();
        msg_->header.frame_id = pub_frame_id_;
        msg_->header.stamp = now();
        msg_->child_frame_id = pub_child_frame_id_;
        br_->sendTransform(*msg_);
      }
    );
  }

private:
  std::string pub_frame_id_;
  std::string pub_child_frame_id_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> br_;
};

class TransformToTransform : public rclcpp::Node
{
public:
  TransformToTransform(
    std::string node_name,
    std::string sub_frame_id,
    std::string sub_child_frame_id,
    std::string pub_frame_id,
    std::string pub_child_frame_id
  )
  : Node(node_name),
    sub_frame_id_(sub_frame_id),
    sub_child_frame_id_(sub_child_frame_id),
    pub_frame_id_(pub_frame_id),
    pub_child_frame_id_(pub_child_frame_id)
  {
    br_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    tf_buffer_ =
      std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ =
      std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    timer_ = create_wall_timer(
      0.1s, [&]() {
        try {
          geometry_msgs::msg::TransformStamped tf_sub;
          geometry_msgs::msg::TransformStamped tf_pub;
          tf_sub = tf_buffer_->lookupTransform(
            sub_child_frame_id_, sub_frame_id_,
            tf2::TimePointZero);
          tf_pub.header.stamp = now();
          tf_pub.header.frame_id = pub_frame_id_;
          tf_pub.child_frame_id = pub_child_frame_id_;
          br_->sendTransform(tf_pub);
        } catch (tf2::TransformException & ex) {
        }
      });
  }

private:
  std::string sub_frame_id_;
  std::string sub_child_frame_id_;
  std::string pub_frame_id_;
  std::string pub_child_frame_id_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> br_;
};

class TransformToNormal : public rclcpp::Node
{
public:
  TransformToNormal(
    std::string node_name,
    std::string sub_frame_id,
    std::string sub_child_frame_id,
    std::string pub_topic_name
  )
  : Node(node_name),
    sub_frame_id_(sub_frame_id),
    sub_child_frame_id_(sub_child_frame_id),
    pub_topic_name_(pub_topic_name)
  {
    pub_ = create_publisher<sensor_msgs::msg::Image>(pub_topic_name_, 1);
    tf_buffer_ =
      std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ =
      std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    timer_ = create_wall_timer(
      0.1s, [&]() {
        try {
          geometry_msgs::msg::TransformStamped tf_sub;
          geometry_msgs::msg::TransformStamped tf_pub;
          tf_sub = tf_buffer_->lookupTransform(
            sub_child_frame_id_, sub_frame_id_,
            tf2::TimePointZero);
          auto msg = std::make_unique<sensor_msgs::msg::Image>();
          msg->header.stamp = now();
          pub_->publish(std::move(msg));
        } catch (tf2::TransformException & ex) {
        }
      });
  }

private:
  std::string sub_frame_id_;
  std::string sub_child_frame_id_;
  std::string pub_topic_name_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

class MessageDestination : public rclcpp::Node
{
public:
  MessageDestination(std::string node_name, std::string sub_topic_name)
  : Node(node_name)
  {
    sub_ = create_subscription<sensor_msgs::msg::Image>(
      sub_topic_name, 1,
      [&](sensor_msgs::msg::Image::UniquePtr msg) {

      });
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::executors::MultiThreadedExecutor exec;
  auto node_src = std::make_shared<MessageSource>(
    "message_source", "/topic1");
  auto node_n2n = std::make_shared<NormalToNormal>(
    "normal_to_normal", "/topic1", "/topic2");
  auto node_n2t = std::make_shared<NormalToTransform>(
    "normal_to_transform", "/topic1", "earth", "map");
  auto node_t2t = std::make_shared<TransformToTransform>(
    "transform_to_transform", "earth", "map",
    "map", "robot");
  auto node_t2n = std::make_shared<TransformToNormal>(
    "transform_to_normal", "map", "robot",
    "/topic3");
  auto node_dest = std::make_shared<MessageDestination>("message_destination", "/topic3");

  exec.add_node(node_src);
  exec.add_node(node_n2n);
  exec.add_node(node_n2t);
  exec.add_node(node_t2t);
  exec.add_node(node_t2n);
  exec.add_node(node_dest);

  exec.spin();

  rclcpp::shutdown();
  return 0;
}
