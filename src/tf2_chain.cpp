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
#include <tilde/tilde_node.hpp>
#include <tilde/tilde_transform_broadcaster.hpp>
#include <tilde/tilde_transform_listener.hpp>
#include <tilde/tilde_buffer.hpp>
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2_ros/create_timer_ros.h>

#include <memory>
#include <string>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/image.hpp>


using std::placeholders::_1;
using namespace std::chrono_literals;

class MessageSource : public tilde::TildeNode
{
public:
  MessageSource(std::string node_name, std::string topic_name)
  : TildeNode(node_name)
  {
    pub_ = create_tilde_publisher<sensor_msgs::msg::Image>(topic_name, 1);
    timer_ = create_wall_timer(
      1s, [&]() {
        auto msg = std::make_unique<sensor_msgs::msg::Image>();
        pub_->publish(std::move(msg));
      });
  }

private:
  tilde::TildePublisher<sensor_msgs::msg::Image>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

class NormalToNormal : public tilde::TildeNode
{
public:
  NormalToNormal(
    std::string node_name,
    std::string sub_topic_name,
    std::string pub_topic_name)
  : TildeNode(node_name), sub_topic_name_(sub_topic_name)
  {
    pub_ = create_tilde_publisher<sensor_msgs::msg::Image>(pub_topic_name, 1);
    sub_ = create_tilde_subscription<sensor_msgs::msg::Image>(
      sub_topic_name_,
      1,
      [&](sensor_msgs::msg::Image::UniquePtr msg) {
        pub_->add_explicit_input_info(sub_topic_name_, msg->header.stamp);
        pub_->publish(std::move(msg));
      });
  }

private:
  std::string sub_topic_name_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
  tilde::TildePublisher<sensor_msgs::msg::Image>::SharedPtr pub_;
};

class NormalToTransform : public tilde::TildeNode
{
public:
  NormalToTransform(
    std::string node_name,
    std::string sub_topic_name,
    std::string pub_frame_id,
    std::string pub_child_frame_id
  )
  : TildeNode(node_name),
    sub_topic_name_(sub_topic_name),
    pub_frame_id_(pub_frame_id),
    pub_child_frame_id_(pub_child_frame_id)
  {
    br_ = std::make_unique<tilde::TildeTransformBroadcaster>(*this);
    sub_ = create_subscription<sensor_msgs::msg::Image>(
      sub_topic_name,
      1,
      [&](sensor_msgs::msg::Image::UniquePtr msg) {
        auto msg_ = std::make_unique<geometry_msgs::msg::TransformStamped>();
        msg_->header.frame_id = pub_frame_id_;
        msg_->header.stamp = now();
        msg_->child_frame_id = pub_child_frame_id_;
        br_->add_explicit_input_info(sub_topic_name_, msg->header.stamp);
        br_->sendTransform(*msg_);
      }
    );
  }

private:
  std::string sub_topic_name_;
  std::string pub_frame_id_;
  std::string pub_child_frame_id_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
  std::unique_ptr<tilde::TildeTransformBroadcaster> br_;
};

class TransformToTransform : public tilde::TildeNode
{
public:
  TransformToTransform(
    std::string node_name,
    std::string sub_frame_id,
    std::string sub_child_frame_id,
    std::string pub_frame_id,
    std::string pub_child_frame_id
  )
  : TildeNode(node_name),
    sub_frame_id_(sub_frame_id),
    sub_child_frame_id_(sub_child_frame_id),
    pub_frame_id_(pub_frame_id),
    pub_child_frame_id_(pub_child_frame_id)
  {
    br_ = std::make_unique<tilde::TildeTransformBroadcaster>(*this);
    tf_buffer_ =
      std::make_unique<tilde::TildeBuffer>(this->get_clock());
    tf_listener_ =
      std::make_shared<tilde::TildeTransformListener>(*tf_buffer_);
    timer_ = create_wall_timer(
      0.1s, [&]() {
        try {
          geometry_msgs::msg::TransformStamped tf_sub;
          geometry_msgs::msg::TransformStamped tf_pub;
          tf_sub = tf_buffer_->lookupTransform(
            sub_frame_id_, sub_child_frame_id_,
            tf2::TimePointZero);
          tf_pub.header.stamp = now();
          tf_pub.header.frame_id = pub_frame_id_;
          tf_pub.child_frame_id = pub_child_frame_id_;
          br_->add_explicit_input_info(tf_sub);
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
  std::unique_ptr<tilde::TildeBuffer> tf_buffer_;
  std::shared_ptr<tilde::TildeTransformListener> tf_listener_;
  std::unique_ptr<tilde::TildeTransformBroadcaster> br_;
};

class TransformToTransformAsync : public tilde::TildeNode
{
public:
  TransformToTransformAsync(
    std::string node_name,
    std::string sub_frame_id,
    std::string sub_child_frame_id,
    std::string pub_frame_id,
    std::string pub_child_frame_id
  )
  : TildeNode(node_name),
    sub_frame_id_(sub_frame_id),
    sub_child_frame_id_(sub_child_frame_id),
    pub_frame_id_(pub_frame_id),
    pub_child_frame_id_(pub_child_frame_id)
  {
    br_ = std::make_unique<tilde::TildeTransformBroadcaster>(*this);
    tf_buffer_ =
      std::make_unique<tilde::TildeBuffer>(this->get_clock());
    tf_timer_ = std::make_shared<tf2_ros::CreateTimerROS>(
      this->get_node_base_interface(), this->get_node_timers_interface());
    tf_buffer_->setCreateTimerInterface(tf_timer_);
    tf_listener_ =
      std::make_shared<tilde::TildeTransformListener>(*tf_buffer_);
    timer_ = create_wall_timer(
      1s, [&]() {
        auto future = tf_buffer_->waitForTransform(
          sub_frame_id_, sub_child_frame_id_, now(), 3s,
          [](const tf2_ros::TransformStampedFuture &) {});
        future.wait_for(0.5s);
        try {
          auto tf_sub = future.get();

          geometry_msgs::msg::TransformStamped tf_pub;
          tf_pub.header.stamp = now();
          tf_pub.header.frame_id = pub_frame_id_;
          tf_pub.child_frame_id = pub_child_frame_id_;
          br_->add_explicit_input_info(tf_sub);
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
  std::unique_ptr<tilde::TildeBuffer> tf_buffer_;
  std::shared_ptr<tf2_ros::CreateTimerInterface> tf_timer_;
  std::shared_ptr<tilde::TildeTransformListener> tf_listener_;
  std::unique_ptr<tilde::TildeTransformBroadcaster> br_;
};

class TransformToTransformAsyncCallback : public tilde::TildeNode
{
public:
  TransformToTransformAsyncCallback(
    std::string node_name,
    std::string sub_frame_id,
    std::string sub_child_frame_id,
    std::string pub_frame_id,
    std::string pub_child_frame_id
  )
  : TildeNode(node_name),
    sub_frame_id_(sub_frame_id),
    sub_child_frame_id_(sub_child_frame_id),
    pub_frame_id_(pub_frame_id),
    pub_child_frame_id_(pub_child_frame_id)
  {
    br_ = std::make_unique<tilde::TildeTransformBroadcaster>(*this);
    tf_buffer_ =
      std::make_unique<tilde::TildeBuffer>(this->get_clock());
    tf_timer_ = std::make_shared<tf2_ros::CreateTimerROS>(
      this->get_node_base_interface(), this->get_node_timers_interface());
    tf_buffer_->setCreateTimerInterface(tf_timer_);
    tf_listener_ =
      std::make_shared<tilde::TildeTransformListener>(*tf_buffer_);
    timer_ = create_wall_timer(
      1s, [&]() {
        tf_buffer_->waitForTransform(
          sub_frame_id_, sub_child_frame_id_, now(), 0.5s,
          std::bind(
            &TransformToTransformAsyncCallback::tf_ready_callback, this,
            std::placeholders::_1)
        );
      });
  }

  void tf_ready_callback(const tf2_ros::TransformStampedFuture & future)
  {
    // future.wait_for(1s);
    try {
      auto tf_sub = future.get();

      geometry_msgs::msg::TransformStamped tf_pub;
      tf_pub.header.stamp = now();
      tf_pub.header.frame_id = pub_frame_id_;
      tf_pub.child_frame_id = pub_child_frame_id_;
      br_->add_explicit_input_info(tf_sub);
      br_->sendTransform(tf_pub);
    } catch (tf2::TransformException & ex) {
    }
  }

private:
  std::string sub_frame_id_;
  std::string sub_child_frame_id_;
  std::string pub_frame_id_;
  std::string pub_child_frame_id_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<tilde::TildeBuffer> tf_buffer_;
  std::shared_ptr<tf2_ros::CreateTimerInterface> tf_timer_;
  std::shared_ptr<tilde::TildeTransformListener> tf_listener_;
  std::unique_ptr<tilde::TildeTransformBroadcaster> br_;
};

class TransformToNormal : public tilde::TildeNode
{
public:
  TransformToNormal(
    std::string node_name,
    std::string sub_frame_id,
    std::string sub_child_frame_id,
    std::string pub_topic_name
  )
  : TildeNode(node_name),
    sub_frame_id_(sub_frame_id),
    sub_child_frame_id_(sub_child_frame_id),
    pub_topic_name_(pub_topic_name)
  {
    pub_ = create_tilde_publisher<sensor_msgs::msg::Image>(pub_topic_name_, 1);
    tf_buffer_ =
      std::make_unique<tilde::TildeBuffer>(this->get_clock());
    tf_listener_ =
      std::make_shared<tilde::TildeTransformListener>(*tf_buffer_);
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
          pub_->add_explicit_input_info(tf_sub);
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
  tilde::TildePublisher<sensor_msgs::msg::Image>::SharedPtr pub_;
  std::unique_ptr<tilde::TildeBuffer> tf_buffer_;
  std::shared_ptr<tilde::TildeTransformListener> tf_listener_;
};

class MessageDestination : public tilde::TildeNode
{
public:
  MessageDestination(std::string node_name, std::string sub_topic_name)
  : TildeNode(node_name)
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
    "normal_to_transform", "/topic1", "map", "earth");
  auto node_t2t = std::make_shared<TransformToTransform>(
    "transform_to_transform", "map", "earth",
    "robot", "map");
  auto node_async = std::make_shared<TransformToTransformAsync>(
    "transform_to_transform_async", "robot", "map",
    "arm", "robot");
  auto node_async_cb = std::make_shared<TransformToTransformAsyncCallback>(
    "transform_to_transform_async_callback", "arm", "robot",
    "hand", "arm");
  auto node_t2n = std::make_shared<TransformToNormal>(
    "transform_to_normal", "hand", "robot",
    "/topic3");
  auto node_dest = std::make_shared<MessageDestination>("message_destination", "/topic3");

  exec.add_node(node_src);
  exec.add_node(node_n2n);
  exec.add_node(node_n2t);
  exec.add_node(node_t2t);
  exec.add_node(node_t2n);
  exec.add_node(node_async);
  exec.add_node(node_async_cb);
  exec.add_node(node_dest);

  exec.spin();

  rclcpp::shutdown();
  return 0;
}
