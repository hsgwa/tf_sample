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
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/exceptions.h>

#include <memory>
#include <string>

using std::placeholders::_1;
using namespace std::chrono_literals;


class Listener : public rclcpp::Node
{
public:
  Listener(std::string node_name, std::string frame_id_from, std::string frame_id_to)
  : Node(node_name), frame_id_from_(frame_id_from), frame_id_to_(frame_id_to)
  {
    tf_buffer_ =
      std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ =
      std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    timer_ = create_wall_timer(
      1s, std::bind(&Listener::on_timer, this)
    );
  }

private:
  void on_timer()
  {
    geometry_msgs::msg::TransformStamped transformStamped;

    try {
      transformStamped = tf_buffer_->lookupTransform(
        frame_id_from_, frame_id_to_,
        tf2::TimePointZero);
    } catch (tf2::TransformException & ex) {
      RCLCPP_INFO(
        this->get_logger(), "Could not transform %s to %s: %s",
        frame_id_from_.c_str(), frame_id_to_.c_str(), ex.what());
      return;
    }
    RCLCPP_INFO(get_logger(), "succeed to receive transform.");
  }

  std::string frame_id_from_;
  std::string frame_id_to_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::TimerBase::SharedPtr timer_;
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Listener>("tf2_listener", "world", "robot"));
  rclcpp::shutdown();
  return 0;
}
