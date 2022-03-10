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

#include <memory>
#include <string>

#include <geometry_msgs/msg/transform_stamped.hpp>


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

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::executors::SingleThreadedExecutor exec;
  auto node = std::make_shared<Broadcaster>("tf2_broadcaster", "world", "map");
  auto node_ = std::make_shared<Broadcaster>("tf2_broadcaster_", "map", "robot");
  exec.add_node(node);
  exec.add_node(node_);
  exec.spin();

  rclcpp::shutdown();
  return 0;
}
