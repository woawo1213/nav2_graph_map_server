/* Copyright (c) 2018 Intel Corporation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/* Copyright 2019 Rover Robotics
 * Copyright 2010 Brian Gerkey
 * Copyright (c) 2008, Willow Garage, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "nav2_graph_map_server/map_server.hpp"
#include "graph_map_msgs/msg/graph_map.hpp"

#include <string>
#include <memory>
#include <fstream>
#include <stdexcept>
#include <utility>

#include "yaml-cpp/yaml.h"
#include "lifecycle_msgs/msg/state.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

namespace nav2_graph_map_server
{

MapServer::MapServer(const rclcpp::NodeOptions & options)
: nav2_util::LifecycleNode("graph_map_server", "", options)
{
  RCLCPP_INFO(get_logger(), "Creating");

  // Declare the node parameters
  declare_parameter("yaml_filename", rclcpp::PARAMETER_STRING);
  declare_parameter("topic_name", "graph_map");
  declare_parameter("frame_id", "map");
}

MapServer::~MapServer()
{
}

nav2_util::CallbackReturn
MapServer::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Configuring");

  // Get the name of the YAML file to use
  std::string yaml_filename = get_parameter("yaml_filename").as_string();

  std::string topic_name = get_parameter("topic_name").as_string();
  frame_id_ = get_parameter("frame_id").as_string();

  // Load GraphMap form ymal
  YAML::Node config = YAML::LoadFile(yaml_filename);

  YAML::Node nodes = config["nodes"];
  YAML::Node edges = config["edges"];

  msg_.header.frame_id = frame_id_;
  msg_.header.stamp = rclcpp::Clock().now();

  // push the Node Info 
  for (int i = 0; i < (int)nodes.size(); i++) {
    wr_nav_msgs::msg::GraphNode node_msg;

    YAML::Node node = nodes[i];
    YAML::Node pose = node["pose"];
    YAML::Node position = pose["position"];
    YAML::Node orientation = pose["orientation"];

    node_msg.id = node["id"].as<uint32_t>();
    node_msg.pose.position.x = position["x"].as<double>();
    node_msg.pose.position.y = position["y"].as<double>();
    node_msg.pose.position.z = position["z"].as<double>();
    node_msg.pose.orientation.x = orientation["x"].as<double>();
    node_msg.pose.orientation.y = orientation["y"].as<double>();
    node_msg.pose.orientation.z = orientation["z"].as<double>();
    node_msg.pose.orientation.w = orientation["w"].as<double>();
    node_msg.station_name = node["station_name"].as<std::string>();

    msg_.nodes.push_back(node_msg);
  }

  // push the Edge Info
  for (int i = 0; i < (int)edges.size(); i++) {
    wr_nav_msgs::msg::GraphEdge edge_msg;

    YAML::Node edge = edges[i];

    edge_msg.id_0 = edge["id_0"].as<uint32_t>(); // start_id
    edge_msg.id_1 = edge["id_1"].as<uint32_t>(); // end_id
    edge_msg.speed = edge["speed"].as<float>();

    msg_.edges.push_back(edge_msg);
  }

  // Create a publisher for GraphMap.msg, sing the QoS settings to emulate a ROS1 latched topic
  graph_map_pub_ = create_publisher<wr_nav_msgs::msg::GraphMap>(
    topic_name,
    rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  return nav2_util::CallbackReturn::SUCCESS;
}

nnav2_util::CallbackReturn
MapServer::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Activating");

  // Publish the map using the latched topic
  graph_map_pub_.headeron_activate();
  auto graph_ = std::make_unique<wr_nav_msgs::msg::GraphMap>(msg_);
  graph_map_pub_->publish(std::move(graph_));

  // create bond connection
  createBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
MapServer::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Deactivating");

  graph_map_pub_->on_deactivate();

  // destroy bond connection
  destroyBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
MapServer::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Cleaning up");

  graph_map_pub_.reset();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn
MapServer::on_shutdown(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(get_logger(), "Shutting down");
  return nav2_util::CallbackReturn::SUCCESS;
}

}  // namespace nav2_graph_map_server

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(nav2_graph_map_server::MapServer)
