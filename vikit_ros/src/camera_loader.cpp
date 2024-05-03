/*
 * camera_loader.cpp
 *
 *  Created on: Feb 11, 2014
 *      Author: cforster
 */

#include <string>
#include <rclcpp/rclcpp.hpp>
#include <vikit/abstract_camera.h>
#include <vikit/atan_camera.h>
#include <vikit/camera_loader.h>
#include <vikit/omni_camera.h>
#include <vikit/pinhole_camera.h>

namespace vk::camera_loader {

  bool loadFromRosNode(const std::shared_ptr<rclcpp::Node>& node, vk::AbstractCamera*& cam)
  {
    bool res = true;

    auto cam_model = node->declare_parameter<std::string>("cam_model", "");
    auto cam_calib_file = node->declare_parameter<std::string>("cam_calib_file", "");
    auto cam_width = node->declare_parameter<long>("cam_width", 0);
    auto cam_height = node->declare_parameter<long>("cam_height", 0);
    auto cam_fx = node->declare_parameter<double>("cam_fx", 0.0);
    auto cam_fy = node->declare_parameter<double>("cam_fy", 0.0);
    auto cam_cx = node->declare_parameter<double>("cam_cx", 0.0);
    auto cam_cy = node->declare_parameter<double>("cam_cy", 0.0);
    auto cam_d0 = node->declare_parameter<double>("cam_d0", 0.0);
    auto cam_d1 = node->declare_parameter<double>("cam_d1", 0.0);
    auto cam_d2 = node->declare_parameter<double>("cam_d2", 0.0);
    auto cam_d3 = node->declare_parameter<double>("cam_d3", 0.0);

    if(cam_model == "Ocam")
    {
      cam = new vk::OmniCamera(cam_calib_file);
    }
    else if(cam_model == "Pinhole")
    {
      cam = new vk::PinholeCamera(
          static_cast<double>(cam_width),
          static_cast<double>(cam_height),
          cam_fx,
          cam_fy,
          cam_cx,
          cam_cy,
          cam_d0,
          cam_d1,
          cam_d2,
          cam_d3);
    }
    else if(cam_model == "ATAN")
    {
      cam = new vk::ATANCamera(
          static_cast<double>(cam_width),
          static_cast<double>(cam_height),
          cam_fx,
          cam_fy,
          cam_cx,
          cam_cy,
          cam_d0);
    }
    else
    {
      cam = nullptr;
      res = false;
    }
    return res;
  }

} // namespace vk::camera_loader
