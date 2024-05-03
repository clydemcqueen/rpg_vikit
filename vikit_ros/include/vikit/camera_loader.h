/*
 * camera_loader.h
 *
 *  Created on: Feb 11, 2014
 *      Author: cforster
 */

#ifndef VIKIT_CAMERA_LOADER_H_
#define VIKIT_CAMERA_LOADER_H_

#include <string>
#include <rclcpp/rclcpp.hpp>
#include <vikit/abstract_camera.h>
#include <vikit/pinhole_camera.h>
#include <vikit/atan_camera.h>
#include <vikit/omni_camera.h>

namespace vk::camera_loader {

bool loadFromRosNode(const std::shared_ptr<rclcpp::Node>& node, vk::AbstractCamera*& cam);

} // namespace vk::camera_loader

#endif // VIKIT_CAMERA_LOADER_H_
