/*
 * output_helper.h
 *
 *  Created on: Jan 20, 2013
 *      Author: cforster
 */

#ifndef VIKIT_OUTPUT_HELPER_H_
#define VIKIT_OUTPUT_HELPER_H_

#include <string>
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Core>
#include <sophus/se3.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/msg/marker.hpp>

namespace vk::output_helper {

using namespace std;        // TODO remove
using namespace Eigen;      // TODO remove

void
publishTfTransform      (const Sophus::SE3d& T, const rclcpp::Time& stamp,
                         const string& frame_id, const string& child_frame_id,
                         tf2_ros::TransformBroadcaster& br);

void
publishPointMarker      (const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr& pub,
                         const Vector3d& pos,
                         const string& ns,
                         const rclcpp::Time& timestamp,
                         int id,
                         int action,
                         double marker_scale,
                         const Vector3d& color,
                         const rclcpp::Duration& lifetime = std::chrono::nanoseconds::zero());

void
publishLineMarker       (const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr& pub,
                         const Vector3d& start,
                         const Vector3d& end,
                         const string& ns,
                         const rclcpp::Time& timestamp,
                         int id,
                         int action,
                         double marker_scale,
                         const Vector3d& color,
                         const rclcpp::Duration& lifetime = std::chrono::nanoseconds::zero());

void
publishArrowMarker      (const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr& pub,
                         const Vector3d& pos,
                         const Vector3d& dir,
                         double scale,
                         const string& ns,
                         const rclcpp::Time& timestamp,
                         int id,
                         int action,
                         double marker_scale,
                         const Vector3d& color);

void
publishHexacopterMarker (const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr& pub,
                         const string& frame_id,
                         const string& ns,
                         const rclcpp::Time& timestamp,
                         int id,
                         int action,
                         double marker_scale,
                         const Vector3d& color);

void
publishCameraMarker(const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr& pub,
                    const string& frame_id,
                    const string& ns,
                    const rclcpp::Time& timestamp,
                    int id,
                    double marker_scale,
                    const Vector3d& color);
void
publishFrameMarker     (const rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr& pub,
                        const Matrix3d& rot,
                        const Vector3d& pos,
                        const string& ns,
                        const rclcpp::Time& timestamp,
                        int id,
                        int action,
                        double marker_scale,
                        const rclcpp::Duration& lifetime = std::chrono::nanoseconds::zero());


} // namespace vk::output_helper

#endif /* VIKIT_OUTPUT_HELPER_H_ */
