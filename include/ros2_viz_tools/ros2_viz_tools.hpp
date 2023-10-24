#ifndef ROS_VIZ_TOOLS_H
#define ROS_VIZ_TOOLS_H

#include "ros2_viz_tools/color.h"
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <string>

namespace ros2_viz_tools {

using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;
using std_msgs::msg::ColorRGBA;

class RosVizTools : public rclcpp::Node {
public:
    
    explicit RosVizTools(rclcpp::Node::SharedPtr node_handle, const std::string &topic);
    virtual ~RosVizTools(){};

    void publish();

    void clear();

    void append(const Marker &marker);

    static Marker newMaker(const geometry_msgs::msg::Vector3 &scale,
                           const geometry_msgs::msg::Pose &pose,
                           const std::string &ns,
                           const int32_t &id,
                           const ColorRGBA &color,
                           const std::string &frame_id,
                           const int32_t &type);

    static Marker newCubeList(double scale,
                              const std::string &ns,
                              const int32_t &id,
                              const ColorRGBA &color,
                              const std::string &frame_id);

    static Marker newSphereList(const double &scale,
                                const std::string &ns,
                                const int32_t &id,
                                const ColorRGBA &color,
                                const std::string &frame_id);

    static Marker newLineStrip(const double &scale,
                               const std::string &ns,
                               const int32_t &id,
                               const ColorRGBA &color,
                               const std::string &frame_id);

    static Marker newLineList(const double &scale,
                              const std::string &ns,
                              const int32_t &id,
                              const ColorRGBA &color,
                              const std::string &frame_id);

    static Marker newCylinder(const geometry_msgs::msg::Vector3 &scale,
                              const geometry_msgs::msg::Pose &pose,
                              const std::string &ns,
                              const int32_t &id,
                              const ColorRGBA &color,
                              const std::string &frame_id);

    static Marker newCube(const double &scale,
                          const geometry_msgs::msg::Pose &pose,
                          const std::string &ns,
                          const int32_t &id,
                          const ColorRGBA &color,
                          const std::string &frame_id);

    static Marker newSphere(const double &scale,
                            const geometry_msgs::msg::Pose &pose,
                            const std::string &ns,
                            const int32_t &id,
                            const ColorRGBA &color,
                            const std::string &frame_id);

    static Marker newArrow(const geometry_msgs::msg::Vector3 &scale,
                           const geometry_msgs::msg::Pose &pose,
                           const std::string &ns,
                           const int32_t &id,
                           const ColorRGBA &color,
                           const std::string &frame_id);

    static Marker newText(const double &scale,
                          const geometry_msgs::msg::Pose &pose,
                          const std::string &ns,
                          const int32_t &id,
                          const ColorRGBA &color,
                          const std::string &frame_id);

    static Marker newFrame(const double &width,
                           const double &length,
                           const geometry_msgs::msg::Pose &pose,
                           const std::string &ns,
                           const int32_t &id,
                           const std::string &frame_id);

    static geometry_msgs::msg::Pose defaultPose();

private:
    void initPublisher();

private:
    rclcpp::Node::SharedPtr node_handle_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr rviz_pub;
    std::string topic;
    MarkerArray rviz_marker_array;
};
} // namespace ros_viz_tools
#endif //ROS_VIZ_TOOLS_H
