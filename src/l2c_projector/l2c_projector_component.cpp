#include "tf_tool/l2c_projector_component.hpp"

#include <math.h> /* M_PI */
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

using namespace std::chrono_literals;

L2CProjectorComponent::L2CProjectorComponent(const rclcpp::NodeOptions & options)
: Node("hoge")
{
  // dynamic_reconfigure
  // this->declare_parameter<double>("tf_parent2child.x", 0.0);

  // Initialize the transform listener

  // parameter callback
  set_init_param();
  auto parameter_change_cb = std::bind(&L2CProjectorComponent::CB_param_reset, this, std::placeholders::_1);
  reset_param_handler_ = this->add_on_set_parameters_callback(parameter_change_cb);

  // callback
  subscriber_pc = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "input_points", 
    rclcpp::SensorDataQoS(), 
    std::bind(&ColoringComponent::CB_cloud, this, std::placeholders::_1)\
  );

  subscriber_im = this->create_subscription<sensor_msgs::msg::Image>(
    "input_image_raw", 
    rclcpp::SensorDataQoS(), 
    std::bind(&ColoringComponent::CB_image, this, std::placeholders::_1)\
  );

  publisher_pc = this->create_publisher<sensor_msgs::msg::PointCloud2>("/hoge_points", 10);
}

rcl_interfaces::msg::SetParametersResult L2CProjectorComponent::CB_param_reset(const std::vector<rclcpp::Parameter> & params){
  auto result = rcl_interfaces::msg::SetParametersResult();
  for(auto&& param : params){
    if(param.get_name() == "tf_parent2child.x"){
      p2c_x = param.as_double();
    }else if(param.get_name() == "tf_parent2child.y"){
      p2c_y = param.as_double();
    }else if(param.get_name() == "tf_parent2child.z"){
      p2c_z = param.as_double();
    }else if(param.get_name() == "tf_parent2child.roll_deg"){
      p2c_roll = param.as_double() * (M_PI / 180);
    }else if(param.get_name() == "tf_parent2child.pitch_deg"){
      p2c_pitch = param.as_double() * (M_PI / 180);
    }else if(param.get_name() == "tf_parent2child.yaw_deg"){
      p2c_yaw = param.as_double() * (M_PI / 180);
    }
  }

  RCLCPP_INFO(this->get_logger(), "%s -> %s\ntranslate:{x=%f[m], y=%f[m], z=%f[m]}\nrotation:{roll=%f[rad], pitch=%f[rad], yaw=%f[rad]}",
              frame_p.c_str(), frame_c.c_str(), p2c_x, p2c_y, p2c_z, p2c_roll, p2c_pitch, p2c_yaw);

  set_new_tf();

  result.successful = true;
  return result;
}

void L2CProjectorComponent::set_init_param()
{
  declare_parameter("frame_parent", "parent");
  declare_parameter("frame_child", "child");
  frame_p = get_parameter("frame_parent").as_string();
  frame_c = get_parameter("frame_child").as_string();

  p2c_x = get_parameter("tf_parent2child.x").as_double();
  p2c_y = get_parameter("tf_parent2child.y").as_double();
  p2c_z = get_parameter("tf_parent2child.z").as_double();
  p2c_roll = get_parameter("tf_parent2child.roll_deg").as_double() * (M_PI / 180);
  p2c_pitch = get_parameter("tf_parent2child.pitch_deg").as_double() * (M_PI / 180);
  p2c_yaw = get_parameter("tf_parent2child.yaw_deg").as_double() * (M_PI / 180);

  RCLCPP_INFO(this->get_logger(), "%s -> %s\ntranslate:{x=%f[m], y=%f[m], z=%f[m]}\nrotation:{roll=%f[rad], pitch=%f[rad], yaw=%f[rad]}",
              frame_p.c_str(), frame_c.c_str(), p2c_x, p2c_y, p2c_z, p2c_roll, p2c_pitch, p2c_yaw);

  set_new_tf();
}

void L2CProjectorComponent::set_new_tf()
{
  geometry_msgs::msg::TransformStamped t;
  // Read message content and assign it to
  // corresponding tf variables
  t.header.stamp = this->get_clock()->now();
  t.header.frame_id = frame_p.c_str();
  t.child_frame_id = frame_c.c_str();

  t.transform.translation.x = p2c_x;
  t.transform.translation.y = p2c_y;
  t.transform.translation.z = p2c_z;

  tf2::Quaternion q;
  q.setRPY(p2c_roll, p2c_roll, p2c_roll);
  t.transform.rotation.x = q.x();
  t.transform.rotation.y = q.y();
  t.transform.rotation.z = q.z();
  t.transform.rotation.w = q.w();

  // Send the transformation
  tf_broadcaster_->sendTransform(t);
}