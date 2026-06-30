#include "bicycle_to_ackermann_steering_adapter/bicycle_to_ackermann_steering_adapter.hpp"

namespace bicycle_to_ackermann_steering_adapter
{

BicycleToAckermannSteeringAdapter::BicycleToAckermannSteeringAdapter() : controller_interface::ChainableControllerInterface()
{
}

controller_interface::CallbackReturn BicycleToAckermannSteeringAdapter::on_init()
{
  try
  {
    bicycle_to_ackermann_steering_adapter_param_listener_ =
        std::make_shared<bicycle_to_ackermann_steering_adapter::ParamListener>(get_node());
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during controller's init with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn BicycleToAckermannSteeringAdapter::on_configure(
  const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;
  try
  {
    bicycle_to_ackermann_steering_adapter_params_ = bicycle_to_ackermann_steering_adapter_param_listener_->get_params();

    wheelbase_ = bicycle_to_ackermann_steering_adapter_params_.wheelbase;
    track_width_ = bicycle_to_ackermann_steering_adapter_params_.track_width;
    max_steer_angle_ = bicycle_to_ackermann_steering_adapter_params_.max_steer_angle;
    by_reference_or_by_state_ = bicycle_to_ackermann_steering_adapter_params_.by_reference_or_by_state;
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during configure stage with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration BicycleToAckermannSteeringAdapter::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  command_interfaces_config.names.reserve(nr_output_steer_itfs_);
  for (size_t i = 0; i < nr_output_steer_itfs_; i++)
  {
    command_interfaces_config.names.push_back(
      bicycle_to_ackermann_steering_adapter_params_.output_steering_names[i]
      + "/" + hardware_interface::HW_IF_POSITION);
  }
  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration BicycleToAckermannSteeringAdapter::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  if (by_reference_or_by_state_)
  {
    state_interfaces_config.names.push_back(
      bicycle_to_ackermann_steering_adapter_params_.input_steering_name
      + "/" + hardware_interface::HW_IF_POSITION);
  }
  else
  {
    state_interfaces_config.names.reserve(nr_output_steer_itfs_);
    for (size_t i = 0; i < nr_output_steer_itfs_; i++)
    {
      state_interfaces_config.names.push_back(
        bicycle_to_ackermann_steering_adapter_params_.output_steering_names[i]
        + "/" + hardware_interface::HW_IF_POSITION);
    }
  }

  return state_interfaces_config;
}

std::vector<hardware_interface::StateInterface>
BicycleToAckermannSteeringAdapter::on_export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  state_interfaces_values_.reserve(nr_input_steer_itfs_);

  for (size_t i = 0; i < nr_input_steer_itfs_; ++i)
  {
    state_interfaces.emplace_back(
     get_name()
      + "/" + bicycle_to_ackermann_steering_adapter_params_.input_steering_name,
      hardware_interface::HW_IF_POSITION,
      &state_interfaces_values_[i]
    );
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
BicycleToAckermannSteeringAdapter::on_export_reference_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.reserve(nr_input_steer_itfs_);
  reference_interfaces_.resize(nr_input_steer_itfs_);

  for (size_t i = 0; i < nr_input_steer_itfs_; ++i)
  {
    command_interfaces.emplace_back(
      get_name()
      + "/" + bicycle_to_ackermann_steering_adapter_params_.input_steering_name,
      hardware_interface::HW_IF_POSITION,
      &reference_interfaces_[i]
    );
  }

  return command_interfaces;
}

controller_interface::CallbackReturn BicycleToAckermannSteeringAdapter::on_activate(
  const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn BicycleToAckermannSteeringAdapter::on_deactivate(
  const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type BicycleToAckermannSteeringAdapter::update_reference_from_subscribers(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  (void)time;
  (void)period;
  return controller_interface::return_type::OK;

}

controller_interface::return_type BicycleToAckermannSteeringAdapter::update_and_write_commands(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  (void)time;
  (void)period;

  auto logger = get_node()->get_logger();

  right_steering_angle_ = 0.0;
  left_steering_angle_ = 0.0;

  if (by_reference_or_by_state_)
  {
    // by_state: input = hardware sensor (center steering angle) → apply Ackermann kinematics → output individual angles
    auto center_steering_angle_op = state_interfaces_[0].get_optional();
    if (!center_steering_angle_op.has_value())
    {
      RCLCPP_ERROR(logger, "Unable to retrieve position data for center steering angle sensor");
      return controller_interface::return_type::ERROR;
    }

    center_steering_angle_ = center_steering_angle_op.value();
    state_interfaces_values_[0] = center_steering_angle_;

    center_steering_angle_ = std::clamp(center_steering_angle_, -max_steer_angle_, max_steer_angle_);
    center_steering_angle_abs_ = std::abs(center_steering_angle_);

    if (center_steering_angle_abs_ > 1e-6) {
      cot_center_steering_angle_ = std::cos(center_steering_angle_abs_) / std::sin(center_steering_angle_abs_);
      outer_steering_angle_ = std::atan2(1.0, (cot_center_steering_angle_ + (track_width_ / (2 * wheelbase_))));
      inner_steering_angle_ = std::atan2(1.0, (cot_center_steering_angle_ - (track_width_ / (2 * wheelbase_))));

      if (center_steering_angle_ > 0) {
        left_steering_angle_ = inner_steering_angle_;
        right_steering_angle_ = outer_steering_angle_;
      } else {
        right_steering_angle_ = -inner_steering_angle_;
        left_steering_angle_ = -outer_steering_angle_;
      }
    }
  }
  else
  {
    // by_reference: read individual wheel states → compute center angle feedback → apply kinematics from reference
    auto right_steering_angle_op = state_interfaces_[0].get_optional();
    auto left_steering_angle_op = state_interfaces_[1].get_optional();

    if (!right_steering_angle_op.has_value())
    {
      RCLCPP_ERROR(logger, "Unable to retrieve position feedback data for right steering angle");
      return controller_interface::return_type::ERROR;
    }
    if (!left_steering_angle_op.has_value())
    {
      RCLCPP_ERROR(logger, "Unable to retrieve position feedback data for left steering angle");
      return controller_interface::return_type::ERROR;
    }

    right_steering_angle_ = right_steering_angle_op.value();
    left_steering_angle_ = left_steering_angle_op.value();

    right_steering_angle_ = std::atan(1.0 / ((std::cos(right_steering_angle_) / std::sin(right_steering_angle_)) - (track_width_ / (2 * wheelbase_))));
    left_steering_angle_  = std::atan(1.0 / ((std::cos(left_steering_angle_)  / std::sin(left_steering_angle_))  + (track_width_ / (2 * wheelbase_))));
    center_steering_angle_ = (right_steering_angle_ + left_steering_angle_) / 2;
    state_interfaces_values_[0] = center_steering_angle_;

    right_steering_angle_ = 0.0;
    left_steering_angle_ = 0.0;

    if (!std::isnan(reference_interfaces_[0])) {
      center_steering_angle_ = reference_interfaces_[0];
      center_steering_angle_ = std::clamp(center_steering_angle_, -max_steer_angle_, max_steer_angle_);
      center_steering_angle_abs_ = std::abs(center_steering_angle_);

      if (center_steering_angle_abs_ > 1e-6) {
        cot_center_steering_angle_ = std::cos(center_steering_angle_abs_) / std::sin(center_steering_angle_abs_);
        outer_steering_angle_ = std::atan2(1.0, (cot_center_steering_angle_ + (track_width_ / (2 * wheelbase_))));
        inner_steering_angle_ = std::atan2(1.0, (cot_center_steering_angle_ - (track_width_ / (2 * wheelbase_))));

        if (center_steering_angle_ > 0) {
          left_steering_angle_ = inner_steering_angle_;
          right_steering_angle_ = outer_steering_angle_;
        } else {
          right_steering_angle_ = -inner_steering_angle_;
          left_steering_angle_ = -outer_steering_angle_;
        }
      }
    }

    reference_interfaces_[0] = std::numeric_limits<double>::quiet_NaN();
  }

  if (!command_interfaces_[0].set_value(right_steering_angle_)) {
    RCLCPP_WARN(logger, "Failed to set right steering");
  }
  if (!command_interfaces_[1].set_value(left_steering_angle_)) {
    RCLCPP_WARN(logger, "Failed to set left steering");
  }

  return controller_interface::return_type::OK;
}

} // namespace bicycle_to_ackermann_steering_adapter

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  bicycle_to_ackermann_steering_adapter::BicycleToAckermannSteeringAdapter,
  controller_interface::ChainableControllerInterface)
