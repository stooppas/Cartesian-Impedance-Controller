#include <cartesian_impedance_controller/cartesian_impedance_controller_ros.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "controller_interface/helpers.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_action/create_server.hpp"
#include "rclcpp_action/server_goal_handle.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "rclcpp/version.h"
#if RCLCPP_VERSION_GTE(29, 0, 0)
#include "urdf/model.hpp"
#else
#include "urdf/model.h"
#endif

namespace cartesian_impedance_controller
{ 

  /*! \brief Saturate a variable x with the limits x_min and x_max
  *
  * \param[in] x Value
  * \param[in] x_min Minimal value
  * \param[in] x_max Maximum value
  * \return Saturated value
  */
  double saturateValue(double x, double x_min, double x_max)
  {
    return std::min(std::max(x, x_min), x_max);
  }

  /*! \brief Populates a wrench msg with value from Eigen vector
  *
  * It is assumed that the vector has the form transl_x, transl_y, transl_z, rot_x, rot_y, rot_z
  * \param[in] v Input vector
  * \param[out] wrench Wrench message
  */
  void EigenVectorToWrench(const Eigen::Matrix<double, 6, 1> &v, geometry_msgs::msg::Wrench *wrench)
  {
    wrench->force.x = v(0);
    wrench->force.y = v(1);
    wrench->force.z = v(2);
    wrench->torque.x = v(3);
    wrench->torque.y = v(4);
    wrench->torque.z = v(5);
  }

  CartesianImpedanceControllerRos::CartesianImpedanceControllerRos(): controller_interface::ControllerInterface(),
  CartesianImpedanceController()
  {

  }

void CartesianImpedanceControllerRos::updateParams()
{
  this->delta_tau_max_ = params_.delta_tau_max;

  this->filter_params_nullspace_config_ = params_.filtering.nullspace_config;
  this->filter_params_stiffness_ = params_.filtering.stiffness;
  this->filter_params_pose_ = params_.filtering.pose;
  this->filter_params_wrench_ = params_.filtering.wrench;

  this->verbose_print_ = params_.verbosity.verbose_print;
  this->verbose_state_ = params_.verbosity.state_msgs;
  this->verbose_tf_ = params_.verbosity.tf_frames;

  CartesianImpedanceController::setStiffness(saturateValue(params_.stiffness.translation.x, trans_stf_min_, trans_stf_max_),
                                              saturateValue(params_.stiffness.translation.y, trans_stf_min_, trans_stf_max_),
                                              saturateValue(params_.stiffness.translation.z, trans_stf_min_, trans_stf_max_),
                                              saturateValue(params_.stiffness.rotation.x, trans_stf_min_, trans_stf_max_),
                                              saturateValue(params_.stiffness.rotation.y, trans_stf_min_, trans_stf_max_),
                                              saturateValue(params_.stiffness.rotation.z, trans_stf_min_, trans_stf_max_), params_.stiffness.nullspace);

  CartesianImpedanceController::setDampingFactors(
  params_.damping.translation.x, params_.damping.translation.y, params_.damping.translation.z, params_.damping.rotation.x, params_.damping.rotation.y, params_.damping.rotation.z, params_.damping.nullspace);

  Eigen::Vector6d F{Eigen::Vector6d::Zero()};

  F << params_.wrench.f_x, params_.wrench.f_y, params_.wrench.f_z, params_.wrench.tau_x, params_.wrench.tau_y, params_.wrench.tau_z;
  if (!transformWrench(&F, params_.wrench_ee_frame, this->root_frame_))
  {
    RCLCPP_ERROR(get_node()->get_logger(),"Could not transform wrench. Not applying it.");
    return;
  }

  this->applyWrench(F);
}

controller_interface::CallbackReturn CartesianImpedanceControllerRos::on_init()
{
  try
  {
    // Create the parameter listener and get the parameters
    param_listener_ = std::make_shared<ParamListener>(get_node());
    params_ = param_listener_->get_params();
  }
  catch (const std::exception & e)
  {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }

  // get degrees of freedom
  dof_ = (uint32_t)params_.joints.size();

  if (params_.joints.empty())
  {
    RCLCPP_WARN(get_node()->get_logger(), "'joints' parameter is empty.");
  }

  update_frequency_ = get_update_rate();

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_node()->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  tf_br_ = std::make_unique<tf2_ros::TransformBroadcaster>(*(this->get_node()));

  RCLCPP_INFO(get_node()->get_logger(),"Initializing Cartesian impedance controller in namespace: %s", get_node()->get_namespace());

  // Get the URDF XML from the parameter server. Wait if needed.
  
  std::string urdf_string = get_node()->get_parameter("robot_description").as_string();

  if (!this->initMessaging() || !this->initRBDyn(urdf_string))
  {
    return CallbackReturn::ERROR;
  }
  if (!this->initTrajectories())
  {
    return CallbackReturn::ERROR;
  }
  this->root_frame_ = this->rbdyn_wrapper_.root_link();

  updateParams();

  // Initialize base_tools and member variables
  this->setNumberOfJoints((uint32_t)dof_);
  if (this->dof_ < 6)
  {
    RCLCPP_WARN(get_node()->get_logger(),"Number of joints is below 6. Functions might be limited.");
  }
  if (this->dof_ < 7)
  {
    RCLCPP_WARN(get_node()->get_logger(),"Number of joints is below 7. No redundant joint for nullspace.");
  }
  this->tau_m_ = Eigen::VectorXd(this->dof_);

  RCLCPP_INFO(get_node()->get_logger(),"Finished initialization.");

  return CallbackReturn::SUCCESS;
}

  controller_interface::InterfaceConfiguration
  CartesianImpedanceControllerRos::command_interface_configuration() const
  {
    controller_interface::InterfaceConfiguration conf;
    conf.type = controller_interface::interface_configuration_type::ALL;
    if (dof_ == 0)
    {
      fprintf(
        stderr,
        "During ros2_control interface configuration, degrees of freedom is not valid;"
        " it should be positive. Actual DOF is %i\n",
        dof_);
      std::exit(EXIT_FAILURE);
    }
    conf.names.reserve(dof_);
    for (const auto & joint_name : params_.joints)
    {
      conf.names.push_back(joint_name + "/" + hardware_interface::HW_IF_EFFORT);
    }
    return conf;
  }

  controller_interface::InterfaceConfiguration
CartesianImpedanceControllerRos::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf;
  conf.type = controller_interface::interface_configuration_type::ALL;
  conf.names.reserve(dof_ * 3);
  for (const auto & joint_name : params_.joints)
  { 
      conf.names.push_back(joint_name + "/" + hardware_interface::HW_IF_POSITION);
      conf.names.push_back(joint_name + "/" + hardware_interface::HW_IF_VELOCITY);
      conf.names.push_back(joint_name + "/" + hardware_interface::HW_IF_EFFORT);
  }
  return conf;
}

controller_interface::CallbackReturn CartesianImpedanceControllerRos::on_activate(
  const rclcpp_lifecycle::State &)
{
  auto logger = get_node()->get_logger();

  // update the dynamic map parameters
  param_listener_->refresh_dynamic_parameters();

  // get parameters from the listener in case they were updated
  params_ = param_listener_->get_params();

  if (!controller_interface::get_ordered_interfaces(command_interfaces_, params_.joints, hardware_interface::HW_IF_EFFORT, effort_command_interface_))
  {
    RCLCPP_ERROR(logger, "Expected %i '%s' command interfaces, got %zu.", dof_, hardware_interface::HW_IF_EFFORT, effort_command_interface_.size());
    return CallbackReturn::ERROR;
  }

  if (!controller_interface::get_ordered_interfaces(state_interfaces_, params_.joints, hardware_interface::HW_IF_POSITION, position_state_interface_))
  {
    RCLCPP_ERROR(
      logger, "Expected %i '%s' state interfaces, got %zu.", dof_, hardware_interface::HW_IF_POSITION,
      position_state_interface_.size());
    return CallbackReturn::ERROR;
  }

  if (!controller_interface::get_ordered_interfaces(state_interfaces_, params_.joints, hardware_interface::HW_IF_VELOCITY, velocity_state_interface_))
  {
    RCLCPP_ERROR(
      logger, "Expected %i '%s' state interfaces, got %zu.", dof_, hardware_interface::HW_IF_VELOCITY,
      velocity_state_interface_.size());
    return CallbackReturn::ERROR;
  }

  if (!controller_interface::get_ordered_interfaces(state_interfaces_, params_.joints, hardware_interface::HW_IF_EFFORT, effort_state_interface_))
  {
    RCLCPP_ERROR(
      logger, "Expected %i '%s' state interfaces, got %zu.", dof_, hardware_interface::HW_IF_EFFORT,
      effort_state_interface_.size());
    return CallbackReturn::ERROR;
  }

  traj_msg_external_point_ptr_.writeFromNonRT(
    std::shared_ptr<trajectory_msgs::msg::JointTrajectory>());

  this->updateState();

  // Set reference pose to current pose and q_d_nullspace
  this->initDesiredPose(this->position_, this->orientation_);
  this->initNullspaceConfig(this->q_);
  RCLCPP_INFO(logger,"Activated Cartesian Impedance Controller");
  
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CartesianImpedanceControllerRos::on_configure(
  const rclcpp_lifecycle::State &)
{
  auto logger = get_node()->get_logger();

  if (!param_listener_)
  {
    RCLCPP_ERROR(logger, "Error encountered during init");
    return controller_interface::CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CartesianImpedanceControllerRos::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  const auto active_goal = *rt_active_goal_.readFromNonRT();
  if (active_goal)
  {
    rt_has_pending_goal_.writeFromNonRT(false);
    auto action_res = std::make_shared<FollowJTrajAction::Result>();
    action_res->set__error_code(FollowJTrajAction::Result::INVALID_GOAL);
    action_res->set__error_string("Current goal cancelled during deactivate transition.");
    active_goal->setCanceled(action_res);
    rt_active_goal_.writeFromNonRT(RealtimeGoalHandlePtr());
  }

  for (size_t index = 0; index < dof_; ++index)
  {
    effort_command_interface_[index].get().set_value(0.0);
  }

  release_interfaces();

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CartesianImpedanceControllerRos::on_cleanup(
  const rclcpp_lifecycle::State &)
{
  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CartesianImpedanceControllerRos::on_error(
  const rclcpp_lifecycle::State &)
{
  return CallbackReturn::SUCCESS;
}

  bool CartesianImpedanceControllerRos::initMessaging()
  {
    /*
    // Queue size of 1 since we are only interested in the last message
    this->sub_cart_stiffness_ = nh->subscribe("set_cartesian_stiffness", 1,
                                              &CartesianImpedanceControllerRos::cartesianStiffnessCb, this);
    this->sub_cart_wrench_ = nh->subscribe("set_cartesian_wrench", 1,
                                           &CartesianImpedanceControllerRos::wrenchCommandCb, this);
    this->sub_damping_factors_ = nh->subscribe("set_damping_factors", 1,
                                       &CartesianImpedanceControllerRos::cartesianDampingFactorCb, this);
    this->sub_controller_config_ =
        nh->subscribe("set_config", 1, &CartesianImpedanceControllerRos::controllerConfigCb, this);
    this->sub_reference_pose_ = nh->subscribe("reference_pose", 1, &CartesianImpedanceControllerRos::referencePoseCb, this);
    */

    // Initializing the realtime publisher and the message

    this->pub_torques_ros_ = get_node()->create_publisher<std_msgs::msg::Float64MultiArray>("~/commanded_torques", rclcpp::SystemDefaultsQoS());
    this->pub_torques_ = std::make_unique<realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>>(pub_torques_ros_);
    this->pub_torques_->msg_.layout.dim.resize(1);
    this->pub_torques_->msg_.layout.data_offset = 0;
    this->pub_torques_->msg_.layout.dim[0].size = this->dof_;
    this->pub_torques_->msg_.layout.dim[0].stride = 0;
    this->pub_torques_->msg_.data.resize(this->dof_);

    this->pub_state_ros_ = get_node()->create_publisher<cartesian_impedance_controller::msg::ControllerState>("~/controller_state", rclcpp::SystemDefaultsQoS());
    this->pub_state_ = std::make_unique<realtime_tools::RealtimePublisher<cartesian_impedance_controller::msg::ControllerState>>(pub_state_ros_);

    for (size_t i = 0; i < this->dof_; i++)
    {
      this->pub_state_->msg_.joint_state.name.push_back(params_.joints.at(i));
    }
    this->pub_state_->msg_.joint_state.position = std::vector<double>(this->dof_);
    this->pub_state_->msg_.joint_state.velocity = std::vector<double>(this->dof_);
    this->pub_state_->msg_.joint_state.effort = std::vector<double>(this->dof_);
    this->pub_state_->msg_.commanded_torques = std::vector<double>(this->dof_);
    this->pub_state_->msg_.nullspace_config = std::vector<double>(this->dof_);
    
    return true;
  }

  bool CartesianImpedanceControllerRos::initRBDyn(std::string& urdf_string)
  {
    try
    {
      this->rbdyn_wrapper_.init_rbdyn(urdf_string, params_.end_effector);
    }
    catch (std::runtime_error& e)
    {
      RCLCPP_ERROR(get_node()->get_logger(),"Error when intializing RBDyn: %s", e.what());
      return false;
    }
    RCLCPP_INFO_STREAM(get_node()->get_logger(),"Number of joints found in urdf: " << this->rbdyn_wrapper_.n_joints());
    if (this->rbdyn_wrapper_.n_joints() < this->dof_)
    {
      RCLCPP_ERROR(get_node()->get_logger(),"Number of joints in the URDF is smaller than supplied number of joints. %i < %i", this->rbdyn_wrapper_.n_joints(), this->dof_);
      return false;
    }
    else if (this->rbdyn_wrapper_.n_joints() > this->dof_)
    {
      RCLCPP_WARN(get_node()->get_logger(),"Number of joints in the URDF is greater than supplied number of joints: %i > %i. Assuming that the actuated joints come first.", this->rbdyn_wrapper_.n_joints(), this->dof_);
    }
    return true;
  }

  bool CartesianImpedanceControllerRos::initTrajectories()
  {
    this->sub_trajectory_ = get_node()->create_subscription<trajectory_msgs::msg::JointTrajectory>(
      "joint_trajectory", 10, std::bind(&CartesianImpedanceControllerRos::trajCb, this, std::placeholders::_1));

    this->traj_as_ =  rclcpp_action::create_server<control_msgs::action::FollowJointTrajectory>(
          get_node()->get_node_base_interface(), get_node()->get_node_clock_interface(),
          get_node()->get_node_logging_interface(), get_node()->get_node_waitables_interface(),
          std::string(get_node()->get_name()) + "/follow_joint_trajectory",
          std::bind(&CartesianImpedanceControllerRos::goal_received_callback, this, std::placeholders::_1, std::placeholders::_2),
          std::bind(&CartesianImpedanceControllerRos::goal_cancelled_callback, this, std::placeholders::_1),
          std::bind(&CartesianImpedanceControllerRos::goal_accepted_callback, this, std::placeholders::_1));

    return true;
  }
  
  controller_interface::return_type CartesianImpedanceControllerRos::update(const rclcpp::Time & /*time*/, const rclcpp::Duration &/*period*/)
  {
    if (get_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
    {
      return controller_interface::return_type::OK;
    }

    if (param_listener_->is_old(params_))
    {
      params_ = param_listener_->get_params();
      updateParams();
    }

    // don't update goal after we sampled the trajectory to avoid any racecondition
    const auto active_goal = *rt_active_goal_.readFromRT();

    auto new_external_msg = traj_msg_external_point_ptr_.readFromRT();
    // Discard, if a goal is pending but still not active (somewhere stuck in goal_handle_timer_)
    if (*(rt_has_pending_goal_.readFromRT()) && !active_goal)
    {
      sort_to_local_joint_order(*new_external_msg);
      trajStart(*new_external_msg);
    }


    if (*(rt_has_pending_goal_.readFromRT()))
    {
      trajUpdate();
    }
    this->updateState();

    // Apply control law in base library
    this->calculateCommandedTorques();

    for (size_t index = 0; index < dof_; ++index)
    {
      effort_command_interface_[index].get().set_value(this->tau_c_(index));
    }

    publishMsgsAndTf();

    return controller_interface::return_type::OK;
  }

  bool CartesianImpedanceControllerRos::getFk(const Eigen::VectorXd &q, Eigen::Vector3d *position,
                                              Eigen::Quaterniond *orientation) const
  {
    rbdyn_wrapper::EefState ee_state;
    // If the URDF contains more joints than there are controlled, only the state of the controlled ones are known
    if (this->rbdyn_wrapper_.n_joints() != this->dof_)
    {
      Eigen::VectorXd q_rb = Eigen::VectorXd::Zero(this->rbdyn_wrapper_.n_joints());
      q_rb.head(q.size()) = q;
      ee_state = this->rbdyn_wrapper_.perform_fk(q_rb);
    }
    else
    {
      ee_state = this->rbdyn_wrapper_.perform_fk(q);
    }
    *position = ee_state.translation;
    *orientation = ee_state.orientation;
    return true;
  }

  bool CartesianImpedanceControllerRos::getJacobian(const Eigen::VectorXd &q, const Eigen::VectorXd &dq,
                                                    Eigen::MatrixXd *jacobian)
  {
    // If the URDF contains more joints than there are controlled, only the state of the controlled ones are known
    if (this->rbdyn_wrapper_.n_joints() != this->dof_)
    {
      Eigen::VectorXd q_rb = Eigen::VectorXd::Zero(this->rbdyn_wrapper_.n_joints());
      q_rb.head(q.size()) = q;
      Eigen::VectorXd dq_rb = Eigen::VectorXd::Zero(this->rbdyn_wrapper_.n_joints());
      dq_rb.head(dq.size()) = dq;
      *jacobian = this->rbdyn_wrapper_.jacobian(q_rb, dq_rb);
    }
    else
    {
      *jacobian = this->rbdyn_wrapper_.jacobian(q, dq);
    }
    *jacobian = jacobian_perm_ * *jacobian;
    return true;
  }

  void CartesianImpedanceControllerRos::updateState()
  {
    for (size_t i = 0; i < this->dof_; ++i)
    {
      this->q_[i] = this->position_state_interface_[i].get().get_value();
      this->dq_[i] = this->velocity_state_interface_[i].get().get_value();
      this->tau_m_[i] = this->effort_state_interface_[i].get().get_value();
    }
    getJacobian(this->q_, this->dq_, &this->jacobian_);
    getFk(this->q_, &this->position_, &this->orientation_);
  }

  void CartesianImpedanceControllerRos::controllerConfigCb(const cartesian_impedance_controller::msg::ControllerConfig::SharedPtr &msg)
  {
    this->setStiffness(msg->cartesian_stiffness, msg->nullspace_stiffness, false);
    this->setDampingFactors(msg->cartesian_damping_factors, msg->nullspace_damping_factor);

    if (msg->q_d_nullspace.size() == this->dof_)
    {
      Eigen::VectorXd q_d_nullspace(this->dof_);
      for (size_t i = 0; i < this->dof_; i++)
      {
        q_d_nullspace(i) = msg->q_d_nullspace.at(i);
      }
      this->setNullspaceConfig(q_d_nullspace);
    }
    else
    {
      RCLCPP_WARN_STREAM(get_node()->get_logger(),"Nullspace configuration does not have the correct amount of entries. Got " << msg->q_d_nullspace.size() << " expected " << this->dof_ << ". Ignoring.");
    }
  }

  void CartesianImpedanceControllerRos::cartesianDampingFactorCb(const geometry_msgs::msg::Wrench::SharedPtr &msg)
  {
    this->setDampingFactors(*msg, this->damping_factors_[6]);
  }

  void CartesianImpedanceControllerRos::referencePoseCb(const geometry_msgs::msg::PoseStamped::SharedPtr &msg)
  {
    if (!msg->header.frame_id.empty() && msg->header.frame_id != this->root_frame_)
    {
      RCLCPP_WARN_STREAM(get_node()->get_logger(),"Reference poses need to be in the root frame '" << this->root_frame_ << "'. Ignoring.");
      return;
    }
    Eigen::Vector3d position_d;
    position_d << msg->pose.position.x, msg->pose.position.y, msg->pose.position.z;
    const Eigen::Quaterniond last_orientation_d_target(this->orientation_d_);
    Eigen::Quaterniond orientation_d;
    orientation_d.coeffs() << msg->pose.orientation.x, msg->pose.orientation.y, msg->pose.orientation.z,
        msg->pose.orientation.w;
    if (last_orientation_d_target.coeffs().dot(this->orientation_d_.coeffs()) < 0.0)
    {
      this->orientation_d_.coeffs() << -this->orientation_d_.coeffs();
    }
    this->setReferencePose(position_d, orientation_d);
  }

  void CartesianImpedanceControllerRos::cartesianStiffnessCb(const geometry_msgs::msg::WrenchStamped::SharedPtr &msg)
  {
    this->setStiffness(msg->wrench, this->nullspace_stiffness_target_);
  }

  void CartesianImpedanceControllerRos::setDampingFactors(const geometry_msgs::msg::Wrench &cart_damping, double nullspace)
  {
    CartesianImpedanceController::setDampingFactors(saturateValue(cart_damping.force.x, dmp_factor_min_, dmp_factor_max_),
                                             saturateValue(cart_damping.force.y, dmp_factor_min_, dmp_factor_max_),
                                             saturateValue(cart_damping.force.z, dmp_factor_min_, dmp_factor_max_),
                                             saturateValue(cart_damping.torque.x, dmp_factor_min_, dmp_factor_max_),
                                             saturateValue(cart_damping.torque.y, dmp_factor_min_, dmp_factor_max_),
                                             saturateValue(cart_damping.torque.z, dmp_factor_min_, dmp_factor_max_),
                                             saturateValue(nullspace, dmp_factor_min_, dmp_factor_max_));
  }

  void CartesianImpedanceControllerRos::setStiffness(const geometry_msgs::msg::Wrench &cart_stiffness, double nullspace, bool auto_damping)
  {
    CartesianImpedanceController::setStiffness(saturateValue(cart_stiffness.force.x, trans_stf_min_, trans_stf_max_),
                                               saturateValue(cart_stiffness.force.y, trans_stf_min_, trans_stf_max_),
                                               saturateValue(cart_stiffness.force.z, trans_stf_min_, trans_stf_max_),
                                               saturateValue(cart_stiffness.torque.x, rot_stf_min_, rot_stf_max_),
                                               saturateValue(cart_stiffness.torque.y, rot_stf_min_, rot_stf_max_),
                                               saturateValue(cart_stiffness.torque.z, rot_stf_min_, rot_stf_max_),
                                               saturateValue(nullspace, ns_min_, ns_max_), auto_damping);
  }

  void CartesianImpedanceControllerRos::wrenchCommandCb(const geometry_msgs::msg::WrenchStamped::SharedPtr &msg)
  {
    Eigen::Matrix<double, 6, 1> F;
    F << msg->wrench.force.x, msg->wrench.force.y, msg->wrench.force.z, msg->wrench.torque.x, msg->wrench.torque.y,
        msg->wrench.torque.z;

    if (!msg->header.frame_id.empty() && msg->header.frame_id != this->root_frame_)
    {
      if (!transformWrench(&F, msg->header.frame_id, this->root_frame_))
      {
        RCLCPP_ERROR(get_node()->get_logger(),"Could not transform wrench. Not applying it.");
        return;
      }
    }
    else if (msg->header.frame_id.empty())
    {
      if (!transformWrench(&F, params_.wrench_ee_frame, this->root_frame_))
      {
        RCLCPP_ERROR(get_node()->get_logger(),"Could not transform wrench. Not applying it.");
        return;
      }
    }
    this->applyWrench(F);
  }

  bool CartesianImpedanceControllerRos::transformWrench(Eigen::Matrix<double, 6, 1> *cartesian_wrench,
                                                        const std::string &from_frame, const std::string &to_frame) const
  {
    try
    {
      geometry_msgs::msg::TransformStamped transform_msg = tf_buffer_->lookupTransform(to_frame, from_frame, tf2::TimePointZero);
      
      tf2::Transform transform;
      tf2::convert<geometry_msgs::msg::TransformStamped, tf2::Transform>( transform_msg, transform );

      tf2::Vector3 v_f(cartesian_wrench->operator()(0), cartesian_wrench->operator()(1), cartesian_wrench->operator()(2));
      tf2::Vector3 v_t(cartesian_wrench->operator()(3), cartesian_wrench->operator()(4), cartesian_wrench->operator()(5));
      tf2::Vector3 v_f_rot = tf2::quatRotate(transform.getRotation(), v_f);
      tf2::Vector3 v_t_rot = tf2::quatRotate(transform.getRotation(), v_t);
      *cartesian_wrench << v_f_rot[0], v_f_rot[1], v_f_rot[2], v_t_rot[0], v_t_rot[1], v_t_rot[2];
      return true;
    }
    catch (const tf2::TransformException &ex)
    {
      RCLCPP_ERROR_THROTTLE(get_node()->get_logger(),*get_node()->get_clock(),1, "%s", ex.what());
      return false;
    }
  }

  void CartesianImpedanceControllerRos::publishMsgsAndTf()
  {
    // publish commanded torques
    if (this->pub_torques_->trylock())
    {
      for (size_t i = 0; i < this->dof_; i++)
      {
        this->pub_torques_->msg_.data[i] = this->tau_c_[i];
      }
      this->pub_torques_->unlockAndPublish();
    }

    const Eigen::Matrix<double, 6, 1> error{this->getPoseError()};

    if (this->verbose_print_)
    {
      RCLCPP_INFO_STREAM_THROTTLE(get_node()->get_logger(), *get_node()->get_clock() ,0.1, "\nCartesian Position:\n"
                                        << this->position_ << "\nError:\n"
                                        << error << "\nCartesian Stiffness:\n"
                                        << this->cartesian_stiffness_ << "\nCartesian damping:\n"
                                        << this->cartesian_damping_ << "\nNullspace stiffness:\n"
                                        << this->nullspace_stiffness_ << "\nq_d_nullspace:\n"
                                        << this->q_d_nullspace_ << "\ntau_d:\n"
                                        << this->tau_c_);
    }
    
    if (this->verbose_tf_ && get_node()->now() > this->tf_last_time_)
    {
      rclcpp::Time now = get_node()->get_clock()->now();
      geometry_msgs::msg::TransformStamped t;
      t.header.stamp = now;
      t.header.frame_id = this->root_frame_;

      // Publish result of forward kinematics
      t.child_frame_id = params_.end_effector + "_ee_fk";

      t.transform.translation.x = position_.x();
      t.transform.translation.y = position_.y();
      t.transform.translation.z = position_.z();

      t.transform.rotation.x = orientation_.x();
      t.transform.rotation.y = orientation_.y();
      t.transform.rotation.z = orientation_.z();
      t.transform.rotation.w = orientation_.w();

      tf_br_->sendTransform(t);
        
      // Publish tf to the reference pose
      t.child_frame_id = params_.end_effector + "_ee_ref_pose";

      t.transform.translation.x = position_d_.x();
      t.transform.translation.y = position_d_.y();
      t.transform.translation.z = position_d_.z();

      t.transform.rotation.x = orientation_d_.x();
      t.transform.rotation.y = orientation_d_.y();
      t.transform.rotation.z = orientation_d_.z();
      t.transform.rotation.w = orientation_d_.w();

      tf_br_->sendTransform(t);
         
      this->tf_last_time_ = get_node()->now();
    }
    
    /*
    if (this->verbose_state_ && this->pub_state_->trylock())
    {
      this->pub_state_->msg_.header.stamp = get_node()->now();

      this->pub_state_->msg_.current_pose.position = tf2::toMsg<Eigen::Vector3d,geometry_msgs::msg::Point>(this->position_);
      this->pub_state_->msg_.current_pose.orientation = tf2::toMsg<Eigen::Quaterniond,geometry_msgs::msg::Quaternion>(this->orientation_);
      this->pub_state_->msg_.reference_pose.position = tf2::toMsg<Eigen::Vector3d,geometry_msgs::msg::Point>(this->position_d_);
      this->pub_state_->msg_.reference_pose.orientation = tf2::toMsg<Eigen::Quaterniond,geometry_msgs::msg::Quaternion>(this->orientation_d_);
      this->pub_state_->msg_.pose_error.position = tf2::toMsg<Eigen::Vector3d,geometry_msgs::msg::Point>(error.head(3));
      Eigen::Quaterniond q = Eigen::AngleAxisd(error(3), Eigen::Vector3d::UnitX()) * Eigen::AngleAxisd(error(4), Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(error(5), Eigen::Vector3d::UnitZ());
      this->pub_state_->msg_.pose_error.orientation = tf2::toMsg<Eigen::Quaterniond,geometry_msgs::msg::Quaternion>(q);

      EigenVectorToWrench(this->cartesian_stiffness_.diagonal(), &this->pub_state_->msg_.cartesian_stiffness);
      EigenVectorToWrench(this->cartesian_damping_.diagonal(), &this->pub_state_->msg_.cartesian_damping);
      EigenVectorToWrench(this->getAppliedWrench(), &this->pub_state_->msg_.commanded_wrench);

      for (size_t i = 0; i < this->dof_; i++)
      {
        this->pub_state_->msg_.joint_state.position.at(i) = this->q_(i);
        this->pub_state_->msg_.joint_state.velocity.at(i) = this->dq_(i);
        this->pub_state_->msg_.joint_state.effort.at(i) = this->tau_m_(i);
        this->pub_state_->msg_.nullspace_config.at(i) = this->q_d_nullspace_(i);
        this->pub_state_->msg_.commanded_torques.at(i) = this->tau_c_(i);
      }
      this->pub_state_->msg_.nullspace_stiffness = this->nullspace_stiffness_;
      this->pub_state_->msg_.nullspace_damping = this->nullspace_damping_;
      const Eigen::Matrix<double, 6, 1> dx = this->jacobian_ * this->dq_;
      this->pub_state_->msg_.cartesian_velocity = sqrt(dx(0) * dx(0) + dx(1) * dx(1) + dx(2) * dx(2));

      this->pub_state_->unlockAndPublish();
    }
    */
    
  }

  void CartesianImpedanceControllerRos::trajCb(const std::shared_ptr<trajectory_msgs::msg::JointTrajectory> msg)
  {
    RCLCPP_INFO(get_node()->get_logger(),"Got trajectory msg from trajectory topic.");

    preempt_active_goal();
    traj_msg_external_point_ptr_.writeFromNonRT(msg);
    rt_is_holding_.writeFromNonRT(false);
    
    trajStart(msg);
  }

  bool CartesianImpedanceControllerRos::validate_trajectory_msg(
  const trajectory_msgs::msg::JointTrajectory & trajectory) const
{
  if (trajectory.joint_names.size() != dof_)
  {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Joints on incoming trajectory don't match the controller joints.");
    return false;
  }

  if (trajectory.joint_names.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Empty joint names on incoming trajectory.");
    return false;
  }

  if (trajectory.points.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Empty trajectory received.");
    return false;
  }

  const auto trajectory_start_time = static_cast<rclcpp::Time>(trajectory.header.stamp);
  // If the starting time it set to 0.0, it means the controller should start it now.
  // Otherwise we check if the trajectory ends before the current time,
  // in which case it can be ignored.
  if (trajectory_start_time.seconds() != 0.0)
  {
    auto const trajectory_end_time =
      trajectory_start_time + trajectory.points.back().time_from_start;
    if (trajectory_end_time < get_node()->now())
    {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "Received trajectory with non-zero start time (%f) that ends in the past (%f)",
        trajectory_start_time.seconds(), trajectory_end_time.seconds());
      return false;
    }
  }

  for (size_t i = 0; i < trajectory.joint_names.size(); ++i)
  {
    const std::string & incoming_joint_name = trajectory.joint_names[i];

    auto it = std::find(params_.joints.begin(), params_.joints.end(), incoming_joint_name);
    if (it == params_.joints.end())
    {
      RCLCPP_ERROR(
        get_node()->get_logger(), "Incoming joint %s doesn't match the controller's joints.",
        incoming_joint_name.c_str());
      return false;
    }
  }

  rclcpp::Duration previous_traj_time(0ms);
  for (size_t i = 0; i < trajectory.points.size(); ++i)
  {
    if ((i > 0) && (rclcpp::Duration(trajectory.points[i].time_from_start) <= previous_traj_time))
    {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "Time between points %zu and %zu is not strictly increasing, it is %f and %f respectively",
        i - 1, i, previous_traj_time.seconds(),
        rclcpp::Duration(trajectory.points[i].time_from_start).seconds());
      return false;
    }
    previous_traj_time = trajectory.points[i].time_from_start;

    const size_t joint_count = trajectory.joint_names.size();
    const auto & points = trajectory.points;

    if (joint_count != points[i].positions.size())
    {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "Mismatch between joint_names size (%zu) and %s (%zu) at point #%zu.", joint_count,
        "positions", points[i].positions.size(), i);
      return false;
    }

    // reject effort entries
    if (!points[i].effort.empty() || !points[i].velocities.empty() ||  !points[i].accelerations.empty())
    {
      RCLCPP_ERROR(
        get_node()->get_logger(), "Trajectories with effort, velocity or acceleration fields are currently not supported.");
      return false;
    }
  }
  return true;
}

  rclcpp_action::GoalResponse CartesianImpedanceControllerRos::goal_received_callback(const rclcpp_action::GoalUUID &, std::shared_ptr<const control_msgs::action::FollowJointTrajectory::Goal> goal)
  {
     RCLCPP_INFO(get_node()->get_logger(), "Received new action goal");

    // Precondition: Running controller
    if (get_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
    {
      RCLCPP_ERROR(
        get_node()->get_logger(), "Can't accept new action goals. Controller is not running.");
      return rclcpp_action::GoalResponse::REJECT;
    }

    if (!validate_trajectory_msg(goal->trajectory))
    {
      return rclcpp_action::GoalResponse::REJECT;
    }

    RCLCPP_INFO(get_node()->get_logger(), "Accepted new action goal");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse CartesianImpedanceControllerRos::goal_cancelled_callback(const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>> goal_handle)
  {
    RCLCPP_INFO(get_node()->get_logger(), "Got request to cancel goal");

    // Check that cancel request refers to currently active goal (if any)
    const auto active_goal = *rt_active_goal_.readFromNonRT();
    if (active_goal && active_goal->gh_ == goal_handle)
    {
      RCLCPP_INFO(
        get_node()->get_logger(), "Canceling active action goal because cancel callback received.");

      // Mark the current goal as canceled
      rt_has_pending_goal_.writeFromNonRT(false);
      auto action_res = std::make_shared<FollowJTrajAction::Result>();
      active_goal->setCanceled(action_res);
      rt_active_goal_.writeFromNonRT(RealtimeGoalHandlePtr());

      // Enter hold current position mode
      //traj_msg_external_point_ptr_.writeFromNonRT(traj_msg);
      //add_new_trajectory_msg(set_hold_position());
    }
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void CartesianImpedanceControllerRos::goal_accepted_callback(std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>> goal_handle)
  {
    // mark a pending goal
    rt_has_pending_goal_.writeFromNonRT(true);

    // Update new trajectory
    {
      preempt_active_goal();
      auto traj_msg =
        std::make_shared<trajectory_msgs::msg::JointTrajectory>(goal_handle->get_goal()->trajectory);

      traj_msg_external_point_ptr_.writeFromNonRT(traj_msg);
      rt_is_holding_.writeFromNonRT(false);
    }

    // Update the active goal
    RealtimeGoalHandlePtr rt_goal = std::make_shared<RealtimeGoalHandle>(goal_handle);
    rt_goal->preallocated_feedback_->joint_names = params_.joints;
    rt_goal->execute();
    rt_active_goal_.writeFromNonRT(rt_goal);

    // Set smartpointer to expire for create_wall_timer to delete previous entry from timer list
    goal_handle_timer_.reset();

    // Setup goal status checking timer
    goal_handle_timer_ = get_node()->create_wall_timer(
      action_monitor_period_.to_chrono<std::chrono::nanoseconds>(),
      std::bind(&RealtimeGoalHandle::runNonRealtime, rt_goal));
  }

  void CartesianImpedanceControllerRos::preempt_active_goal()
  {
    const auto active_goal = *rt_active_goal_.readFromNonRT();
    if (active_goal)
    {
      auto action_res = std::make_shared<FollowJTrajAction::Result>();
      action_res->set__error_code(FollowJTrajAction::Result::INVALID_GOAL);
      action_res->set__error_string("Current goal cancelled due to new incoming action.");
      active_goal->setCanceled(action_res);
      rt_active_goal_.writeFromNonRT(RealtimeGoalHandlePtr());
    }
  }

  void CartesianImpedanceControllerRos::sort_to_local_joint_order(
  std::shared_ptr<trajectory_msgs::msg::JointTrajectory> trajectory_msg) const
  {
    std::vector<size_t> mapping_vector(trajectory_msg->joint_names.size());
    for (auto t1_it = trajectory_msg->joint_names.begin(); t1_it != trajectory_msg->joint_names.end(); ++t1_it)
    {
      auto t2_it = std::find(params_.joints.begin(), params_.joints.end(), *t1_it);

      const size_t t1_dist = std::distance(trajectory_msg->joint_names.begin(), t1_it);
      const size_t t2_dist = std::distance(params_.joints.begin(), t2_it);
      mapping_vector[t1_dist] = t2_dist;
    }

    auto remap = [this](
                  const std::vector<double> & to_remap,
                  const std::vector<size_t> & mapping) -> std::vector<double>
    {
      if (to_remap.empty())
      {
        return to_remap;
      }
      if (to_remap.size() != mapping.size())
      {
        RCLCPP_WARN(
          get_node()->get_logger(), "Invalid input size (%zu) for sorting", to_remap.size());
        return to_remap;
      }
      static std::vector<double> output(dof_, 0.0);
      // Only resize if necessary since it's an expensive operation
      if (output.size() != mapping.size())
      {
        output.resize(mapping.size(), 0.0);
      }
      for (size_t index = 0; index < mapping.size(); ++index)
      {
        auto map_index = mapping[index];
        output[map_index] = to_remap[index];
      }
      return output;
    };

    for (size_t index = 0; index < trajectory_msg->points.size(); ++index)
    {
      trajectory_msg->points[index].positions =
        remap(trajectory_msg->points[index].positions, mapping_vector);

      trajectory_msg->points[index].velocities =
        remap(trajectory_msg->points[index].velocities, mapping_vector);

      trajectory_msg->points[index].accelerations =
        remap(trajectory_msg->points[index].accelerations, mapping_vector);

      trajectory_msg->points[index].effort =
        remap(trajectory_msg->points[index].effort, mapping_vector);
    }
  }

  void CartesianImpedanceControllerRos::trajStart(const trajectory_msgs::msg::JointTrajectory::SharedPtr &trajectory)
  {
    this->traj_duration_ = trajectory->points[trajectory->points.size() - 1].time_from_start;
    RCLCPP_INFO_STREAM(get_node()->get_logger(),"Starting a new trajectory with " << trajectory->points.size() << " points that takes " << this->traj_duration_.seconds() << "s.");
    this->trajectory_ = trajectory;
    this->traj_start_ = get_node()->now();
    this->traj_index_ = 0;
    trajUpdate();
    if (this->nullspace_stiffness_ < 5.)
    {
      RCLCPP_WARN(get_node()->get_logger(),"Nullspace stiffness is low. The joints might not follow the planned path.");
    }
  }

  void CartesianImpedanceControllerRos::trajUpdate()
  {
    const auto active_goal = *rt_active_goal_.readFromRT();

    if (get_node()->now() > (this->traj_start_ + trajectory_->points.at(this->traj_index_).time_from_start))
    {
      // Get end effector pose
      Eigen::VectorXd q = Eigen::VectorXd::Map(trajectory_->points.at(this->traj_index_).positions.data(),
                                               trajectory_->points.at(this->traj_index_).positions.size());
      if (this->verbose_print_)
      {
        RCLCPP_INFO_STREAM(get_node()->get_logger(),"Index " << this->traj_index_ << " q_nullspace: " << q.transpose());
      }
      // Update end-effector pose and nullspace
      getFk(q, &this->position_d_target_, &this->orientation_d_target_);
      this->setNullspaceConfig(q);
      this->traj_index_++;
    }

    if (get_node()->now() > (this->traj_start_ + this->traj_duration_))
    {
      RCLCPP_INFO_STREAM(get_node()->get_logger(),"Finished executing trajectory.");

      auto result = std::make_shared<FollowJTrajAction::Result>();
      result->set__error_code(FollowJTrajAction::Result::SUCCESSFUL);
      result->set__error_string("Goal successfully reached!");

      active_goal->setSucceeded(result);
      rt_active_goal_.writeFromNonRT(RealtimeGoalHandlePtr());
      rt_has_pending_goal_.writeFromNonRT(false);
    }
  }
} // namespace cartesian_impedance_controller

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(cartesian_impedance_controller::CartesianImpedanceControllerRos,controller_interface::ControllerInterface);

