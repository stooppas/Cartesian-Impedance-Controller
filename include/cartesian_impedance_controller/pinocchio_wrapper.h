#pragma once

#include <vector>
#include <Eigen/Geometry>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>

class pinocchio_wrapper
{
public:
  struct EefState
  {
    Eigen::Vector3d translation;
    Eigen::Quaterniond orientation;
  };

  void init_pinocchio(const std::string &urdf_string, const std::string &end_effector)
  {
    // Build model from URDF
    pinocchio::urdf::buildModelFromXML(urdf_string, model);
    data = pinocchio::Data(model);

    // Get end-effector frame ID
    _ef_frame_id = model.getFrameId(end_effector);
    if (_ef_frame_id >= model.nframes)
      throw std::runtime_error("Frame for end effector " + end_effector + " not found in URDF.");
  }

  Eigen::MatrixXd jacobian(const Eigen::VectorXd &q, const Eigen::VectorXd &dq)
  {
    pinocchio::Data local_data = data;
    
    pinocchio::forwardKinematics(model, local_data, q, dq);
    pinocchio::computeJointJacobians(model, local_data, q);
    pinocchio::computeJointJacobiansTimeVariation(model, local_data,q,dq);
    pinocchio::updateFramePlacements(model, local_data);
    
    Eigen::MatrixXd J = pinocchio::getFrameJacobian(model, local_data, _ef_frame_id,pinocchio::LOCAL_WORLD_ALIGNED);
    
    return J.leftCols(7);
  }

  EefState perform_fk(const Eigen::VectorXd &q) const
  {
    Eigen::VectorXd clamped_q = q;
    for(int i = 0; i < model.nq; ++i)
    {
      double jt = q[i];
      jt = wrap_angle(jt);
      if(model.nq == model.lowerPositionLimit.size())
        jt = std::clamp(jt, model.lowerPositionLimit[i], model.upperPositionLimit[i]);
      clamped_q[i] = jt;
    }

    pinocchio::Data local_data = data;
    pinocchio::forwardKinematics(model, local_data, clamped_q);
    pinocchio::updateFramePlacements(model, local_data);

    const pinocchio::SE3& tf = local_data.oMf[_ef_frame_id];
    return {tf.translation(), Eigen::Quaterniond(tf.rotation()).normalized()};
  }

  uint32_t n_joints() const
  {
    return static_cast<uint32_t>(model.nq);
  }

  std::string root_link() const
  {
    for(const auto& frame : model.frames)
      if(frame.type == pinocchio::BODY && frame.parent == 0)
        return frame.name;
    throw std::runtime_error("Root link not found");
  }

private:
  double wrap_angle(double angle) const
  {
    if(angle <= M_PI && angle >= -M_PI) return angle;
    return angle < 0.0 
      ? std::fmod(angle - M_PI, 2*M_PI) + M_PI
      : std::fmod(angle + M_PI, 2*M_PI) - M_PI;
  }

  pinocchio::Model model;
  mutable pinocchio::Data data;
  pinocchio::FrameIndex _ef_frame_id;
};