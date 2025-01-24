#pragma once

#include <vector>

#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/parsers/urdf.hpp>

class pinocchio_wrapper
{
public:
  struct EefState
  {
    Eigen::Vector3d translation;
    Eigen::Quaterniond orientation;
  };

  void init_pinocchio(const std::string &urdf_string, const std::string &end_effector, std::vector<std::string>& joints)
  {
    // Convert URDF to RBDyn
    pinocchio::urdf::buildModelFromXML(urdf_string, model);

    if(!model.existFrame(end_effector))
    {
      throw std::runtime_error("Index for end effector link " + end_effector + " not found in URDF. Aborting.");
    }

    // Find the index of the end effector in the model
    ee_frame_id = model.getFrameId(end_effector);

    // Identify the joints to be used for computation
    _pinocchio_indices.clear();
    for (int i = 0; i < model.njoints; ++i)
    {
      for(int j = 0; j < joints.size(); j++)
      {
        if(model.names[i] == joints[j])
        {
          _pinocchio_indices.push_back(i);
        }
      }
    }
      
  }

  Eigen::MatrixXd jacobian(const Eigen::VectorXd &q, const Eigen::VectorXd &dq)
  {
    pinocchio::Data data(model);
    
    // Update the joint configuration
    pinocchio::forwardKinematics(model, data, q, dq);

    // Compute the Jacobian for the end effector
    Eigen::MatrixXd jac(n_joints(),model.nv);
    pinocchio::computeFrameJacobian(model, data, q, ee_frame_id, jac);
    return jac;
  }

  EefState perform_fk(const Eigen::VectorXd &q) const
  {
    pinocchio::Data data(model);
    
    // Perform forward kinematics
    pinocchio::forwardKinematics(model, data, q);

    // Get the transformation matrix of the end effector
    const pinocchio::SE3 &transformation = data.oMi[ee_frame_id];
    Eigen::Matrix4d eig_tf = transformation.toHomogeneousMatrix();
    
    // Extract translation and rotation
    Eigen::Vector3d trans = eig_tf.col(3).head(3);
    Eigen::Matrix3d rot_mat = eig_tf.block(0, 0, 3, 3);
    Eigen::Quaterniond quat(rot_mat);

    return {trans, quat};
  }

  uint32_t n_joints() const
  {
    return (uint32_t)_pinocchio_indices.size();
  }

  std::string root_link() const
  {
    return model.frames[0].name;
  }

private:
  pinocchio::Model model;
  std::vector<size_t> _pinocchio_indices;
  pinocchio::FrameIndex ee_frame_id;
};