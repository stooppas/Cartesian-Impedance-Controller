#pragma once

#include <vector>

#include <RBDyn/FK.h>
#include <RBDyn/FV.h>
#include <RBDyn/Jacobian.h>
#include <SpaceVecAlg/Conversions.h>
#include <RBDyn/parsers/urdf.h>
#include <Eigen/Geometry>

class rbdyn_wrapper
{
public:
  struct EefState
  {
    Eigen::Vector3d translation;
    Eigen::Quaterniond orientation;
  };

  void init_rbdyn(const std::string &urdf_string, const std::string &end_effector)
  {
    // Convert URDF to RBDyn
    _parser_result = rbd::parsers::from_urdf(urdf_string);

    _rbd_indices.clear();

    for (int32_t i = 0; i < _parser_result.mb.nrJoints(); i++)
    {
      if (_parser_result.mb.joint(i).type() != rbd::Joint::Fixed)
        _rbd_indices.push_back(i);
    }

    for (int32_t i = 0; i < _parser_result.mb.nrBodies(); i++)
    {
      if (_parser_result.mb.body(i).name() == end_effector)
      {
        _ef_index = i;
        return;
      }
    }
    throw std::runtime_error("Index for end effector link " + end_effector + " not found in URDF. Aborting.");
  }

  Eigen::MatrixXd jacobian(const Eigen::VectorXd &q, const Eigen::VectorXd &dq)
  {
    rbd::parsers::ParserResult parser_result = _parser_result;

    parser_result.mbc.zero(parser_result.mb);

    _update_urdf_state(parser_result, q, dq);

    // Compute jacobian
    rbd::Jacobian jac(parser_result.mb, parser_result.mb.body(_ef_index).name());

    // // TO-DO: Check if we need this
    rbd::forwardKinematics(parser_result.mb, parser_result.mbc);
    rbd::forwardVelocity(parser_result.mb, parser_result.mbc);

    return jac.jacobian(parser_result.mb, parser_result.mbc);
  }

  EefState perform_fk(const Eigen::VectorXd &q) const
  {
    rbd::parsers::ParserResult parser_result = _parser_result;

    Eigen::VectorXd q_low = Eigen::VectorXd::Ones(_rbd_indices.size());
    Eigen::VectorXd q_high = q_low;

    for (uint32_t i = 0; i < _rbd_indices.size(); i++)
    {
      uint32_t index = _rbd_indices[i];
      q_low(i) = parser_result.limits.lower[parser_result.mb.joint(index).name()][0];
      q_high(i) = parser_result.limits.upper[parser_result.mb.joint(index).name()][0];
    }

    parser_result.mbc.zero(parser_result.mb);

    for (size_t i = 0; i < _rbd_indices.size(); i++)
    {
      size_t rbd_index = _rbd_indices[i];
      double jt = q[i];
      // wrap in [-pi,pi]
      jt = _wrap_angle(jt);
      // enforce limits
      if (jt < q_low(i))
        jt = q_low(i);
      if (jt > q_high(i))
        jt = q_high(i);

      parser_result.mbc.q[rbd_index][0] = jt;
    }

    rbd::forwardKinematics(parser_result.mb, parser_result.mbc);

    sva::PTransformd tf = parser_result.mbc.bodyPosW[_ef_index];

    Eigen::Matrix4d eig_tf = sva::conversions::toHomogeneous(tf);
    Eigen::Vector3d trans = eig_tf.col(3).head(3);
    Eigen::Matrix3d rot_mat = eig_tf.block(0, 0, 3, 3);
    Eigen::Quaterniond quat = Eigen::Quaterniond(rot_mat).normalized();

    return {trans, quat};
  }

  uint32_t n_joints() const
  {
    return (uint32_t)_rbd_indices.size();
  }

  std::string root_link() const
  {
    return _parser_result.mb.body(0).name();
  }

private:
  void _update_urdf_state(rbd::parsers::ParserResult &parser_result, const Eigen::VectorXd &q,
                          const Eigen::VectorXd &dq)
  {
    for (uint32_t i = 0; i < _rbd_indices.size(); i++)
    {
      uint32_t rbd_index = _rbd_indices[i];

      if (q.size() > i)
        parser_result.mbc.q[rbd_index][0] = q[i];
      if (dq.size() > i)
        parser_result.mbc.alpha[rbd_index][0] = dq[i];
    }
  }

  double _wrap_angle(const double &angle) const
  {
    double wrapped;
    if ((angle <= M_PI) && (angle >= -M_PI))
    {
      wrapped = angle;
    }
    else if (angle < 0.0)
    {
      wrapped = std::fmod(angle - M_PI, 2.0 * M_PI) + M_PI;
    }
    else
    {
      wrapped = std::fmod(angle + M_PI, 2.0 * M_PI) - M_PI;
    }
    return wrapped;
  }

  rbd::parsers::ParserResult _parser_result;
  std::vector<uint32_t> _rbd_indices;
  uint32_t _ef_index;
};