/// \file
/// \brief Computes forward and inverse kinematics of turtlebot

#include <iostream>
#include "turtlelib/diff_drive.hpp"

namespace turtlelib
{
DiffDrive::DiffDrive(double track, double radius)
: track{track}, radius{radius}, phi{0.0, 0.0}, q{0.0, 0.0, 0.0} {}

DiffDrive::DiffDrive(double track, double radius, Phi phi, Config q)
: track{track}, radius{radius}, phi{phi}, q{q} {}

/// \brief calculated from Forward Kinematics section in notes
/// Used pseudoinverse of H matrix from Inverse Kinematic calculations
/// Reference equations 3 and 4 on page 6 of notes
Twist2D DiffDrive::calc_body_twist(Phi p)
{
  /// equation 3
  double rob_theta = (radius / track) * (-p.l + p.r);
  /// equation 4
  double rob_x = (radius / 2.0) * (p.l + p.r);
  double rob_y = 0.0;
  return {rob_theta, rob_x, rob_y};
}

DiffDrive DiffDrive::Forward_Kin(Phi p)
{
  Twist2D twist = calc_body_twist(p);
  /// Integrating the calculated body twist to get a transformation
  Transform2D T = integrate_twist(twist);
  /// Transforming the body transform into the world frame
  double Tw_x = T.translation().x * std::cos(q.theta) - T.translation().y * std::sin(q.theta);
  double Tw_y = T.translation().x * std::sin(q.theta) + T.translation().y * std::cos(q.theta);
  /// updating the robot values
  q.theta += T.rotation();
  q.x += Tw_x;
  q.y += Tw_y;

  phi.r += p.r;
  phi.l += p.l;

  return *this;
}

/// \brief calculated from Inverse Kinematics section in notes
/// Calculate wheel left and right twists from body twist and adjoints on pages 1 and 2
/// Found H matrix on pages 3 and 4 using angular wheel rotations (phi)
/// Reference equations 1 and 2 on page 5 of notes
Phi DiffDrive::Inverse_Kin(Twist2D twist)
{
  if (twist.getY() != 0.0) {
    throw std::logic_error("Wheels will slip");
  }
  /// equation 2
  double phi_r = (twist.getX() / radius) + (track / 2.0) * (twist.getW() / radius);
  /// equation 1
  double phi_l = (twist.getX() / radius) - (track / 2.0) * (twist.getW() / radius);
  return {phi_r, phi_l};
}
}
