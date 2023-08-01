/// \file
/// \brief perform 2D rigid body transformations and other functionality

#include <iostream>
#include "turtlelib/rigid2d.hpp"

namespace turtlelib
{
std::ostream & operator<<(std::ostream & os, const Vector2D & v)
{
  os << "[" << v.x << " " << v.y << "]";
  return os;
}

std::istream & operator>>(std::istream & is, Vector2D & v)
{
  if (is.peek() == '[') {
    is.get();
    is >> v.x >> v.y;
  } else {
    is >> v.x >> v.y;
  }
  is.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
  return is;
}

Transform2D::Transform2D()
: translate{0.0, 0.0}, theta{0.0} {}

Transform2D::Transform2D(Vector2D trans)
: translate{trans}, theta{0.0} {}

Transform2D::Transform2D(double radians)
: translate{0.0, 0.0}, theta{radians} {}

Transform2D::Transform2D(Vector2D trans, double radians)
: translate{trans}, theta{radians} {}

Vector2D Transform2D::operator()(Vector2D v) const
{
  double x_new = std::cos(theta) * v.x - std::sin(theta) * v.y + translate.x;
  double y_new = std::sin(theta) * v.y + std::cos(theta) * v.x + translate.y;
  Vector2D vec = {x_new, y_new};
  return vec;
}

Transform2D Transform2D::inv() const
{
  double x_new = -std::cos(theta) * translate.x - std::sin(theta) * translate.y;
  double y_new = -std::cos(theta) * translate.y + std::sin(theta) * translate.x;
  Vector2D new_vec = {x_new, y_new};
  double new_radians = -theta;
  return Transform2D(new_vec, new_radians);
}

Transform2D & Transform2D::operator*=(const Transform2D & rhs)
{
  this->translate.x = this->translate.x + rhs.translate.x * std::cos(this->theta) -
    rhs.translate.y * std::sin(this->theta);
  this->translate.y = this->translate.y + rhs.translate.x * std::sin(this->theta) +
    rhs.translate.y * std::cos(this->theta);
  this->theta = this->theta + rhs.theta;
  if (std::abs(this->theta) > 2 * PI) {
    this->theta = std::fmod(this->theta, 2 * PI);
  }
  return *this;
}

Vector2D Transform2D::translation() const
{
  return {translate.x, translate.y};
}

double Transform2D::rotation() const
{
  return theta;
}

/// deg: 90 x: 3 y: 5
std::ostream & operator<<(std::ostream & os, const Transform2D & tf)
{
  double theta_deg = rad2deg(tf.theta);
  os << "deg: " << theta_deg << " x: " << tf.translate.x << " y: " << tf.translate.y;
  return os;
}


std::istream & operator>>(std::istream & is, Transform2D & tf)
{
  std::string str, str2, str3;
  double rotate = 0.0;
  Vector2D trans = {0.0, 0.0};
  if (is.peek() == 'd') {
    is >> str >> rotate >> str2 >> trans.x >> str3 >> trans.y;
  } else {
    is >> rotate >> trans.x >> trans.y;
  }
  const auto theta_rad = deg2rad(rotate);
  tf = Transform2D{trans, theta_rad};
  is.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
  return is;
}

Transform2D operator*(Transform2D lhs, const Transform2D & rhs)
{
  lhs *= rhs;
  return lhs;
}

std::ostream & operator<<(std::ostream & os, const Twist2D & VT)
{
  os << "[" << VT.w << " " << VT.x << " " << VT.y << "]";
  return os;
}

std::istream & operator>>(std::istream & is, Twist2D & VT)
{
  double w = 0.0;
  double x = 0.0;
  double y = 0.0;
  if (is.peek() == '[') {
    is.get();
    is >> w >> x >> y;
  } else {
    is >> w >> x >> y;
  }
  VT = Twist2D{w, x, y};
  is.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
  return is;
}

Twist2D Transform2D::operator()(Twist2D twist) const
{
  double w = 0.0;
  double x = 0.0;
  double y = 0.0;
  w = twist.getW();
  x = twist.getW() * translate.y + twist.getX() * std::cos(theta) - twist.getY() * std::sin(theta);
  y = -twist.getW() * translate.x + twist.getX() * std::sin(theta) + twist.getY() * std::cos(theta);
  return {w, x, y};
}

Vector2D normalize_vec(Vector2D vec)
{
  Vector2D norm;
  double mag = sqrt(pow(vec.x, 2) + pow(vec.y, 2));
  return {vec.x / mag, vec.y / mag};
}

double normalize_angle(double rad)
{
  while (rad > PI || rad <= -PI) {
    if (rad > PI) {
      rad = rad - 2 * PI + 0.01745;
    } else if (rad <= -PI) {
      rad = rad + 2 * PI;
    }
  }
  return rad;
}

Vector2D & Vector2D::operator+=(const Vector2D & rhs)
{
  this->x = this->x + rhs.x;
  this->y = this->y + rhs.y;
  return *this;
}

Vector2D operator+(Vector2D lhs, const Vector2D & rhs)
{
  lhs += rhs;
  return lhs;
}

Vector2D & Vector2D::operator-=(const Vector2D & rhs)
{
  this->x = this->x - rhs.x;
  this->y = this->y - rhs.y;
  return *this;
}

Vector2D operator-(Vector2D lhs, const Vector2D & rhs)
{
  lhs -= rhs;
  return lhs;
}

Vector2D & Vector2D::operator*=(const Vector2D & rhs)
{
  this->x = this->x * rhs.x;
  this->y = this->y * rhs.y;
  return *this;
}

Vector2D operator*(Vector2D lhs, const Vector2D & rhs)
{
  lhs *= rhs;
  return lhs;
}

double dot(const Vector2D & vec1, const Vector2D & vec2)
{
  return vec1.x * vec2.x + vec1.y * vec2.y;
}

double magnitude(const Vector2D & vec)
{
  return std::sqrt(std::pow(vec.x, 2) + std::pow(vec.y, 2));
}

double angle(const Vector2D & vec1, const Vector2D & vec2)
{
  return std::acos(dot(vec1, vec2) / (magnitude(vec1) * magnitude(vec2)));
}

Transform2D integrate_twist(Twist2D twist)
{
  Vector2D vec;
  double x = twist.getX();
  double y = twist.getY();
  double w = twist.getW();
  if (w == 0.0) {
    Transform2D T = {{x, y}, 0.0};
    return T;
  } else {
    vec.x = y / w;
    vec.y = -x / w;
    Transform2D T_sb = {vec, 0.0};
    Transform2D T_bs = T_sb.inv();
    Transform2D T_ss = {{0.0, 0.0}, w};
    Transform2D T_bb = T_bs * T_ss * T_sb;
    return T_bb;
  }
}

}
