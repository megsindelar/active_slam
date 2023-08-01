/// \file
/// \brief Tests functionality of rigid2D

#include <iostream>
#include "turtlelib/rigid2d.hpp"

/// \brief Tests:
/// Transform multiplication and inverse
/// Normalizing a vector and transforming a vector
/// Transforming a twist
/// component
int main()
{
  turtlelib::Transform2D T_ab, T_ba, T_bc, T_cb, T_ac, T_ca;
  std::cout << "Enter transform T_{a,b}:" << std::endl;
  std::cin >> T_ab;
  std::cin.clear();

  std::cout << "Enter transform T_{b,c}:" << std::endl;
  std::cin >> T_bc;
  std::cin.clear();

  T_ba = T_ab.inv();
  T_cb = T_bc.inv();
  T_ac = T_ab * T_bc;
  T_ca = T_ac.inv();
  std::cout << "T_{a,b}: " << T_ab << "\n";
  std::cout << "T_{b,a}: " << T_ba << "\n";
  std::cout << "T_{b,c}: " << T_bc << "\n";
  std::cout << "T_{c,b}: " << T_cb << "\n";
  std::cout << "T_{a,c}: " << T_ac << "\n";
  std::cout << "T_{c,a}: " << T_ca << "\n" << std::endl;

  turtlelib::Vector2D vec_b, vec_b_hat, vec_a, vec_c;
  std::cout << "Enter vector v_b:" << std::endl;
  std::cin >> vec_b;
  vec_b_hat = turtlelib::normalize_vec(vec_b);
  vec_a = T_ab(vec_b);
  vec_c = T_cb(vec_b);
  std::cout << "v_bhat: " << vec_b_hat << "\n";
  std::cout << "v_a: " << vec_a << "\n";
  std::cout << "v_b: " << vec_b << "\n";
  std::cout << "v_c: " << vec_c << std::endl;
  std::cin.clear();

  turtlelib::Twist2D V_a = {0.0, 0.0, 0.0};
  turtlelib::Twist2D V_b = {0.0, 0.0, 0.0};
  turtlelib::Twist2D V_c = {0.0, 0.0, 0.0};
  std::cout << "Enter twist V_b:" << std::endl;
  std::cin >> V_b;
  V_a = T_ab(V_b);
  V_c = T_cb(V_b);
  std::cout << "V_a " << V_a << "\n";
  std::cout << "V_b " << V_b << "\n";
  std::cout << "V_c " << V_c << std::endl;
}
