#include "turtlelib/discriminant.hpp"

double turtlelib::calc_disc(const turtlelib::Disc & disc)
{
  return disc.b * disc.b - 4 * disc.a * disc.c;
}

double turtlelib::calc_x(const turtlelib::Disc & disc, int sign)
{
  return (-disc.b + sign * sqrt(calc_disc(disc))) / (2 * disc.a);
}
