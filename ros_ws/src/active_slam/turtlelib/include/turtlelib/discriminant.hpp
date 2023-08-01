#ifndef DISCRIMINANT_INCLUDE_GUARD_HPP
#define DISCRIMINANT_INCLUDE_GUARD_HPP
/// \file
/// \brief Calculating the discriminant of a quadratic.

#include<cmath>  // contains math definitions

namespace turtlelib
{
    /// \brief Struct for the discriminant of a quadratic
    struct Disc{
        /// \brief the a term of discriminant
        double a = 0.0;

        /// \brief the b term of discriminant
        double b = 0.0;

        /// \brief the c term of discriminant
        double c = 0.0;
    };

    /// \brief compute the discriminant
    /// \param disc - the discriminant object to calculate with
    /// \return the discriminant
    double calc_disc(const Disc & disc);

    /// \brief compute x from the quadratic formula
    /// \param disc - the discriminant object to calculate with
    /// \param sign - input a negative or positive 1 for the +/- in the equation
    /// \return the value of x
    double calc_x(const Disc & disc, int sign);
}
#endif