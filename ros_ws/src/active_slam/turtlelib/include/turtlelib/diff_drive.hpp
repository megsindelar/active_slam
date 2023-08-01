#ifndef DIFFDRIVE_INCLUDE_GUARD_HPP
#define DIFFDRIVE_INCLUDE_GUARD_HPP
/// \file
/// \brief Kinematics of wheeled mobile robots.


#include<iosfwd> // contains forward definitions for iostream objects
#include<cmath>  // contains math definitions
#include"turtlelib/rigid2d.hpp"   //contains Vector2D and Transform2D class

namespace turtlelib
{
    /// \brief Struct for robot wheel rotations
    struct Phi{
        /// \brief the right wheel
        double r = 0.0;

        /// \brief the left wheel
        double l = 0.0;

    };

    /// \brief Struct for robot configuration
    struct Config{
        /// \brief the rotation of the robot
        double theta = 0.0;

        /// \brief the robot position in the x-direction
        double x = 0.0;

        /// \brief the robot position in the y-direction
        double y = 0.0;

    };    

    /// \brief Robot kinematics for any general robot.
    class DiffDrive
    {
    private:
        double track, radius;
        Phi phi {0.0, 0.0};
        Config q {0.0, 0.0, 0.0};
    public:
        /// \brief robot with no position or configuration
        /// \param track - the distance between the two wheels
        /// \param radius - the wheel radius
        DiffDrive(double track, double radius);

        /// \brief robot with full position and configuration
        /// \param track - the distance between the two wheels
        /// \param radius - the wheel radius
        /// \param phi - the rotation of the robot wheels
        /// \param q - the configuration of the robot
        DiffDrive(double track, double radius, Phi phi, Config q);

        /// \brief setter for private variable right phi
        void phi_r_set(double new_phi_r) {phi.r = new_phi_r;}

        /// \brief setter for private variable left phi
        void phi_l_set(double new_phi_l) {phi.l = new_phi_l;}

        /// \brief to get the private variable right phi
        double phi_r_get() const {return phi.r;}

        /// \brief to get the private variable left phi
        double phi_l_get() const {return phi.l;}

        /// \brief to get the private variable theta
        double theta_get() const {return q.theta;}

        /// \brief setter for private variable right phi
        void x_set(double x) {q.x = x;}

        /// \brief setter for private variable left phi
        void y_set(double y) {q.y = y;}
        
        /// \brief to get the private variable theta
        double x_get() const {return q.x;}
        
        /// \brief to get the private variable theta
        double y_get() const {return q.y;}

        /// \brief compute the Forward Kinematics to update configuration
        /// \param p - the change in rotation of the robot wheels
        /// \return a robot with updated configuration and wheels
        DiffDrive Forward_Kin(Phi p);

        /// \brief compute the Inverse Kinematics of the robot
        /// \param twist - the twist used to find new wheel rotations
        /// \return new wheel rotoations based on inverse kinematics
        Phi Inverse_Kin(Twist2D twist);

        /// \brief compute the Forward Kinematics to update configuration
        /// \param p - the change in rotation of the robot wheels
        /// \return a twist from wheel rotations
        Twist2D calc_body_twist(Phi p);
    };
}

#endif