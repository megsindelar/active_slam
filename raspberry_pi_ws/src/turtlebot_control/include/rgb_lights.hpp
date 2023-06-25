#ifndef RGB_LIGHTS_HPP_
#define RGB_LIGHTS_HPP_

#include <chrono>
#include <fstream>
#include <iostream>
#include "rclcpp/rclcpp.hpp"

namespace turtlebot_control
{
    void writeGPIO(const std::string &pin, const std::string &value)
    {
        std::ofstream file_export("/sys/class/gpio/export");
        if (file_export.is_open())
        {
            file_export << pin;
            file_export.close();
        }
        else{
            RCLCPP_INFO(rclcpp::get_logger("message"), "Failed to open GPIO export file");
        }
        sleep(1);

        std::ofstream file_dir("/sys/class/gpio/gpio" + pin + "/direction");
        if (file_dir.is_open())
        {
            file_dir << "out";
            file_dir.close();
        }
        else{
            RCLCPP_INFO(rclcpp::get_logger("message"), "Failed to open GPIO direction file");
        }
        sleep(1);

        std::ofstream file_val("/sys/class/gpio/gpio" + pin + "/value");
        if (file_val.is_open())
        {
            file_val << value;
            file_val.close();
        }
        else{
            RCLCPP_INFO(rclcpp::get_logger("message"), "Failed to open GPIO value file");
        }
        sleep(1);

        std::ofstream file_unexport("/sys/class/gpio/unexport");
        if (file_unexport.is_open())
        {
            file_unexport << pin;
            file_unexport.close();
        }
        else{
            RCLCPP_INFO(rclcpp::get_logger("message"), "Failed to open GPIO export file");
        }
    }
} // namespace turtlebot_control

#endif  // RGB_LIGHTS_HPP_