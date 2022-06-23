#pragma once
#include <iostream>
#include <vector>

struct config{
    std::string port = "/dev/ttyACM0";
    int baud_rate = 115200;
};

struct IMU{
    int time[2] = {0, 0};  // s, ms
    float linear_acceleration[3] = {0.0, 0.0, 0.0};
    float angular_velocity[3] = {0.0, 0.0, 0.0};
    float magnetic_field[3] = {0.0, 0.0, 0.0};
    float linear_acceleration_cov;
    float angular_velocity_cov;
    float orientation_cov;
    float magnetic_field_cov;
    float roll;
    float pitch;
    float yaw;
};
