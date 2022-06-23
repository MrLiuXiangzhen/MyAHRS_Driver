//
// Created by liuxiangzhen on 22-6-23.
//
#include "myahrs_plus.hpp"
#include "config.hpp"

using namespace std;
using namespace WithRobot;


class MyAhrsDriver: public iMyAhrsPlus{
private:
    Platform::Mutex lock_;
    SensorData sensor_data_;
    double linear_acceleration_stddev_;
    double angular_velocity_stddev_;
    double magnetic_field_stddev_;
    double orientation_stddev_;

    void OnSensorData(int sensor_id, SensorData data) {
        LockGuard _l(lock_);
        sensor_data_ = data;
        publish_imu(sensor_id);
    }

    void OnAttributeChange(int sensor_id, std::string attribute_name, std::string value) {
        printf("OnAttributeChange(id %d, %s, %s)\n", sensor_id, attribute_name.c_str(), value.c_str());
    }

public:
    MyAhrsDriver(std::string port="", int baud_rate=115200) : iMyAhrsPlus(port, baud_rate){ }

    ~MyAhrsDriver() {}

    bool initialize() {
        bool ok = false;
        do {
            if(!start()) break;
            //Euler angle(x, y, z axis)
            //IMU(linear_acceleration, angular_velocity, magnetic_field)
            if(!cmd_binary_data_format("EULER, IMU")) break;
            // 100Hz
            if(!cmd_divider("1")) break;
            // Binary and Continue mode
            if(!cmd_mode("BC")) break;
            ok = true;
        } while(0);

        return ok;
    }

    inline void get_data(SensorData& data) {
        LockGuard _l(lock_);
        data = sensor_data_;
    }

    inline SensorData get_data() {
        LockGuard _l(lock_);
        return sensor_data_;
    }

    void publish_imu(int sensor_id) {
        IMU imu_data;
        double linear_acceleration_cov = linear_acceleration_stddev_ * linear_acceleration_stddev_;
        double angular_velocity_cov    = angular_velocity_stddev_ * angular_velocity_stddev_;
        double magnetic_field_cov      = magnetic_field_stddev_ * magnetic_field_stddev_;
        double orientation_cov         = orientation_stddev_ * orientation_stddev_;
        imu_data.angular_velocity_cov = angular_velocity_cov;
        imu_data.linear_acceleration_cov = linear_acceleration_cov;
        imu_data.magnetic_field_cov = magnetic_field_cov;
        imu_data.orientation_cov = orientation_cov;

        static double convertor_g2a  = 9.80665;    // for linear_acceleration (g to m/s^2)
        static double convertor_d2r  = M_PI/180.0; // for angular_velocity (degree to radian)
        static double convertor_r2d  = 180.0/M_PI; // for easy understanding (radian to degree)
        static double convertor_ut2t = 1000000;    // for magnetic_field (uT to Tesla)
        static double convertor_c    = 1.0;        // for temperature (celsius)

        double roll, pitch, yaw;

        // original sensor data used the degree unit, convert to radian (see ROS REP103)
        // we used the ROS's axes orientation like x forward, y left and z up
        // so changed the y and z aixs of myAHRS+ board
        roll  =  sensor_data_.euler_angle.roll*convertor_d2r;
        pitch = -sensor_data_.euler_angle.pitch*convertor_d2r;
        yaw   = -sensor_data_.euler_angle.yaw*convertor_d2r;
        imu_data.roll = roll;
        imu_data.pitch = pitch;
        imu_data.yaw = yaw;

        ImuData<float>& imu = sensor_data_.imu;

        struct timeval time_val;  // from 1970
        gettimeofday(&time_val, NULL);
        imu_data.time[0] = (int)time_val.tv_sec;
        imu_data.time[1] = (int)time_val.tv_usec;
        // original data used the g unit, convert to m/s^2
        imu_data.linear_acceleration[0] = imu.ax * convertor_g2a;
        imu_data.linear_acceleration[1] = -imu.ay * convertor_g2a;
        imu_data.linear_acceleration[2] = -imu.az * convertor_g2a;

        // original data used the degree/s unit, convert to radian/s
        imu_data.angular_velocity[0] = imu.gx * convertor_d2r;
        imu_data.angular_velocity[1] = -imu.gy * convertor_d2r;
        imu_data.angular_velocity[2] = -imu.gz * convertor_d2r;

        // original data used the uTesla unit, convert to Tesla
        imu_data.magnetic_field[0] = imu.mx / convertor_ut2t;
        imu_data.magnetic_field[1] = -imu.my / convertor_ut2t;
        imu_data.magnetic_field[2] = -imu.mz / convertor_ut2t;

        printf("%d:%d -> linearAcc: %f, %f, %f | angVel: %f, %f, %f \n", imu_data.time[0], imu_data.time[1], imu_data.linear_acceleration[0], imu_data.linear_acceleration[1], imu_data.linear_acceleration[2], imu_data.angular_velocity[0], imu_data.angular_velocity[1], imu_data.angular_velocity[2]);
    }
};

int main() {
    config cfg;
    cout << "begin" << endl;
    MyAhrsDriver sensor(cfg.port, cfg.baud_rate);
    if(!sensor.initialize()) {
        cout << "Initialize() returns false, please check your devices." << endl;
        return 0;
    }
    else {
        cout <<  "Initialization OK!" << endl;
    }
    while(true){
        sleep(1);
    }
    cout << "ok" << endl;
}