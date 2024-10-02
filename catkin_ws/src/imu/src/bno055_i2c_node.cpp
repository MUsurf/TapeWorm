/* bno055_i2c_node.cpp
 * Author: Dheera Venkatraman <dheera@dheera.net>
 *
 * Instantiates a BNO055I2CDriver class, as well as
 * a Watchdog that causes this node to die if things aren't
 * working.
 */

#include "imu_bno055/bno055_i2c_driver.h"
#include "watchdog/watchdog.h"
#include <csignal>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/Temperature.h>
#include <std_srvs/Trigger.h>
#include <std_msgs/UInt8.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/KeyValue.h>

#include <memory>

class BNO055I2CNode {
    public:
        BNO055I2CNode(int argc, char* argv[]);
        void run();
        bool readAndPublish();
        void stop();
        bool onSrvReset(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
    private:
        ros::NodeHandle* nh;
        ros::NodeHandle* nh_priv;
        std::unique_ptr<imu_bno055::BNO055I2CDriver> imu;

        std::string param_device;
        int param_address;
        double param_rate;
        std::string param_frame_id;

        diagnostic_msgs::DiagnosticStatus current_status;

        ros::Publisher pub_IMUdata;
        ros::Publisher pub_IMUraw;
        ros::Publisher pub_IMUmag;
        ros::Publisher pub_IMUtemp;
        ros::Publisher pub_IMUstatus;
        ros::ServiceServer srv_reset;

        std::unique_ptr<ros::Rate> rate;

        watchdog::Watchdog watchdog;

        int seq;
};

BNO055I2CNode::BNO055I2CNode(int argc, char* argv[]) {
    ros::init(argc, argv, "bno055_node");
    nh = new ros::NodeHandle();
    nh_priv = new ros::NodeHandle("~");

    if(!nh || !nh_priv) {
        ROS_FATAL("Failed to initialize node handles");
        ros::shutdown();
        return;
    }

    nh_priv->param("device", param_device, (std::string)"/dev/i2c-8");
    nh_priv->param("address", param_address, (int)BNO055_ADDRESS_A);
    nh_priv->param("frame_id", param_frame_id, (std::string)"imu");
    nh_priv->param("rate", param_rate, (double)100);
 
    imu = std::make_unique<imu_bno055::BNO055I2CDriver>(param_device, param_address);

    imu->init();

    pub_IMUdata = nh->advertise<sensor_msgs::Imu>("IMUdata", 1);
    pub_IMUraw = nh->advertise<sensor_msgs::Imu>("IMUraw", 1);
    pub_IMUmag = nh->advertise<sensor_msgs::MagneticField>("IMUmag", 1);
    pub_IMUtemp = nh->advertise<sensor_msgs::Temperature>("IMUtemp", 1);
    pub_IMUstatus = nh->advertise<diagnostic_msgs::DiagnosticStatus>("IMUstatus", 1);

    srv_reset = nh->advertiseService("reset", &BNO055I2CNode::onSrvReset, this);

    seq = 0;


    current_status.level = 0;
    current_status.name = "BNO055 IMU";
    current_status.hardware_id = "bno055_i2c";

    diagnostic_msgs::KeyValue calib_stat;
    calib_stat.key = "Calibration status";
    calib_stat.value = "";
    current_status.values.push_back(calib_stat);

    diagnostic_msgs::KeyValue selftest_result;
    selftest_result.key = "Self-test result";
    selftest_result.value = "";
    current_status.values.push_back(selftest_result);

    diagnostic_msgs::KeyValue intr_stat;
    intr_stat.key = "Interrupt status";
    intr_stat.value = "";
    current_status.values.push_back(intr_stat);

    diagnostic_msgs::KeyValue sys_clk_stat;
    sys_clk_stat.key = "System clock status";
    sys_clk_stat.value = "";
    current_status.values.push_back(sys_clk_stat);

    diagnostic_msgs::KeyValue sys_stat;
    sys_stat.key = "System status";
    sys_stat.value = "";
    current_status.values.push_back(sys_stat);

    diagnostic_msgs::KeyValue sys_err;
    sys_err.key = "System error";
    sys_err.value = "";
    current_status.values.push_back(sys_err);


    rate =std::make_unique<ros::Rate>(param_rate);
}

void BNO055I2CNode::run() {
    while(ros::ok()) {
        rate->sleep();
        if(readAndPublish()) {
            watchdog.refresh();
        }
    }
}

bool BNO055I2CNode::readAndPublish() {
    imu_bno055::IMURecord record;

    try {
        record = imu->read();
    } catch(const std::runtime_error& e) {
        ROS_WARN_STREAM(e.what());
    }

    ros::Time time = ros::Time::now();

    sensor_msgs::Imu msg_raw;
    msg_raw.header.stamp = time;
    msg_raw.header.frame_id = param_frame_id;
    msg_raw.header.seq = seq;
    msg_raw.linear_acceleration.x = (double)record.raw_linear_acceleration_x / 100.0;
    msg_raw.linear_acceleration.y = (double)record.raw_linear_acceleration_y / 100.0;
    msg_raw.linear_acceleration.z = (double)record.raw_linear_acceleration_z / 100.0;
    msg_raw.angular_velocity.x = (double)record.raw_angular_velocity_x / 900.0;
    msg_raw.angular_velocity.y = (double)record.raw_angular_velocity_y / 900.0;
    msg_raw.angular_velocity.z = (double)record.raw_angular_velocity_z / 900.0;

    sensor_msgs::MagneticField msg_mag;
    msg_mag.header.stamp = time;
    msg_mag.header.frame_id = param_frame_id;
    msg_mag.header.seq = seq;
    msg_mag.magnetic_field.x = (double)record.raw_magnetic_field_x / 16.0;
    msg_mag.magnetic_field.y = (double)record.raw_magnetic_field_y / 16.0;
    msg_mag.magnetic_field.z = (double)record.raw_magnetic_field_z / 16.0;

    sensor_msgs::Imu msg_data;
    msg_data.header.stamp = time;
    msg_data.header.frame_id = param_frame_id;
    msg_data.header.seq = seq;

    double fused_orientation_norm = std::pow(
      std::pow(record.fused_orientation_w, 2) +
      std::pow(record.fused_orientation_x, 2) +
      std::pow(record.fused_orientation_y, 2) +
      std::pow(record.fused_orientation_z, 2), 0.5);

    msg_data.orientation.w = (double)record.fused_orientation_w / fused_orientation_norm;
    msg_data.orientation.x = (double)record.fused_orientation_y / fused_orientation_norm;
    msg_data.orientation.y = (double)record.fused_orientation_x /(-1 * fused_orientation_norm);
    msg_data.orientation.z = (double)record.fused_orientation_z / fused_orientation_norm;
    msg_data.linear_acceleration.x = (double)record.fused_linear_acceleration_y / 100.0;
    msg_data.linear_acceleration.y = (double)record.fused_linear_acceleration_x /(-1 * 100.0);
    msg_data.linear_acceleration.z = (double)record.fused_linear_acceleration_z / 100.0;
    msg_data.angular_velocity.x = (double)record.raw_angular_velocity_y / 900.0;
    msg_data.angular_velocity.y = (double)record.raw_angular_velocity_x /(-1 * 900.0);
    msg_data.angular_velocity.z = (double)record.raw_angular_velocity_z / 900.0;

    sensor_msgs::Temperature msg_temp;
    msg_temp.header.stamp = time;
    msg_temp.header.frame_id = param_frame_id;
    msg_temp.header.seq = seq;
    msg_temp.temperature = (double)record.temperature;

    pub_IMUdata.publish(msg_data);
    pub_IMUraw.publish(msg_raw);
    pub_IMUmag.publish(msg_mag);
    pub_IMUtemp.publish(msg_temp);

    if((seq++) % 50 == 0) {
        current_status.values[DIAG_CALIB_STAT].value = std::to_string(record.calibration_status);
        current_status.values[DIAG_SELFTEST_RESULT].value = std::to_string(record.self_test_result);
        current_status.values[DIAG_INTR_STAT].value = std::to_string(record.interrupt_status);
        current_status.values[DIAG_SYS_CLK_STAT].value = std::to_string(record.system_clock_status);
        current_status.values[DIAG_SYS_STAT].value = std::to_string(record.system_status);
        current_status.values[DIAG_SYS_ERR].value = std::to_string(record.system_error_code);
        pub_IMUstatus.publish(current_status);
    }

    return true;
}

void BNO055I2CNode::stop() {
    ROS_INFO("stopping");
    if(pub_IMUdata) pub_IMUdata.shutdown();
    if(pub_IMUraw) pub_IMUraw.shutdown();
    if(pub_IMUmag) pub_IMUmag.shutdown();
    if(pub_IMUtemp) pub_IMUtemp.shutdown();
    if(pub_IMUstatus) pub_IMUstatus.shutdown();
    if(srv_reset) srv_reset.shutdown();
}

bool BNO055I2CNode::onSrvReset(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
    if(!(imu->reset())) {
        throw std::runtime_error("chip reset failed");
        return false;
    }
    return true;
}

int main(int argc, char *argv[]) {
    BNO055I2CNode node(argc, argv);
    node.run();
    return 0;
}
