#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "geometry_msgs/TransformStamped.h"
#include <sstream>
#include <cmath>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>

#define IMU9250_ADDR 0x68
#define AK8963_ADDR 0x0C

// IMU9250 Register Addresses
#define PWR_MGMT_1 0x6B
#define INT_PIN_CFG 0x37
#define ACCEL_CONFIG 0x1C
#define GYRO_CONFIG 0x1B
#define ACCEL_XOUT_H 0x3B
#define ACCEL_YOUT_H 0x3D
#define ACCEL_ZOUT_H 0x3F
#define GYRO_XOUT_H 0x43
#define GYRO_YOUT_H 0x45
#define GYRO_ZOUT_H 0x47

// Magnetometer Registers (AK8963)
#define AK8963_ST1 0x02
#define AK8963_CNTL1 0x0A
#define AK8963_CNTL2 0x0B
#define AK8963_ASTC 0x0C
#define MAG_XOUT_L 0x03
#define MAG_YOUT_L 0x05
#define MAG_ZOUT_L 0x07
#define MAG_ASAX 0x10
#define MAG_ASAY 0x11
#define MAG_ASAZ 0x12

// Scaling factors
#define ACCEL_SCALE_MODIFIER_2G 16384.0
#define GYRO_SCALE_MODIFIER_250DEG 131
#define GYRO_SCALE_MODIFIER_500DEG 65.5
#define GYRO_SCALE_MODIFIER_2000DEG 16.4
#define MAG_SCALE_MODIFIER 0.6

#define GRAVITY 9.80665

struct IMUOffsets {
    double accel_x_offset;
    double accel_y_offset;
    double accel_z_offset;
    double gyro_x_offset;
    double gyro_y_offset;
    double gyro_z_offset;
};

static inline __s32 i2c_smbus_access(int file, char read_write, __u8 command, int size, union i2c_smbus_data *data)
{
    struct i2c_smbus_ioctl_data args;
    args.read_write = read_write;
    args.command = command;
    args.size = size;
    args.data = data;
    return ioctl(file, I2C_SMBUS, &args);
}

static inline __s32 i2c_smbus_read_byte_data(int file, __u8 command)
{
    union i2c_smbus_data data;
    if (i2c_smbus_access(file, I2C_SMBUS_READ, command, I2C_SMBUS_BYTE_DATA, &data))
        return -1;
    else
        return 0x0FF & data.byte;
}

static inline __s32 i2c_smbus_write_byte_data(int file, __u8 command, uint8_t data_in)
{
    union i2c_smbus_data data;
    data.byte = data_in;
    if (i2c_smbus_access(file, I2C_SMBUS_WRITE, command, I2C_SMBUS_BYTE_DATA, &data))
        return -1;
    else
        return 0x0FF & data.byte;
}

uint8_t i2c_read(int file, uint8_t dev_addr, uint8_t reg)
{
    int rc;
    uint8_t read_back;

    rc = ioctl(file, I2C_SLAVE, dev_addr); // Sets the device address
    if (rc < 0)
    {
        ROS_INFO("ERROR - couldn't set device address");
        return false;
    }

    read_back = i2c_smbus_read_byte_data(file, reg);
    return read_back;
}

uint8_t i2c_write(int file, uint8_t dev_addr, uint8_t reg, uint8_t data_in)
{
    int rc;
    uint8_t read_back;

    rc = ioctl(file, I2C_SLAVE, dev_addr); // Sets the device address
    if (rc < 0)
    {
        ROS_INFO("ERROR - couldn't set device address");
        return false;
    }

    read_back = i2c_smbus_write_byte_data(file, reg, data_in);
    return read_back;
}

int16_t read_word_2c(int file, uint8_t dev_addr, uint8_t reg)
{
    uint8_t high = i2c_read(file, dev_addr, reg);
    uint8_t low = i2c_read(file, dev_addr, reg + 1);
    int16_t value = (high << 8) + low;
    if (value >= 0x8000)
    {
        value = -(65536 - value);
    }
    return value;
}

bool perform_self_test(int file)
{
    // Perform a simple read/write test to check if the sensors respond correctly
    uint8_t original_value, test_value;

    // Test MPU6050
    original_value = i2c_read(file, IMU9250_ADDR, GYRO_CONFIG);
    i2c_write(file, IMU9250_ADDR, GYRO_CONFIG, 0x08);
    test_value = i2c_read(file, IMU9250_ADDR, GYRO_CONFIG);
    if (test_value != 0x08)
    {
        ROS_INFO("IMU9250 self-test failed.");
        return false;
    }
    i2c_write(file, IMU9250_ADDR, GYRO_CONFIG, original_value);

    // Test HMC5883L
/*    original_value = i2c_read(file, HMC5883L_ADDR, MODE_REG);
    i2c_write(file, HMC5883L_ADDR, MODE_REG, 0x01);
    test_value = i2c_read(file, HMC5883L_ADDR, MODE_REG);
    if (test_value != 0x01)
    {
        ROS_INFO("HMC5883L self-test failed.");
        return false;
    }
    i2c_write(file, HMC5883L_ADDR, MODE_REG, original_value);
*/
    ROS_INFO("Self-test passed.");
    return true;
}

IMUOffsets calibrateIMU(int file) {
    const int calibration_samples = 500;
    IMUOffsets offsets = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    for (int i = 0; i < calibration_samples; ++i) {
        int16_t accel_x_raw = read_word_2c(file, IMU9250_ADDR, ACCEL_XOUT_H);
        int16_t accel_y_raw = read_word_2c(file, IMU9250_ADDR, ACCEL_YOUT_H);
        int16_t accel_z_raw = read_word_2c(file, IMU9250_ADDR, ACCEL_ZOUT_H);
        int16_t gyro_x_raw = read_word_2c(file, IMU9250_ADDR, GYRO_XOUT_H);
        int16_t gyro_y_raw = read_word_2c(file, IMU9250_ADDR, GYRO_YOUT_H);
        int16_t gyro_z_raw = read_word_2c(file, IMU9250_ADDR, GYRO_ZOUT_H);

        offsets.accel_x_offset += accel_x_raw;
        offsets.accel_y_offset += accel_y_raw;
        offsets.accel_z_offset += accel_z_raw;
        offsets.gyro_x_offset += gyro_x_raw;
        offsets.gyro_y_offset += gyro_y_raw;
        offsets.gyro_z_offset += gyro_z_raw;

        ros::Duration(0.05).sleep(); // Sleep for 50ms between samples
    }

    // Average the samples
/*    offsets.accel_x_offset /= calibration_samples;
    offsets.accel_y_offset /= calibration_samples;
    offsets.accel_z_offset /= calibration_samples;
    // Assuming the accelerometer z-axis should measure 1g (gravity)
    offsets.accel_z_offset -= ACCEL_SCALE_MODIFIER_2G;
    */
    offsets.accel_x_offset = 0;
    offsets.accel_y_offset = 0;
    offsets.accel_z_offset = 0;
    offsets.gyro_x_offset /= calibration_samples;
    offsets.gyro_y_offset /= calibration_samples;
    offsets.gyro_z_offset /= calibration_samples;

    
    

    return offsets;
}
void publish_static_transform()
{
    static tf2_ros::StaticTransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;

    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "base_link";
    transformStamped.child_frame_id = "imu_frame";
    transformStamped.transform.translation.x = 0.0;
    transformStamped.transform.translation.y = 0.0;
    transformStamped.transform.translation.z = 0.0;
    transformStamped.transform.rotation.x = 0.0;
    transformStamped.transform.rotation.y = 0.0;
    transformStamped.transform.rotation.z = 0.0;
    transformStamped.transform.rotation.w = 1.0;

    br.sendTransform(transformStamped);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "imu_node");
    ros::NodeHandle nh;

    ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("imu/data_raw", 2);
    ros::Publisher mag_pub = nh.advertise<sensor_msgs::MagneticField>("imu/mag", 2);
    
    ros::Rate loop_rate(20);

    int file;
    file = open("/dev/i2c-1", O_RDWR);
    if (file < 0)
    {
        ROS_INFO("ERROR - Unable to open I2C bus");
        return 1;
    }

    // Initialize IMU9250
    i2c_write(file, IMU9250_ADDR, PWR_MGMT_1, 0x80); // Reset and Wake up IMU9250
    ros::Duration(0.1).sleep();
    i2c_write(file, IMU9250_ADDR, INT_PIN_CFG, 0x02); // i2c passthrough
    //i2c_write(file, AK8963_ADDR, AK8963_CNTL2, 0x01);
    ros::Duration(0.2).sleep();
    ROS_INFO("IMU9250_ADDR id: %i", i2c_read(file, IMU9250_ADDR, 0x75));
    ROS_INFO("ak8963 id: %i", i2c_read(file, AK8963_ADDR, 0x00));

    // Initialize IMU9520
    i2c_write(file, IMU9250_ADDR, 0x6B, 0x08); // Reset MPU6050
    if (!perform_self_test(file))
    {
        ROS_ERROR("Self-test failed. Exiting...");
        close(file);
        return -1;
    }
    ros::Duration(0.2).sleep();

    // Calibration
    double accel_x_offset, accel_y_offset, accel_z_offset;
    double gyro_x_offset, gyro_y_offset, gyro_z_offset;


    i2c_write(file, IMU9250_ADDR, ACCEL_CONFIG, 0x00); // ±2g
//    i2c_write(file, IMU9250_ADDR, GYRO_CONFIG, 0x00);  // ±250°/s (FS_SEL = 0)
    i2c_write(file, IMU9250_ADDR, GYRO_CONFIG, 0x18);  // ±2000°/s (FS_SEL = 3)

    ros::Duration(0.1).sleep();

    ROS_INFO("Calibrating... Please keep the sensor stationary.");
    IMUOffsets offsets = calibrateIMU(file);
    ROS_INFO("Calibration complete.");

// Initialize AK8963 Magnetometer
/*
i2c_write(file, AK8963_ADDR, AK8963_CNTL1, 0x00); // Power down magnetometer
ros::Duration(0.01).sleep();
i2c_write(file, AK8963_ADDR, AK8963_CNTL1, 0x0F); // Enter Fuse ROM access mode
ros::Duration(0.01).sleep();

// Read sensitivity adjustment values
uint8_t asax = i2c_read(file, AK8963_ADDR, MAG_ASAX);
uint8_t asay = i2c_read(file, AK8963_ADDR, MAG_ASAY);
uint8_t asaz = i2c_read(file, AK8963_ADDR, MAG_ASAZ);

ROS_INFO("Asa %i %i %i", asax, asay, asaz);


// Sensitivity adjustment values conversion to proper scaling
double mag_adj_x = ((asax - 128) / 256.0) + 1.0;
double mag_adj_y = ((asay - 128) / 256.0) + 1.0;
double mag_adj_z = ((asaz - 128) / 256.0) + 1.0;
i2c_write(file, AK8963_ADDR, AK8963_CNTL1, 0x00);
//i2c_write(file, AK8963_ADDR, AK8963_CNTL2, 0x01);
ros::Duration(0.01).sleep();
*/
double mag_adj_x = 1;
double mag_adj_y = 1;
double mag_adj_z = 1;
i2c_write(file, AK8963_ADDR, AK8963_CNTL1, 0x11);

    //publish_static_transform();

    while (ros::ok())
    {
        
        // Read IMU9250 data
        int16_t accel_x_raw = read_word_2c(file, IMU9250_ADDR, ACCEL_XOUT_H);
        int16_t accel_y_raw = read_word_2c(file, IMU9250_ADDR, ACCEL_YOUT_H);
        int16_t accel_z_raw = read_word_2c(file, IMU9250_ADDR, ACCEL_ZOUT_H);
        int16_t gyro_x_raw = read_word_2c(file, IMU9250_ADDR, GYRO_XOUT_H);
        int16_t gyro_y_raw = read_word_2c(file, IMU9250_ADDR, GYRO_YOUT_H);
        int16_t gyro_z_raw = read_word_2c(file, IMU9250_ADDR, GYRO_ZOUT_H);

        // Apply offsets
        accel_x_raw -= offsets.accel_x_offset;
        accel_y_raw -= offsets.accel_y_offset;
        accel_z_raw -= offsets.accel_z_offset;
        gyro_x_raw -= offsets.gyro_x_offset;
        gyro_y_raw -= offsets.gyro_y_offset;
        gyro_z_raw -= offsets.gyro_z_offset;

        // Convert raw accel and gyro values to acceleration in m/s², angular velocity in rad/s
        double accel_x = (accel_x_raw / ACCEL_SCALE_MODIFIER_2G) * GRAVITY;
        double accel_y = (accel_y_raw / ACCEL_SCALE_MODIFIER_2G) * GRAVITY;
        double accel_z = (accel_z_raw / ACCEL_SCALE_MODIFIER_2G) * GRAVITY;
        double gyro_x = (gyro_x_raw / GYRO_SCALE_MODIFIER_2000DEG) * (M_PI / 180.0);
        double gyro_y = (gyro_y_raw / GYRO_SCALE_MODIFIER_2000DEG) * (M_PI / 180.0);
        double gyro_z = (gyro_z_raw / GYRO_SCALE_MODIFIER_2000DEG) * (M_PI / 180.0);

    i2c_write(file, AK8963_ADDR, AK8963_CNTL1, 0x11);        
    uint8_t ST1 = 0;
    do //wait until data has arrived
    {
     ST1 = i2c_read(file, AK8963_ADDR, AK8963_ST1);
    } while (!(ST1 & 0x01));


    // Read AK8963 magnetometer data
    int16_t mag_x_raw = read_word_2c(file, AK8963_ADDR, MAG_XOUT_L);
    int16_t mag_y_raw = read_word_2c(file, AK8963_ADDR, MAG_YOUT_L);
    int16_t mag_z_raw = read_word_2c(file, AK8963_ADDR, MAG_ZOUT_L);

    // Apply sensitivity adjustments and scaling. Magnetic field in µT
    double mag_x = mag_x_raw * mag_adj_x * MAG_SCALE_MODIFIER;
    double mag_y = mag_y_raw * mag_adj_y * MAG_SCALE_MODIFIER;
    double mag_z = mag_z_raw * mag_adj_z * MAG_SCALE_MODIFIER;

        // Prepare IMU message
        sensor_msgs::Imu imu_msg;
        imu_msg.header.stamp = ros::Time::now();
        imu_msg.header.frame_id = "imu_link";
//        imu_msg.header.frame_id = "base_link";
/*
            imu_msg.orientation.x = 1.0;
            imu_msg.orientation.y = 0.0;
            imu_msg.orientation.z = 0.0;
            imu_msg.orientation.w = 0.0;
            imu_msg.orientation_covariance[0] = 0.01;
            imu_msg.orientation_covariance[4] = 0.01;
            imu_msg.orientation_covariance[8] = 0.01;
            imu_msg.angular_velocity_covariance[0] = 0.03;
            imu_msg.angular_velocity_covariance[4] = 0.03;
            imu_msg.angular_velocity_covariance[8] = 0.03;
            imu_msg.linear_acceleration_covariance[0] = 10;
            imu_msg.linear_acceleration_covariance[4] = 10;
            imu_msg.linear_acceleration_covariance[8] = 10;
*/
        imu_msg.linear_acceleration.x = -accel_y;
        imu_msg.linear_acceleration.y = accel_x;
        imu_msg.linear_acceleration.z = accel_z;

        imu_msg.angular_velocity.x = -gyro_y;
        imu_msg.angular_velocity.y = gyro_x;
        imu_msg.angular_velocity.z = gyro_z;

        imu_pub.publish(imu_msg);

        // Prepare MagneticField message
        sensor_msgs::MagneticField mag_msg;
        mag_msg.header.stamp = imu_msg.header.stamp;
        mag_msg.magnetic_field.x = mag_x * 1e-3; // Convert to mTesla
        mag_msg.magnetic_field.y = mag_y * 1e-3; // Convert to mTesla
        mag_msg.magnetic_field.z = mag_z * 1e-3; // Convert to mTesla
        //ROS_INFO("MAG: %f %f %f", mag_msg.magnetic_field.x, mag_msg.magnetic_field.y,  mag_msg.magnetic_field.z);

/*            mag_msg.magnetic_field_covariance[0] = 0.01;
            mag_msg.magnetic_field_covariance[4] = 0.01;
            mag_msg.magnetic_field_covariance[8] = 0.01;*/
        mag_pub.publish(mag_msg);

        ros::spinOnce();
        loop_rate.sleep();
    }

    close(file);
    return 0;
}
