// Created by Clemens Elflein on 3/28/22.
// Rewritten by Daniel Wiegert on 21 May 2023.
// Copyright (c) 2022 Clemens Elflein, Daniel Wiegert. All rights reserved.
//
// This work is licensed under a Creative Commons Attribution-NonCommercial-ShareAlike 4.0 International License.
//
// Feel free to use the design in your private/educational projects, but don't try to sell the design or products based on it without getting our consent first.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//
//

#include <queue>
#include <atomic>

#include "ros/ros.h"

#include <boost/thread.hpp>
#include <boost/bind.hpp>
#include "boost/crc.hpp"

#include "spidev_lib++.h"

#include "rapidjson/document.h"
#include "rapidjson/writer.h"
#include "rapidjson/stringbuffer.h"

#include "std_msgs/Empty.h"
#include "std_msgs/Bool.h"

#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>

#include <mower_msgs/Status.h>
#include "mower_msgs/MowerControlSrv.h"
#include "mower_msgs/EmergencyStopSrv.h"
#include "mower_msgs/ImuRaw.h"
#include "mower_msgs/HighLevelControlSrv.h"
#include "mower_msgs/HighLevelStatus.h"

#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"

#include <xbot_msgs/WheelTick.h>

using namespace rapidjson;

#define NOP 0x00
#define SOF 0x01
#define EOF 0xFF
#define buflen  250

spi_config_t spi_config;

std::atomic<bool> keep_running(true);

std::queue<std::string> spi_rx_queue;
std::queue<std::string> spi_tx_queue;
boost::mutex spi_rx_queue_mutex;
boost::mutex spi_tx_queue_mutex;

ros::ServiceClient highLevelClient;

ros::Publisher status_pub;
ros::Publisher wheel_tick_pub;

ros::Publisher sensor_imu_pub;
ros::Publisher sensor_mag_pub;

ros::Time last_cmd_vel(0.0);

sensor_msgs::MagneticField sensor_mag_msg;
sensor_msgs::Imu sensor_imu_msg;
mower_msgs::HighLevelStatus last_high_level_status;

bool emergency_ROS = false;
bool emergency_WORX = false;
bool clear_emergency_WORX = false;

int speed_l = 0, speed_r = 0, speed_mow = 0;
int last_speed_l = 0, last_speed_r = 0, last_speed_mow = 0;
double wheel_ticks_per_m = 0.0;
double wheel_distance_m = 0.0;

int ros_msg_state = -1;
std::string ros_msg_substate = "";
std::string motorState = "";

float v_battery = 0;
float charge_current = 0;
float battery_temp = 0;

#define MAXSPEED 1320 // 2047 = 100% PWM, 1320 = 65% PWM

void velReceived(const geometry_msgs::Twist::ConstPtr &msg) {
    last_cmd_vel = ros::Time::now();
    speed_r = (msg->linear.x + 0.5*wheel_distance_m*msg->angular.z) * MAXSPEED;
    speed_l = (msg->linear.x - 0.5*wheel_distance_m*msg->angular.z) * MAXSPEED;
    

    if (speed_l >= MAXSPEED) {
        speed_l = MAXSPEED;
    } else if (speed_l <= -MAXSPEED) {
        speed_l = -MAXSPEED;
    }
    if (speed_r >= MAXSPEED) {
        speed_r = MAXSPEED;
    } else if (speed_r <= -MAXSPEED) {
        speed_r = -MAXSPEED;
    }
}

void ping_timer(const ros::TimerEvent& event) {
  ROS_INFO("::: Timertest!");
}

bool is_emergency() {
    return emergency_ROS || emergency_WORX;
}

void publishActuators() {
// emergency or timeout -> send 0 speeds
    if (is_emergency()) {
        speed_l = 0;
        speed_r = 0;
        speed_mow = 0;
    }
    if (ros::Time::now() - last_cmd_vel > ros::Duration(5.0)) {
        speed_l = 0;
        speed_r = 0;

    }
    if (ros::Time::now() - last_cmd_vel > ros::Duration(25.0)) {
        speed_l = 0;
        speed_r = 0;
        speed_mow = 0;
    }

    if (speed_l != last_speed_l || speed_r != last_speed_r || speed_mow != last_speed_mow) {
        last_speed_l = speed_l;
        last_speed_r = speed_r;
        last_speed_mow = speed_mow;

        rapidjson::Document document;
        document.SetObject();

        // Create a JSON object for "setpwm"
        rapidjson::Value setpwmObject(rapidjson::kObjectType);

        // Add "left", "right", and "mow" key-value pairs to the "setpwm" object
        setpwmObject.AddMember("left", speed_l, document.GetAllocator());
        setpwmObject.AddMember("right", speed_r, document.GetAllocator());
        setpwmObject.AddMember("mow", speed_mow, document.GetAllocator());

        // Add the "setpwm" object to the main document
        document.AddMember("MOTORREQ_SETSPEED", setpwmObject, document.GetAllocator());

        // Convert the document to a JSON string
        rapidjson::StringBuffer buffer;
        rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
        document.Accept(writer);

        boost::unique_lock<boost::mutex> lock(spi_tx_queue_mutex);
        spi_tx_queue.push(buffer.GetString());
        document.GetAllocator().Clear();
        document.SetNull();

    }
    
}

bool setMowEnabled(mower_msgs::MowerControlSrvRequest &req, mower_msgs::MowerControlSrvResponse &res) {
    if (req.mow_enabled && !is_emergency()) {
        speed_mow = 2047;
    } else {
        speed_mow = 0;
    }
    ROS_INFO_STREAM("Setting mow enabled to " << speed_mow);
    return true;
}

bool setEmergencyStop(mower_msgs::EmergencyStopSrvRequest &req, mower_msgs::EmergencyStopSrvResponse &res) {
    
    if (req.emergency) {
        ROS_ERROR_STREAM("Setting emergency!!");
        clear_emergency_WORX = false;
    } else {
        clear_emergency_WORX = true;
    }
    // Set the high level emergency instantly. Low level value will be set on next update.
    emergency_ROS = req.emergency;
    publishActuators();
    return true;
}

void rosStatusRecieved(const mower_msgs::HighLevelStatus::ConstPtr &msg) {
    rapidjson::Document document;
    rapidjson::Document::AllocatorType* allocator;
    rapidjson::StringBuffer buffer;
    rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
    rapidjson::Value setObject; 
    document.SetObject();
    allocator = &document.GetAllocator();

    /*struct ll_high_level_state hl_state = {
            .type = PACKET_ID_LL_HIGH_LEVEL_STATE,
            .current_mode = msg->state,
            .gps_quality = static_cast<uint8_t>(msg->gps_quality_percent*100.0)
    };
uint8 HIGH_LEVEL_STATE_NULL=0
uint8 HIGH_LEVEL_STATE_IDLE=1
uint8 HIGH_LEVEL_STATE_AUTONOMOUS=2
uint8 HIGH_LEVEL_STATE_RECORDING=3

uint8 SUBSTATE_1=0
uint8 SUBSTATE_2=1
uint8 SUBSTATE_3=2
uint8 SUBSTATE_4=3
uint8 SUBSTATE_SHIFT=6
    */
    if (ros_msg_substate != msg->sub_state_name ) {
        ros_msg_substate = msg->sub_state_name;
        ROS_INFO("### rosSubStatusRecieved: %s", msg->sub_state_name.c_str());
    }

    bool motorStateCheck = (motorState == "MOTORREQ_IDLE" || motorState == "MOTORREQ_DISABLE") && msg->state > 1;

    if (ros_msg_state != msg->state || motorStateCheck) {
        ros_msg_state = msg->state;
        if (msg->state > 1 || motorStateCheck) {
            document.SetObject();
            setObject.SetObject();
            document.AddMember("MOTORREQ_ENABLE", setObject, *allocator);
            buffer.Clear();
            writer.Reset(buffer);
            document.Accept(writer);

            boost::unique_lock<boost::mutex> lock(spi_tx_queue_mutex);
            spi_tx_queue.push(buffer.GetString());
            ROS_INFO("### rosStatusRecieved: %i", msg->state);
        } else {
            document.SetObject();
            setObject.SetObject();
            document.AddMember("MOTORREQ_DISABLE", setObject, *allocator);
            buffer.Clear();
            writer.Reset(buffer);
            document.Accept(writer);

            boost::unique_lock<boost::mutex> lock(spi_tx_queue_mutex);
            spi_tx_queue.push(buffer.GetString());
            ROS_INFO("### rosStatusRecieved: %i", msg->state);
        }
    }
    document.GetAllocator().Clear();
    document.SetNull();
}

void processI2C_IMU(const rapidjson::Value& i2c_imu) {
//{"I2C_IMU":{"Yaw":-117,"Pitch":-316,"Roll":4,"AccX":63,"AccY":52,"AccZ":1034}}
    sensor_imu_msg.header.stamp = ros::Time::now();
    sensor_imu_msg.header.seq++;
    sensor_imu_msg.header.frame_id = "base_link";
/*
    sensor_mag_msg.header.stamp = ros::Time::now();
    sensor_mag_msg.header.seq++;
    sensor_mag_msg.header.frame_id = "base_link";
    sensor_mag_msg.magnetic_field.x = 0.0001f;
    sensor_mag_msg.magnetic_field.y = 0.0001f;
    sensor_mag_msg.magnetic_field.z = 0.0001f;
  */  
    if (i2c_imu.HasMember("Yaw"))
        sensor_imu_msg.angular_velocity.x = i2c_imu["Yaw"].GetInt() / 100000.0f;
    if (i2c_imu.HasMember("Pitch"))
        sensor_imu_msg.angular_velocity.y = i2c_imu["Pitch"].GetInt() / 100000.0f;
    if (i2c_imu.HasMember("Roll"))
        sensor_imu_msg.angular_velocity.z = i2c_imu["Roll"].GetInt() / 100000.0f;
    if (i2c_imu.HasMember("AccX"))
        sensor_imu_msg.linear_acceleration.x = i2c_imu["AccX"].GetInt() / 1000.0f;
    if (i2c_imu.HasMember("AccY"))
        sensor_imu_msg.linear_acceleration.y = i2c_imu["AccY"].GetInt() / 1000.0f;
    if (i2c_imu.HasMember("AccZ"))
        sensor_imu_msg.linear_acceleration.z = i2c_imu["AccZ"].GetInt() / 1000.0f;

 
    sensor_imu_pub.publish(sensor_imu_msg);
    //sensor_mag_pub.publish(sensor_mag_msg);
}

void processMotorTicks(const rapidjson::Value& MotorPulse) {
//{"MotorPulse":{"Left":890,"Right":884,"Mow":2,"DirLeft":0,"DirRight":0}}
    xbot_msgs::WheelTick wheel_tick_msg;
    wheel_tick_msg.wheel_tick_factor = static_cast<unsigned int>(wheel_ticks_per_m);
    wheel_tick_msg.stamp = ros::Time::now();

    if (MotorPulse.HasMember("Left"))
        wheel_tick_msg.wheel_ticks_rl = MotorPulse["Left"].GetInt();
    if (MotorPulse.HasMember("DirLeft"))
        wheel_tick_msg.wheel_direction_rl = MotorPulse["DirLeft"].GetInt();
    if (MotorPulse.HasMember("Right"))
        wheel_tick_msg.wheel_ticks_rr = MotorPulse["Right"].GetInt();
    if (MotorPulse.HasMember("DirRight"))
        wheel_tick_msg.wheel_direction_rr = MotorPulse["DirRight"].GetInt();

    wheel_tick_pub.publish(wheel_tick_msg);
}

void publishStatus() {
    mower_msgs::Status status_msg;
    status_msg.stamp = ros::Time::now();

    status_msg.mower_status = mower_msgs::Status::MOWER_STATUS_INITIALIZING;
    status_msg.mower_status = mower_msgs::Status::MOWER_STATUS_OK;
    
    status_msg.raspberry_pi_power = 1;
    status_msg.gps_power = 1;
    status_msg.esc_power = 1;
    status_msg.rain_detected = 0;
    status_msg.sound_module_available = 0;
    status_msg.sound_module_busy = 0;
    status_msg.ui_board_available = 0;

    for (uint8_t i = 0; i < 5; i++) {
        status_msg.ultrasonic_ranges[i] = 0;
    }

    // TODO Reset Worx emergancy status 
    // Send message, wait for not emergancy and be happy :)
    // overwrite emergency with the LL value.
    /*emergency_low_level = last_ll_status.emergency_bitmask > 0;
    if (!emergency_WORX) {
        // it obviously worked, reset the request
        clear_emergency_WORX = false;
    } else {
        ROS_ERROR_STREAM_THROTTLE(1, "Low Level Emergency. Bitmask was: " << (int)last_ll_status.emergency_bitmask);
    }*/

    // True, if high or low level emergency condition is present
    status_msg.emergency = is_emergency();

    status_msg.v_battery = v_battery;
    if (charge_current > 0) {
        status_msg.v_charge = 30;
        status_msg.charge_current = charge_current;
    } else {
        status_msg.v_charge = 0;
        status_msg.charge_current = 0;
    }
    
    // TODO Check motor current for stuck wheel, check motorcontroller error pin etc

/*
ESC_STATUS_DISCONNECTED
ESC_STATUS_ERROR
ESC_STATUS_STALLED
ESC_STATUS_OK
ESC_STATUS_RUNNING
*/
    status_msg.mow_esc_status.status = mower_msgs::ESCStatus::ESC_STATUS_OK;
    status_msg.mow_esc_status.tacho = 0;
    status_msg.mow_esc_status.current = 0;
    status_msg.mow_esc_status.temperature_motor = 0;
    status_msg.mow_esc_status.temperature_pcb = battery_temp;

    status_msg.left_esc_status.status = mower_msgs::ESCStatus::ESC_STATUS_OK;
    status_msg.left_esc_status.tacho = 0;
    status_msg.left_esc_status.current = 0;
    status_msg.left_esc_status.temperature_motor = 0;
    status_msg.left_esc_status.temperature_pcb = battery_temp;

    status_msg.right_esc_status.status = mower_msgs::ESCStatus::ESC_STATUS_OK;
    status_msg.right_esc_status.tacho = 0;
    status_msg.right_esc_status.current = 0;
    status_msg.right_esc_status.temperature_motor = 0;
    status_msg.right_esc_status.temperature_pcb = battery_temp;

    status_pub.publish(status_msg);


}

void publishActuatorsTimerTask(const ros::TimerEvent &timer_event) {
    publishActuators();
    publishStatus();
}

void spi_thread(std::atomic<bool>& keep_running, ros::NodeHandle& n, const std::string& ll_spi_dev_name) {
    char rxBuffer[buflen];
    char txBuffer[buflen];
    char rxmsgBuffer[buflen];
    bool receiving = false;
    uint8_t rxByte = 0;
    size_t rxPos = 0;
    
    // Initialize ROS time
    ros::Time::init(); 
    
    // Configure the SPI settings
    spi_config.mode=0;
    spi_config.speed=1500000;
    spi_config.delay=0;
    spi_config.bits_per_word=8;

    // Initialize the SPI device
    ROS_INFO_STREAM("Opening spi device: " << ll_spi_dev_name);
    SPI *SPIdev = NULL;
    SPIdev = new SPI(ll_spi_dev_name.c_str(), &spi_config);
    if (SPIdev->begin()) {
        ROS_INFO_STREAM("Connecting SPI interface: " << ll_spi_dev_name);
    } else {
        keep_running.store(false);
        ROS_ERROR_STREAM("Error opening spi device - check your udev permissions.");
    }

    //ros::Timer timer = n.createTimer(ros::Duration(1.0), ping_timer);

    ros::Duration loopDelay(0, 1000000); // 1ms

    while (ros::ok() && keep_running.load()) {
        // Clear the transmit and receive buffers
        memset(txBuffer, 0x00, buflen);
        memset(rxBuffer, 0x00, buflen);
        
        // Lock the transmit queue mutex to ensure thread-safe access, take the first one and prepare it for transmission
        {
            boost::unique_lock<boost::mutex> lock(spi_tx_queue_mutex);
            if (!spi_tx_queue.empty()) {
                std::string txMsg;
                txMsg = spi_tx_queue.front();
                spi_tx_queue.pop();
                strncpy(txBuffer + 1, txMsg.c_str(), buflen);
                txBuffer[0] = SOF;
                txBuffer[strlen(txBuffer)] = EOF;
                //ROS_INFO("<<< %s", txBuffer + 1);
                ROS_INFO_STREAM_THROTTLE(1, "<<< " << txMsg.c_str());
            }
        }
        
        // Transfer the data between the SPI device and the buffers
        SPIdev->xfer(reinterpret_cast<uint8_t*>(txBuffer), buflen, reinterpret_cast<uint8_t*>(rxBuffer), buflen);

        // Process the received data
        for (size_t i = 0; i <= buflen; ++i) {
            rxByte = rxBuffer[i];
            if (receiving) {
                // If the received byte is an EOF byte, terminate the received string and store it in the receive queue
                if (rxByte == EOF) {
                    rxmsgBuffer[rxPos] = '\0'; // Null-terminate the received string
                    
                    boost::unique_lock<boost::mutex> lock(spi_rx_queue_mutex);
                    if (rxPos > 0) {
                        spi_rx_queue.push(rxmsgBuffer);
                        //ROS_INFO("<<< %s", rxmsgBuffer); // fix rate limit debug output
                    }
                    receiving=false;
                    continue;
                } else if (rxByte != NOP) {
                    rxmsgBuffer[rxPos++] = rxByte;
                }
            // If the received byte is a SOF byte, start receiving and clear the received message buffer
            } else if (rxByte == SOF) {
                receiving = true;
                rxPos = 0;
                memset(rxmsgBuffer, 0x00, buflen);
            }
        }
        
        loopDelay.sleep();
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "worx_comms");

    rapidjson::Document document;
    rapidjson::Document::AllocatorType* allocator;
    rapidjson::StringBuffer buffer;
    rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
    rapidjson::Value setObject; 
    document.SetObject();
    allocator = &document.GetAllocator();

    ros::NodeHandle n;
    ros::NodeHandle paramNh("~");
    
    highLevelClient = n.serviceClient<mower_msgs::HighLevelControlSrv>(
            "mower_service/high_level_control");

    // spi dev to use
    std::string ll_spi_dev_name;
    if (!paramNh.getParam("ll_spi_dev_name", ll_spi_dev_name)) {
        ROS_ERROR_STREAM("Error getting ll_spi_dev_name. Quitting.");
        return 1;
    }

    if (!paramNh.getParam("wheel_ticks_per_m",wheel_ticks_per_m)) {
        ROS_ERROR_STREAM("wheel_ticks_per_m not set. Quitting.");
        return 1;
    }

    if (!paramNh.getParam("wheel_distance_m",wheel_distance_m)) {
        ROS_ERROR_STREAM("wheel_distance_m not set. Quitting.");
        return 1;
    }
 
    // Fire up the communication thread!
    boost::thread spiThread(boost::bind(&spi_thread, std::ref(keep_running), std::ref(n), ll_spi_dev_name));

    status_pub = n.advertise<mower_msgs::Status>("mower/status", 1);
    wheel_tick_pub = n.advertise<xbot_msgs::WheelTick>("mower/wheel_ticks", 1);
    sensor_imu_pub = n.advertise<sensor_msgs::Imu>("imu/data_raw", 1);
    sensor_mag_pub = n.advertise<sensor_msgs::MagneticField>("imu/mag", 1);
    ros::ServiceServer mow_service = n.advertiseService("mower_service/mow_enabled", setMowEnabled);
    ros::ServiceServer emergency_service = n.advertiseService("mower_service/emergency", setEmergencyStop);
    ros::Subscriber cmd_vel_sub = n.subscribe("cmd_vel", 0, velReceived, ros::TransportHints().tcpNoDelay(true));
    ros::Subscriber ros_status_sub = n.subscribe("/mower_logic/current_state", 0, rosStatusRecieved);
    ros::Timer publish_timer = n.createTimer(ros::Duration(0.02), publishActuatorsTimerTask);

    sensor_mag_msg.header.seq = 0;
    sensor_imu_msg.header.seq = 0;

    ros::Duration retryDelay(5, 0);
    ros::Duration loopDelay(0, 5000000); // 5ms
    ros::AsyncSpinner spinner(1);

    int pingCounter = 0;
    std::string rxMsg;

    ros::Time last_ping_time = ros::Time::now();
    ros::Time current_time = ros::Time::now();

    spinner.start();
    ros::Duration(0.5).sleep(); // sleep for half a second
    
    while (ros::ok() && keep_running.load()) {
        current_time = ros::Time::now();

        // Ping to tell worx we are still alive, will reset watchdogSPI in worx fw if recieved.
        // this could be moved to a timer (created by spithread if all is well) instead. (half implemented - ping_timer() )
        if ((current_time - last_ping_time).toSec() > 2.0) {
            pingCounter++;
            last_ping_time = current_time;

            document.SetObject();
            rapidjson::Value pingObject(rapidjson::kObjectType);
            pingObject.AddMember("count", pingCounter, *allocator);
            document.AddMember("ping", pingObject, *allocator);
            buffer.Clear();
            writer.Reset(buffer);
            document.Accept(writer);

            boost::unique_lock<boost::mutex> lock(spi_tx_queue_mutex);
            spi_tx_queue.push(buffer.GetString());
            document.GetAllocator().Clear();
            document.SetNull();
        }

        // Check and fetch message from queue
        {
            boost::unique_lock<boost::mutex> lock(spi_rx_queue_mutex);
            if (!spi_rx_queue.empty()) {
                rxMsg = spi_rx_queue.front();
                spi_rx_queue.pop();
                //ROS_INFO(">>> %s", rxMsg.c_str()); // we need to rate limit this or something, logs gets large... fast.. 
                // ROS_INFO_STREAM_THROTTLE exists apparently lol

            }
        }

        if(rxMsg.find("DEBUG") == 0) {
            ROS_WARN("!!! %s", rxMsg.c_str());
        }

        if(rxMsg.find("State") == 0) {
            ROS_INFO("!!! %s", rxMsg.c_str());
        }

        ParseResult result = document.Parse(rxMsg.c_str());
        if (result) {
            if (document.HasMember("Analog")) {
                //{"Analog":{"Rain":3850,"boardTemp":3089}} // Boardtemp raw value unknown conversion
            }
            if (document.HasMember("Battery")) {
                //{"Battery":{"mV":28014,"mA":0,"Temp":152,"CellLow":1,"CellHigh":1,"InCharger":1}}
                ROS_INFO_STREAM_THROTTLE(10, ">>> " << rxMsg.c_str());
                
                const rapidjson::Value& json = document["Battery"];
                if (json.HasMember("mV"))
                    v_battery = (float)json["mV"].GetInt() / 1000.0;
                if (json.HasMember("mA"))
                    charge_current = (float)json["mA"].GetInt() / 1000.0;
                if (json.HasMember("Temp"))
                    battery_temp = (float)json["Temp"].GetInt() / 10.0; // Really BATTERY TEMPERATURE!
            }
            if (document.HasMember("Boundary")) {
                //{"Boundary":{"sleft":524272,"sright":338912,"nleft":102,"nright":43}}
            }
            if (document.HasMember("Digital")) {
                //{"Digital":{"Stuck":1,"Stuck2":1,"Door":1,"Door2":1,"Lift":1,"Collision":0,"Stop":0,"Rain":0}}
                ROS_INFO_STREAM_THROTTLE(2, ">>> " << rxMsg.c_str());
            }
            if (document.HasMember("I2C_IMU")) {
                //{"I2C_IMU":{"Yaw":-117,"Pitch":-316,"Roll":4,"AccX":63,"AccY":52,"AccZ":1034}}
                processI2C_IMU(document["I2C_IMU"]);
            }
            if (document.HasMember("MotorCurrent")) {
                //{"MotorCurrent":{"Left":102,"Right":15,"MowRPM":0}}
                ROS_INFO_STREAM_THROTTLE(2, ">>> " << rxMsg.c_str());
            }
            if (document.HasMember("MotorPulse")) {
                //{"MotorPulse":{"Left":890,"Right":884,"Mow":2,"DirLeft":0,"DirRight":0}}
                ROS_INFO_STREAM_THROTTLE(2, ">>> " << rxMsg.c_str());
                processMotorTicks(document["MotorPulse"]);
            }
            if (document.HasMember("MotorPWM")) {
                //{"MotorPWM":{"Left":0,"Right":0,"Mow":0}}
                ROS_INFO_STREAM_THROTTLE(5, ">>> " << rxMsg.c_str());
            }
            if (document.HasMember("powerState")) {
                ROS_INFO_STREAM("!!! " << rxMsg.c_str());
            }
            if (document.HasMember("motorState")) {
                motorState = document["motorState"].GetString();
                ROS_INFO_STREAM("!!! " << rxMsg.c_str());
            }
        }
        document.GetAllocator().Clear();
        document.SetNull();

        rxMsg.clear();
        loopDelay.sleep();
    }
    
    // Shutdown procedure
    {
        document.SetObject();
        setObject.SetObject();
        document.AddMember("MOTORREQ_DISABLE", setObject, *allocator);

        buffer.Clear();
        writer.Reset(buffer);
        document.Accept(writer);

        boost::unique_lock<boost::mutex> lock(spi_tx_queue_mutex);
        spi_tx_queue.push(buffer.GetString());
    }
    ros::Duration(1).sleep();

    spinner.stop();
    spiThread.join();

    return 0;
}
