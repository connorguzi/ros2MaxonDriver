//============================================================================

// Name        : maxon_driver.cpp
// Author      : Connor Guzikowksi
// Version     :
// Description : ROS2 Maxon EPOS2 Driver for roboendo project - should work for EPOS4 with farily minimal changes
// Usage       : $ ros2 run maxon_driver maxon_driver
// NOTES       : run $ source install/setup.bash first to allow for the custom messages
//               - Currently only uses position control but should be easily modifiable to use velocity control (change VCS_position commands to VCS_velocity)
//============================================================================

#include <iostream>
#include "maxon_driver/Definitions.h"
#include <string.h>
#include <sstream>
#include <unistd.h>
#include <getopt.h>
#include <stdlib.h>
#include <stdio.h>
#include <cstdio>
#include <list>
#include <math.h>
#include <sys/types.h>
#include <unistd.h>
#include <stdio.h>
#include <sys/times.h>
#include <sys/time.h>
#include <rclcpp/rclcpp.hpp>
#include "maxon_epos_msgs/msg/motor_state.hpp"
#include "maxon_epos_msgs/msg/motor_states.hpp"

typedef void* HANDLE;
typedef int BOOL;

enum EAppMode
{
	AM_UNKNOWN,
	AM_DEMO,
	AM_INTERFACE_LIST,
	AM_PROTOCOL_LIST,
	AM_VERSION_INFO
};

#define POSITION 0
#define VELOCITY 1

#ifndef MMC_SUCCESS
	#define MMC_SUCCESS 0
#endif

#ifndef MMC_FAILED
	#define MMC_FAILED 1
#endif

#ifndef MMC_MAX_LOG_MSG_SIZE
	#define MMC_MAX_LOG_MSG_SIZE 512
#endif


using namespace std;
using MotorStates = maxon_epos_msgs::msg::MotorStates;

// using maxon_msgs::msg::MotorStates;


class MaxonDriverNode : public rclcpp::Node
{
public:
    MaxonDriverNode(int argc, char** argv) : Node("maxon_driver_node")
    {
        // Initialization of the drivers:;
            // First open the gateway motor via USB, and then subsequent sub motors via CAN
            // Then activate the position mode of the motors to enable position control
            // Finally enable the motors to run
        SetDefaultParameters();
        ParseArguments(argc, argv);
        OpenDevices(&ulErrorCode);
        if(controlMethod == POSITION){
            activatePositionMode(gateway_handle, 1, ulErrorCode);
            activatePositionMode(sub_handle, 2, ulErrorCode);
            activatePositionMode(sub_handle, 3, ulErrorCode);
            activatePositionMode(sub_handle, 4, ulErrorCode);
        }
        else{
            activateVelocityMode(gateway_handle, 1, ulErrorCode);
            activateVelocityMode(sub_handle, 2, ulErrorCode);
            activateVelocityMode(sub_handle, 3, ulErrorCode);
            activateVelocityMode(sub_handle, 4, ulErrorCode);            
        }

        VCS_SetEnableState(gateway_handle, 1, &ulErrorCode);
        VCS_SetEnableState(sub_handle, 2, &ulErrorCode);
        VCS_SetEnableState(sub_handle, 3, &ulErrorCode);
        VCS_SetEnableState(sub_handle, 4, &ulErrorCode);

        // Create a subscriber to the topic "/maxon_bringup/set_all_states"
        motor_states_subscriber_ = this->create_subscription<MotorStates>(
            "/maxon_bringup/set_all_states", 10,
            std::bind(&MaxonDriverNode::motorStatesCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "MaxonDriverNode initialized and ready to control motors.");
    }

private:
    // The handles are used to determine whether to sent the motor commands to the gateway motor or sub motors
    // They are properly initialized in the OpenDevices() function
    void* gateway_handle = 0;
    void* sub_handle = 0;

    // Motor IDs are determined by the binary switches on the motor driver boards
    unsigned short m1_nodeID = 1;
    unsigned short m2_nodeID = 2;
    unsigned short m3_nodeID = 3;
    unsigned short m4_nodeID = 4;

    // Variables used later for opening the devices and error handling
    string g_deviceName;
    string g_protocolStackName;
    string g_interfaceName;
    string g_portName;
    int controlMethod;
    int g_baudrate = 0;
	unsigned int ulErrorCode = 0;
    EAppMode g_eAppMode = AM_DEMO;
        
    // Subscriber to the MotorStates topic
    rclcpp::Subscription<MotorStates>::SharedPtr motor_states_subscriber_;

    // Callback function for processing incoming motor states
    void motorStatesCallback(const MotorStates::SharedPtr msg)
    {
        // Loop through the motor states in the received message
        for (const auto &state : msg->states)
        {
            unsigned short motor = std::stoul(state.motor_name);
            long position = state.position;
            long velocity = state.velocity;

            RCLCPP_INFO(this->get_logger(), "Received command for motor ID: %d, Position: %ld, Velocity: %ld",
                        motor, position, velocity);

            switch (motor)
            {
            // If the first motor is being controlled then use the gateway handle, else use the subdevice handle
            case 1:
                controlMotor(gateway_handle, motor, ulErrorCode, position, velocity);
                
                break;
            
            case 2:
            case 3:
            case 4:
                controlMotor(sub_handle, motor, ulErrorCode, position, velocity);

                break;
            }
        }
    }

    // Function to send commands to a specific motor
    void controlMotor(HANDLE p_DeviceHandle, unsigned short motor_id, unsigned int & p_rlErrorCode, long position, long velocity)
    {
        RCLCPP_INFO(this->get_logger(), "Setting motor %d to Position: %ld",
                    motor_id, position);

        if(controlMethod == POSITION){
            if(VCS_MoveToPosition(p_DeviceHandle, motor_id, position, 0, 1, &p_rlErrorCode) == 0)
                    {
                        RCLCPP_FATAL(this->get_logger(), "VCS_MoveToPosition Failed");
                    }
        }

        else{
            if(VCS_MoveWithVelocity(p_DeviceHandle, motor_id, velocity, &p_rlErrorCode) == 0)
                    {
                        RCLCPP_FATAL(this->get_logger(), "VCS_MoveWithVelocity Failed");
                    }   
        }

    }

    // Enables for position control for the motors
    void activatePositionMode(HANDLE p_DeviceHandle, unsigned short motor_id, unsigned int & p_rlErrorCode){
        VCS_ActivateProfilePositionMode(p_DeviceHandle, motor_id, &p_rlErrorCode);
    }
    // Enable velocity control 
    void activateVelocityMode(HANDLE p_DeviceHandle, unsigned short motor_id, unsigned int & p_rlErrorCode){
        VCS_ActivateProfileVelocityMode(p_DeviceHandle, motor_id, &p_rlErrorCode);
    }

    void SetDefaultParameters()
    {

        // Communication to gateway motor is via USB with the baudrate of 1000000
        g_deviceName = "EPOS2"; 
        g_protocolStackName = "MAXON SERIAL V2"; 
        g_interfaceName = "USB"; 
        g_portName = "USB0"; 
        g_baudrate = 1000000; 
        controlMethod = POSITION;
    }

    int OpenDevices(unsigned int* p_pErrorCode)
    {
        int lResult = 1;

        char* pDeviceName = new char[255];
        char* pProtocolStackName = new char[255];
        char* pInterfaceName = new char[255];
        char* pPortName = new char[255];
        string device = "EPOS2";
        string protocol = "CANopen";

        char* sub_device = device.data();
        char* sub_protocol = protocol.data();

        // Needed to get the correct types for the VCS functions
        strcpy(pDeviceName, g_deviceName.c_str());
        strcpy(pProtocolStackName, g_protocolStackName.c_str());
        strcpy(pInterfaceName, g_interfaceName.c_str());
        strcpy(pPortName, g_portName.c_str());

        RCLCPP_INFO(this->get_logger(), "Opening Gateway Motor");

        // Initialize the gateway handle, this is important and will be used when sending any command to the gateway motor
        gateway_handle = VCS_OpenDevice(pDeviceName, pProtocolStackName, pInterfaceName, pPortName, p_pErrorCode);

        // Just setting the proper settings for the motor and making sure there are no errors
            // Most of this is taken from Maxon's code
        if(gateway_handle!=0 && *p_pErrorCode == 0)
        {
            unsigned int lBaudrate = 0;
            unsigned int lTimeout = 0;

            if(VCS_GetProtocolStackSettings(gateway_handle, &lBaudrate, &lTimeout, p_pErrorCode)!=0)
            {
                if(VCS_SetProtocolStackSettings(gateway_handle, g_baudrate, lTimeout, p_pErrorCode)!=0)
                {
                    if(VCS_GetProtocolStackSettings(gateway_handle, &lBaudrate, &lTimeout, p_pErrorCode)!=0)
                    {
                        if(g_baudrate==(int)lBaudrate)
                        {
                            lResult = 0;
                        }
                    }
                }
            }
            // Initializing the sub_handle
                // NOTE: The 3 sub motors are controlled with one handle while the gateway motor has it's own handle
                // The gateway motor needs its own handle because it handles all of the CAN communication
            RCLCPP_INFO(this->get_logger(), "Opening Sub Motors");
            sub_handle = VCS_OpenSubDevice(gateway_handle, sub_device, sub_protocol, p_pErrorCode);

        }
        else
        {
            gateway_handle = 0;
        }



        delete []pDeviceName;
        delete []pProtocolStackName;
        delete []pInterfaceName;
        delete []pPortName;

        return lResult;
    }

    int ParseArguments(int argc, char** argv)
    {
        int lOption;
        int lResult = MMC_SUCCESS;

        opterr = 0;

        while((lOption = getopt(argc, argv, "hlrvd:s:i:p:b:n:c:")) != -1)
        {
            switch (lOption)
            {
                case 'h':
                    PrintUsage();
                    lResult = 1;
                    break;
                case 'd':
                    g_deviceName = optarg;
                    break;
                case 's':
                    g_protocolStackName = optarg;
                    break;
                case 'i':
                    g_interfaceName = optarg;
                    break;
                case 'p':
                    g_portName = optarg;
                    break;
                case 'b':
                    g_baudrate = atoi(optarg);
                    break;
                case 'n':
                    m1_nodeID = (unsigned short)atoi(optarg);
                    break;
                case 'l':
                    g_eAppMode = AM_INTERFACE_LIST;
                    break;
                case 'r':
                    g_eAppMode = AM_PROTOCOL_LIST;
                    break;
                case 'v':
                    g_eAppMode = AM_VERSION_INFO;
                    break;
                case 'c':
                    controlMethod = atoi(optarg);
                    break;
                case '?':  // unknown option...
                    stringstream msg;
                    msg << "Unknown option: '" << char(optopt) << "'!";
                    LogInfo(msg.str());
                    PrintUsage();
                    lResult = MMC_FAILED;
                    break;
            }
        }

        return lResult;
    }
    void PrintUsage()
    {
        cout << "Usage: HelloEposCmd" << endl;
        cout << "\t-h : this help" << endl;
        cout << "\t-n : node id (default 1)" << endl;
        cout << "\t-d   : device name (EPOS2, EPOS4, default - EPOS4)"  << endl;
        cout << "\t-s   : protocol stack name (MAXON_RS232, CANopen, MAXON SERIAL V2, default - MAXON SERIAL V2)"  << endl;
        cout << "\t-i   : interface name (RS232, USB, CAN_ixx_usb 0, CAN_kvaser_usb 0,... default - USB)"  << endl;
        cout << "\t-p   : port name (COM1, USB0, CAN0,... default - USB0)" << endl;
        cout << "\t-b   : baudrate (115200, 1000000,... default - 1000000)" << endl;
        cout << "\t-l   : list available interfaces (valid device name and protocol stack required)" << endl;
        cout << "\t-r   : list supported protocols (valid device name required)" << endl;
        cout << "\t-v   : display device version" << endl;
        cout << "\t-c   : control method (0 for position and 1 for velocity)" << endl;
    }

    void LogInfo(string message)
    {
        cout << message << endl;
    }

};


int main(int argc, char ** argv)
{
//   rclcpp::init(argc, argv);
  rclcpp::init(0, nullptr);


//   std::shared_ptr<MaxonDriverNode> maxon_driver = MaxonDriverNode();

  rclcpp::spin(std::make_shared<MaxonDriverNode>(argc, argv));
  rclcpp::shutdown();
  return 0;
}
