#ifndef ROBOCLAW_HARDWARE_INTERFACE_H
#define ROBOCLAW_HARDWARE_INTERFACE_H

// ROS
//#include <ros/ros.h>
#include <ros/node_handle.h>
// rosparam
#include <XmlRpcException.h>
#include <XmlRpcValue.h>

// ROS control
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

// Roboclaw driver
#include <libroboclaw/roboclaw_driver.h>

namespace roboclaw_hardware_interface
{
  /**
   * Generic hardware interface for roboclaw drivers
   */
  class RoboclawHardwareInterface : public hardware_interface::RobotHW
  {
  public:
    /**
     * Default constructor
     */
    RoboclawHardwareInterface();

    /**
     * Default destructor, stops movement on all roboclaw motors
     */
    ~RoboclawHardwareInterface();

    /**
      * Fully initialize the hardware interface with ROS.
      * This includes reading parameters from the parameter server for determining
      * roboclaw settings and which motors use which interface
      *
      * @param[in] nh A node handle in the root namespace of the control node
      * @param[in] private_nh A node handle in the namespace of the robot hw
      * @return true if successful
      */
    bool init(ros::NodeHandle& nh, ros::NodeHandle& private_nh);

    /**
      * Read and parse parameters from the parameter server
      *
      * @param[in] nh A node handle in the root namespace of the control node
      * @param[in] private_nh A node handle in the namespace of the robot hw
      * @return true if successful
      */
    bool initParameters(ros::NodeHandle& nh, ros::NodeHandle& private_nh);

    /**
      * Initialize roboclaws based on read parameters
      *
      * @param[in] serial_port. e.g. "/dev/serial0"
      * @param[in] baudrate. e.g. 115200
      *
      * @return true if successful
      */
    bool initRoboclaws(std::string serial_port, int baudrate);

    /**
      * Register position, velocity, effort, and state interfaces
      *
      * @param[in]
      * @return true if successful
      */
    bool registerInterfaces();

    /**
     * Read data from the roboclaws
     */
    void read();
    /**
     * Write data to the roboclaws
     */
    void write();

  private:
    // serial connection to roboclaws
    libroboclaw::driver *roboclaws_conn;

    // roboclaws and motor mappings
    std::map<int,std::map<std::string,unsigned int>> _roboclaw_mapping;
    std::vector<std::string> _joint_names;
    std::vector<std::string> _command_interfaces;

    // ROS control interfaces
    hardware_interface::JointStateInterface _jnt_state_interface;
    hardware_interface::PositionJointInterface _jnt_pos_interface;
    hardware_interface::VelocityJointInterface _jnt_vel_interface;
    hardware_interface::EffortJointInterface _jnt_eff_interface;

    // command, position, velocity, effort state holders, shared memory with the controller
    std::vector<double> _joint_commands;
    std::vector<double> _joint_positions;
    std::vector<double> _joint_velocities;
    std::vector<double> _joint_efforts;

  };
}

#endif  // ROBOCLAW_HARDWARE_INTERFACE_H