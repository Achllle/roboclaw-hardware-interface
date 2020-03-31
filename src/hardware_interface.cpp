#include <roboclaw-hardware-interface/hardware_interface.h>

using namespace roboclaw_hardware_interface;

RoboclawHardwareInterface::RoboclawHardwareInterface() {}

bool RoboclawHardwareInterface::init(ros::NodeHandle& nh, ros::NodeHandle& private_nh) {

  if (!initParameters(nh, private_nh)) {
    ROS_ERROR("RoboclawHardwareInterface: failed to read or parse parameters from rosparam.");
    return false;
  }

  std::string serial_port;
  int baudrate;
  private_nh.param<std::string>("serial_port", serial_port, "/dev/serial0");
  private_nh.param("baudrate", baudrate, 115200);
  if (!initRoboclaws(serial_port, baudrate)) {
    ROS_ERROR("RoboclawHardwareInterface: failed to connect to roboclaws via serial.");
    return false;
  }

  if (!registerInterfaces()) {
    ROS_ERROR("RoboclawHardwareInterface: failed to connect interfaces.");
    return false;
  }
}

bool RoboclawHardwareInterface::initParameters(ros::NodeHandle& nh, ros::NodeHandle& private_nh) {
  XmlRpc::XmlRpcValue roboclaw_params;
  if (!private_nh.getParam("roboclaws", roboclaw_params)) {
    ROS_ERROR("Couldn't find parameter 'roboclaws' on the parameter server.");
    return false;
  }

  try {
    unsigned int index = 0;
    _roboclaw_mapping = std::map<int, std::map<std::string, unsigned int>>{};
    for (XmlRpc::XmlRpcValue::ValueStruct::const_iterator it = roboclaw_params.begin(); it != roboclaw_params.end(); ++it) {
      _joint_names[index] = it->first;
      std::string channel;
      int address;
      if (!it->second.hasMember("channel")) {
        ROS_ERROR_STREAM("Joint " << it->first << " is missing a channel field.");
        return false;
      }
      else
        channel = static_cast<std::string>(roboclaw_params[it->first]["channel"]);
      if (!it->second.hasMember("address")) {
        ROS_ERROR_STREAM("Joint " << it->first << " is missing an address field.");
        return false;
      }
      else
        address = static_cast<int>(roboclaw_params[it->first]["address"]);

      if (it->second.hasMember("command_interface")) {
        _command_interfaces[index] = static_cast<std::string>(roboclaw_params[it->first]["command_interface"]);
      }
      else {
        ROS_ERROR_STREAM("Joint " << it->first << " is missing a command_interface field.");
        return false;
      }
      std::map<std::string, unsigned int> vals;
      vals.emplace(channel, index);
      _roboclaw_mapping.emplace(address, vals);
      index++;
    }
  }
  catch (XmlRpc::XmlRpcException &xmlexc) {
    ROS_FATAL_STREAM("Exception while attempting to read roboclaw parameter configuration:\n" << xmlexc.getMessage());
    return false;
  }

  return true;
}

bool RoboclawHardwareInterface::initRoboclaws(std::string serial_port, int baudrate) {
  roboclaws_conn = new libroboclaw::driver(serial_port, baudrate);
}

bool RoboclawHardwareInterface::registerInterfaces() {
  for (unsigned int index = 0; index < _joint_names.size(); index++) {
    // connect the joint state interface
    hardware_interface::JointStateHandle state_handle(_joint_names[index],
                                                      &_joint_positions[index],
                                                      &_joint_velocities[index],
                                                      &_joint_efforts[index]);
    _jnt_state_interface.registerHandle(state_handle);

    // connect to joint position, velocity or effort interface depending on command_interface
    if (_command_interfaces[index] == "position") {
      hardware_interface::JointHandle pos_handle(_jnt_state_interface.getHandle(_joint_names[index]), &_joint_commands[index]);
      _jnt_pos_interface.registerHandle(pos_handle);
    }
    else if (_command_interfaces[index] == "velocity") {
      hardware_interface::JointHandle vel_handle(_jnt_state_interface.getHandle(_joint_names[index]), &_joint_commands[index]);
      _jnt_vel_interface.registerHandle(vel_handle);
    }
    else if (_command_interfaces[index] == "effort") {
      hardware_interface::JointHandle eff_handle(_jnt_state_interface.getHandle(_joint_names[index]), &_joint_commands[index]);
      _jnt_eff_interface.registerHandle(eff_handle);
    }
    else {
      ROS_ERROR_STREAM("Unsupported command interface " << _command_interfaces[index] << " received"
                       << " for joint " << _joint_names[index] << ". Value must be either position, velocity, or effort");
      return false;
    }
  }

  registerInterface(&_jnt_state_interface);
  registerInterface(&_jnt_pos_interface);
  registerInterface(&_jnt_vel_interface);
  registerInterface(&_jnt_eff_interface);
}


void RoboclawHardwareInterface::read() {
  for (auto const& addressPair : _roboclaw_mapping) {
    std::pair<int, int> readings = std::pair<int, int>(0, 0);
    std::pair<std::string, std::string> interfaces;
    std::pair<unsigned int, unsigned int> indices;
    if (addressPair.second.count("M1") == 1) {
      unsigned int idx = addressPair.second.at("M1");
      indices.first = idx;
      interfaces.first = _command_interfaces[idx];
    }
    if (addressPair.second.count("M2") == 1) {
      unsigned int idx = addressPair.second.at("M2");
      indices.second = idx;
      interfaces.second = _command_interfaces[idx];
    }

    if (interfaces.first != interfaces.second && (interfaces.first != "" || interfaces.second != "")) {
      ROS_ERROR_STREAM_THROTTLE(2, "Received different command interfaces for roboclaw with address " <<
                                   addressPair.first << ". Currently unsupported. Dropping command");
    }
    else {
      try {
        if (interfaces.first == "velocity") {
          readings = roboclaws_conn->get_encoders(addressPair.first);  // TODO change to get_velocity
          _joint_velocities[indices.first] = readings.first;
          _joint_velocities[indices.second] = readings.second;
        }
        else if (interfaces.first == "duty") {
          readings = roboclaws_conn->get_encoders(addressPair.first);  // TODO change to get_duty
          _joint_efforts[indices.first] = readings.first;
          _joint_efforts[indices.second] = readings.second;
        }
        else
          ROS_ERROR_STREAM("Received unexpected command interface " << interfaces.first << " for address " << addressPair.first);
      } catch(libroboclaw::crc_exception &e){
        ROS_ERROR_STREAM("RoboClaw CRC error during getting " << interfaces.first);
        continue;
      } catch(timeout_exception &e){
        ROS_ERROR_STREAM("RoboClaw timout during getting " << interfaces.first);
        continue;
      }
    }
  }
}

void RoboclawHardwareInterface::write() {
  for (auto const& addressPair : _roboclaw_mapping) {
    std::pair<int, int> commands;
    std::pair<std::string, std::string> interfaces;
    if (addressPair.second.count("M1") == 1) {
      unsigned int idx = addressPair.second.at("M1");
      interfaces.first = _command_interfaces[idx];
      commands.first = _joint_commands[idx];
    }
    else
      commands.first = 0;

    if (addressPair.second.count("M2") == 1) {
      unsigned int idx = addressPair.second.at("M2");
      interfaces.second = _command_interfaces[idx];
      commands.second = _joint_commands[idx];
    }
    else
      commands.second = 0;

    if (interfaces.first != interfaces.second && (interfaces.first != "" || interfaces.second != "")) {
      ROS_ERROR_STREAM_THROTTLE(2, "Received different command interfaces for roboclaw with address " <<
                                   addressPair.first << ". Currently unsupported. Dropping command");
    }
    else {
      if (interfaces.first == "velocity") {
        try {
          roboclaws_conn->set_velocity(addressPair.first, commands);
        } catch(libroboclaw::crc_exception &e){
          ROS_ERROR("RoboClaw CRC error during set velocity!");
        } catch(timeout_exception &e){
          ROS_ERROR("RoboClaw timout during set velocity!");
        }
      }
      else if (interfaces.first == "effort") {
        try {
          roboclaws_conn->set_duty(addressPair.first, commands);
        } catch(libroboclaw::crc_exception &e){
          ROS_ERROR("Roboclaw CRC error during set duty!");
        } catch(timeout_exception &e){
          ROS_ERROR("Roboclaw timout during set duty!");
        }
      }
      else {
        ROS_ERROR_STREAM_THROTTLE(2, "Unsupported command interface " << commands.first <<
                                     ". Options are velocity of effort");
      }
    }
  }
}
