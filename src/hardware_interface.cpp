#include <roboclaw-hardware-interface/hardware_interface.h>

using namespace roboclaw_hardware_interface;

RoboclawHardwareInterface::RoboclawHardwareInterface() {
  // set the log level to debug programmatically (should be done trough rqt_logger_level instead)
  if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
     ros::console::notifyLoggerLevelsChanged();
  }
  ROS_DEBUG("Set log level to debug");
}
RoboclawHardwareInterface::~RoboclawHardwareInterface() {}

bool RoboclawHardwareInterface::init(ros::NodeHandle& nh, ros::NodeHandle& private_nh) {

  ROS_DEBUG("Initializing hardware interface...");

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
  ROS_DEBUG("Established connection to Roboclaws via serial.");

  if (!registerInterfaces()) {
    ROS_ERROR("RoboclawHardwareInterface: failed to connect interfaces.");
    return false;
  }
  ROS_DEBUG("Successfully registered hardware interfaces.");

  return true;
}

bool RoboclawHardwareInterface::initParameters(ros::NodeHandle& nh, ros::NodeHandle& private_nh) {
  XmlRpc::XmlRpcValue roboclaw_params;
  if (!private_nh.getParam("roboclaws", roboclaw_params)) {
    ROS_ERROR("Couldn't find parameter 'roboclaws' on the parameter server.");
    return false;
  }
  ROS_DEBUG_STREAM("Found roboclaws on param server: " << roboclaw_params);
  ROS_ASSERT(roboclaw_params.getType() == XmlRpc::XmlRpcValue::TypeStruct);

  try {
    unsigned int index = 0;
    // _roboclaw_mapping = std::map<unsigned int, std::map<bool, unsigned int>>;
    ROS_DEBUG("init roboclaw_mapping ");
    for (auto it = roboclaw_params.begin(); it != roboclaw_params.end(); ++it) {
      ROS_DEBUG_STREAM("working on joint " << it->first);
      _joint_names.push_back((std::string)(it->first));

      bool channel;
      std::string channel_str;
      if (!it->second.hasMember("channel")) {
        ROS_ERROR_STREAM("Joint " << it->first << " is missing a channel field.");
        return false;
      }
      else
        channel_str = static_cast<std::string>(it->second["channel"]);
        if (channel_str == "M1")
          channel = 0;
        else if (channel_str == "M2")
          channel = 1;
        else {
          ROS_ERROR_STREAM("Joint " << it->first << " has an unsupported channel '" << channel_str << "'.");
          return false;
        }
        ROS_DEBUG_STREAM("which has channel " << channel_str);

      unsigned int address;
      if (!it->second.hasMember("address")) {
        ROS_ERROR_STREAM("Joint " << it->first << " is missing an address field.");
        return false;
      }
      else
        address = static_cast<int>(it->second["address"]);
        ROS_DEBUG_STREAM("and has address " << address);

      if (it->second.hasMember("command_interface")) {
        _command_interfaces.push_back(static_cast<std::string>(it->second["command_interface"]));
        ROS_DEBUG_STREAM("command interface: " << _command_interfaces[index]);
      }
      else {
        ROS_ERROR_STREAM("Joint " << it->first << " is missing a command_interface field.");
        return false;
      }
      ROS_DEBUG_STREAM("channel" << channel);
      // add to addresses map
      _roboclaw_mapping[address].emplace(channel, index);
      _joint_positions.push_back(0);
      _joint_velocities.push_back(0);
      _joint_efforts.push_back(0);
      _joint_commands.push_back(0);
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
  // TODO should test connection here to make sure it works, e.g. get version
  return true;
}

bool RoboclawHardwareInterface::registerInterfaces() {
  for (unsigned int index = 0; index < _joint_names.size(); index++) {
    // connect the joint state interface
    hardware_interface::JointStateHandle state_handle(_joint_names[index],
                                                      &_joint_positions[index],
                                                      &_joint_velocities[index],
                                                      &_joint_efforts[index]);
    _jnt_state_interface.registerHandle(state_handle);
    ROS_DEBUG_STREAM("registered state handle for joint " << _joint_names[index]);

    // connect to joint position, velocity or effort interface depending on command_interface
    if (_command_interfaces[index] == "position") {
      hardware_interface::JointHandle pos_handle(_jnt_state_interface.getHandle(_joint_names[index]), &_joint_commands[index]);
      _jnt_pos_interface.registerHandle(pos_handle);
      ROS_DEBUG_STREAM("registered position handle for joint " << _joint_names[index]);
    }
    else if (_command_interfaces[index] == "velocity") {
      hardware_interface::JointHandle vel_handle(_jnt_state_interface.getHandle(_joint_names[index]), &_joint_commands[index]);
      _jnt_vel_interface.registerHandle(vel_handle);
      ROS_DEBUG_STREAM("registered velocity handle for joint " << _joint_names[index]);
    }
    else if (_command_interfaces[index] == "effort") {
      hardware_interface::JointHandle eff_handle(_jnt_state_interface.getHandle(_joint_names[index]), &_joint_commands[index]);
      _jnt_eff_interface.registerHandle(eff_handle);
      ROS_DEBUG_STREAM("registered effort handle for joint " << _joint_names[index]);
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

  return true;
}


void RoboclawHardwareInterface::read() {
  // loop over the addresses
  for (auto const& addressPair : _roboclaw_mapping) {
    // ROS_DEBUG_STREAM_THROTTLE(2, "interfaces: " << interfaces.first << ", " << interfaces.second);
    // if (interfaces.first != interfaces.second || interfaces.first == "" || interfaces.second == "") {
    //   ROS_ERROR_STREAM_THROTTLE(2, "Received different command interfaces for roboclaw with address " <<
    //                                addressPair.first << ". Currently unsupported. Dropping command");
    // }
    // else {

    std::pair<int, int> pos_readings;
    std::pair<int, int> vel_readings;
    try {
      pos_readings = roboclaws_conn->get_encoders(addressPair.first);
      vel_readings = roboclaws_conn->get_velocity(addressPair.first);
    } catch(libroboclaw::crc_exception &e){
      ROS_ERROR_STREAM("RoboClaw CRC error during getting readings for " << addressPair.first);
      continue;
    } catch(timeout_exception &e){
      ROS_ERROR_STREAM("RoboClaw timout during getting readings for " << addressPair.first);
      continue;
    }
    // if channel exists, populate the interface
    if (addressPair.second.count(0) == 1) {
      _joint_positions[addressPair.second.at(0)] = pos_readings.first;
      _joint_velocities[addressPair.second.at(0)] = vel_readings.first;
    }
    if (addressPair.second.count(1) == 1) {
      _joint_positions[addressPair.second.at(1)] = pos_readings.second;
      _joint_velocities[addressPair.second.at(1)] = vel_readings.second;
    }
  }
}

void RoboclawHardwareInterface::write() {
  // loop over the addresses
  for (auto const& addressPair : _roboclaw_mapping) {
    // if (interfaces.first != interfaces.second && (interfaces.first != "" || interfaces.second != "")) {
    //   ROS_ERROR_STREAM_THROTTLE(2, "Received different command interfaces for roboclaw with address " <<
    //                                addressPair.first << ". Currently unsupported. Dropping command");
    // }
    // else {

    std::pair<int, int> commands(0, 0);
    // if channel exists, update the commands
    if (addressPair.second.count(0) == 1)
      commands.first = _joint_commands[addressPair.second.at(0)];
    if (addressPair.second.count(1) == 1)
      commands.second = _joint_commands[addressPair.second.at(1)];

    try {
      roboclaws_conn->set_velocity(addressPair.first, commands);
    } catch(libroboclaw::crc_exception &e){
      ROS_ERROR("RoboClaw CRC error during set velocity!");
    } catch(timeout_exception &e){
      ROS_ERROR("RoboClaw timout during set velocity!");
    }
  }
}
