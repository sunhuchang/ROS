/*********************************************************************
 *This is demo for ROS refering to xv_11_laser_driver.
 rosrun LH_laser_driver LH_laser_publisher _frame_id:=LH_laser _port:=/dev/ttyUSB0 _baud_rate:=230400 _firmware_version:=2
 echo LOCONH >/dev/ttyUSB0 //with intensity
 echo LNCONH >/dev/ttyUSB0 //without intensity
 echo LSTARH >/dev/ttyUSB0 //working begins
 echo LSTOPH >/dev/ttyUSB0 //working stops
 echo LRESTH >/dev/ttyUSB0 //working stops
 echo LSRPM:xxxH >/dev/ttyUSB0 //set motor RPM
 *********************************************************************/

//#include <ros/ros.h>
#include <rclcpp/rclcpp.hpp>
//#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/msg/laser_scan.h>
#include <asio.hpp>
#include <LH_laser_driver/LH_laser.h>
#include <std_msgs/msg/u_int16.hpp>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("LH_laser_publisher");
  //ros::NodeHandle n;
  //ros::NodeHandle priv_nh("~");

  std::string port="/dev/ttyTHS1";
  int baud_rate = 230400;
  std::string frame_id="LH_laser";
  int firmware_number=2;

  std_msgs::msg::UInt16 rpms; 
/*
  priv_nh.param("port", port, std::string("/dev/ttyTHS1"));
  priv_nh.param("baud_rate", baud_rate, 230400);
  priv_nh.param("frame_id", frame_id, std::string("LH_laser"));
  priv_nh.param("firmware_version", firmware_number, 2);
*/
node->get_parameter("port", port);
node->get_parameter("baud_rate", baud_rate);
node->get_parameter("frame_id", frame_id);
node->get_parameter("firmware_version", firmware_number);

  asio::io_service io;
  try {
//RCLCPP_ERROR(node->get_logger(),"sunhuchang pos0");
    LH_laser_driver::LHLaser laser(port, baud_rate, firmware_number, io);
//RCLCPP_ERROR(node->get_logger(),"sunhuchang pos2");
    //rclcpp::Publisher laser_pub = n.advertise<sensor_msgs::msg::LaserScan>("scan", 1500);
    auto laser_pub = node->create_publisher<sensor_msgs::msg::LaserScan>("scan",rmw_qos_profile_default);
    //rclcpp::Publisher motor_pub = n.advertise<std_msgs::UInt16>("rpms",1500);
    auto motor_pub = node->create_publisher<std_msgs::msg::UInt16>("rpms",rmw_qos_profile_default);
    while (rclcpp::ok()) {
//RCLCPP_ERROR(node->get_logger(),"sunhuchang debug LH_laser pos1");
      sensor_msgs::msg::LaserScan::SharedPtr scan(new sensor_msgs::msg::LaserScan);
      scan->header.frame_id = frame_id;
      rclcpp::Clock ros_clock(RCL_ROS_TIME);
      scan->header.stamp = ros_clock.now();
//RCLCPP_ERROR(node->get_logger(),"sunhuchang debug LH_laser pos2");
      laser.poll(scan);
//RCLCPP_ERROR(node->get_logger(),"sunhuchang debug LH_laser pos3");
      rpms.data=laser.rpms;
      laser_pub->publish(scan);
      motor_pub->publish(rpms);
//      RCLCPP_ERROR(node->get_logger(),"sunhuchang debug LH_laser rpms=%d", rpms.data);

    }
    laser.close();
    return 0;
  } catch (asio::system_error ex) {
    RCLCPP_ERROR(node->get_logger(),"LH_laser Error:%s", ex.what());
    return -1;
  }
}
