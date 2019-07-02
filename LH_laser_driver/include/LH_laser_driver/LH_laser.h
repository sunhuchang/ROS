/*********************************************************************
 *This is demo for ROS refering to xv_11_laser_driver.
 *********************************************************************/

#include <sensor_msgs/msg/laser_scan.hpp>
#include <asio.hpp>
//#include <array.hpp>
#include <string>

namespace LH_laser_driver {
    class LHLaser {
        public:
	      uint16_t rpms; 
            LHLaser(const std::string& port, uint32_t baud_rate, uint32_t firmware, asio::io_service& io);
            ~LHLaser() {};

            void poll(sensor_msgs::msg::LaserScan::SharedPtr scan);

            void close() { shutting_down_ = true; };

        private:
            std::string port_; 
            uint32_t baud_rate_; 
            uint32_t firmware_; 

            bool shutting_down_; 
            asio::serial_port serial_; 
	    uint16_t motor_speed_; 
    };
};
