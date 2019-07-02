#include <LH_laser_driver/LH_laser.h>
#include "boost/multi_array.hpp"


namespace LH_laser_driver {
using asio::serial_port;
using asio::serial_port_base;
  LHLaser::LHLaser(const std::string& port,
                   uint32_t baud_rate,
                   uint32_t firmware,
                   asio::io_service& io) : port_(port),
                                                  baud_rate_(baud_rate),
                                                  firmware_(firmware),
                                                  shutting_down_(false),
                                                  serial_(io, port_) {
    serial_.set_option(serial_port_base::baud_rate(baud_rate_));
    serial_.set_option(serial_port::flow_control(serial_port::flow_control::none)); // NOLINT
    serial_.set_option(serial_port::parity(serial_port::parity::none));
    serial_.set_option(serial_port::stop_bits(serial_port::stop_bits::one));
    serial_.set_option(serial_port::character_size(8));

    int fd = serial_.native_handle();
    struct termios opt;
    tcgetattr(fd, &opt);
    opt.c_cflag &= ~CSIZE;
    opt.c_cflag |= CS8;
    opt.c_cflag &= ~CSTOPB;
    opt.c_cflag &= ~PARENB;
    opt.c_cflag &= ~INPCK;
    opt.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

    opt.c_oflag &= ~OPOST;
    opt.c_oflag &= ~(ONLCR | OCRNL);

    opt.c_iflag &= ~(ICRNL | INLCR);
    opt.c_iflag &= ~(IXON | IXOFF | IXANY);

    tcflush(fd, TCIOFLUSH);

    printf("configure complete\n");

    if (tcsetattr(fd, TCSANOW, &opt) != 0) {
      printf("serial error");
    } else {
      printf("start send and receive data\n");
      fcntl(fd, F_SETFL, 0);
    }
  }

  int K = 0;
  void LHLaser::poll(sensor_msgs::msg::LaserScan::SharedPtr scan) {
    uint8_t temp_char;
    uint8_t start_count = 0;
    bool got_scan = false;
    if(firmware_ == 2) {
      std::array<uint8_t, 1980> raw_bytes;
      start_count = 0;
      typedef boost::multi_array<uint16_t, 2> j_array;
      j_array A(boost::extents[10][510]);
      for (int i=0; i<10; i++) {
        A[i][0] = 0;
      }
      uint8_t good_sets = 0;
      uint32_t motor_speed = 0;
      uint16_t j_angle=0,j_size=0,j_num_readings=0,j_s_num_readings=0;
      rpms = 0;
      int index=0;
      while (!shutting_down_ && !got_scan) {
        asio::read(serial_,
                          asio::buffer(&raw_bytes[start_count],
                          1));
        if (start_count == 0) {
          //printf("\nsunhuchang debug raw_bytes[%d]=0x%x",start_count,raw_bytes[start_count]);
          if (raw_bytes[start_count] == 0xCE) {
            start_count = 1;
          }
#if 1
            asio::read(serial_,
                          asio::buffer(&raw_bytes[0],
                          256));
            for (int index=0;index<256;index++){
                if (index%16==0)
                   printf("\n");
                printf("0x%x ",raw_bytes[index]);
            }
#endif          
        } else if (start_count == 1) {
          //printf("\nsunhuchang debug raw_bytes[%d]=0x%x",start_count,raw_bytes[start_count]);
          if (raw_bytes[start_count] == 0xFA) {
            start_count = 0;
            asio::read(serial_,asio::buffer(&raw_bytes[2], 4));
            //for (int index=0;index<4;index++)
             //   printf("\nsunhuchang debug raw_bytes[2+index]=0x%x ",raw_bytes[2+index]);
            uint32_t CheckSum=0;
            uint32_t CheckSum1=0;
            j_num_readings=raw_bytes[2]+raw_bytes[3]*256;
            if(j_num_readings>200)continue;
            j_angle=raw_bytes[4]+raw_bytes[5]*256;
            A[j_angle/360][0]=j_num_readings;
            CheckSum+=j_angle;
            CheckSum+=A[j_angle/360][0];
            printf("\nj_angle%d;points%d\n",j_angle,j_num_readings);
            asio::read(serial_,
                              asio::buffer(&raw_bytes[6],
                                                  j_num_readings * 2 + 2));
            for (int index=0;index<(j_num_readings * 2 + 2);index++){
                if (index%16==0)
                   printf("\n");
                printf("sunhuchang debug raw_bytes[6+index]=0x%x ",raw_bytes[6+index]);
            }
            for(uint16_t i = 0; i < j_num_readings; i++){
              A[j_angle/360][i+1]=raw_bytes[6+i*2] + (raw_bytes[7+i*2])*256;
              CheckSum+=A[j_angle/360][i+1];
            }
            if ((uint16_t)CheckSum==(uint16_t)(raw_bytes[6+j_num_readings*2] + (raw_bytes[7 +j_num_readings*2])*256u))
              printf("j_angle%d 's reading is ok\n",j_angle);
            else {
              printf("j_angle%d 's CheckSum is bad\n",j_angle);
              continue;
            }
            if (j_angle == 3240) {
              j_s_num_readings = 0;
              for(int i = 0; i < 10; i++) {
                if (A[i][0] == 0 || (A[i][0]>150)) {
                  j_s_num_readings = 0;
                  continue;
                }
                j_s_num_readings = j_s_num_readings + A[i][0];
              }
              if (j_s_num_readings > 0 &&
                  j_s_num_readings < 1200) {
                got_scan = true;
                scan->angle_min = 0;
                scan->angle_max = 2.0 * M_PI;
                scan->angle_increment = 2.0 * M_PI / j_s_num_readings;
                scan->time_increment = 60 / 400 / j_s_num_readings;
                scan->range_min = 0;
                scan->range_max = 30.0;
                scan->ranges.resize(j_s_num_readings);
                scan->intensities.resize(j_s_num_readings);
                printf("thr cloud point size%d\n", j_s_num_readings);
                j_size=0;
                for(int j=0;j<10;j++) {
                  for(uint16_t i = 1; i < A[j][0]+1; i++){
                    uint16_t range = A[j][i] & 0x1fffu;  // A[j][i]
                    uint16_t intensity=(A[j][i]&0xe000u)>>13;
                    scan->ranges.push_back(range / 100.0);
                    scan->intensities.push_back(intensity);
                  }
                }
              }
            }
          }
        }
      }
    }
  }
};
