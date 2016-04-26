#ifndef SERIALNODE_H
#define SERIALNODE_H

#include <ros/ros.h>
#include <basketball_msgs/robot_message.h>
#include <basketball_msgs/robot_state.h>
#include <basketball_base_serial/SerialPort.h>
#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <map>

using namespace serial ;
using namespace boost ;

namespace exp_serial {
class ExpSerial
{
public:
    ExpSerial(ros::NodeHandle node) ;
    ~ExpSerial() ;

    void serialCall(ByteVector current_data,int id) ;

private:


    string serial_port_ ; 
    int baud_rate_ ; 
    shared_ptr<SerialPort>main_serial ;

    ros::NodeHandle node ;
    ros::Subscriber data_sub ;
    void dataCallBack(const basketball_msgs::robot_message::ConstPtr &ptr) ;
    void dumpBuffer(const char *buffer, int elements) ;
    map<u_int8_t,ros::Publisher> pub_ ;
protected:
};
}
#endif // SERIALNODE_H
