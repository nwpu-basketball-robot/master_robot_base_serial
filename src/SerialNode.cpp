#include <basketball_base_serial/SerialNode.h>
#include <stdio.h>
#include <string.h>
#include <sstream>
#include <boost/bind.hpp>

using namespace exp_serial ;
using namespace boost ;
ExpSerial::ExpSerial(ros::NodeHandle node)
   :node(node)
{
   node.param("serial_port" , serial_port_  , string("/dev/ttyUSB0")) ;
   node.param("baud_rate" , baud_rate_ , 115200) ;
   SerialParams serial_params(serial_port_,baud_rate_)  ; 
    main_serial = make_shared<SerialPort>() ;
    main_serial->setSerialParams(serial_params) ; 
    main_serial->setCallbackFunc(bind(&ExpSerial::serialCall,this,_1,_2));
   data_sub = node.subscribe("/robot_cmd" , 1000 , &ExpSerial::dataCallBack , this) ;
    main_serial->startThread() ;
}

ExpSerial::~ExpSerial()
{
    node.shutdown();
}

void ExpSerial::dumpBuffer(const char *buffer, int elements)
{
    int j;
            printf(" [");
            for (j = 0; j < elements; j++)
            {
                if (j > 0)
                    printf(" ");
                printf("0x%02X", (unsigned int)(buffer[j]&0x0ff));
            }
            printf("]\n");
}


void ExpSerial::serialCall(ByteVector current_data, int id)
{
    ros::Publisher &serialPub = pub_[id] ;
    stringstream recv_name ;	
    if(!serialPub)
    {
    //    stringstream recv_name ;
        recv_name<<"/RecvData/"<<id;
        serialPub  = node.advertise<basketball_msgs::robot_state>(recv_name.str(),100) ;
    }
    basketball_msgs::robot_state pub_data ;
    pub_data.id = id ;
    pub_data.header.stamp = ros::Time::now() ;
    //第一位代表数据id , 所以从1开始 
    for(int i = 1 ; i < current_data.size() ; i += 4)
    {
        float data = *(float *)(current_data.data()+i) ;
        pub_data.data.push_back(data) ;
    }
    serialPub.publish(pub_data) ;
}

void ExpSerial::dataCallBack(const basketball_msgs::robot_message::ConstPtr &ptr)
{
    ByteVector current_data ;
    size_t data_len = ptr->data.size() ;
    current_data.resize(data_len+1,0);

    uint32_t check_sum  = 0 ;

    for(size_t i = 0 ; i < data_len ; i++)
    {
        if(i>=2)check_sum += ptr->data[i] ;
        current_data[i] = ptr->data[i] ;
    }
    current_data[data_len] = (uint8_t)(check_sum%255) ; //set the last position as the checksum
    main_serial->writeRaw(current_data) ;
}


