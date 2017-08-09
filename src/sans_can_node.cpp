#include "ros/ros.h"
#include "sans_can/sans_can_msgs_rtk.h"
#include "sans_can/sans_can_msgs_vn300.h"
#include <PCANBasic.h>
#include <boost/thread.hpp>

#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <linux/can.h>
#include <linux/can/raw.h>

class CANNode {


public :
    ros::NodeHandle nh;
    ros::Publisher can_pub1;
    ros::Publisher can_pub2;

    inline uint16_t big16(uint8_t *c)
    {
        uint8_t i = 0;
        uint16_t temp = 0x00;

        for (i = 0; i < 2; i++)
        {
            temp |= c[i];

            if (i == 1) break;
            else temp = temp << 8;

        }

        return temp;
    }

    inline uint32_t big32(uint8_t *c)
    {
        uint8_t i = 0;
        uint32_t temp = 0x00;

        for (i = 0; i < 4; i++)
        {
            temp |= c[i];

            if (i == 3) break;
            else temp = temp << 8;

        }

        return temp;
    }

    inline uint64_t big64(uint8_t *c)
    {
        uint8_t i = 0;
        uint64_t temp = 0x00;

        for (i = 0; i < 8; i++)
        {
            temp |= c[i];

            if (i == 7) break;
            else temp = temp << 8;

        }

        return temp;
    }


    void InitCan(int & fd, fd_set & fds, int pcan_interface_channel)
    {
        TPCANStatus status = CAN_Initialize(pcan_interface_channel, PCAN_BAUD_500K, 0, 0, 0);

        // event
        CAN_GetValue(PCAN_USBBUS1, PCAN_RECEIVE_EVENT, &fd,sizeof(int));
        FD_ZERO(&fds);
        FD_SET(fd, &fds);
    }

    void MainCAN()
    {
        // PCAN data
        TPCANStatus status;
        TPCANMsg message;
        uint32_t temp = 0x00;

        static uint8_t bitFlagM8P = 0x00;
        static uint16_t bitFlagVN300 = 0x0000;
        // ================== end mapping ==================

        // PCAN spin
        while (select(can_fd + 1, &can_fds, NULL, NULL, NULL) > 0)
        {
            status = CAN_Read(PCAN_USBBUS1, &message, NULL);
            if (status != PCAN_ERROR_OK)
            {
                break;
            }

            switch(message.ID)
            {
            case 0x001 :
                can_msg_rtk.header.stamp = ros::Time::now();

                can_msg_rtk.WGS84_Lat = big32(&message.DATA[0])/10000000.0;
                can_msg_rtk.WGS84_Lon = big32(&message.DATA[4])/10000000.0;

                bitFlagM8P |= 0x01;

                break;

            case 0x002 :
                

                can_msg_rtk.WGS84_Alt = big32(&message.DATA[0])/1000.0;
                temp = big32(&message.DATA[4]);
                memcpy(&can_msg_rtk.NED_N, &temp, sizeof(float));

                bitFlagM8P |= 0x02;

                break;

            case 0x003 :
                

                temp = big32(&message.DATA[0]);
                memcpy(&can_msg_rtk.NED_E, &temp, sizeof(float));


                temp = big32(&message.DATA[4]);
                memcpy(&can_msg_rtk.NED_D, &temp, sizeof(float));

                bitFlagM8P |= 0x04;

                break;

            case 0x004 :

                can_msg_rtk.Hori_Accuracy = big32(&message.DATA[0])/10000.0;
                can_msg_rtk.Vert_Accuracy = big32(&message.DATA[4])/10000.0;

                bitFlagM8P |= 0x08;

                break;

            case 0x005 :

                can_msg_rtk.FixType = message.DATA[0];
                can_msg_rtk.Flags = message.DATA[1];
                can_msg_rtk.Flags2 = message.DATA[2];

                bitFlagM8P |= 0x10;

                break;



            case 0x011 :
                

                can_msg_vn300.header.stamp = ros::Time::now();

                temp = big32(&message.DATA[0]);
                memcpy(&can_msg_vn300.ROLL, &temp, sizeof(float));


                temp = big32(&message.DATA[4]);
                memcpy(&can_msg_vn300.PITCH, &temp, sizeof(float));

                bitFlagVN300 |= 0x0001;
                break;

            case 0x012 :
                
                temp = big32(&message.DATA[0]);
                memcpy(&can_msg_vn300.YAW, &temp, sizeof(float));


                temp = big32(&message.DATA[4]);
                memcpy(&can_msg_vn300.Uncertain_ROLL, &temp, sizeof(float));

                bitFlagVN300 |= 0x0002;
                break;

            case 0x013 :
                
                temp = big32(&message.DATA[0]);
                memcpy(&can_msg_vn300.Uncertain_PITCH, &temp, sizeof(float));


                temp = big32(&message.DATA[4]);
                memcpy(&can_msg_vn300.Uncertain_YAW, &temp, sizeof(float));

                bitFlagVN300 |= 0x0004;
                break;

            case 0x014 :
            

                temp = big64(&message.DATA[0]);
                memcpy(&can_msg_vn300.WGS84_Lat, &temp, sizeof(double));

                bitFlagVN300 |= 0x0008;
                break;

            case 0x015 :
            

                temp = big64(&message.DATA[0]);
                memcpy(&can_msg_vn300.WGS84_Lon, &temp, sizeof(double));

                bitFlagVN300 |= 0x0010;
                break;

            case 0x016 :
            

                temp = big64(&message.DATA[0]);
                memcpy(&can_msg_vn300.WGS84_Alt, &temp, sizeof(double));

                bitFlagVN300 |= 0x0020;
                break;

            case 0x017 :
                

                temp = big32(&message.DATA[0]);
                memcpy(&can_msg_vn300.NED_N, &temp, sizeof(float));


                temp = big32(&message.DATA[4]);
                memcpy(&can_msg_vn300.NED_E, &temp, sizeof(float));

                bitFlagVN300 |= 0x0040;
                break;

            case 0x018 :
                

                temp = big32(&message.DATA[0]);
                memcpy(&can_msg_vn300.NED_D, &temp, sizeof(float));


                temp = big32(&message.DATA[4]);
                memcpy(&can_msg_vn300.VEL_N, &temp, sizeof(float));

                bitFlagVN300 |= 0x0080;
                break;

            case 0x019 :
                
                temp = big32(&message.DATA[0]);
                memcpy(&can_msg_vn300.VEL_E, &temp, sizeof(float));


                temp = big32(&message.DATA[4]);
                memcpy(&can_msg_vn300.VEL_D, &temp, sizeof(float));

                bitFlagVN300 |= 0x0100;
                
                break;

            case 0x020 :
                
                temp = big32(&message.DATA[0]);
                memcpy(&can_msg_vn300.COMP_ACC_X, &temp, sizeof(float));


                temp = big32(&message.DATA[4]);
                memcpy(&can_msg_vn300.COMP_ACC_Y, &temp, sizeof(float));

                bitFlagVN300 |= 0x0200;
                
                break;

            case 0x021 :
                

                temp = big32(&message.DATA[0]);
                memcpy(&can_msg_vn300.COMP_ACC_Z, &temp, sizeof(float));


                temp = big32(&message.DATA[4]);
                memcpy(&can_msg_vn300.COMP_GYRO_X, &temp, sizeof(float));

                bitFlagVN300 |= 0x0400;
                
                break;

            case 0x022 :
                

                temp = big32(&message.DATA[0]);
                memcpy(&can_msg_vn300.COMP_GYRO_Y, &temp, sizeof(float));


                temp = big32(&message.DATA[4]);
                memcpy(&can_msg_vn300.COMP_GYRO_Z, &temp, sizeof(float));

                bitFlagVN300 |= 0x0800;
                
                break;



            }

            if (bitFlagM8P == 0x1F)
            {
                bitFlagM8P = 0x00;
                can_pub1.publish(can_msg_rtk);
            }

            if (bitFlagVN300 == 0x0FFF)
            {
                bitFlagVN300 = 0x0000;
                can_pub2.publish(can_msg_vn300);
            }




        }
    }

private :
    // can object
    int can_fd;
    fd_set can_fds;
    sans_can::sans_can_msgs_rtk         can_msg_rtk;
    sans_can::sans_can_msgs_vn300       can_msg_vn300;

public:


    CANNode(ros::NodeHandle _nh)
    {
        can_pub1 = nh.advertise<sans_can::sans_can_msgs_rtk>("sans_can_msg_rtk", 1000);
        can_pub2 = nh.advertise<sans_can::sans_can_msgs_vn300>("sans_can_msg_vn300", 1000);

        InitCan(can_fd, can_fds, PCAN_USBBUS1);

        boost::thread grab_thread = boost::thread(boost::bind(&CANNode::MainCAN, this));
    }

    ~CANNode() {}

};


int main(int argc, char* argv[]) 
{
    ros::init(argc, argv, "sans_can_node");
    ros::NodeHandle nh;
    CANNode kn(nh);
    ros::spin();
	return 0;
}

