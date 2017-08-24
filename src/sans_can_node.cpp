#include "ros/ros.h"
#include "sans_can/sans_can_msgs_rtk.h"
#include "sans_can/sans_can_msgs_sdins.h"
#include "sans_can/sans_can_msgs_vn300.h"
#include "sans_can/sans_can_msgs_bestpos.h"
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
    ros::Publisher can_pub3;
    ros::Publisher can_pub4;


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

        ros::Time bf_rtk = ros::Time::now(), bf_vn300 = ros::Time::now(), bf_bestpos = ros::Time::now(), bf_sdins = ros::Time::now();

        static uint8_t  bitFlagM8P = 0x00;
        static uint16_t bitFlagVN300 = 0x0000;
        static uint8_t  bitFlagBESTPOS = 0x00;
        static uint8_t  bitFlagSDINS = 0x00;

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


            /** @C94-M8P Begin *///////////////////////////////////////////////////
            case 0x001 :

                if (can_msg_rtk.header.stamp == bf_rtk)
                {
                    can_msg_rtk.header.stamp = ros::Time::now();
                }

                can_msg_rtk.WGS84_Lat = big32(&message.DATA[0])/10000000.0;
                can_msg_rtk.WGS84_Lon = big32(&message.DATA[4])/10000000.0;

                bitFlagM8P |= 0x01;

                break;

            case 0x002 :

                if (can_msg_rtk.header.stamp == bf_rtk)
                {
                    can_msg_rtk.header.stamp = ros::Time::now();
                }
                

                can_msg_rtk.WGS84_Alt = big32(&message.DATA[0])/1000.0;
                temp = big32(&message.DATA[4]);
                memcpy(&can_msg_rtk.NED_N, &temp, sizeof(float));

                bitFlagM8P |= 0x02;

                break;

            case 0x003 :

                if (can_msg_rtk.header.stamp == bf_rtk)
                {
                    can_msg_rtk.header.stamp = ros::Time::now();
                }
                

                temp = big32(&message.DATA[0]);
                memcpy(&can_msg_rtk.NED_E, &temp, sizeof(float));


                temp = big32(&message.DATA[4]);
                memcpy(&can_msg_rtk.NED_D, &temp, sizeof(float));

                bitFlagM8P |= 0x04;

                break;

            case 0x004 :

                if (can_msg_rtk.header.stamp == bf_rtk)
                {
                    can_msg_rtk.header.stamp = ros::Time::now();
                }

                can_msg_rtk.Hori_Accuracy = big32(&message.DATA[0])/10000.0;
                can_msg_rtk.Vert_Accuracy = big32(&message.DATA[4])/10000.0;

                bitFlagM8P |= 0x08;

                break;

            case 0x005 :

                if (can_msg_rtk.header.stamp == bf_rtk)
                {
                    can_msg_rtk.header.stamp = ros::Time::now();
                }

                can_msg_rtk.FixType = message.DATA[0];
                can_msg_rtk.Flags = message.DATA[1];
                can_msg_rtk.Flags2 = message.DATA[2];

                bitFlagM8P |= 0x10;

                break;

            /** @C94-M8P End *///////////////////////////////////////////////////


            /** @VN-300 Begin *///////////////////////////////////////////////////        
            case 0x011 :

                if (can_msg_vn300.header.stamp == bf_vn300)
                {
                    can_msg_vn300.header.stamp = ros::Time::now();
                }
                
                temp = big32(&message.DATA[0]);
                memcpy(&can_msg_vn300.ROLL, &temp, sizeof(float));


                temp = big32(&message.DATA[4]);
                memcpy(&can_msg_vn300.PITCH, &temp, sizeof(float));

                bitFlagVN300 |= 0x0001;
                break;

            case 0x012 :

                if (can_msg_vn300.header.stamp == bf_vn300)
                {
                    can_msg_vn300.header.stamp = ros::Time::now();
                }
                
                temp = big32(&message.DATA[0]);
                memcpy(&can_msg_vn300.YAW, &temp, sizeof(float));


                temp = big32(&message.DATA[4]);
                memcpy(&can_msg_vn300.Uncertain_ROLL, &temp, sizeof(float));

                bitFlagVN300 |= 0x0002;
                break;

            case 0x013 :

                if (can_msg_vn300.header.stamp == bf_vn300)
                {
                    can_msg_vn300.header.stamp = ros::Time::now();
                }
                
                temp = big32(&message.DATA[0]);
                memcpy(&can_msg_vn300.Uncertain_PITCH, &temp, sizeof(float));


                temp = big32(&message.DATA[4]);
                memcpy(&can_msg_vn300.Uncertain_YAW, &temp, sizeof(float));

                bitFlagVN300 |= 0x0004;
                break;

            case 0x014 :

                if (can_msg_vn300.header.stamp == bf_vn300)
                {
                    can_msg_vn300.header.stamp = ros::Time::now();
                }

                temp = big32(&message.DATA[0]);
                memcpy(&can_msg_vn300.WGS84_Lat, &temp, sizeof(float));

                temp = big32(&message.DATA[4]);
                memcpy(&can_msg_vn300.WGS84_Lon, &temp, sizeof(float));

                bitFlagVN300 |= 0x0008;
                break;

            case 0x015 :

                if (can_msg_vn300.header.stamp == bf_vn300)
                {
                    can_msg_vn300.header.stamp = ros::Time::now();
                }
            
                temp = big32(&message.DATA[0]);
                memcpy(&can_msg_vn300.WGS84_Alt, &temp, sizeof(float));

                temp = big32(&message.DATA[4]);
                memcpy(&can_msg_vn300.NED_N, &temp, sizeof(float));

                bitFlagVN300 |= 0x0010;
                break;

            case 0x016 :

                if (can_msg_vn300.header.stamp == bf_vn300)
                {
                    can_msg_vn300.header.stamp = ros::Time::now();
                }

                temp = big32(&message.DATA[0]);
                memcpy(&can_msg_vn300.NED_E, &temp, sizeof(float));

                temp = big32(&message.DATA[4]);
                memcpy(&can_msg_vn300.NED_D, &temp, sizeof(float));

                bitFlagVN300 |= 0x0020;
                break;

            case 0x017 :

                if (can_msg_vn300.header.stamp == bf_vn300)
                {
                    can_msg_vn300.header.stamp = ros::Time::now();
                }
                
                temp = big32(&message.DATA[0]);
                memcpy(&can_msg_vn300.VEL_N, &temp, sizeof(float));


                temp = big32(&message.DATA[4]);
                memcpy(&can_msg_vn300.VEL_E, &temp, sizeof(float));

                bitFlagVN300 |= 0x0040;
                break;

            case 0x018 :

                if (can_msg_vn300.header.stamp == bf_vn300)
                {
                    can_msg_vn300.header.stamp = ros::Time::now();
                }

                temp = big32(&message.DATA[0]);
                memcpy(&can_msg_vn300.VEL_D, &temp, sizeof(float));


                bitFlagVN300 |= 0x0080;
                break;

            case 0x019 :

                if (can_msg_vn300.header.stamp == bf_vn300)
                {
                    can_msg_vn300.header.stamp = ros::Time::now();
                }
                
                temp = big32(&message.DATA[0]);
                memcpy(&can_msg_vn300.COMP_ACC_X, &temp, sizeof(float));


                temp = big32(&message.DATA[4]);
                memcpy(&can_msg_vn300.COMP_ACC_Y, &temp, sizeof(float));

                bitFlagVN300 |= 0x0100;
                
                break;

            case 0x01A :

                if (can_msg_vn300.header.stamp == bf_vn300)
                {
                    can_msg_vn300.header.stamp = ros::Time::now();
                }
                

                temp = big32(&message.DATA[0]);
                memcpy(&can_msg_vn300.COMP_ACC_Z, &temp, sizeof(float));


                temp = big32(&message.DATA[4]);
                memcpy(&can_msg_vn300.COMP_GYRO_X, &temp, sizeof(float));

                bitFlagVN300 |= 0x0200;
                
                break;

            case 0x01B :

                if (can_msg_vn300.header.stamp == bf_vn300)
                {
                    can_msg_vn300.header.stamp = ros::Time::now();
                }
                

                temp = big32(&message.DATA[0]);
                memcpy(&can_msg_vn300.COMP_GYRO_Y, &temp, sizeof(float));


                temp = big32(&message.DATA[4]);
                memcpy(&can_msg_vn300.COMP_GYRO_Z, &temp, sizeof(float));

                bitFlagVN300 |= 0x0400;
                
                break;


            case 0x01C :

                if (can_msg_vn300.header.stamp == bf_vn300)
                {
                    can_msg_vn300.header.stamp = ros::Time::now();
                }
                
                temp = big32(&message.DATA[0]);
                memcpy(&can_msg_vn300.UNCOMP_ACC_X, &temp, sizeof(float));


                temp = big32(&message.DATA[4]);
                memcpy(&can_msg_vn300.UNCOMP_ACC_Y, &temp, sizeof(float));

                bitFlagVN300 |= 0x0800;
                
                break;

            case 0x01D :

                if (can_msg_vn300.header.stamp == bf_vn300)
                {
                    can_msg_vn300.header.stamp = ros::Time::now();
                }
                

                temp = big32(&message.DATA[0]);
                memcpy(&can_msg_vn300.UNCOMP_ACC_Z, &temp, sizeof(float));


                temp = big32(&message.DATA[4]);
                memcpy(&can_msg_vn300.UNCOMP_GYRO_X, &temp, sizeof(float));

                bitFlagVN300 |= 0x1000;
                
                break;

            case 0x01E :

                if (can_msg_vn300.header.stamp == bf_vn300)
                {
                    can_msg_vn300.header.stamp = ros::Time::now();
                }
                

                temp = big32(&message.DATA[0]);
                memcpy(&can_msg_vn300.UNCOMP_GYRO_Y, &temp, sizeof(float));


                temp = big32(&message.DATA[4]);
                memcpy(&can_msg_vn300.UNCOMP_GYRO_Z, &temp, sizeof(float));

                bitFlagVN300 |= 0x2000;
                
                break;

            case 0x01F :

                if (can_msg_vn300.header.stamp == bf_vn300)
                {
                    can_msg_vn300.header.stamp = ros::Time::now();
                }
                

                temp = big32(&message.DATA[0]);
                memcpy(&can_msg_vn300.TEMPRATURE, &temp, sizeof(float));


                temp = big32(&message.DATA[4]);
                memcpy(&can_msg_vn300.PRESSURE, &temp, sizeof(float));

                bitFlagVN300 |= 0x4000;
                
                break;
            /** @VN-300 End *///////////////////////////////////////////////////


              
            /** @Novatel Begin *///////////////////////////////////////////////////
            case 0x030 :

                if (can_msg_bestpos.header.stamp == bf_bestpos)
                {
                    can_msg_bestpos.header.stamp = ros::Time::now();
                }

                temp = big32(&message.DATA[0]);
                memcpy(&can_msg_bestpos.WGS84_Lat, &temp, sizeof(float));

                temp = big32(&message.DATA[4]);
                memcpy(&can_msg_bestpos.WGS84_Lon, &temp, sizeof(float));

                bitFlagBESTPOS |= 0x01;
                
                break;

            case 0x031 :

                if (can_msg_bestpos.header.stamp == bf_bestpos)
                {
                    can_msg_bestpos.header.stamp = ros::Time::now();
                }

                temp = big32(&message.DATA[0]);
                memcpy(&can_msg_bestpos.WGS84_Alt, &temp, sizeof(float));

                temp = big32(&message.DATA[4]);
                memcpy(&can_msg_bestpos.NED_N, &temp, sizeof(float));

                bitFlagBESTPOS |= 0x02;
                
                break;

            case 0x032 :

                if (can_msg_bestpos.header.stamp == bf_bestpos)
                {
                    can_msg_bestpos.header.stamp = ros::Time::now();
                }

                temp = big32(&message.DATA[0]);
                memcpy(&can_msg_bestpos.NED_E, &temp, sizeof(float));

                temp = big32(&message.DATA[4]);
                memcpy(&can_msg_bestpos.NED_D, &temp, sizeof(float));

                bitFlagBESTPOS |= 0x04;
                
                break;

            case 0x033 :

                if (can_msg_bestpos.header.stamp == bf_bestpos)
                {
                    can_msg_bestpos.header.stamp = ros::Time::now();
                }

                temp = big32(&message.DATA[0]);
                memcpy(&can_msg_bestpos.STD_DEV_LAT, &temp, sizeof(float));

                temp = big32(&message.DATA[4]);
                memcpy(&can_msg_bestpos.STD_DEV_LON, &temp, sizeof(float));

                bitFlagBESTPOS |= 0x08;
                
                break;

            case 0x034 :

                if (can_msg_bestpos.header.stamp == bf_bestpos)
                {
                    can_msg_bestpos.header.stamp = ros::Time::now();
                }

                temp = big32(&message.DATA[0]);
                memcpy(&can_msg_bestpos.STD_DEV_ALT, &temp, sizeof(float));

                can_msg_bestpos.POSTYPE = big32(&message.DATA[4]);


                bitFlagBESTPOS |= 0x10;
                
                break;
            /** @SDINS Begin *///////////////////////////////////////////////////
            case 0x040 :

                if (can_msg_sdins.header.stamp == bf_sdins)
                {
                    can_msg_sdins.header.stamp = ros::Time::now();
                }

                temp = big32(&message.DATA[0]);
                memcpy(&can_msg_sdins.WGS84_Lat, &temp, sizeof(float));

                temp = big32(&message.DATA[4]);
                memcpy(&can_msg_sdins.WGS84_Lon, &temp, sizeof(float));

                bitFlagSDINS |= 0x01;
                
                break;

            case 0x041 :

                if (can_msg_sdins.header.stamp == bf_sdins)
                {
                    can_msg_sdins.header.stamp = ros::Time::now();
                }

                temp = big32(&message.DATA[0]);
                memcpy(&can_msg_sdins.WGS84_Alt, &temp, sizeof(float));

                temp = big32(&message.DATA[4]);
                memcpy(&can_msg_sdins.NED_N, &temp, sizeof(float));

                bitFlagSDINS |= 0x02;
                
                break;

            case 0x042 :

                if (can_msg_sdins.header.stamp == bf_sdins)
                {
                    can_msg_sdins.header.stamp = ros::Time::now();
                }

                temp = big32(&message.DATA[0]);
                memcpy(&can_msg_sdins.NED_E, &temp, sizeof(float));

                temp = big32(&message.DATA[4]);
                memcpy(&can_msg_sdins.NED_D, &temp, sizeof(float));

                bitFlagSDINS |= 0x04;
                
                break;

            }
            /** @Novatel End *///////////////////////////////////////////////////
            


            if (bitFlagM8P == 0x1F)
            {
                bitFlagM8P = 0x00;
                can_pub1.publish(can_msg_rtk);
                can_msg_rtk.header.stamp = bf_rtk;
            }

            if (bitFlagVN300 == 0x7FFF)
            {
                bitFlagVN300 = 0x0000;
                can_pub2.publish(can_msg_vn300);
                can_msg_vn300.header.stamp = bf_vn300;
            }

            if (bitFlagBESTPOS == 0x1F)
            {
                bitFlagBESTPOS = 0x00;
                can_pub3.publish(can_msg_bestpos);
                can_msg_bestpos.header.stamp = bf_bestpos;
            }

            if ( bitFlagSDINS == 0x07)
            {
                bitFlagSDINS = 0x00;
                can_pub4.publish(can_msg_sdins);
                can_msg_sdins.header.stamp = bf_sdins;   

            }



        }
    }

private :
    // can object
    int can_fd;
    fd_set can_fds;
    sans_can::sans_can_msgs_rtk         can_msg_rtk;
    sans_can::sans_can_msgs_vn300       can_msg_vn300;
    sans_can::sans_can_msgs_bestpos     can_msg_bestpos;
    sans_can::sans_can_msgs_sdins       can_msg_sdins;

public:


    CANNode(ros::NodeHandle _nh)
    {
        can_pub1 = nh.advertise<sans_can::sans_can_msgs_rtk>("sans_can_msg_rtk", 1000);
        can_pub2 = nh.advertise<sans_can::sans_can_msgs_vn300>("sans_can_msg_vn300", 1000);
        can_pub3 = nh.advertise<sans_can::sans_can_msgs_bestpos>("sans_can_msg_bestpos", 1000);
        can_pub4 = nh.advertise<sans_can::sans_can_msgs_sdins>("sans_can_msg_sdins", 1000);

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

