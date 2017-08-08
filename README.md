sans_can Messages

Modified Date : 2017. 08. 08.


#include "sans_can/sans_can_msgs_rtk.h"
#include "sans_can/sans_can_msgs_vn300.h"



Topic

#1 	sans_can_msg_rtk

		Header header
		float32 WGS84_Lat
		float32 WGS84_Lon
		float32 WGS84_Alt
		float32 NED_N
		float32 NED_E
		float32 NED_D
		float32 Hori_Accuracy
		float32 Vert_Accuracy
		uint8 	FixType
		uint8 	Flags
		uint8 	Flags2


#2 	sans_can_msg_vn300


		float32 ROLL
		float32 PITCH
		float32 YAW
		float32 Uncertain_ROLL
		float32 Uncertain_PITCH
		float32 Uncertain_YAW
		float64 WGS84_Lat
		float64 WGS84_Lon
		float64 WGS84_Alt
		float32 NED_N
		float32 NED_E
		float32 NED_D
		float32 VEL_N
		float32 VEL_E
		float32 VEL_D
		float32 COMP_ACC_X
		float32 COMP_ACC_Y
		float32 COMP_ACC_Z
		float32 COMP_GYRO_X
		float32 COMP_GYRO_Y
		float32 COMP_GYRO_Z