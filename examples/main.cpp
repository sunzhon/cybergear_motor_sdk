//
//  main.c
//
//  Created by Eric Wu on 2023/9/4.
//  Modified by Tao Sun on 2024/12/5
//  Modified by Guohui Liu on 2024/12/10
//

#include "can_usb.hpp"



std::string port_ = "/dev/ttyACM0"; // 串口端口
int baudrate_ = 115200;				// 波特率


int main(int argc, const char *argv[])
{

	// open serial
	open_serial(port_, baudrate_);
    	printf("Select Motor Mode\n");
	select_mode(CONTROL_MODE_VEL,0x7f);//set motion mode, here is velocity mode
	usleep(250);
	// cyber_gear_can_t frame;
	printf("Enable Motor\n");
	enable_motor(0x7f);
	usleep(250);
	printf("Control Motor\n");
	// control_motion(0x7f, 1.0, 0.02, 0.0, 3, 0.0);// this status do not need to set CONTORL_MODE_XX
	//control_pos(0x7f, 1,1.0);
	control_vel(0x7f,1.0);
	sleep(3);// move 3 seconds

	printf("Stop Motor\n");
	stop_motor(0x7f);
	usleep(250);
	close_serial();
	return 0;
}
