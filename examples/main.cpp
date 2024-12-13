//
//  main.c
//
//  Created by Eric Wu on 2023/9/4.
//  Modified by Tao Sun on 2024/12/5
//  Modified by Guohui Liu on 2024/12/10
//

#include "can_usb.hpp"



std::string port = "/dev/ttyACM0"; // 串口端口
int baudrate = 115200;				// 波特率
std::vector<int> ids;
std::shared_ptr<CyberGearCan> cybergear;

int main(int argc, const char *argv[])
{
	ids.resize(1);
	ids[0] = 0x7f;
	cybergear = std::make_shared<CyberGearCan>(port, baudrate,ids);

	//move(uint8_t motor_id, float kp, float kd, float torque, float pos, float vel){
	cybergear->move(ids[0], 1.0,  0.02,  0.0, 0.5, 0.0);
	printf("Control Motor\n");
	sleep(3);// move 3 seconds

	cybergear->stop();

	return 0;
}
