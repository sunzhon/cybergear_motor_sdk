
#include "can_usb.hpp"


//std::string data_;
//extern "C" {
int float_to_byte(float data)
{

    unsigned int longdata = 0;
    longdata = *((unsigned int *)&data);
    return longdata;
}
//}


CyberGearCan::CyberGearCan(std::string port_, int baudrate_, std::vector<int> motor_ids_, int mode_){

	// open serial
	std::cout<<"Open serial port " << port_ <<std::endl;
	std::cout<<"Serial port baudrate " << baudrate_ <<std::endl;
	open_serial(port_, baudrate_);
	motor_mode = mode_;
	motor_ids.resize(motor_ids_.size());
	// set motor mode and enable motors
	printf("Select Motor Mode\n");
	for (uint8_t idx=0; idx<motor_ids_.size(); idx++){
		motor_ids[idx]=motor_ids_[idx];
		if(mode_>0 && mode_<=CONTROL_MODE_CUR){
			//set motion mode, here is velocity mode
			select_mode(mode_/*motor mode*/, motor_ids_[idx]/*motor id*/);
		}
		usleep(250);
		// cyber_gear_can_t frame;
		printf("Enable Motor\n");
		enable_motor(motor_ids_[idx]);
		usleep(250);
	}

}

void CyberGearCan::move(uint8_t motor_id, float kp, float kd, float torque, float pos, float vel){
	if(motor_mode == 0){
		control_motion(motor_id, kp, kd, torque, pos, vel);// this status do not need to set CONTORL_MODE_XX
	}
	else if(motor_mode==CONTROL_MODE_POS){
		control_pos(motor_id, pos, vel);
	}
	else if(motor_mode==CONTROL_MODE_VEL){
		control_vel(motor_id, vel);
	}
	else{
		printf("motor_mode is wrong!");
		exit(-1);
	}

}


void CyberGearCan::stop(){

	printf("Stop Motor\n");
	for(uint8_t idx=0;idx<motor_ids.size();idx++){
		stop_motor(motor_ids[idx]);
		usleep(250);
	}
	close_serial();
}


void CyberGearCan::write_serial_normal(const cyber_gear_can_t *frame)
{

    uint8_t data[4 + 12];
    // serial frame head and end
    data[0] = 0xaa;
    data[1] = 0x01;
    data[2] = 0x00;
    data[3] = 0x08;
    // put can frame into serial frame
    data[4] = frame->can_id.bytes[3];
    data[5] = frame->can_id.bytes[2];
    data[6] = frame->can_id.bytes[1];
    data[7] = frame->can_id.bytes[0];

    data[8] = frame->can_data.bytes[1];
    data[9] = frame->can_data.bytes[0];
    data[10] = frame->can_data.bytes[3];
    data[11] = frame->can_data.bytes[2];
    data[12] = frame->can_data.bytes[5];
    data[13] = frame->can_data.bytes[4];
    data[14] = frame->can_data.bytes[7];
    data[15] = frame->can_data.bytes[6];

    /*
    printf("command frame:\n");
    for (uint8_t idx = 0; idx < 16; idx++)
    {
        printf("%x ", data[idx]);
    }
    printf("\n");
    */

    serial_.write((uint8_t *)data, 16);
}

void CyberGearCan::write_serial_little_endian(const cyber_gear_can_t *frame)
{
    uint8_t data[4 + 12];
    // serial frame head and end
    data[0] = 0xaa;
    data[1] = 0x01;
    data[2] = 0x00;
    data[3] = 0x08;
    // put can frame into serial frame
    data[4] = frame->can_id.bytes[3];
    data[5] = frame->can_id.bytes[2];
    data[6] = frame->can_id.bytes[1];
    data[7] = frame->can_id.bytes[0];

    data[8] = frame->can_data.bytes[0];
    data[9] = frame->can_data.bytes[1];
    data[10] = frame->can_data.bytes[2];
    data[11] = frame->can_data.bytes[3];
    data[12] = frame->can_data.bytes[4];
    data[13] = frame->can_data.bytes[5];
    data[14] = frame->can_data.bytes[6];
    data[15] = frame->can_data.bytes[7];

    printf("command frame:\n");
    for (uint8_t idx = 0; idx < 16; idx++)
    {
        printf("%x ", data[idx]);
    }
    printf("\n");

    serial_.write((uint8_t *)data, 16);
}

void CyberGearCan::close_serial()
{
    std::cout << "Close motor device" << std::endl;
    if (serial_.isOpen())
    {
        serial_.close();
    }
}

void CyberGearCan::read_serial()
{
	uint8_t data_len = 0;
	cyber_gear_can_t frame;
	cyber_gear_can_communication_type_t comm_type;
	uint8_t feedback[100];
	while (serial_.isOpen())
	{
		data_len = serial_.available();
		if (data_len > 0)
		{
			serial_.read(feedback, data_len);
			if(feedback[0]==0xaa && feedback[1]==0x01 && feedback[3]==0x08){

				for(uint8_t idx=0; idx<4;idx++){
					frame.can_id.bytes[3-idx]=feedback[4+idx];
				}

				for(uint8_t idx=0; idx<8;idx++){
					frame.can_data.bytes[idx] = feedback[8+idx];

				}

				comm_type = cyber_gear_get_can_id_communication_type(&frame);
				if(comm_type == COMMUNICATION_STATUS_REPORT){
					motor_status = cyber_gear_parse_motor_status_frame(&frame);
					/*
					printf("host can id: %i \n",motor_status.host_can_id); // 目标电机 CAN ID
					printf("motor can id: %i \n",motor_status.motor_can_id); // 目标电机 CAN ID
					printf("torques: %f \n", motor_status.current_torque); // 力矩 [-12, 12] 单位 N/m
					printf("pos: %f\n", motor_status.current_location); // 当前角度 [-4pi, 4pi]
					printf("vel: %f\n", motor_status.current_speed); // 当前角速度 [-30rad/s, 30rad/s]
					printf("temp: %f\n", motor_status.current_temperature); // 当前温度:Temp(摄氏度) * 10
					*/

					if(motor_status.has_calibration_error) // 标定错误
						printf(" motor: %d calibration error \n", motor_status.motor_can_id);
					//int has_hall_encode_error; // HALL 编码故障
					if(motor_status.has_calibration_error) // 标定错误
						printf(" motor: %d calibration error \n", motor_status.motor_can_id);
					//int has_magnetic_encoding_error; // 磁编码错误
					if(motor_status.has_magnetic_encoding_error) // 标定错误
						printf(" motor: %d magnetic encodering error \n", motor_status.motor_can_id);
					//int has_over_temperature; // 过温故障
					if(motor_status.has_over_temperature) // 标定错误
						printf(" motor: %d over tempature error \n", motor_status.motor_can_id);
					//int has_over_current; // 过流故障
					if(motor_status.has_over_current) // 标定错误
						printf(" motor: %d ove current error \n", motor_status.motor_can_id);
					//int has_undervoltage; // 欠压故障
					if(motor_status.has_undervoltage) // 标定错误
						printf(" motor: %d undervoltage error \n", motor_status.motor_can_id);
					//cyber_gear_motor_mode_t mode_type; // 运行模式
				}
				/*
				   printf("can usb frame:\n");
				   for (uint8_t i = 0; i < data_len; i++)
				   {
				   printf("%d  %x\n", i, feedback[i]);
				   }



				   printf("motor status frame:\n");
				   for (uint8_t i = 0; i < 12; i++)
				   {
				   printf("%d  %x\n", i, *((uint8_t*)&frame+i));
				   }
				   */
			}
			break;
		}
		usleep(1000);
	}
}

void CyberGearCan::open_serial(std::string port_, int baudrate_)
{
	/*
	   Open serial port

*/
	try
	{
		std::cout << "Successfully open motor device:" << port_.c_str() << ", baud rate:" << baudrate_ << std::endl;
		serial_.setPort(port_);
		serial_.setBaudrate(baudrate_);
		serial::Timeout to = serial::Timeout::simpleTimeout(1000);
		serial_.setTimeout(to);
		serial_.open();
	}
	catch (serial::IOException &e)
	{
		std::cout << "Unable to open imu device:" << port_.c_str() << ", baud rate:" << baudrate_ << std::endl;
	}

	if (serial_.isOpen())
	{
		std::cout << "Serial port: " << serial_.getPort().c_str() << " is opened and initialized" << std::endl;
	}
}

void CyberGearCan::select_mode(uint16_t mode, uint8_t motor_id)
{
    cyber_gear_can_t frame;
    frame.can_data.value = 0x00;
    frame.can_id.value = 0x00;
    frame.can_id.bytes[0] = motor_id;
    frame.can_id.bytes[1] = 0x00;
    frame.can_id.bytes[2] = 0x00;
    frame.can_id.bytes[3] = 0x00;
    cyber_gear_build_parameter_write_frame_with_int_value(&frame, PARAMETER_RUN_MODE, mode);
    write_serial_little_endian(&frame);
}

void CyberGearCan::enable_motor(uint8_t motor_id)
{

    cyber_gear_can_t frame;
    frame.can_data.value = 0x00;
    frame.can_id.value = 0x00;
    frame.can_id.bytes[0] = motor_id;
    frame.can_id.bytes[1] = 0x00;
    frame.can_id.bytes[2] = 0x00;
    frame.can_id.bytes[3] = 0x00;
    cyber_gear_set_can_id_communication_type(&frame, COMMUNICATION_ENABLE_DEVICE);
    write_serial_normal(&frame);
    read_serial();
}

void CyberGearCan::stop_motor(uint8_t motor_id)
{
    /*
    Stop motor
    */

    cyber_gear_can_t frame;
    frame.can_data.value = 0x00;
    frame.can_id.value = 0x00;
    frame.can_id.bytes[0] = motor_id;
    frame.can_id.bytes[1] = 0x00;
    frame.can_id.bytes[2] = 0x00;
    frame.can_id.bytes[3] = 0x00;
    cyber_gear_set_can_id_communication_type(&frame, COMMUNICATION_DISABLE_DEVICE);
    write_serial_normal(&frame);
    read_serial();
}

void CyberGearCan::control_motion(uint8_t motor_id, float kp, float kd, float torque, float pos, float vel)
{
    /*
    Motion control mode
    */
    cyber_gear_can_t frame = {0};
    cyber_gear_motion_control_t control_parameter;
    control_parameter.kd = kd;
    control_parameter.kp = kp;
    control_parameter.motor_can_id = motor_id;
    control_parameter.target_location = pos;
    control_parameter.target_speed = vel;
    control_parameter.torque = torque;
    cyber_gear_build_motion_control_frame(&frame, control_parameter);
    write_serial_normal(&frame);
    read_serial();
}

void CyberGearCan::control_pos(uint8_t motor_id, float pos, float spd)
{
    /*
    Velocity control
    */

    int loc = float_to_byte(pos);
    int vel = float_to_byte(spd);
    cyber_gear_can_t frame;
    frame.can_data.value = 0x00;
    frame.can_id.value = 0x00;
    frame.can_id.bytes[0] = motor_id;
    frame.can_id.bytes[1] = 0x00;
    frame.can_id.bytes[2] = 0x00;
    frame.can_id.bytes[3] = 0x00;
    cyber_gear_build_parameter_write_frame_with_int_value(&frame, PARAMETER_LIMIT_SPD, vel);
    write_serial_little_endian(&frame);
    usleep(300);
    cyber_gear_build_parameter_write_frame_with_int_value(&frame, PARAMETER_LOC_REF, loc);
    write_serial_little_endian(&frame);
    read_serial();
}

void CyberGearCan::control_vel(uint8_t motor_id, float spd)
{
    int vel = float_to_byte(spd);
    cyber_gear_can_t frame;
    frame.can_data.value = 0x00;
    frame.can_id.value = 0x00;
    frame.can_id.bytes[0] = motor_id;
    frame.can_id.bytes[1] = 0x00;
    frame.can_id.bytes[2] = 0x00;
    frame.can_id.bytes[3] = 0x00;
    cyber_gear_build_parameter_write_frame_with_int_value(&frame, PARAMETER_SPD_REF, vel);
    write_serial_little_endian(&frame);
    usleep(300);
    read_serial();
}

