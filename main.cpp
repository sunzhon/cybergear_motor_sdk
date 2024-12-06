//
//  main.c
//
//  Created by Eric Wu on 2023/9/4.
//  Modified by Tao Sun on 2024/12/5
//
#include <iostream>
#include <stdio.h>
#include <string.h>

#include <serial/serial.h>
extern "C"{
#include <cyber_gear_protocol.h>
}


std::string port_ = "/dev/ttyACM0";                      //串口端口
 int baudrate_ = 115200;                          //波特率
 serial::Serial serial_;                 //串口实例
 std::string data_;

void write_serial(const cyber_gear_can_t * frame){
	uint8_t data[4+13];
	// serial frame head and end
	data[0] = 0x41;
	data[1] = 0x54;
	data[15] = 0x0d;
	data[16] = 0x0a;
	// put can frame into serial frame
	data[3] = frame->can_id.bytes[0];
	data[4] = frame->can_id.bytes[1];
	data[5] = frame->can_id.bytes[2];
	data[6] = frame->can_id.bytes[3];

	data[7] = frame->can_data.bytes[0];
	data[8] = frame->can_data.bytes[1];
	data[9] = frame->can_data.bytes[2];
	data[10] = frame->can_data.bytes[3];
	data[11] = frame->can_data.bytes[4];
	data[12] = frame->can_data.bytes[5];
	data[13] = frame->can_data.bytes[6];
	data[14] = frame->can_data.bytes[7];
	
	printf("command frame:\n");
	for(uint8_t idx=0;idx<17;idx++){
		printf("%x ", data[idx]);
	}
	printf("\n");

	serial_.write((uint8_t*)data,17);
}



 void close_serial(){
	 std::cout<<"Close yesense device"<<std::endl;                                                                                           
	 if(serial_.isOpen())                                                                                                                    
	 {                                                                                                                                       
		 serial_.close();
	 }
 }


void read_serial(){
                                                                
      if (serial_.available())                                                                                                
      {                                                                                                                       
              data_ = serial_.read(serial_.available());                                                                      
	      
                
              {                                                                                                               
			//boost::mutex::scoped_lock lock(m_mutex_data_buffer_);                                                   
			      
		      printf("sensory feedback:\n");
                      for(uint8_t i=0;i<data_.length();i++)                                                                   
                      {                                                                                                       
                              //data_buffer_ptr_->push_back(data_[i]);                                                          
			      printf("%x",data_[i]);
                      }                                                                                                       
		      printf("\n");
              } 
		      
                                                                                                                              
      } 
}




void open_serial(std::string port_, int baudrate_){
	try
	{
		std::cout <<  "Sucessfully open imu device:" << port_.c_str() << ", buad rate:"<<baudrate_<<  std::endl;
		serial_.setPort(port_);
		serial_.setBaudrate(baudrate_);
		serial::Timeout to = serial::Timeout::simpleTimeout(1000);
		serial_.setTimeout(to);
		serial_.open();
	}
	catch (serial::IOException &e)
	{
		std::cout <<  "Unable to open imu device:" << port_.c_str() << ", buad rate:"<<baudrate_<<  std::endl;
	}


	if (serial_.isOpen())
	{
		std::cout<<"Serial port: " << serial_.getPort().c_str() << " is opened and initialized" <<std::endl;

	}
}                  



int main(int argc, const char * argv[]) {
	// open serial 
	open_serial(port_, baudrate_);

	cyber_gear_can_t frame;
	cyber_gear_can_init(&frame);
	cyber_gear_set_can_id_host_can_id(&frame, 1);
	cyber_gear_set_can_id_target_can_id(&frame, 127);

	printf("Fetch current motor status\n");
	cyber_gear_set_can_id_communication_type(&frame, COMMUNICATION_FETCH_DEVICE_ID);
	cyber_gear_can_dump(&frame);

	//send frame to motor via serial port
	write_serial(&frame);
	read_serial();
	printf("data:");
	close_serial();
	
	/*

	printf("Set Run mode I\n");
	bzero(frame.can_data.bytes, 8);
	cyber_gear_build_parameter_write_frame_with_int_value(&frame, PARAMETER_RUN_MODE, 2);
	cyber_gear_can_dump(&frame);

	printf("Enable Motor\n");
	bzero(frame.can_data.bytes, 8);
	cyber_gear_set_can_id_communication_type(&frame, COMMUNICATION_ENABLE_DEVICE);
	cyber_gear_can_dump(&frame);

	printf("Set SPD_REF\n");
	bzero(frame.can_data.bytes, 8);
	cyber_gear_build_parameter_write_frame_with_float_value(&frame, PARAMETER_SPD_REF, 30.0);
	cyber_gear_can_dump(&frame);    

	*/
	return 0;
}




