
#include "can_usb.hpp"


serial::Serial serial_; // 串口实例
std::string data_;

int float_to_byte(float data)
{

    unsigned int longdata = 0;
    longdata = *((unsigned int *)&data);
    return longdata;
}

void write_serial_normal(const cyber_gear_can_t *frame)
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

    printf("command frame:\n");
    for (uint8_t idx = 0; idx < 16; idx++)
    {
        printf("%x ", data[idx]);
    }
    printf("\n");

    serial_.write((uint8_t *)data, 16);
}

void write_serial_little_endian(const cyber_gear_can_t *frame)
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

void close_serial()
{
    std::cout << "Close motor device" << std::endl;
    if (serial_.isOpen())
    {
        serial_.close();
    }
}

void read_serial()
{
    uint8_t data_len = 0;
    while (serial_.isOpen())
    {
        data_len = serial_.available();
        if (data_len > 0)
        {
            uint8_t feedback[100];
            serial_.read(feedback, data_len);
            {
                printf("motor feedback:\n");
                for (uint8_t i = 0; i < data_len; i++)
                {
                    printf("%d  %x\n", i, feedback[i]);
                }
            }
            break;
        }
        usleep(1000);
    }
}

void open_serial(std::string port_, int baudrate_)
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

void select_mode(uint16_t mode, uint8_t motor_id)
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

void enable_motor(uint8_t motor_id)
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

void stop_motor(uint8_t motor_id)
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

void control_motion(uint8_t motor_id, float kp, float kd, float torque, float pos, float vel)
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

void control_pos(uint8_t motor_id, float pos, float spd)
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

void control_vel(uint8_t motor_id, float spd)
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
