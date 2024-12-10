#include <unistd.h>
#include <iostream>
#include <stdio.h>
#include <string.h>
#include <serial/serial.h>
extern "C"
{
#include <cyber_gear_protocol.h>

enum
{
    /// 电机位置控制模式
    CONTROL_MODE_POS = 0x01,
    /// 电机速度控制模式
    CONTROL_MODE_VEL,
    /// 电机电流控制模式
    CONTROL_MODE_CUR,
};
/* float型数据转化为int型数据
 * @param: data 要转化float型数据
 * @return: int 值数据
 * */
int float_to_byte(float data);

/* can_usb模块协议数据帧正常写入
 * @param: frame 要设置CAN数组首地址
 * */
void write_serial_normal(const cyber_gear_can_t *frame);

/* can_usb模块协议数据帧小端写入
 * @param: frame 要设置CAN数组首地址
 * */
void write_serial_little_endian(const cyber_gear_can_t *frame);

/* 串口关闭* */
void close_serial();
/* 保存串口接收数据* */
void read_serial();

/* 串口打开
 * @param: port_ 设置串口名
 * @param: baudrate_ 设置串口波特率
 * */
void open_serial(std::string port_, int baudrate_);
/* 选择电机控制模式
 * @param: mode 电机工作模式
 * @param: motor_id 工作电机ID
 * */
void select_mode(uint16_t mode, uint8_t motor_id);
/* 使能电机
 * @param: motor_id 工作电机ID
 * */
void enable_motor(uint8_t motor_id);
/* 停止电机
 * @param: motor_id 工作电机ID
 * */
void stop_motor(uint8_t motor_id);
/* 电机运动控制
 * @param: motor_id 工作电机ID
 * @param: kp 电机参数kp
 * @param: kd 电机参数kd
 * @param: torque 电机前馈力矩
 * @param: pos 电机目标位置
 * @param: vel 电机目标速度
 * */
void control_motion(uint8_t motor_id, float kp, float kd, float torque, float pos, float vel);
/* 电机位置环控制
 * @param: motor_id 工作电机ID
 * @param: pos 电机目标位置
 * @param: spd 电机速度限制
 * */
void control_pos(uint8_t motor_id, float pos, float spd);
/* 电机速度环控制
 * @param: motor_id 工作电机ID
 * @param: spd 电机目标速度
 * */
void control_vel(uint8_t motor_id, float spd);

}
