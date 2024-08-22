#include "stm32f1xx_hal.h"
#include "mpu6050.h"

/**
 * @brief MPU6050初始化
 * @return uint8_t 返回0表示初始化成功，返回1表示初始化失败
 */
uint8_t MPU_Init(void)
{
    uint8_t res;

    MPU_IIC_Init();                          // 初始化IIC总线
    MPU_Write_Byte(MPU_PWR_MGMT1_REG, 0X80); // 复位MPU6050
    delay_ms(100);
    MPU_Write_Byte(MPU_PWR_MGMT1_REG, 0X00); // 唤醒MPU6050
    MPU_Set_Gyro_Fsr(3);                     // 陀螺仪传感器,±2000dps
    MPU_Set_Accel_Fsr(0);                    // 加速度传感器,±2g
    MPU_Set_Rate(100);                       // 设置采样率100Hz
    MPU_Write_Byte(MPU_INT_EN_REG, 0X00);    // 关闭所有中断
    MPU_Write_Byte(MPU_USER_CTRL_REG, 0X00); // I2C主模式关闭
    MPU_Write_Byte(MPU_FIFO_EN_REG, 0X00);   // 关闭FIFO
    MPU_Write_Byte(MPU_INTBP_CFG_REG, 0X80); // INT引脚低电平有效
    res = MPU_Read_Byte(MPU_DEVICE_ID_REG);
    if (res == MPU_ADDR) // 器件ID正确
    {
        MPU_Write_Byte(MPU_PWR_MGMT1_REG, 0X01); // 设置CLKSEL,PLL X轴为参考
        MPU_Write_Byte(MPU_PWR_MGMT2_REG, 0X00); // 加速度与陀螺仪都工作
        MPU_Set_Rate(100);                       // 设置采样率为100Hz
    }
    else
        return 1;
    return 0;
}

/**
 * @brief 设置MPU6050陀螺仪传感器满量程范围
 * @param fsr 0,±250dps;1,±500dps;2,±1000dps;3,±2000dps
 * @return uint8_t 0,设置成功;其他,设置失败
 */
uint8_t MPU_Set_Gyro_Fsr(uint8_t fsr)
{
    return MPU_Write_Byte(MPU_GYRO_CFG_REG, fsr << 3); // 设置陀螺仪满量程范围
}

/**
 * @brief 设置MPU6050加速度传感器满量程范围
 * @param fsr 0,±2g;1,±4g;2,±8g;3,±16g
 * @return uint8_t 0,设置成功;其他,设置失败
 */
uint8_t MPU_Set_Accel_Fsr(uint8_t fsr)
{
    return MPU_Write_Byte(MPU_ACCEL_CFG_REG, fsr << 3); // 设置加速度传感器满量程范围
}

/**
 * @brief 设置MPU6050的数字低通滤波器
 * @param lpf 数字低通滤波频率(Hz)
 * @return uint8_t 0,设置成功;其他,设置失败
 */
uint8_t MPU_Set_LPF(uint16_t lpf)
{
    uint8_t data = 0;
    if (lpf >= 188)
        data = 1;
    else if (lpf >= 98)
        data = 2;
    else if (lpf >= 42)
        data = 3;
    else if (lpf >= 20)
        data = 4;
    else if (lpf >= 10)
        data = 5;
    else
        data = 6;
    return MPU_Write_Byte(MPU_CFG_REG, data); // 设置数字低通滤波器
}

/**
 * @brief 设置MPU6050的采样率(假定Fs=1KHz)
 * @param rate 4~1000(Hz)
 * @return uint8_t 0,设置成功;其他,设置失败
 */
uint8_t MPU_Set_Rate(uint16_t rate)
{
    uint8_t data;
    if (rate > 1000)
        rate = 1000;
    if (rate < 4)
        rate = 4;
    data = 1000 / rate - 1;
    data = MPU_Write_Byte(MPU_SAMPLE_RATE_REG, data); // 设置数字低通滤波器
    return MPU_Set_LPF(rate / 2);                     // 自动设置LPF为采样率的一半
}

/**
 * @brief 获取温度值
 * @return short 温度值(扩大了100倍)
 */
short MPU_Get_Temperature(void)
{
    uint8_t buf[2];
    short raw;
    float temp;
    MPU_Read_Len(MPU_ADDR, MPU_TEMP_OUTH_REG, 2, buf);
    raw = ((uint16_t)buf[0] << 8) | buf[1];
    temp = 36.53 + ((double)raw) / 340;
    return temp * 100;
    ;
}

/**
 * @brief 获取陀螺仪值(原始值)
 * @param gx 陀螺仪x轴的原始读数(带符号)
 * @param gy 陀螺仪y轴的原始读数(带符号)
 * @param gz 陀螺仪z轴的原始读数(带符号)
 * @return uint8_t 0,成功;其他,错误代码
 */
uint8_t MPU_Get_Gyroscope(short *gx, short *gy, short *gz)
{
    uint8_t buf[6], res;
    res = MPU_Read_Len(MPU_ADDR, MPU_GYRO_XOUTH_REG, 6, buf);
    if (res == 0)
    {
        *gx = ((uint16_t)buf[0] << 8) | buf[1];
        *gy = ((uint16_t)buf[2] << 8) | buf[3];
        *gz = ((uint16_t)buf[4] << 8) | buf[5];
    }
    return res;
    ;
}

/**
 * @brief 获取加速度值(原始值)
 * @param ax 加速度x轴的原始读数(带符号)
 * @param ay 加速度y轴的原始读数(带符号)
 * @param az 加速度z轴的原始读数(带符号)
 * @return uint8_t 0,成功;其他,错误代码
 */
uint8_t MPU_Get_Accelerometer(short *ax, short *ay, short *az)
{
    uint8_t buf[6], res;
    res = MPU_Read_Len(MPU_ADDR, MPU_ACCEL_XOUTH_REG, 6, buf);
    if (res == 0)
    {
        *ax = ((uint16_t)buf[0] << 8) | buf[1];
        *ay = ((uint16_t)buf[2] << 8) | buf[3];
        *az = ((uint16_t)buf[4] << 8) | buf[5];
    }
    return res;
    ;
}

/**
 * @brief IIC连续写
 * @param addr 器件地址
 * @param reg 寄存器地址
 * @param len 写入长度
 * @param buf 数据区
 * @return uint8_t 0,正常;其他,错误代码
 */
uint8_t MPU_Write_Len(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)
{
    uint8_t i;
    MPU_IIC_Start();
    MPU_IIC_Send_Byte((addr << 1) | 0); // 发送器件地址+写命令
    if (MPU_IIC_Wait_Ack())             // 等待应答
    {
        MPU_IIC_Stop();
        return 1;
    }
    MPU_IIC_Send_Byte(reg); // 写寄存器地址
    MPU_IIC_Wait_Ack();     // 等待应答
    for (i = 0; i < len; i++)
    {
        MPU_IIC_Send_Byte(buf[i]); // 发送数据
        if (MPU_IIC_Wait_Ack())    // 等待ACK
        {
            MPU_IIC_Stop();
            return 1;
        }
    }
    MPU_IIC_Stop();
    return 0;
}

/**
 * @brief IIC连续读
 * @param addr 器件地址
 * @param reg 寄存器地址
 * @param len 读取长度
 * @param buf 数据区
 * @return uint8_t 0,正常;其他,错误代码
 */
uint8_t MPU_Read_Len(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf)
{
    MPU_IIC_Start();
    MPU_IIC_Send_Byte((addr << 1) | 0); // 发送器件地址+写命令
    if (MPU_IIC_Wait_Ack())             // 等待应答
    {
        MPU_IIC_Stop();
        return 1;
    }
    MPU_IIC_Send_Byte(reg); // 写寄存器地址
    MPU_IIC_Wait_Ack();     // 等待应答
    MPU_IIC_Start();
    MPU_IIC_Send_Byte((addr << 1) | 1); // 发送器件地址+读命令
    MPU_IIC_Wait_Ack();                 // 等待应答
    while (len)
    {
        if (len == 1)
            *buf = MPU_IIC_Read_Byte(0); // 读数据,发送nACK
        else
            *buf = MPU_IIC_Read_Byte(1); // 读数据,发送ACK
        len--;
        buf++;
    }
    MPU_IIC_Stop(); // 产生一个停止条件
    return 0;
}

/**
 * @brief IIC写一个字节
 * @param reg 寄存器地址
 * @param data 数据
 * @return uint8_t 0,正常;其他,错误代码
 */
uint8_t MPU_Write_Byte(uint8_t reg, uint8_t data)
{
    MPU_IIC_Start();
    MPU_IIC_Send_Byte((MPU_ADDR << 1) | 0); // 发送器件地址+写命令
    if (MPU_IIC_Wait_Ack())                 // 等待应答
    {
        MPU_IIC_Stop();
        return 1;
    }
    MPU_IIC_Send_Byte(reg);  // 写寄存器地址
    MPU_IIC_Wait_Ack();      // 等待应答
    MPU_IIC_Send_Byte(data); // 发送数据
    if (MPU_IIC_Wait_Ack())  // 等待ACK
    {
        MPU_IIC_Stop();
        return 1;
    }
    MPU_IIC_Stop();
    return 0;
}

/**
 * @brief IIC读一个字节
 * @param reg 寄存器地址
 * @return uint8_t 读到的数据
 */
uint8_t MPU_Read_Byte(uint8_t reg)
{
    uint8_t res;
    MPU_IIC_Start();
    MPU_IIC_Send_Byte((MPU_ADDR << 1) | 0); // 发送器件地址+写命令
    MPU_IIC_Wait_Ack();                     // 等待应答
    MPU_IIC_Send_Byte(reg);                 // 写寄存器地址
    MPU_IIC_Wait_Ack();                     // 等待应答
    MPU_IIC_Start();
    MPU_IIC_Send_Byte((MPU_ADDR << 1) | 1); // 发送器件地址+读命令
    MPU_IIC_Wait_Ack();                     // 等待应答
    res = MPU_IIC_Read_Byte(0);             // 读取数据,发送nACK
    MPU_IIC_Stop();                         // 产生一个停止条件
    return res;
}
