#include "PID.h"
#include "Encoder.h"
#include "mpu6050.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "tim.h"
#include "Motor.h"
#include "Bluetooth.h"
#include "HC_SR04.h"
#include "usart.h"
#include <stdio.h>

volatile float Vertical_angle_Kp; // 直立环P参数 0 ~ 1000
volatile float Vertical_angle_Kd; // 直立环D参数 -10 ~ 0
volatile float Vertical_gyro_Kp;  // 直立环角速度P参数 0 ~ 1
volatile float Vertical_gyro_Kd;  // 直立环角速度D参数 -5 ~ 0
volatile float Velocity_Kp;       // 速度环P参数 0 ~ 1
volatile float Velocity_Ki;       // 速度环I参数 根据工程经验 Kp / 200
volatile float Turn_Kp;           // 转向环P参数
volatile float Turn_Kd;           // 转向环D参数 -1 ~ 0
volatile uint8_t Dis_Flag;        // 测距标志位

extern volatile int8_t STOP_Flag; // 立即停止标志位

float Pitch, Roll, Yaw;          // 姿态角
short Gyro_X, Gyro_Y, Gyro_Z;    // 陀螺仪数据（角速度）
volatile float Med_Angle = 0.0f; // 直立环机械中值角度

int Vertical_Out, Velocity_Out, Turn_Out;       // 直立环、速度环、转向环输出
volatile int Speed_Target = 0, Turn_Target = 0; // 速度环目标速度、转向环目标角度
int MotorA_PWM, MotorB_PWM;                     // 电机A、B的PWM输出

/**
 * @brief 直立环二阶PD控制器
 * @param angle_target 目标角度
 * @param angle 实际角度
 * @param gyro_Y 实际角速度
 * @param accy 实际角加速度
 * @retval 电机PWM输出
 */
int Vertical_PD(float angle_target, float angle, float gyro_Y)
{
    /* 计算误差 */
    float err_angle, err_gyro; // 偏差值
    static float err_gyro_last = 0.0f;
    err_angle = angle_target - angle;
    err_gyro = -gyro_Y;
    /* 计算PD输出 */
    int angle_output = Vertical_angle_Kp * err_angle + Vertical_angle_Kd * gyro_Y;
    int gyro_output = Vertical_gyro_Kp * err_gyro + Vertical_gyro_Kd * (err_gyro - err_gyro_last);
    /* 更新误差 */
    err_gyro_last = err_gyro;
    /* 电机PWM输出 */
    return angle_output + gyro_output;
}

/**
 * @brief 速度环PI控制器
 * @param speed_target 目标速度
 * @param speed_A 实际速度（编码器A）
 * @param speed_B 实际速度（编码器B）
 * @retval 目标角度
 */
int Velocity_PI(int speed_target, int speed_A, int speed_B)
{
    int err, err_LowOut;
    static int err_LowOut_Last, Encoder_In;
    const float alpha = 0.7; // 滤波系数
    /* 偏差值 */
    err = speed_A + speed_B - speed_target * 2;
    /* 低通滤波 */
    err_LowOut = (1 - alpha) * err + alpha * err_LowOut_Last;
    err_LowOut_Last = err_LowOut;
    /* 编码器积分 */
    if (Speed_Target != 0 || Turn_Target != 0 || STOP_Flag == 0)
    {
        Encoder_In += err_LowOut;
    }
    /* 积分限幅 */
    if (Encoder_In > 20000)
        Encoder_In = 20000;
    else if (Encoder_In < -20000)
        Encoder_In = -20000;
    if (Turn_Target != 0 || Speed_Target != 0)
    {
        Encoder_In = 0;
    }
    /* 立即停止信号 */
    if (STOP_Flag)
    {
        STOP_Flag = Speed_Target; // 缓冲减速，避免过冲
        Turn_Target = 0;          // 转向环目标角度清零
        if (STOP_Flag > 0)
        {
            Speed_Target--;
        }
        else if (STOP_Flag < 0)
        {
            Speed_Target++;
        }
        else if (STOP_Flag == 0)
        {
            PID_Param_buf[7] = *(float *)(FLASH_USER_START_ADDR + 7 * 4); // 转向约束开启
            Encoder_In = 0;                                               // 速度环积分清零
        }
    }
    /* 速度环计算 */
    return Velocity_Kp * err_LowOut + Velocity_Ki * Encoder_In;
}

/**
 * @brief 转向环PD控制器
 * @param angle_target 目标角度
 * @param gyro_Z 实际角速度
 * @note Kd参数作用为转向约束，需在执行转向时切换为0
 */
int Turn_PD(float angle_target, float gyro_Z)
{
    return Turn_Kp * angle_target + Turn_Kd * gyro_Z;
}

/**
 * @brief PID核心控制
 * @note 速度环 -> 直立环   转向环
 */
void PID_Control(void)
{
    int PWM_Out; // 电机PWM输出
    /* 读取编码器和陀螺仪数据 */
    Encoder_Left = GetEncoderVal(&htim2);
    Encoder_Right = -GetEncoderVal(&htim4);       // 右轮方向相反
    mpu_dmp_get_data(&Pitch, &Roll, &Yaw);        // 获取姿态角
    MPU_Get_Gyroscope(&Gyro_X, &Gyro_Y, &Gyro_Z); // 获取陀螺仪数据
    /* 蓝牙遥控 */
    if (Car_State == ControlMode)
    {
        if (UP_Flag)
        {
            Speed_Target += 5;
            UP_Flag = 0;
        }
        if (DOWN_Flag)
        {
            Speed_Target -= 5;
            DOWN_Flag = 0;
        }
        if (LEFT_Flag)
        {
            Turn_Target -= 30;
            PID_Param_buf[7] = 0.0f; // 转向约束关闭
            LEFT_Flag = 0;
        }
        if (RIGHT_Flag)
        {
            Turn_Target += 30;
            PID_Param_buf[7] = 0.0f; // 转向约束关闭
            RIGHT_Flag = 0;
        }

        /* 防撞 */
        if (distance < 20 && Dis_Flag == 0)
        {
            Dis_Flag = 1;  // 转换进入防撞状态
            STOP_Flag = 1; // 立即停止
        }
        if (Dis_Flag && distance > 30)
        {
            Dis_Flag = 0; // 转换退出防撞状态
        }
        /* 限幅 */
        if (Speed_Target > SPEED_MAX)
            Speed_Target = SPEED_MAX;
        else if (Speed_Target < SPEED_MIN)
            Speed_Target = SPEED_MIN;
        if (Turn_Target > TURN_MAX)
            Turn_Target = TURN_MAX;
        else if (Turn_Target < -TURN_MAX)
            Turn_Target = -TURN_MAX;
    }

    /* 数据转入PID控制器，计算电机输出转速 */
    Velocity_Out = Velocity_PI(Speed_Target, Encoder_Left, Encoder_Right); // 速度环输出目标角度
    Vertical_Out = Vertical_PD(Velocity_Out + Med_Angle, Roll, Gyro_X);    // 直立环输出目标转速
    Turn_Out = Turn_PD(Turn_Target, Gyro_Z);                               // 转向环输出目标角度
    /* 电机PWM输出 */
    PWM_Out = Vertical_Out;
    MotorA_PWM = PWM_Out - Turn_Out;
    MotorB_PWM = PWM_Out + Turn_Out; // Turn_Out为正时，左转，为负时，右转
    /* 电机PWM限幅 */
    Motor_Limit(&MotorA_PWM, &MotorB_PWM);
    /* 电机控制 */
    Motor_Control(MotorA_PWM, MotorB_PWM);
    /* 上位机调试 */
    // printf("%d,%d\n", Gyro_Z, PWM_Out);
}
// 重定义fputc函数
int fputc(int ch, FILE *f)
{
    while ((USART1->SR & 0X40) == 0)
        ; // 循环发送,直到发送完毕
    USART1->DR = (uint8_t)ch;
    return ch;
}
