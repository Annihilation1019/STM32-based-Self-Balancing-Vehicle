#include "PID.h"
#include "Encoder.h"
#include "mpu6050.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "tim.h"
#include "Motor.h"
#include "Bluetooth.h"
#include "HC_SR04.h"

volatile float Vertical_Kp = 200.0f; // 直立环P参数 0 ~ 1000
volatile float Vertical_Kd = 0.0f;   // 直立环D参数 -10 ~ 0
volatile float Velocity_Kp = 0.0f;   // 速度环P参数 0 ~ 1
volatile float Velocity_Ki = 0.0f;   // 速度环I参数 根据工程经验 Kp / 200
volatile float Turn_Kp = 0.0f;       // 转向环P参数
volatile float Turn_Kd = 0.0f;       // 转向环D参数

extern volatile uint8_t STOP_Flag; // 立即停止标志位

float Pitch, Roll, Yaw;          // 姿态角
short Gyro_X, Gyro_Y, Gyro_Z;    // 陀螺仪数据（角速度）
short aacx, aacy, aacz;          // 加速度计数据
volatile float Med_Angle = 0.0f; // 直立环机械中值角度

int Vertical_Out, Velocity_Out, Turn_Out;       // 直立环、速度环、转向环输出
volatile int Speed_Target = 0, Turn_Target = 0; // 速度环目标速度、转向环目标角度
int MotorA_PWM, MotorB_PWM;                     // 电机A、B的PWM输出

/**
 * @brief 直立环PD控制器
 * @param angle_target 目标角度
 * @param angle 实际角度
 * @param gyro_Y 角速度
 * @retval 电机PWM输出
 */
int Vertical_PD(float angle_target, float angle, float gyro_Y)
{
    float err; // 角度误差
    err = angle_target - angle;
    return Vertical_Kp * err + Vertical_Kd * gyro_Y;
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
    Encoder_In += err_LowOut;
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
        Encoder_In = 0;                                               // 速度环积分清零
        Turn_Target = 0;                                              // 转向环目标角度清零
        Speed_Target = 0;                                             // 速度环目标速度清零
        PID_Param_buf[5] = *(float *)(FLASH_USER_START_ADDR + 5 * 4); // 转向约束开启
        STOP_Flag = 0;
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
    MPU_Get_Accelerometer(&aacx, &aacy, &aacz);   // 获取加速度计数据
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
            PID_Param_buf[5] = 0.0f; // 转向约束关闭
            LEFT_Flag = 0;
        }
        if (RIGHT_Flag)
        {
            Turn_Target += 30;
            PID_Param_buf[5] = 0.0f; // 转向约束关闭
            RIGHT_Flag = 0;
        }

        /* 防撞 */
        if (distance < 20)
        {
            STOP_Flag = 1;
        }
        /* 限幅 */
        if (Speed_Target > SPEED_MAX)
            Speed_Target = SPEED_MAX;
        else if (Speed_Target < -SPEED_MAX)
            Speed_Target = -SPEED_MAX;
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
}