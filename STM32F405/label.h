#pragma once
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/*
ͷ�ļ��в�Ҫ����ȫ�ֱ������壬�ɽ��к궨��
ȫ�ֱ�����extern����������ͷ�ļ��������multiple defined
*/

#define CAN1_MOTOR_NUM 5
#define CAN2_MOTOR_NUM 3

#define CHASSIS_MOTOR_NUM 4
#define PANTILE_MOTOR_NUM 2
#define SHOOTER_MOTOR_NUM 2
#define SUPPLY_MOTOR_NUM 1

#define RcQueueHandle Uart2QueueHandler
#define ImuQueueHandle Uart5QueueHandler
#define JudgementQueueHandle Uart3QueueHandler

#define degreeToMechanical(a) ((a)*8192.f/360.f)
#define mechanicalToDegree(a) ((a)*360.f/8192.f)

#ifndef PI
#define PI 3.1415926
#endif // !PI


class PARAMETER
{
public:

	float pitch_max{}, imu_pitch_max{}, pitch_min{}, imu_pitch_min{}, orgin_pitch{}, initial_pitch{}, initial_yaw{};
	int32_t ace_speed{}, max_speed{}, rota_speed{};
	int32_t pitch_speed{}, yaw_speed{};

	PARAMETER& Init();


};


//�������ȼ�
#define START_TASK_PRIO		1
//�����ջ��С	
#define START_STK_SIZE 		128
//������
extern TaskHandle_t StartTask_Handler;
//������

#define LED_TASK_PRIO		1
#define LED_STK_SIZE 		128
extern TaskHandle_t LedTask_Handler;

#define DECODE_TASK_PRIO		3
#define DECODE_STK_SIZE 		128
extern TaskHandle_t DecodeTask_Handler;

#define RC_TASK_PRIO		3	
#define RC_STK_SIZE 		256  
extern TaskHandle_t RcTask_Handler;

#define CONTROL_TASK_PRIO		2
#define CONTROL_STK_SIZE 		128  
extern TaskHandle_t ControlTask_Handler;

#define MOTOR_TASK_PRIO		2
#define MOTOR_STK_SIZE 		256  
extern TaskHandle_t MotorTask_Handler;

#define CANTX_TASK_PRIO		2
#define CANTX_STK_SIZE 		256 
extern TaskHandle_t CanTxTask_Handler;

extern PARAMETER para;