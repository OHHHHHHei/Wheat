#include "label.h"
#include "taskslist.h"
#include "can.h"
#include "motor.h"
#include "imu.h"
#include "RC.h"
#include "tim.h"
#include "control.h"
#include "led.h"
#include "delay.h"
#include "HTmotor.h"
#include "Power_read.h"
#include "xuc.h"
extern float Kp = 10;
extern float Kd = 0.6;
extern int start_flag;
void TASK::Init()
{
	//创建开始任务
	xTaskCreate((TaskFunction_t)start_task,            //任务函数
		(const char*)"start_task",          //任务名称
		(uint16_t)START_STK_SIZE,        //任务堆栈大小
		(void*)NULL,                  //传递给任务函数的参数
		(UBaseType_t)START_TASK_PRIO,       //任务优先级
		(TaskHandle_t*)&StartTask_Handler);   //任务句柄              
	vTaskStartScheduler();          //开启任务调度
}

/*
开始任务任务函数
*/
void start_task(void* pvParameters)
{
	taskENTER_CRITICAL();           //进入临界区
	//创建任务

	xTaskCreate((TaskFunction_t)ArmTask,
		(const char*)"ArmTask",
		(uint16_t)LED_STK_SIZE,
		(void*)NULL,
		(UBaseType_t)LED_TASK_PRIO,
		(TaskHandle_t*)&LedTask_Handler);

	xTaskCreate((TaskFunction_t)DecodeTask,
		(const char*)"DecodeTask",
		(uint16_t)DECODE_STK_SIZE,
		(void*)NULL,
		(UBaseType_t)DECODE_TASK_PRIO,
		(TaskHandle_t*)&DecodeTask_Handler);

	xTaskCreate((TaskFunction_t)MotorUpdateTask,
		(const char*)"MotorUpdateTask",
		(uint16_t)MOTOR_STK_SIZE,
		(void*)NULL,
		(UBaseType_t)MOTOR_TASK_PRIO,
		(TaskHandle_t*)&MotorTask_Handler);

	xTaskCreate((TaskFunction_t)CanTransmitTask,
		(const char*)"CanTransmitTask",
		(uint16_t)CANTX_STK_SIZE,
		(void*)NULL,
		(UBaseType_t)CANTX_TASK_PRIO,
		(TaskHandle_t*)&CanTxTask_Handler);

	xTaskCreate((TaskFunction_t)ControlTask,
		(const char*)"ControlTask",
		(uint16_t)CONTROL_STK_SIZE,
		(void*)NULL,
		(UBaseType_t)CONTROL_TASK_PRIO,
		(TaskHandle_t*)&ControlTask_Handler);

	vTaskDelete(StartTask_Handler); //删除开始任务
	taskEXIT_CRITICAL();            //退出临界区
}
int CNT = 0;
void MotorUpdateTask(void* pvParameters)
{
	
	while (1)
	{
	TickType_t xlastWakeTime = xTaskGetTickCount();
	
		for (auto& motor : can1_motor)motor.Ontimer(can1.data, can1.temp_data);

		for (auto& motor : can2_motor)motor.Ontimer(can2.data, can2.temp_data);

		DMmotor[0].State_Decode(can2, can2.jointidata).DMmotor_Ontimer(can2, DMmotor[1].Kp, DMmotor[1].Kd, can2.jointpdata[0]);


	vTaskDelayUntil(&xlastWakeTime, pdMS_TO_TICKS(2));//开始执行该任务之后1ms再执行该任务
}
}

void CanTransmitTask(void* pvParameters)
{
	while (true)
	{

		TickType_t xlastWakeTime1 = xTaskGetTickCount();

		switch ((timer.counter++) % 3)
		{
		case 0:
				DMmotor[0].DMmotor_transmit(1);  //发送达妙电机数据
			break;
		case 1:
			can1.Transmit(0x1ff, can1.temp_data + 8); //发送can1的数据 云台
			can2.Transmit(0x1ff, can2.temp_data + 8); //发送can2的数据
			break;
		case 2:
			can1.Transmit(0x200, can1.temp_data); //发送can1的数据  底盘
			can2.Transmit(0x200, can2.temp_data); //发送can2的数据
		default:
			break;
		}
		
		vTaskDelayUntil(&xlastWakeTime1, pdMS_TO_TICKS(1));//开始执行该任务之后1ms再执行该任务

	}
}

void ControlTask(void* pvParameters)
{
	// MODIFIED: 为了实现固定的控制周期，使用vTaskDelayUntil
	TickType_t xlastWakeTime;
	const TickType_t xFrequency = pdMS_TO_TICKS(5); // 设定5ms的固定周期 (200Hz)
	xlastWakeTime = xTaskGetTickCount(); // 在循环外初始化xlastWakeTime
	while (true)
	{
		ctrl.chassis.Update(); //底盘电机更新
		ctrl.pantile.Update();  //云台电机更新
		ctrl.shooter.Update();  //摩擦轮电机更新
		rc.Update();
		// MODIFIED: 使用vTaskDelayUntil代替vTaskDelay(5)。
		// 这可以确保此任务严格按照5ms的周期执行，消除了抖动（Jitter），
		// 这对于PID控制，尤其是D项的稳定计算至关重要。
		xuc.Encode();
		vTaskDelayUntil(&xlastWakeTime, xFrequency);
	}
}


void DecodeTask(void* pvParameters)
{
	// MODIFIED: 同样地，为解码任务也使用vTaskDelayUntil来确保固定的执行周期
	TickType_t xlastWakeTime;
	const TickType_t xFrequency = pdMS_TO_TICKS(5); // 设定5ms的固定周期 (200Hz)
	xlastWakeTime = xTaskGetTickCount(); // 初始化
	while (true)
	{
		rc.Decode(); //遥控器解码
		xuc.Decode();//xuc解码
		imu_pantile.Decode(); //陀螺仪解码
	
		vTaskDelayUntil(&xlastWakeTime, xFrequency);
	}
}

void ArmTask(void* pvParameters)
{
	while (true)
	{
		//初始化达妙电机
		DMmotor[0].DMmotorinit();
		power.Send();
		vTaskDelay(100);
	}
}





