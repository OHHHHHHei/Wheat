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
	//������ʼ����
	xTaskCreate((TaskFunction_t)start_task,            //������
		(const char*)"start_task",          //��������
		(uint16_t)START_STK_SIZE,        //�����ջ��С
		(void*)NULL,                  //���ݸ��������Ĳ���
		(UBaseType_t)START_TASK_PRIO,       //�������ȼ�
		(TaskHandle_t*)&StartTask_Handler);   //������              
	vTaskStartScheduler();          //�����������
}

/*
��ʼ����������
*/
void start_task(void* pvParameters)
{
	taskENTER_CRITICAL();           //�����ٽ���
	//��������

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

	vTaskDelete(StartTask_Handler); //ɾ����ʼ����
	taskEXIT_CRITICAL();            //�˳��ٽ���
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


	vTaskDelayUntil(&xlastWakeTime, pdMS_TO_TICKS(2));//��ʼִ�и�����֮��1ms��ִ�и�����
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
				DMmotor[0].DMmotor_transmit(1);  //���ʹ���������
			break;
		case 1:
			can1.Transmit(0x1ff, can1.temp_data + 8); //����can1������ ��̨
			can2.Transmit(0x1ff, can2.temp_data + 8); //����can2������
			break;
		case 2:
			can1.Transmit(0x200, can1.temp_data); //����can1������  ����
			can2.Transmit(0x200, can2.temp_data); //����can2������
		default:
			break;
		}
		
		vTaskDelayUntil(&xlastWakeTime1, pdMS_TO_TICKS(1));//��ʼִ�и�����֮��1ms��ִ�и�����

	}
}

void ControlTask(void* pvParameters)
{
	while (true)
	{
		xuc.Encode();
		ctrl.chassis.Update();
		ctrl.pantile.Update();
		ctrl.shooter.Update();
		rc.Update();
		vTaskDelay(1);
	}
}


void DecodeTask(void* pvParameters)
{
	while (true)
	{
		rc.Decode();
		imu_pantile.Decode();
		xuc.Decode();
		vTaskDelay(5);
	}
}

void ArmTask(void* pvParameters)
{
	while (true)
	{
		//��ʼ��������
		DMmotor[0].DMmotorinit();
		power.Send();
		vTaskDelay(100);
	}
}





