  /*
   *__/\\\_______/\\\__/\\\\____________/\\\\__/\\\________/\\\______________/\\\\\\\\\____________/\\\\\\\\\_____/\\\\\\\\\\\___
   * _\///\\\___/\\\/__\/\\\\\\________/\\\\\\_\/\\\_______\/\\\____________/\\\///////\\\_______/\\\////////____/\\\/////////\\\_
   *  ___\///\\\\\\/____\/\\\//\\\____/\\\//\\\_\/\\\_______\/\\\___________\/\\\_____\/\\\_____/\\\/____________\//\\\______\///__
   *   _____\//\\\\______\/\\\\///\\\/\\\/_\/\\\_\/\\\_______\/\\\___________\/\\\\\\\\\\\/_____/\\\_______________\////\\\_________
   *    ______\/\\\\______\/\\\__\///\\\/___\/\\\_\/\\\_______\/\\\___________\/\\\//////\\\____\/\\\__________________\////\\\______
   *     ______/\\\\\\_____\/\\\____\///_____\/\\\_\/\\\_______\/\\\___________\/\\\____\//\\\___\//\\\____________________\////\\\___
   *      ____/\\\////\\\___\/\\\_____________\/\\\_\//\\\______/\\\____________\/\\\_____\//\\\___\///\\\___________/\\\______\//\\\__
   *       __/\\\/___\///\\\_\/\\\_____________\/\\\__\///\\\\\\\\\/_____________\/\\\______\//\\\____\////\\\\\\\\\_\///\\\\\\\\\\\/___
   *        _\///_______\///__\///_____________\///_____\/////////_______________\///________\///________\/////////____\///////////_____
  */

#include <stm32f4xx_hal.h>
#include <../CMSIS_RTOS/cmsis_os.h>
#include "can.h"
#include "usart.h"
#include "taskslist.h"
#include "tim.h"
#include "sysclk.h"
#include "delay.h"
#include "imu.h"
#include "motor.h"
#include "RC.h"
#include "control.h"
#include "judgement.h"
#include "led.h"
#include "HTmotor.h"
#include "Power_read.h"
#include "xuc.h"
#include "power_limit.h"
   
Motor can1_motor[CAN1_MOTOR_NUM] = {
	Motor(M3508,SPD, chassis, ID1, PID(2.3f, 0.f, 6.49e-4f, 0.f)),
	Motor(M3508,SPD, chassis, ID4, PID(2.3f, 0.f, 6.49e-4f, 0.f)),
	Motor(M3508,SPD, chassis, ID2, PID(2.3f, 0.f, 6.49e-4f, 0.f)),
	Motor(M3508,SPD, chassis, ID3, PID(2.3f, 0.f, 6.49e-4f, 0.f)),    //can1[0]~can1[3]底盘电机

	//陀螺仪控制云台
	Motor(M6020,POS_IMU, pantile, ID7, PID(2000.f, 1.f, 0.f, 0.f), PID(1.3f, 0.f, 0.6f, 0.25f)
								, PID(0.f, 0.f, 0.f,0.f))//pid 0 speed     1 posititon

	//Motor(M6020,POS2, pantile, ID7, PID(490.f, 0.f, 610.f, 0.f), PID(1.4f, 0.002f, 80.f, 0.25f)
	//							, PID(0.f, 0.f, 0.f,0.f))//pid 0 speed     1 posititon

	//神经网络
	//Motor(M6020,POS2, pantile, ID7, PID(490.f, 0.f, 610.f, 0.f), PID(1.5f, 0.003f, 70.f, 0.25f)
	//							, PID(0.f, 0.f, 0.f,0.f))//pid 0 speed     1 posititon

	//手控
	//Motor(M6020,POS2, pantile, ID7, PID(500.f, 0.f, 600.f, 0.f), PID(3.f, 0.f, 190.f, 0.25f)
	//							, PID(0.f, 0.f, 0.f,0.f))//pid 0 speed     1 posititon

	//机械角控制云台
	/*Motor(M6020,POS, pantile, ID7, PID(500.f, 0.f, 600.f, 0.f), PID(0.1f, 0.f, 1.4f, 0.25f)
								, PID(0.f, 0.f, 0.f,0.f))*/
};
Motor can2_motor[CAN2_MOTOR_NUM] = {
	Motor(M3508, SPD, shooter, ID1, PID(7.5f, 0.f, 0.02f,0.f)),
	Motor(M3508, SPD, shooter, ID2, PID(7.5f, 0.f, 0.02f,0.f)),
	//Motor(M2006, SPD, supply, ID7, PID(3.5f, 0.1f, 5.f, 0.f)),	//SPD 拨弹轮

	Motor(M2006, ACE, supply, ID7, PID(3.0f, 0.01f, 10.f),PID(0.5f, 0.01f, 2.f,0.f)), // ACE 拨弹轮

	//Motor(M6020, POS, pantile ,ID6, PID(140.f, 0.1f, 60.f,0.f),PID(0.3f, 0.f, 2.f,0.f))
};
DMMOTOR DMmotor[1] = {
	DMMOTOR(0x01, P_S, L_F), 
};


CAN can1, can2;
UART uart1, uart2, uart3, uart4, uart5, uart6;
TIM  timer;
IMU imu_pantile;
DELAY delay;
RC rc;
POWER power;
LED led1, led2, led3, led4;
TASK task; 
CONTROL ctrl;
Judgement judgement;
PARAMETER para;
XUC xuc;

int main(void)
{
	SystemClockConfig();
	delay.Init(168);
	HAL_Init(); 
	can1.Init(CAN1);
	//HAL_CAN_Start(&hcan1);
	//HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

	can2.Init(CAN2);
	timer.Init(BASE, TIM3, 1000).BaseInit();
	imu_pantile.Init(&uart1, USART1, 115200, CH010);
	rc.Init(&uart4, UART4, 100000);
	power.Init(&uart5,UART5, 9600);//10.7.23.00确认u4口正常，5.6.2口异常
	xuc.Init(&uart3, USART3, 460800);
	para.Init();
	ctrl.Init(std::vector<Motor*>{
		&can2_motor[0],
			& can2_motor[1],
			& can2_motor[2],
			& can2_motor[3],
			& can2_motor[4],
			& can2_motor[5]
	});
	ctrl.Init(std::vector<Motor*>{
		&can1_motor[0],
			& can1_motor[1],
			& can1_motor[2],
			& can1_motor[3],
			& can1_motor[4],
			& can1_motor[5]
	});
	DMmotor[0].DMmotorinit();

	// 初始化功率控制器
	// 参数: 底盘电机指针数组, 功率上限(W), k1, k2, k3
	// 功率上限可根据实际情况调整
	powerLimiter.Init(ctrl.chassis_motor, 80.0f, 0.17f, 1.08f, 5.3f);

	task.Init();
	for (;;)
		;
}





