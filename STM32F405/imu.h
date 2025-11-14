#pragma once
#include "stm32f4xx.h"
#include "usart.h"
#include <string.h>
enum IMU_TYPE { IMU601 = 0, CH010, HI226 };

class IMU
{
public:
	float ACC_FSR = 4.f, GYRO_FSR = 2000.f;
	typedef struct
	{
		float roll, yaw, pitch, pitch_filtered;
	}Angle, AngularVelocity;
	typedef struct
	{
		float x{}, y{}, z{};
	}Acceleration;

	void Init(UART* huart, USART_TypeDef* Instance, const uint32_t BaudRate, IMU_TYPE type);
	void Decode();
	bool Check(uint8_t* pdata, uint8_t len, uint32_t com);
	float GetAngleYaw();
	float GetAnglePitch();
	float GetAngleRoll();
	float getangularvelocitypitch();
	float* GetAcceleration();
	void GetQ();
	int16_t getword(uint8_t HighBit, uint8_t LowBits);

	BaseType_t pd_Rx = false;
	QueueHandle_t* queueHandler = NULL;

	Angle angle;
	AngularVelocity angularvelocity;
	Acceleration acceleration;
private:
	
	uint16_t crc, len;
	IMU_TYPE type;


	// ===== Pitch均值滤波器（3点窗口，最小时滞） =====
	// 使用3点滑动窗口进行均值滤波，平衡噪声抑制和响应速度
	// 时滞约为 1.5 * 采样周期（CH010为115200波特率，约13ms延迟）
	float pitch_filter_buffer[7] = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };  // 循环缓冲区
	uint8_t pitch_filter_index = 0;                      // 当前写入位置
	float pitch_filtered = 0.0f;                         // 滤波后的pitch值



	uint8_t rxData[UART_MAX_LEN];
	UART* m_uart;

};

static uint16_t U2(uint8_t* p) { uint16_t u; memcpy(&u, p, 2); return u; }
static uint32_t U4(uint8_t* p) { uint32_t u; memcpy(&u, p, 4); return u; }
static float    R4(uint8_t* p) { float    r; memcpy(&r, p, 4); return r; }

extern IMU imu_chassis, imu_pantile;