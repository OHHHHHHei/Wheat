#pragma once
#include "usart.h."
#include "FreeRTOS.h"
#include <cmath>
#include <cinttypes>

/*
����s[0],�Ҳ���s[1]
�ϣ�1 �У�3 �£�2

��ҡ�� ���� ch[1]
��ҡ�� ���� ch[0]
��ҡ�� ���� ch[2]
��ҡ�� ���� ch[3]

*/

class RC
{
public:
	int gear;
	bool top_mode = true;
	bool fix = false;

	struct
	{
		int16_t ch[4];
		uint8_t s[2];
	}rc, pre_rc;

	enum POSITION { UP = 1, DOWN, MID };
	struct PC
	{
		int16_t x, y, z;
		uint8_t press_l, press_r;

		uint8_t key_h, key_l;
		const float spdratio = 1.f;
	}pc;

	uint8_t* GetDMARx(void) { return m_frame; }

	bool judement_start = false;
	void Decode();
	void OnRC();
	void OnPC();
	void Update();
	void Init(UART* huart, USART_TypeDef* Instance, const uint32_t BaudRate);
	bool Shift_mode();

private:
	QueueHandle_t* queueHandler = NULL;
	BaseType_t pd_Rx, pd_Tx;
	UART* m_uart;
	uint8_t m_frame[UART_MAX_LEN]{};
};

extern RC rc;
