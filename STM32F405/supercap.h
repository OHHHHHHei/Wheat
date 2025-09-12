#pragma once
#include "stm32f4xx_hal.h"
#include "usart.h"
#include "can.h"
#include "queue.h"
#include "pid.h"
#include "string.h"

enum E_ENABLE {
	__DISABLE = 0,
	__ENABLE = 1
};

enum E_LOOP {
	REAL_POWER_LOOP = 0,		/* !����!�е���������(Ӣ�ۺͲ���)��ʹ�õ���������Ĳ������ʽ��е�����ʿ��� */
	REMAIN_ENERGY_LOOP = 1	/* !����!�޵��������������(�ڱ�)��ʹ�ò���ϵͳ���磬ʹ��ʣ���������е�����ʿ��� */
};

typedef enum {
	WORKING = 0, //��������
	DISCHARGE, //��¼ʱ�ŵ�
	SHUT, //���̶ϵ�ʱ�ر�PWM
} working_state; //����״̬

struct RxSurPacket
{
	uint8_t header1;
	uint8_t header2;
	uint8_t connect_flag;
	float cap_energy;
	float U;
	float I;
	
};

struct TxSurPacket
{
	uint8_t header1 = 0x4A; //֡ͷ��0x4A 1
	uint8_t header2 = 0x4B; //֡ͷ��0x4B 1
	uint16_t limit; //�������� 2
	working_state state; //����״̬ 
	uint8_t buffer; //��������
	uint8_t count;
};

class SUPERCAP
{
public:
	/*union
	{
		uint8_t supertcap_data[20];
		struct
		{
			uint8_t header1;
			uint8_t header2;
			float buffer;
			float U;
			float I;
			uint16_t stop;
			uint8_t open_flag;
		}supercap;

	}supercap_union;*/

	working_state s = WORKING;
	TxSurPacket Txsuper;
	RxSurPacket Rxsuper;
	
	float I_pre;
	int count;
	bool connect = false;
	void Init(UART* huart, uint32_t baud, USART_TypeDef* uart_base);
	void encode();
	void decode();

	int update_cnt = 0;

	/*<! ������ر��� Ϊ�˷������ ����public */
	float RF_power;
	float motor_power;
	float remain_energy;
	float motor_power_target;
	float RF_power_target = 60.f;
	float remain_energy_target = 0.f;
	//�����Ƿ�ϵ����������ֵı�����
	float Pin, w, Icmd, Ct = 1.99688994e-6f, k1, k2;

	template<typename Type>
	Type _PowerCtrl_Constrain(Type input, Type min, Type max) {
		if (input <= min)
			return min;
		else if (input >= max)
			return max;
		else return input;
	}

	/*void Load_capChargeController(float(*pFunc)(const float current, const float target));
	void Load_motorLimitController(float(*pFunc)(const float current, const float target));*/

	/* ������ */
	void Control(float _RF_power, float _motor_power, float _remain_energy);

	/* �趨Ŀ����� */
	void Set_PE_Target();

	/* ��õ��ݳ�繩�� */
	float Get_capChargePower(void);

	/* ��ù��ʿ���ֵ */
	float Get_limScale(void);

	uint8_t motor_num=4; 					/*<! ���̵�����������ֵ��� = 8�����ֵ��� = 4  */
	E_ENABLE cap_charge_enable; /*<! ���ݳ�翪�� */
	E_LOOP control_loop = REMAIN_ENERGY_LOOP; // �� REAL_POWER_LOOP   /*<! ʵʱ���ʻ�/ʣ�������� */
	uint8_t ctrl_period=20;				/*<! ��������,����Ԥ���� ��λms */
	float power_sum_total = 0.0f;
	int max_current_out=16000;			  /*<! ���������ֵ���������ú͵��������ֵһ�£���C620Ϊ16384 */
	int max_power_out = 200; //��������ֵ

	float cap_charge_power;		  /*<! ���������繦�� */
	float lim_scale;						/*<! �޷�������ÿ��������Ӧ���������ֵ */

	//float(*capChargeController)(const float current, const float target);
	//float(*motorLimitController)(const float current, const float target);
	PID motorLimitController, capChargeController;

	void Update(float _RF_power, float _motor_power, float _remain_energy);
	void Calc_motorLimit();
	void Calc_capChargePower();

private:
	BaseType_t pd_Rx = false;
	QueueHandle_t* queueHandler;
	uint8_t rxData[UART_MAX_LEN];
	uint8_t tx_data[UART_MAX_LEN];
	UART* m_uart;
	CAN* m_can;

	float u8_to_float(uint8_t* p) {
		float s{}; uint8_t ch[4];
		ch[0] = p[3]; ch[1] = p[2]; ch[2] = p[1]; ch[3] = p[0];

		memcpy(&s, p, 4); return s;
	}

};

extern SUPERCAP supercap;

