#include "supercap.h"
#include "imu.h"
#include "label.h"
#include "control.h"
#include "motor.h"
#include <judgement.h>


float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
	float span = x_max - x_min;
	float offset = x_min;
	return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
};

int float_to_uint(float x, float x_min, float x_max, int bits)
{
	float span = x_max - x_min;
	float offset = x_min;
	return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

void SUPERCAP::Init(UART* huart, uint32_t baud, USART_TypeDef* uart_base)
{
	huart->Init(uart_base, baud).DMARxInit(nullptr).DMATxInit();
	m_uart = huart;
	queueHandler = &huart->UartQueueHandler;

	//Set_PE_Target();
}

void SUPERCAP::decode()
{
	pd_Rx = xQueueReceive(*queueHandler, rxData, NULL);

	if (Rxsuper.I == I_pre) { count += 1; }
	else { count = 0; }
	if (count > 200) { connect = false; }
	else { connect = true; }

	/*memcpy(supercap_union.supertcap_data, rxData, 20);*/
	I_pre = Rxsuper.I;

	Rxsuper.header1 = rxData[0];
	Rxsuper.header2 = rxData[1];
	if (rxData[0] == 0x4c)
	{
		if (rxData[1] == 0x0b)
		{
			Rxsuper.connect_flag = rxData[2];
			Rxsuper.cap_energy = u8_to_float(rxData + 4);
			Rxsuper.U = u8_to_float(rxData + 8);
			Rxsuper.I = u8_to_float(rxData + 12);
			
			/*Rxsuper.stop = (rxData[15] << 8) | (0x00ff & rxData[14]);
			Rxsuper.open_flag = rxData[16];*/
		}
	}
}

void SUPERCAP::encode()
{
	//���� Ҫ�Ķ�����rc���
	int packet_size = sizeof(Txsuper);
	Txsuper.count++;
	
	// �����ݰ����Ƶ����ͻ�����
	memcpy(tx_data, &Txsuper, packet_size);
	if (Txsuper.count == 100)
	{
		Txsuper.count = 0;
	}


	// ���㲢���� CRC16 У���
	//appendCRC16CheckSum(tx_data, packet_size);
	
	// ��������
	m_uart->UARTTransmit(tx_data, packet_size);
}

/**
 * @brief  ���ʿ��Ƶ�������
 * @param  _RF_power:����ϵͳ����
 * @param  _motor_power:���ʵʱ����
 * @param  _remain_energy:����ϵͳʣ������
 * @param  motor_out_raw:�ٶȻ����ֵ��Ҳ������ֵ
 * @author kainan
 */
void SUPERCAP::Control(float _RF_power, float _motor_power, float _remain_energy)
{
	/* ���¹������� */
	Update(_RF_power, _motor_power, _remain_energy);

	/* �޷�ֵ���� */
	Calc_motorLimit();

	/* ��繦�ʼ��� */
	//if (cap_charge_enable == __ENABLE)
	//	Calc_capChargePower();
	//else
	//	cap_charge_power = 0;
}

/**
 * @brief  �õ����ݳ���������
 * @param  None
 * @retval cap_charge_power�����ݳ���������
 * @author kainan
 */
float SUPERCAP::Get_capChargePower(void)
{
	return cap_charge_power;
}

/**
 * @brief  �õ��������޷�����
 * @param  None
 * @retval lim_scale���޷�������ÿ��������Ӧ���������ֵ
 * @author kainan
 */
float SUPERCAP::Get_limScale(void)
{
	return lim_scale;
}


/**
 * @brief  �������Ƿ������ƣ������ڲ��õ��޷�ֵ
 * @param  motor_out_raw������ٶȻ������ֵ��������ֵ
 * @retval ����Ƿ��յ���������
 * @author kainan
 */

void SUPERCAP::Calc_motorLimit()
{
	int limit_power_total = 0;/* ����ܵ��� */

	/* ����ʹ�õ��ݹ���(Ӣ�ۺͲ���)��ʹ�õ�·������Ĳ������ʽ��е�����ʿ��� */
	if (control_loop == REAL_POWER_LOOP)
	{
		/* motor_power_target * 10�൱�ڵ���һ��ǰ������PID��õ�һЩ */
		limit_power_total = motor_power_target * 10 + motorLimitController.Position(motor_power_target-motor_power,100.f);
	}
	/* �޵��������壬����ʹ�ò���ϵͳ����(�ڱ�)��ʹ��ʣ���������е�����ʿ��� */
	else
	{
		/*limit_current_total = RF_power_target * 10 
			+ motorLimitController.Position(remain_energy_target- remain_energy,100.f);*/
		limit_power_total = motorLimitController.Position(remain_energy_target - remain_energy, 100.f);
	}

	/* ��������޷�����������ʱ��limitation������й������ƣ�scaleҲ���� */
	limit_power_total = _PowerCtrl_Constrain(limit_power_total, 0, max_power_out);

	float scale = 1.0f;
	float current_sum = 0.0f;
	float motor_current_max = 0.0f;

	/* ����ܵ��� */
	for (uint8_t i = 0; i < motor_num; i++)
		current_sum += abs(ctrl.chassis_motor[i]->setcurrent) * 20.f / 16384.f;
	power_sum_total = current_sum * 24.f;

	/* �ܵ�����������ֵ */
	if (power_sum_total > limit_power_total)
	{
		scale = limit_power_total / power_sum_total;
	}
	else {
		scale = 1.0f;
	}

	lim_scale = scale;
}


/**
 * @brief  ����������ݳ�繦��
 * @param  None
 * @retval None
 * @author kainan
 */
void SUPERCAP::Calc_capChargePower(void)
{
	cap_charge_power = RF_power_target - capChargeController.Position(remain_energy_target - remain_energy, 100.f);
	if (cap_charge_power < 0)
		cap_charge_power = 0;
	else {}
}

/**
 * @brief  ���¹������ݵ�ֵ
 * @param  ��ع��ʣ�������ʣ�RFʣ������
 * @retval None
 * @author kainan
 */
void SUPERCAP::Update(float _RF_power, float _motor_power, float _remain_energy)
{
	static float last_remian_energy = 0;
	RF_power = _RF_power;
	motor_power = _motor_power;

	/* ����ϵͳ�������Ƿ���� */
	if (_remain_energy != last_remian_energy)
	{
		remain_energy = _remain_energy;
		last_remian_energy = remain_energy;
	}
	else
		remain_energy += (RF_power_target - RF_power) * ctrl_period / 1000.0f;
	if (remain_energy >= 60.0f)
		remain_energy = 60.0f;

	/* ʹ�õ������Ԥ�⹦��ֵ */
	
	Pin = Ct * Icmd * w + k1 * w * w + k2 * Icmd * Icmd;
}

/**
 * @brief  ���ò���ϵͳ���ʣ�������ʣ�ʣ��������Ŀ��ֵ
 * @param  _RF_power_target:����ϵͳ����Ŀ��ֵ
 * @param  _motor_power_target:�������Ŀ��ֵ
 * @param  _remain_energy_target:ʣ������Ŀ��ֵ
 * @retval None
 * @author kainan
 */
void SUPERCAP::Set_PE_Target()
{
	motorLimitController.m_Kp = 10.f;
	motorLimitController.m_Ti = 0.f;
	motorLimitController.m_Td = 0.f;
}