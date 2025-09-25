#include "imu.h"
#include "label.h"
#include "xuc.h"
#include <cmath>
void IMU::Init(UART* huart, USART_TypeDef* Instance, const uint32_t BaudRate, IMU_TYPE type)
{
	huart->Init(Instance, BaudRate).DMARxInit();
	m_uart = huart;
	this->type = type;
	queueHandler = &huart->UartQueueHandler;
}

void IMU::Decode()
{
	if (queueHandler == NULL || *queueHandler == NULL) {
		return;  // ���߱���
	}
	else {
		pd_Rx = xQueueReceive(*queueHandler, rxData, NULL);
	}

	if (type == IMU601)
	{
		if (rxData[0] == 0x55 && rxData[1] == 0x55)
		{
			uint8_t* data = rxData + 4;
			if (rxData[2] == 0x01 && Check(rxData, rxData[3] + 4, rxData[rxData[3] + 4]))
			{
				angle.roll = (float)getword(data[1], data[0]) * 180.f / 32768.f;
				angle.pitch = (float)getword(data[3], data[2]) * 180.f / 32768.f;
				angle.yaw = (float)getword(data[5], data[4]) * 180.f / 32768.f;
			}
			else if (rxData[2] == 0x03 && Check(rxData, rxData[3] + 4, rxData[rxData[3] + 4]))
			{
				uint8_t Ax, Ay, Az, Gx, Gy, Gz;
				Ax = getword(data[1], data[0]);
				Ay = getword(data[3], data[2]);
				Az = getword(data[5], data[4]);
				Gx = getword(data[7], data[6]);
				Gy = getword(data[9], data[8]);
				Gz = getword(data[11], data[10]);
				acceleration.x = (float)Ax / 32768 * ACC_FSR;
				acceleration.y = (float)Ay / 32768 * ACC_FSR;
				acceleration.z = (float)Az / 32768 * ACC_FSR;
				angularvelocity.roll = (float)Gx / 32768 * GYRO_FSR;
				angularvelocity.pitch = (float)Gy / 32768 * GYRO_FSR;
				angularvelocity.yaw = (float)Gz / 32768 * GYRO_FSR;

			}
		}
	}
	else if (type == CH010)
	{
		if (rxData[0] == 0x5A && rxData[1] == 0xA5)
		{
			//Check(rxData, 4, 0);
			if (Check(rxData + 6, rxData[3] << 8 + rxData[2], rxData[5] << 8 + rxData[4]))
			{
				if (rxData[6] == 0x91)
				{
					int offset = 6;
					acceleration.x = R4(rxData + offset + 12);
					acceleration.y = R4(rxData + offset + 16);
					acceleration.z = R4(rxData + offset + 20);
					angularvelocity.pitch = R4(rxData + offset + 24);
					angularvelocity.roll = R4(rxData + offset + 28);
					angularvelocity.yaw = R4(rxData + offset + 32);
					angle.pitch = R4(rxData + offset + 48);
					angle.roll = R4(rxData + offset + 52);
					angle.yaw = R4(rxData + offset + 56);

				}
			}
			crc = 0;
		}
	}
	else if (type == HI226)
	{
		if (rxData[0] == 0x5A && rxData[1] == 0xA5)
		{
			//Check(rxData, 4, 0);
			if (Check(rxData + 6, rxData[3] << 8 + rxData[2], rxData[5] << 8 + rxData[4]))
			{
				if (rxData[6] == 0x91)
				{
					int offset = 6;
					acceleration.x = R4(rxData + offset + 12);
					acceleration.y = R4(rxData + offset + 16);
					acceleration.z = R4(rxData + offset + 20);
					angularvelocity.pitch = R4(rxData + offset + 24);
					angularvelocity.roll = R4(rxData + offset + 28);
					angularvelocity.yaw = R4(rxData + offset + 32);
					angle.pitch = R4(rxData + offset + 48);
					angle.roll = R4(rxData + offset + 52);
					angle.yaw = R4(rxData + offset + 56);
				}
			}
		}
	}
}

float IMU::GetAngleYaw()//��ȡIMU��YAWֵ
{
	return angle.yaw;
}

float IMU::getangularvelocitypitch()
{
	return angularvelocity.pitch;
}

float IMU::GetAnglePitch()//��ȡIMU��PITCHֵ
{
	return angle.pitch;
}

float IMU::GetAngleRoll()//��ȡIMU��ROLLֵ
{
	return angle.roll;
}

float* IMU::GetAcceleration()//û�б����ã��÷�����
{
	float temp[3]{};
	temp[0] = acceleration.x;
	temp[1] = acceleration.y;
	temp[2] = acceleration.z;
	return temp;
}

void IMU::GetQ()//��ά������
{
	xuc.TxNuc_TJ.q_TJ[0] = cos(angle.yaw / 2) * cos(angle.pitch / 2) * cos(angle.roll / 2) + sin(angle.yaw / 2) * sin(angle.pitch / 2) * sin(angle.roll / 2);
	xuc.TxNuc_TJ.q_TJ[1] = sin(angle.yaw / 2) * cos(angle.pitch / 2) * cos(angle.roll / 2) - cos(angle.yaw / 2) * sin(angle.pitch / 2) * sin(angle.roll / 2);
	xuc.TxNuc_TJ.q_TJ[2] = cos(angle.yaw / 2) * sin(angle.pitch / 2) * cos(angle.roll / 2) + sin(angle.yaw / 2) * cos(angle.pitch / 2) * sin(angle.roll / 2);
	xuc.TxNuc_TJ.q_TJ[3] = cos(angle.yaw / 2) * cos(angle.pitch / 2) * sin(angle.roll / 2) - sin(angle.yaw / 2) * sin(angle.pitch / 2) * cos(angle.roll / 2);
}

bool IMU::Check(uint8_t* pdata, uint8_t len, uint32_t com)
{
	if (type == IMU601)
	{
		uint8_t t = 0;
		for (int i = 0; i < len; i++)
		{
			t += pdata[i];
		}
		return t == com;
	}
	else if (type == CH010)
	{
		for (int j = 0; j < len; ++j)
		{
			uint32_t i;
			uint32_t byte = pdata[j];
			crc ^= byte << 8;
			for (i = 0; i < 8; ++i)
			{
				uint32_t temp = crc << 1;
				if (crc & 0x8000)
				{
					temp ^= 0x1021;
				}
				crc = temp;
			}
		}
		return crc == com;
	}
}


int16_t IMU::getword(uint8_t HighBit, uint8_t LowBits)
{
	return HighBit << 8 | LowBits;
}