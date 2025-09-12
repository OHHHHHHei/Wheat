#include "label.h"
#include "Power_read.h"
#include <cstdlib>   // abs() ����
uint8_t CNT_transmate = 0;
void POWER::Init(UART* huart, USART_TypeDef* Instance, const uint32_t BaudRate)
{
    // ��ʼ�� UART�������� DMA ���պͷ���
    huart->Init(Instance, BaudRate)
        .DMARxInit(nullptr)
        .DMATxInit();   // <-- ���ӷ��� DMA ��ʼ��

    m_uart = huart;
    queueHandler = &huart->UartQueueHandler;
}

void POWER::Receive()
{
    if (queueHandler == NULL || *queueHandler == NULL) {
        return;  // ���߱���
    }
    pd_Rx = xQueueReceive(*queueHandler, m_uartrx, 0);
}
void POWER::Send()
{
    if (m_uart == nullptr) return;

    // ʹ��������ʽ����
    m_uart->UARTTransmit((uint8_t*)"AT+P\r\n", strlen("AT+P\r\n"));
}

char uint8_to_char(uint8_t num)
{
    return (char)(num + '0');
}

void POWER::Decode()
{
	if (m_uartrx[0] == '+' && m_uartrx[1] == 'P' && m_uartrx[2] == '=') {
		if (m_uartrx[4] == '.' && m_uartrx[8] == '\r')
		{
			power_now = (m_uartrx[3] - '0') + (m_uartrx[5] - '0') * 0.1 + (m_uartrx[6] - '0') * 0.01;
		}
		else if (m_uartrx[5] == '.' && m_uartrx[9] == '\r')
		{
			power_now = (m_uartrx[3] - '0') * 10 + (m_uartrx[4] - '0') + (m_uartrx[6] - '0') * 0.1 + (m_uartrx[7] - '0') * 0.01;
		}
		else if (m_uartrx[6] == '.' && m_uartrx[10] == '\r')
		{
			power_now = (m_uartrx[3] - '0') * 100 + (m_uartrx[4] - '0') * 10 + (m_uartrx[5] - '0') + (m_uartrx[7] - '0') * 0.1 + (m_uartrx[8] - '0') * 0.01;
		}
		else if (m_uartrx[7] == '.' && m_uartrx[11] == '\r')
		{
			power_now = (m_uartrx[3] - '0') * 1000 + (m_uartrx[4] - '0') * 100 + (m_uartrx[5] - '0') * 10 + (m_uartrx[6] - '0') + (m_uartrx[8] - '0') * 0.1 + (m_uartrx[9] - '0') * 0.01;
		}
	}
}
