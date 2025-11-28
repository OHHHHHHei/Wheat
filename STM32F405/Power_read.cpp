#include "label.h"
#include "Power_read.h"
#include <cstdlib>   // abs() 函数
uint8_t CNT_transmate = 0;
void POWER::Init(UART* huart, USART_TypeDef* Instance, const uint32_t BaudRate)
{
    // 初始化 UART，并启用 DMA 接收和发送
    huart->Init(Instance, BaudRate)
        .DMARxInit(nullptr) // 启用DMA接收
        .DMATxInit();   // <-- 增加发送 DMA 初始化

    m_uart = huart;
    queueHandler = &huart->UartQueueHandler; // 获取接收队列
}

void POWER::Receive()
{
    if (queueHandler == NULL || *queueHandler == NULL) {
        return;  // 或者报错
    }
    pd_Rx = xQueueReceive(*queueHandler, m_uartrx, 0);
}
void POWER::Send()
{
    if (m_uart == nullptr) return;

    // 使用阻塞方式发送查询指令
    m_uart->UARTTransmit((uint8_t*)"AT+P\r\n", strlen("AT+P\r\n"));
}

char uint8_to_char(uint8_t num)
{
    return (char)(num + '0');
}

void POWER::Decode()
{
	// 检查数据帧格式: +P=XXX.XX\r\n
	if (m_uartrx[0] == '+' && m_uartrx[1] == 'P' && m_uartrx[2] == '=') {
		// 情况1: X.XX (个位数, 如 5.23W)
		if (m_uartrx[4] == '.' && m_uartrx[8] == '\r')
		{
			power_now = (m_uartrx[3] - '0') + (m_uartrx[5] - '0') * 0.1 + (m_uartrx[6] - '0') * 0.01;
		}
		// 情况2: XX.XX (十位数, 如 52.34W)
		else if (m_uartrx[5] == '.' && m_uartrx[9] == '\r')
		{
			power_now = (m_uartrx[3] - '0') * 10 + (m_uartrx[4] - '0') + (m_uartrx[6] - '0') * 0.1 + (m_uartrx[7] - '0') * 0.01;
		}
		// 情况3: XXX.XX (百位数, 如 123.45W)
		else if (m_uartrx[6] == '.' && m_uartrx[10] == '\r')
		{
			power_now = (m_uartrx[3] - '0') * 100 + (m_uartrx[4] - '0') * 10 + (m_uartrx[5] - '0') + (m_uartrx[7] - '0') * 0.1 + (m_uartrx[8] - '0') * 0.01;
		}
		// 情况4: XXXX.XX (千位数, 如 1234.56W)
		else if (m_uartrx[7] == '.' && m_uartrx[11] == '\r')
		{
			power_now = (m_uartrx[3] - '0') * 1000 + (m_uartrx[4] - '0') * 100 + (m_uartrx[5] - '0') * 10 + (m_uartrx[6] - '0') + (m_uartrx[8] - '0') * 0.1 + (m_uartrx[9] - '0') * 0.01;
		}
	}
}
