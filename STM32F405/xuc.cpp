#include "xuc.h"
#include "label.h"
#include "imu.h"
#include "CRC.h"
#include "math.h"
#include "RC.h"

int a = 0;

void XUC::Init(UART* huart, USART_TypeDef* Instance, uint32_t BaudRate)
{
	huart->Init(Instance, BaudRate).DMARxInit(nullptr).DMATxInit();
	m_uart = huart;
	frame = m_uart->m_uartrx;
	queue_handler = &huart->UartQueueHandler;

	autoaim_controller[0].m_Kp = 0.005f;
	autoaim_controller[0].m_Td = 0.004f;

	autoaim_controller[1].m_Kp = 0.0014f;
	autoaim_controller[1].m_Td = 0.001f;
}

void XUC::Decode()
{
	a = 1;
	pd_Rx = xQueueReceive((m_uart->UartQueueHandler), m_frame, NULL);
	if (m_frame[0] == 0xA5)
	{
		/*if (!verifyCRC16CheckSum(frame, 22))
			return;*/

		yaw_pre = yaw;
		yaw_spd = ((yaw - yaw_pre) / 0.004) * 2 * PI / 60;//0.004����tasklist�ķ���Ƶ�� �������yaw_spd�ĵ�λ��rpm

		xuc.pitch = u8_to_float(m_frame + 1) * PI / 180.f;
		xuc.yaw = u8_to_float(m_frame + 5);
		xuc.yaw_diff = u8_to_float(m_frame + 9);
		xuc.pitch_diff = u8_to_float(m_frame + 13) * PI / 180.f;
		xuc.distance = u8_to_float(m_frame + 17);
		xuc.fireadvice = m_frame[21] & 0x01;
		xuc.v_y = u8_to_float(m_frame + 25);


		if (distance > 0.f)
		{
			test_cont++;
		}
		else
		{
			test_cont = 0;
		}

		if (test_cont >= 50)
		{
			track_flag = true;
		}
		else
		{
			track_flag = false;
		}

	}
	//ͬ��
	else if (m_frame[0] == 'S' && m_frame[1] == 'P') 
	{
		if (verifyCRC16CheckSum(frame, 29))//������ݴ���δ���ִ���
		{
			RxNuc_TJ.mode_TJ = m_frame[2];//���
			RxNuc_TJ.yaw_TJ = u8_to_float(m_frame + 3);//Ԥ�ڽǶ�
			RxNuc_TJ.yaw_vel_TJ = u8_to_float(m_frame + 7);//yaw�ٶ�
			RxNuc_TJ.yaw_acc_TJ = u8_to_float(m_frame + 11);//yaw���ٶ�
			RxNuc_TJ.pitch_TJ = u8_to_float(m_frame + 15);//pitchԤ�ڽǶ�
			RxNuc_TJ.pitch_vel_TJ = u8_to_float(m_frame + 19);//pitch�ٶ�
			RxNuc_TJ.pitch_acc_TJ = u8_to_float(m_frame + 23);//pitch���ٶ�
		}
	}
}

void XUC::Encode()
{
	own_color = judgement.data.robot_status_t.robot_id <= 7 ? RED : BLUE;
	//TxPacket TxNuc;  // ����һ�����ݰ�ʵ��

	TxNuc_TJ.head[0] = 'S';
	TxNuc_TJ.head[1] = 'P';
	TxNuc_TJ.mode_TJ = 1;  // 0: ����, 1: ����, 2: С��, 3: ���
	imu_pantile.GetQ();
	for (int i = 0; i < 4; i++) {
		TxNuc_TJ.q_TJ[i];
	}   // wxyz˳��
	TxNuc_TJ.yaw_TJ = imu_pantile.angle.yaw;
	TxNuc_TJ.yaw_vel_TJ = imu_pantile.angularvelocity.yaw;
	TxNuc_TJ.pitch_TJ = imu_pantile.angle.pitch;
	TxNuc_TJ.pitch_vel_TJ = imu_pantile.angularvelocity.pitch;
	TxNuc_TJ.bullet_speed_TJ = 1.f;
	TxNuc_TJ.bullet_count_TJ = 1.f; // �ӵ��ۼƷ��ʹ���

	//TxNuc.header = 0x5A;
	//TxNuc.detect_color = !own_color;
	//TxNuc.reset_tracker = 0;
	//TxNuc.reserved = 15;
	//TxNuc.roll = imu_pantile.GetAngleRoll();
	//TxNuc.pitch = imu_pantile.GetAnglePitch();
	//TxNuc.yaw = imu_pantile.GetAngleYaw();
	//TxNuc.aim_x = aim_x;
	//TxNuc.aim_y = aim_y;
	//TxNuc.aim_z = aim_z;
	//TxNuc.checksum = 0;  // ��ʼ��У���Ϊ0

	// �������ݰ����ܴ�С
	int packet_size = sizeof(TxNuc_TJ);

	// �����ݰ����Ƶ����ͻ�����
	memcpy(tx_data, &TxNuc_TJ, packet_size);

	// ���㲢���� CRC16 У���
	appendCRC16CheckSum(tx_data, packet_size);

	// ��������
	m_uart->UARTTransmit(tx_data, packet_size);
}

//����CRC������pchMessageΪ����ָ�룬dwLengthΪ���ݳ��ȣ�wCRCΪ��ʼCRCֵ
uint16_t XUC::getCRC16CheckSum(const uint8_t* pchMessage, uint32_t dwLength, uint16_t wCRC)
{
	uint8_t ch_data;

	if (pchMessage == nullptr) return 0xFFFF;//������ָ��Ϊ�գ�����Ĭ��CRCֵ0xFFFF
	while (dwLength--) {
		ch_data = *pchMessage++;
		(wCRC) =
			((uint16_t)(wCRC) >> 8) ^ CRC_TAB[((uint16_t)(wCRC) ^ (uint16_t)(ch_data)) & 0x00ff];
	}

	return wCRC;
}

//����CRC��������֤����������
uint32_t XUC::verifyCRC16CheckSum(const uint8_t* pchMessage, uint32_t dwLength)
{
	uint16_t w_expected = 0;

	if ((pchMessage == nullptr) || (dwLength <= 2)) return false;//�յ�����Ϊ�գ����߳��Ȳ���

	w_expected = getCRC16CheckSum(pchMessage, dwLength - 2, CRC16_INIT);//dwLength-2��2��Ϊ׷��CRCԤ���ĳ���
	return (
		(w_expected & 0xff) == pchMessage[dwLength - 2] &&
		((w_expected >> 8) & 0xff) == pchMessage[dwLength - 1]);
}

//׷��CRC��������CRC16У��ֵ���㲢׷�ӵ����ݰ���ĩβ���γ�����������֡
void XUC::appendCRC16CheckSum(uint8_t* pchMessage, uint32_t dwLength)
{
	uint16_t w_crc = 0;

	if ((pchMessage == nullptr) || (dwLength <= 2)) return;

	w_crc = getCRC16CheckSum(reinterpret_cast<uint8_t*>(pchMessage), dwLength - 2, CRC16_INIT);

	//׷��CRCֵ
	pchMessage[dwLength - 2] = (uint8_t)(w_crc & 0x00ff);//���ֽ�
	pchMessage[dwLength - 1] = (uint8_t)((w_crc >> 8) & 0x00ff);//���ֽ�
}
