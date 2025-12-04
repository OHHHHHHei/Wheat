// judgement.cpp
#include "judgement.h"
#include "label.h"
#include <cstdarg>
#include "imu.h"
#include "control.h"
#include "nuc.h"
#include "RC.h"
#include "supercap.h"

//void Judgement::BuffData()
//{
//    //if (m_uart->updateFlag)
//    //{
//    //    m_uart->updateFlag = false;
//
//    if (queueHandler == NULL || *queueHandler == NULL)
//        return;  // 或者报错
//
//    // ★ 必须判断有没有真的收到队列数据
//    if (xQueueReceive(*queueHandler, m_uartrx, 0) != pdPASS)
//        return;
//
//    m_readnum = m_uart->dataDmaNum;
//
//    if ((m_whand + m_readnum) < (m_FIFO + BUFSIZE))
//    {
//        memcpy(m_whand, m_uartrx, m_readnum);
//        m_whand = m_whand + m_readnum;
//    }
//    else if ((m_whand + m_readnum) == (m_FIFO + BUFSIZE))
//    {
//        memcpy(m_whand, m_uartrx, m_readnum);
//        m_whand = m_FIFO;
//    }
//    else
//    {
//        const uint8_t left_size = m_FIFO + BUFSIZE - m_whand;
//        memcpy(m_whand, m_uartrx, left_size);
//        m_whand = m_FIFO;
//        memcpy(m_whand, m_uartrx + left_size, m_readnum - left_size);
//        m_whand = m_FIFO + m_readnum - left_size;
//    }
//    m_leftsize = m_leftsize + m_readnum;
//
//    supercap.Txsuper.limit = data.robot_status_t.chassis_power_limit;
//    supercap.Txsuper.buffer = data.power_heat_data_t.buffer_energy;
//    //}
//}

// 在 judgement.cpp 顶部（和其他调试变量一起放）加上：
uint16_t judge_debug_len = 0;
uint8_t  judge_debug_b0 = 0;
uint8_t  judge_debug_b1 = 0;
uint8_t  judge_debug_b2 = 0;


void Judgement::BuffData()
{
    if (queueHandler == NULL || *queueHandler == NULL)
        return;

    // ★ 从队列里收一包原始数据到 m_uartrx
    if (xQueueReceive(*queueHandler, m_uartrx, 0) != pdPASS)
        return;

    // ★ 先不要再信 m_uart->dataDmaNum 了
    //   先用队列元素大小（这里假设就是 DMA_RX_SIZE）
    m_readnum = DMA_RX_SIZE;

    // 调试看看队列里来的东西是不是对的
    judge_debug_len = m_readnum;
    judge_debug_b0 = m_uartrx[0];
    judge_debug_b1 = m_uartrx[1];
    judge_debug_b2 = m_uartrx[2];

    // === 先把 FIFO 全部跳过，避免扰乱 ===
    // 如果你后面想用 FIFO + Transmit 那套，再单独整理

    /*
    if ((m_whand + m_readnum) < (m_FIFO + BUFSIZE))
    {
        memcpy(m_whand, m_uartrx, m_readnum);
        m_whand = m_whand + m_readnum;
    }
    else if ((m_whand + m_readnum) == (m_FIFO + BUFSIZE))
    {
        memcpy(m_whand, m_uartrx, m_readnum);
        m_whand = m_FIFO;
    }
    else
    {
        const uint8_t left_size = m_FIFO + BUFSIZE - m_whand;
        memcpy(m_whand, m_uartrx, left_size);
        m_whand = m_FIFO;
        memcpy(m_whand, m_uartrx + left_size, m_readnum - left_size);
        m_whand = m_FIFO + m_readnum - left_size;
    }
    m_leftsize = m_leftsize + m_readnum;
    */

    supercap.Txsuper.limit = data.robot_status_t.chassis_power_limit;
    supercap.Txsuper.buffer = data.power_heat_data_t.buffer_energy;
}



void Judgement::Init(UART* huart, uint32_t baud, USART_TypeDef* uart_base)
{
    huart->Init(uart_base, baud).DMARxInit();
    m_uart = huart;
    queueHandler = &huart->UartQueueHandler;
}

// 根据协议自己定义一个最大 data 长度，防止异常长度把缓冲区撑爆
#define JUDGEMENT_MAX_DATA_LEN  200u

// 加几个调试变量（放在 Judgement 类里或者 cpp 顶部加 static/全局）:
uint8_t judge_debug_step = 0;
uint16_t judge_debug_dlen = 0;
uint16_t judge_debug_frame_len = 0;


void Judgement::GetData(void)
{
    // ★ 一进函数立刻标记一下
    judge_debug_step = 1;

    uint16_t len = (uint16_t)m_readnum;
    if (len < 9)
    {
        // 帮你再看一眼当前长度
        judge_debug_dlen = len;
        // 这里就先返回
        return;
    }

    judge_debug_dlen = len;  // 保存一下本帧总长度

    uint16_t idx = 0;

    while (idx + 9 <= len)
    {
        // 1. 找 SOF
        if (m_uartrx[idx] != 0xA5)
        {
            idx++;
            continue;
        }

        judge_debug_step = 2;

        // 2. CRC8
        if (idx + 5 > len)
            break;

        if (!VerifyCRC8CheckSum(&m_uartrx[idx], 5))
        {
            judge_debug_step = 3;
            idx++;
            continue;
        }

        judge_debug_step = 4;

        // 3. data_length
        const uint16_t data_length =
            (uint16_t)m_uartrx[idx + 1] |
            (uint16_t)(m_uartrx[idx + 2] << 8);

        judge_debug_dlen = data_length;

        if (data_length == 0 || data_length > JUDGEMENT_MAX_DATA_LEN)
        {
            judge_debug_step = 5;
            idx++;
            continue;
        }

        const uint16_t frame_length = (uint16_t)(data_length + 9);
        judge_debug_frame_len = frame_length;

        if (idx + frame_length > len)
        {
            judge_debug_step = 6;
            break;
        }

        judge_debug_step = 7;

        if (!VerifyCRC16CheckSum(&m_uartrx[idx], frame_length))
        {
            judge_debug_step = 8;
            idx++;
            continue;
        }

        // 一帧完整 OK
        judge_debug_step = 9;
        Decode(&m_uartrx[idx]);

        idx += frame_length;
    }
}




void Judgement::DisplayStaticUI()
{
    string_data_struct_t staticStringUI;

    switch (count % 100)
    {
    case 0:
    {
        char UIRPData[5] = {};

        UIRPData[0] = 'G';
        UIRPData[1] = 'E';
        UIRPData[2] = 'A';
        UIRPData[3] = 'R';

        Char_Draw(&staticStringUI, (char*)"PI", UI_Graph_ADD, 9, UI_Color_Green,
            30, 5, 3, 60, 820, UIRPData);

        Char_ReFresh(&staticStringUI);
        break;
    }
    case 10:
        break;
    case 20:
    {
        char UICapData[5] = {};
        UICapData[0] = 'C';
        UICapData[1] = 'A';
        UICapData[2] = 'P';

        Char_Draw(&staticStringUI, (char*)"CA", UI_Graph_ADD, 7, UI_Color_Green,
            30, 3, 3, 60, 760, UICapData);

        Char_ReFresh(&staticStringUI);
        break;
    }
    case 30:
    {
        char UIModeData[5] = {};
        UIModeData[0] = 'M';
        UIModeData[1] = 'O';
        UIModeData[2] = 'D';
        UIModeData[3] = 'E';

        Char_Draw(&staticStringUI, (char*)"MO", UI_Graph_ADD, 6, UI_Color_Green,
            30, 5, 3, 60, 700, UIModeData);

        Char_ReFresh(&staticStringUI);
        break;
    }
    case 40:
    {
        char UICaptureData[7] = {};

        UICaptureData[0] = 'O';
        UICaptureData[1] = 'P';
        UICaptureData[2] = 'E';
        UICaptureData[3] = 'N';
        UICaptureData[4] = 'R';
        UICaptureData[5] = 'U';
        UICaptureData[6] = 'B';

        Char_Draw(&staticStringUI, (char*)"CPT", UI_Graph_ADD, 5, UI_Color_Green,
            30, 7, 3, 60, 640, UICaptureData);

        Char_ReFresh(&staticStringUI);
        break;
    }
    case 50:
        break;
    case 60:
        break;
    case 70:
    {
        char UICapDisplayData3[3] = {};
        UICapDisplayData3[0] = '2';
        UICapDisplayData3[1] = '4';
        UICapDisplayData3[2] = 'V';

        Char_Draw(&staticStringUI, (char*)"C2", UI_Graph_ADD, 3, UI_Color_Green,
            20, 3, 2, 1300, 70, UICapDisplayData3);

        Char_ReFresh(&staticStringUI);
        break;
    }
    case 80:
    {
        graphic_data_struct_t staticGraohUI3[5] = {};
        Rectangle_Draw(&staticGraohUI3[0], (char*)"CR", UI_Graph_ADD, 3,
            UI_Color_Green, 2, 600, 100, 1320, 150);
        LineDraw(&staticGraohUI3[1], (char*)"CL", UI_Graph_ADD, 3,
            UI_Color_Yellow, 2, 1032, 100, 1032, 150);
        LineDraw(&staticGraohUI3[2], (char*)"LL1", UI_Graph_ADD, 3,
            UI_Color_Main, 2, 480, 100, 700, 400);
        LineDraw(&staticGraohUI3[3], (char*)"LL2", UI_Graph_ADD, 3,
            UI_Color_Main, 2, 1440, 100, 1220, 400);

        UI_ReFresh(5, staticGraohUI3);
        break;
    }
    case 90:
    {
        graphic_data_struct_t staticGraohUI2[7] = {};

        LineDraw(&staticGraohUI2[0], (char*)"L1", UI_Graph_ADD, 2,
            UI_Color_Purplish_red, 2, 1011, 280, 1011, 600);
        Circle_Draw(&staticGraohUI2[3], (char*)"R1", UI_Graph_ADD, 2,
            UI_Color_Purplish_red, 3, 1011, 505, 10);

        UI_ReFresh(7, staticGraohUI2);
        break;
    }
    default:
        break;
    }
}

void Judgement::DisplayRP(int flag)
{
    char RP[2] = {};
    string_data_struct_t RPData;

    switch (flag)
    {
    case 0:
        RP[0] = 'D';
        RP[1] = '0';
        break;
    case 1:
        RP[0] = 'D';
        RP[1] = '1';
        break;
    case 2:
        RP[0] = 'D';
        RP[1] = '2';
        break;
    default:
        break;
    }

    if (!graphInit)
    {
        Char_Draw(&RPData, (char*)"aa", UI_Graph_ADD, 5, UI_Color_Green,
            30, 2, 3, 280, 820, RP);
    }
    else
    {
        Char_Draw(&RPData, (char*)"aa", UI_Graph_Change, 5, UI_Color_Green,
            30, 2, 3, 280, 820, RP);
    }

    Char_ReFresh(&RPData);
}

void Judgement::DisplayCapState(uint8_t capState)
{
    char capStateChar[5] = {};
    string_data_struct_t capStateData;

    if (capState == WORKING)
    {
        capStateChar[0] = 'W';
        capStateChar[1] = 'O';
        capStateChar[2] = 'R';
        capStateChar[3] = 'K';
    }
    else if (capState == DISCHARGE)
    {
        capStateChar[0] = 'D';
        capStateChar[1] = 'I';
        capStateChar[2] = 'S';
        capStateChar[3] = 'C';
        capStateChar[4] = 'H';
    }
    else if (capState == SHUT)
    {
        capStateChar[0] = 'S';
        capStateChar[1] = 'H';
        capStateChar[2] = 'U';
        capStateChar[3] = 'T';
    }

    if (!graphInit)
    {
        Char_Draw(&capStateData, (char*)"CO1", UI_Graph_ADD, 7, UI_Color_Green,
            30, 5, 3, 280, 760, capStateChar);
    }
    else
    {
        Char_Draw(&capStateData, (char*)"CO1", UI_Graph_Change, 7, UI_Color_Green,
            30, 5, 3, 280, 760, capStateChar);
    }

    Char_ReFresh(&capStateData);
}

void Judgement::DisplpayMode(uint8_t mode)
{
    CONTROL::MODE _mode = (CONTROL::MODE)mode;

    char modeChar[7] = {};
    string_data_struct_t modeData;

    switch (_mode)
    {
    case CONTROL::MODE::LOCK:
        modeChar[0] = 'L';
        modeChar[1] = 'O';
        modeChar[2] = 'C';
        modeChar[3] = 'K';
        break;
    case CONTROL::MODE::FOLLOW:
        modeChar[0] = 'F';
        modeChar[1] = 'O';
        modeChar[2] = 'L';
        modeChar[3] = 'L';
        modeChar[4] = 'O';
        modeChar[5] = 'W';
        break;
    case CONTROL::MODE::ROTATION:
        modeChar[0] = 'R';
        modeChar[1] = 'O';
        modeChar[2] = 'T';
        modeChar[3] = 'A';
        break;
    case CONTROL::MODE::SEPARATE:
        modeChar[0] = 'S';
        modeChar[1] = 'E';
        modeChar[2] = 'P';
        break;
    default:
        break;
    }

    if (!graphInit)
    {
        Char_Draw(&modeData, (char*)"MD", UI_Graph_ADD, 6, UI_Color_Green,
            30, 7, 3, 280, 700, modeChar);
    }
    else
    {
        Char_Draw(&modeData, (char*)"MD", UI_Graph_Change, 6, UI_Color_Green,
            30, 7, 3, 280, 700, modeChar);
    }

    Char_ReFresh(&modeData);
}

void Judgement::DisplayCapture(bool isCapture)
{
    char captureChar[5] = {};
    string_data_struct_t captureData;

    if (isCapture)
    {
        captureChar[0] = 'T';
        captureChar[1] = 'R';
        captureChar[2] = 'U';
        captureChar[3] = 'E';
    }
    else
    {
        captureChar[0] = 'F';
        captureChar[1] = 'A';
        captureChar[2] = 'L';
        captureChar[3] = 'S';
        captureChar[4] = 'E';
    }

    if (!graphInit)
    {
        Char_Draw(&captureData, (char*)"CO", UI_Graph_ADD, 5, UI_Color_Green,
            30, 5, 3, 280, 640, captureChar);
    }
    else
    {
        Char_Draw(&captureData, (char*)"CO", UI_Graph_Change, 5, UI_Color_Green,
            30, 5, 3, 280, 640, captureChar);
    }

    Char_ReFresh(&captureData);
}

void Judgement::DisplayCapVoltage(float capVoltage)
{
    uint32_t voltagePos{};
    graphic_data_struct_t voltageData;

    voltagePos = static_cast<uint32_t>(capVoltage * 720.0f / 2000.0f) + 600;

    if (voltagePos < 600)
        voltagePos = 600;
    else if (voltagePos > 1320)
        voltagePos = 1320;

    if (!graphInit)
    {
        LineDraw(&voltageData, (char*)"VD", UI_Graph_ADD, 3, UI_Color_Yellow,
            50, 600, 125, voltagePos, 125);
    }
    else
    {
        LineDraw(&voltageData, (char*)"VD", UI_Graph_Change, 3, UI_Color_Yellow,
            50, 600, 125, voltagePos, 125);
    }

    UI_ReFresh(1, &voltageData);
}

void Judgement::SendData(void)
{
    // 根据机器人 ID 自动匹配客户端 ID
    robotId = data.robot_status_t.robot_id;
    clientId = robotId | 0x100;

    if (count < 200)
    {
        DisplayStaticUI();
    }
    else
    {
        switch (count % 20)
        {
        case 0:
            DisplayRP(rc.gear);
            break;
        case 4:
            DisplayCapState(supercap.Txsuper.state);
            break;
        case 8:
            DisplayCapVoltage(supercap.Rxsuper.cap_energy);
            break;
        case 12:
            DisplayCapture(ctrl.shooter.openRub);
            break;
        case 16:
            // DisplpayMode(ctrl.mode[now]);
            break;
        default:
            break;
        }
    }

    if (count > 2000)
    {
        graphInit = true;
    }

    count++;
}

void Judgement::Decode(uint8_t* m_frame)
{
    const uint16_t cmdID =
        static_cast<uint16_t>(m_frame[5] | (m_frame[6] << 8));
    data.CmdID = cmdID;
    uint8_t* rawdata = &m_frame[7];

    switch (cmdID)
    {
        // 0x0001 比赛状态
    case 0x0001:
        data.game_status_t.game_type =
            static_cast<uint8_t>(rawdata[0] & 0x0F);
        data.game_status_t.game_progress =
            static_cast<uint8_t>(rawdata[0] >> 4);
        data.game_status_t.stage_remain_time =
            static_cast<uint16_t>(rawdata[1] | (rawdata[2] << 8));
        {
            uint64_t ts = 0;
            for (int i = 0; i < 8; ++i)
            {
                ts |= (static_cast<uint64_t>(rawdata[3 + i]) << (8 * i));
            }
            data.game_status_t.SyncTimeStamp = ts;
        }
        break;

        // 0x0002 比赛结果
    case 0x0002:
        data.game_result_t.winner = rawdata[0];
        break;

        // 0x0003 己方血量
    case 0x0003:
        data.game_robot_HP_t.ally_1_robot_HP =
            static_cast<uint16_t>(rawdata[0] | (rawdata[1] << 8));
        data.game_robot_HP_t.ally_2_robot_HP =
            static_cast<uint16_t>(rawdata[2] | (rawdata[3] << 8));
        data.game_robot_HP_t.ally_3_robot_HP =
            static_cast<uint16_t>(rawdata[4] | (rawdata[5] << 8));
        data.game_robot_HP_t.ally_4_robot_HP =
            static_cast<uint16_t>(rawdata[6] | (rawdata[7] << 8));
        data.game_robot_HP_t.reserved =
            static_cast<uint16_t>(rawdata[8] | (rawdata[9] << 8));
        data.game_robot_HP_t.ally_7_robot_HP =
            static_cast<uint16_t>(rawdata[10] | (rawdata[11] << 8));
        data.game_robot_HP_t.ally_outpost_HP =
            static_cast<uint16_t>(rawdata[12] | (rawdata[13] << 8));
        data.game_robot_HP_t.ally_base_HP =
            static_cast<uint16_t>(rawdata[14] | (rawdata[15] << 8));
        break;

        // 0x0101 场地事件
    case 0x0101:
        data.event_data_t.event_data =
            static_cast<uint32_t>(rawdata[0]) |
            (static_cast<uint32_t>(rawdata[1]) << 8) |
            (static_cast<uint32_t>(rawdata[2]) << 16) |
            (static_cast<uint32_t>(rawdata[3]) << 24);
        break;

        // 0x0102 补给站动作
    case 0x0102:
        data.ext_supply_projectile_action_t.reserved = rawdata[0];
        data.ext_supply_projectile_action_t.supply_robot_id = rawdata[1];
        data.ext_supply_projectile_action_t.supply_projectile_step = rawdata[2];
        data.ext_supply_projectile_action_t.supply_projectile_num = rawdata[3];
        break;

        // 0x0104 裁判警告
    case 0x0104:
        data.referee_warning_t.level = rawdata[0];
        data.referee_warning_t.foul_robot_id = rawdata[1];
        data.referee_warning_t.count = rawdata[2];
        break;

        // 0x0105 飞镖倒计时
    case 0x0105:
        data.dart_dart_info_t.dart_remaining_time = rawdata[0];
        data.dart_dart_info_t.dart_info =
            static_cast<uint16_t>(rawdata[1] | (rawdata[2] << 8));
        break;

        // 0x0201 机器人状态
    case 0x0201:
        data.robot_status_t.robot_id = rawdata[0];
        judgementready = true;
        data.robot_status_t.robot_level =
            rawdata[1];
        data.robot_status_t.current_HP =
            static_cast<uint16_t>(rawdata[2] | (rawdata[3] << 8));
        data.robot_status_t.maximum_HP =
            static_cast<uint16_t>(rawdata[4] | (rawdata[5] << 8));
        data.robot_status_t.shooter_barrel_cooling_value =
            static_cast<uint16_t>(rawdata[6] | (rawdata[7] << 8));
        data.robot_status_t.shooter_barrel_heat_limit =
            static_cast<uint16_t>(rawdata[8] | (rawdata[9] << 8));
        data.robot_status_t.chassis_power_limit =
            static_cast<uint16_t>(rawdata[10] | (rawdata[11] << 8));
        data.robot_status_t.power_management_gimbal_output =
            static_cast<uint16_t>(rawdata[12] & 0x01);
        data.robot_status_t.power_management_chassis_output =
            static_cast<uint16_t>((rawdata[12] & 0x02) >> 1);
        data.robot_status_t.power_management_shooter_output =
            static_cast<uint16_t>((rawdata[12] & 0x04) >> 2);
        break;

        // 0x0202 缓冲能量 & 热量
    case 0x0202:
        powerheatready = true;

        data.power_heat_data_t.reserved0 =
            static_cast<uint16_t>(rawdata[0] | (rawdata[1] << 8));
        data.power_heat_data_t.reserved1 =
            static_cast<uint16_t>(rawdata[2] | (rawdata[3] << 8));
        data.power_heat_data_t.reserved2 = u32_to_float(&rawdata[4]);
        data.power_heat_data_t.buffer_energy =
            static_cast<uint16_t>(rawdata[8] | (rawdata[9] << 8));
        data.power_heat_data_t.shooter_17mm_1_barrel_heat =
            static_cast<uint16_t>(rawdata[10] | (rawdata[11] << 8));
        data.power_heat_data_t.shooter_42mm_barrel_heat =
            static_cast<uint16_t>(rawdata[12] | (rawdata[13] << 8));

        supercap.Txsuper.limit = data.robot_status_t.chassis_power_limit;
        supercap.Txsuper.buffer = data.power_heat_data_t.buffer_energy;
        break;

        // 0x0203 位置
    case 0x0203:
        data.robot_pos_t.x = u32_to_float(&rawdata[0]);
        data.robot_pos_t.y = u32_to_float(&rawdata[4]);
        data.robot_pos_t.angle = u32_to_float(&rawdata[8]);
        break;

        // 0x0204 增益
    case 0x0204:
        data.buff_t.recovery_buff = rawdata[0];
        data.buff_t.cooling_buff =
            static_cast<uint16_t>(rawdata[1] | (rawdata[2] << 8));
        data.buff_t.defence_buff = rawdata[3];
        data.buff_t.vulnerability_buff = rawdata[4];
        data.buff_t.attack_buff =
            static_cast<uint16_t>(rawdata[5] | (rawdata[6] << 8));
        data.buff_t.remaining_energy = rawdata[7];
        break;

        // 0x0205 空中能量
    case 0x0205:
        data.air_support_data_t.airforce_status = rawdata[0];
        data.air_support_data_t.time_remain = rawdata[1];
        break;

        // 0x0206 伤害
    case 0x0206:
        data.hurt_data_t.armor_id =
            static_cast<uint8_t>(rawdata[0] & 0x0F);
        data.hurt_data_t.HP_deduction_reason =
            static_cast<uint8_t>(rawdata[0] >> 4);
        break;

        // 0x0207 射击
    case 0x0207:
        data.shoot_data_t.bullet_type = rawdata[0];
        data.shoot_data_t.shooter_number = rawdata[1];
        data.shoot_data_t.bullet_freq = rawdata[2];
        data.shoot_data_t.bullet_speed = u32_to_float(&rawdata[3]);

        if (prebulletspd != data.shoot_data_t.bullet_speed)
        {
            nBullet++;
            prebulletspd = data.shoot_data_t.bullet_speed;
        }
        break;

        // 0x0208 发弹量
    case 0x0208:
        data.projectile_allowance_t.projectile_allowance_17mm =
            static_cast<uint16_t>(rawdata[0] | (rawdata[1] << 8));
        data.projectile_allowance_t.projectile_allowance_42mm =
            static_cast<uint16_t>(rawdata[2] | (rawdata[3] << 8));
        data.projectile_allowance_t.remaining_gold_coin =
            static_cast<uint16_t>(rawdata[4] | (rawdata[5] << 8));
        data.projectile_allowance_t.projectile_allowance_fortress =
            static_cast<uint16_t>(rawdata[6] | (rawdata[7] << 8));
        break;

        // 0x0209 RFID
    case 0x0209:
    {
        uint32_t rfid =
            static_cast<uint32_t>(rawdata[0]) |
            (static_cast<uint32_t>(rawdata[1]) << 8) |
            (static_cast<uint32_t>(rawdata[2]) << 16) |
            (static_cast<uint32_t>(rawdata[3]) << 24);

        data.rfid_status_t.rfid_status = rfid;
        data.rfid_status_t.rfid_status_2 = rawdata[4];

        baseRFID =
            static_cast<uint8_t>((rfid & (1u << 0)) ? 1 : 0);
        highlandRFID =
            static_cast<uint8_t>(((rfid & (1u << 1)) || (rfid & (1u << 2))) ? 1 : 0);
        feipoRFID =
            static_cast<uint8_t>(((rfid & (1u << 5)) || (rfid & (1u << 6)) ||
                (rfid & (1u << 7)) || (rfid & (1u << 8))) ? 1 : 0);
        outpostRFID =
            static_cast<uint8_t>((rfid & (1u << 18)) ? 1 : 0);
        resourseRFID =
            static_cast<uint8_t>(((rfid & (1u << 19)) || (rfid & (1u << 20))) ? 1 : 0);

        energyRFID = 0; // 能量机关可从 event_data_t 等其它地方推
        break;
    }

    // 0x020A 飞镖客户端指令
    case 0x020A:
        data.dart_client_cmd_t.dart_launch_opening_status = rawdata[0];
        data.dart_client_cmd_t.reserved = rawdata[1];
        data.dart_client_cmd_t.target_change_time =
            static_cast<uint16_t>(rawdata[2] | (rawdata[3] << 8));
        data.dart_client_cmd_t.latest_launch_cmd_time =
            static_cast<uint16_t>(rawdata[4] | (rawdata[5] << 8));
        break;

        // 0x020B 地面机器人位置
    case 0x020B:
        data.ground_robot_position_t.hero_x = u32_to_float(&rawdata[0]);
        data.ground_robot_position_t.hero_y = u32_to_float(&rawdata[4]);
        data.ground_robot_position_t.engineer_x = u32_to_float(&rawdata[8]);
        data.ground_robot_position_t.engineer_y = u32_to_float(&rawdata[12]);
        data.ground_robot_position_t.standard_3_x = u32_to_float(&rawdata[16]);
        data.ground_robot_position_t.standard_3_y = u32_to_float(&rawdata[20]);
        data.ground_robot_position_t.standard_4_x = u32_to_float(&rawdata[24]);
        data.ground_robot_position_t.standard_4_y = u32_to_float(&rawdata[28]);
        data.ground_robot_position_t.standard_5_x = u32_to_float(&rawdata[32]);
        data.ground_robot_position_t.standard_5_y = u32_to_float(&rawdata[36]);
        break;

    default:
        break;
    }
}

bool Judgement::Transmit(uint32_t read_size, uint8_t* plate)
{
    if (m_leftsize < read_size) return false;

    if ((m_rhand + read_size) < (m_FIFO + BUFSIZE))
    {
        memcpy(plate, m_rhand, read_size);
        m_rhand = m_rhand + read_size;
    }
    else if ((m_rhand + read_size) == (m_FIFO + BUFSIZE))
    {
        memcpy(plate, m_rhand, read_size);
        m_rhand = m_FIFO;
    }
    else
    {
        const uint8_t left_size = m_FIFO + BUFSIZE - m_rhand;
        memcpy(plate, m_rhand, left_size);
        memcpy(plate + left_size, m_rhand = m_FIFO,
            read_size - left_size);
        m_rhand = m_FIFO + read_size - left_size;
    }

    m_leftsize = m_leftsize - read_size;
    return true;
}

/************************************************绘制直线*************************************************/
void Judgement::LineDraw(graphic_data_struct_t* image, char imagename[3],
    uint32_t Graph_Operate, uint32_t Graph_Layer,
    uint32_t Graph_Color, uint32_t Graph_Width,
    uint32_t Start_x, uint32_t Start_y,
    uint32_t End_x, uint32_t End_y)
{
    int i;
    for (i = 0; i < 3 && imagename[i] != 0; i++)
        image->figure_name[2 - i] = imagename[i];

    image->figure_tpye = UI_Graph_Line;
    image->operate_tpye = Graph_Operate;
    image->layer = Graph_Layer;
    image->color = Graph_Color;
    image->width = Graph_Width;
    image->start_x = Start_x;
    image->start_y = Start_y;
    image->end_x = End_x;
    image->end_y = End_y;
}

/************************************************绘制矩形*************************************************/
void Judgement::Rectangle_Draw(graphic_data_struct_t* image, char imagename[3],
    uint32_t Graph_Operate, uint32_t Graph_Layer,
    uint32_t Graph_Color, uint32_t Graph_Width,
    uint32_t Start_x, uint32_t Start_y,
    uint32_t End_x, uint32_t End_y)
{
    int i;
    for (i = 0; i < 3 && imagename[i] != 0; i++)
        image->figure_name[2 - i] = imagename[i];

    image->figure_tpye = UI_Graph_Rectangle;
    image->operate_tpye = Graph_Operate;
    image->layer = Graph_Layer;
    image->color = Graph_Color;
    image->width = Graph_Width;
    image->start_x = Start_x;
    image->start_y = Start_y;
    image->end_x = End_x;
    image->end_y = End_y;
}

/************************************************绘制整圆*************************************************/
void Judgement::Circle_Draw(graphic_data_struct_t* image, char imagename[3],
    uint32_t Graph_Operate, uint32_t Graph_Layer,
    uint32_t Graph_Color, uint32_t Graph_Width,
    uint32_t Start_x, uint32_t Start_y,
    uint32_t Graph_Radius)
{
    int i;
    for (i = 0; i < 3 && imagename[i] != 0; i++)
        image->figure_name[2 - i] = imagename[i];

    image->figure_tpye = UI_Graph_Circle;
    image->operate_tpye = Graph_Operate;
    image->layer = Graph_Layer;
    image->color = Graph_Color;
    image->width = Graph_Width;
    image->start_x = Start_x;
    image->start_y = Start_y;
    image->radius = Graph_Radius;
}

/************************************************绘制圆弧*************************************************/
void Judgement::Arc_Draw(graphic_data_struct_t* image, char imagename[3],
    uint32_t Graph_Operate, uint32_t Graph_Layer,
    uint32_t Graph_Color, uint32_t Graph_StartAngle,
    uint32_t Graph_EndAngle, uint32_t Graph_Width,
    uint32_t Start_x, uint32_t Start_y,
    uint32_t x_Length, uint32_t y_Length)
{
    int i;
    for (i = 0; i < 3 && imagename[i] != 0; i++)
        image->figure_name[2 - i] = imagename[i];

    image->figure_tpye = UI_Graph_Arc;
    image->operate_tpye = Graph_Operate;
    image->layer = Graph_Layer;
    image->color = Graph_Color;
    image->width = Graph_Width;
    image->start_x = Start_x;
    image->start_y = Start_y;
    image->start_angle = Graph_StartAngle;
    image->end_angle = Graph_EndAngle;
    image->end_x = x_Length;
    image->end_y = y_Length;
}

/************************************************绘制浮点型数据*************************************************/
void Judgement::Float_Draw(float_data_struct_t* image, char imagename[3],
    uint32_t Graph_Operate, uint32_t Graph_Layer,
    uint32_t Graph_Color, uint32_t Graph_Size,
    uint32_t Graph_Digit, uint32_t Graph_Width,
    uint32_t Start_x, uint32_t Start_y,
    float Graph_Float)
{
    int i;
    for (i = 0; i < 3 && imagename[i] != 0; i++)
        image->figure_name[2 - i] = imagename[i];

    image->figure_tpye = UI_Graph_Float;
    image->operate_tpye = Graph_Operate;
    image->layer = Graph_Layer;
    image->color = Graph_Color;
    image->width = Graph_Width;
    image->start_x = Start_x;
    image->start_y = Start_y;
    image->start_angle = Graph_Size;
    image->end_angle = Graph_Digit;

    int32_t temp1 = static_cast<int32_t>(Graph_Float * 1000.0f);
    int32_t temp2 = temp1 / 1024;
    image->end_x = temp2;
    image->radius = temp1 - temp2 * 1024; // 1 -> 1.024e-3
}

/************************************************绘制字符型数据*************************************************/
void Judgement::Char_Draw(string_data_struct_t* image, char imagename[3],
    uint32_t Graph_Operate, uint32_t Graph_Layer,
    uint32_t Graph_Color, uint32_t Graph_Size,
    uint32_t Graph_Digit, uint32_t Graph_Width,
    uint32_t Start_x, uint32_t Start_y,
    char* Char_Data)
{
    int i;
    for (i = 0; i < 3 && imagename[i] != 0; i++)
        image->Graph_Control.figure_name[2 - i] = imagename[i];

    image->Graph_Control.figure_tpye = UI_Graph_Char;
    image->Graph_Control.operate_tpye = Graph_Operate;
    image->Graph_Control.layer = Graph_Layer;
    image->Graph_Control.color = Graph_Color;
    image->Graph_Control.width = Graph_Width;
    image->Graph_Control.start_x = Start_x;
    image->Graph_Control.start_y = Start_y;
    image->Graph_Control.start_angle = Graph_Size;
    image->Graph_Control.end_angle = Graph_Digit;

    for (i = 0; i < static_cast<int>(Graph_Digit); i++)
    {
        image->show_Data[i] = *Char_Data;
        Char_Data++;
    }
}

/************************************************UI删除函数*************************************************/
void Judgement::UIDelete(uint8_t deleteOperator, uint8_t deleteLayer)
{
    uint16_t dataLength;
    CommunatianData_graphic_t UIDeleteData;

    UIDeleteData.txFrameHeader.sof = 0xA5;
    UIDeleteData.txFrameHeader.data_length = 8;
    UIDeleteData.txFrameHeader.seq = UI_seq;

    memcpy(m_uarttx, &UIDeleteData.txFrameHeader, sizeof(frame_header_t));
    AppendCRC8CheckSum(m_uarttx, sizeof(frame_header_t));

    UIDeleteData.CMD = UI_CMD_Robo_Exchange;
    UIDeleteData.txID.data_cmd_id = UI_Data_ID_Del;
    UIDeleteData.txID.receiver_ID = clientId;
    UIDeleteData.txID.sender_ID = robotId;

    memcpy(m_uarttx + 5, (uint8_t*)&UIDeleteData.CMD, 8);

    m_uarttx[13] = deleteOperator;
    m_uarttx[14] = deleteLayer;

    dataLength = sizeof(CommunatianData_graphic_t) + 2;

    AppendCRC16CheckSum(m_uarttx, dataLength);

    m_uart->UARTTransmit(m_uarttx, dataLength);
    UI_seq++;
}

/************************************************UI推送图形*************************************************/
void Judgement::UI_ReFresh(int cnt, graphic_data_struct_t* imageData)
{
    uint8_t dataLength;
    CommunatianData_graphic_t graphicData;
    memset(m_uarttx, 0, DMA_TX_SIZE);

    graphicData.txFrameHeader.sof = UI_SOF;
    graphicData.txFrameHeader.data_length = 6 + cnt * 15;
    graphicData.txFrameHeader.seq = UI_seq;

    memcpy(m_uarttx, &graphicData.txFrameHeader, sizeof(frame_header_t));
    AppendCRC8CheckSum(m_uarttx, sizeof(frame_header_t));

    graphicData.CMD = UI_CMD_Robo_Exchange;
    switch (cnt)
    {
    case 1:
        graphicData.txID.data_cmd_id = UI_Data_ID_Draw1;
        break;
    case 2:
        graphicData.txID.data_cmd_id = UI_Data_ID_Draw2;
        break;
    case 5:
        graphicData.txID.data_cmd_id = UI_Data_ID_Draw5;
        break;
    case 7:
        graphicData.txID.data_cmd_id = UI_Data_ID_Draw7;
        break;
    default:
        break;
    }

    graphicData.txID.sender_ID = robotId;
    graphicData.txID.receiver_ID = clientId;

    memcpy(m_uarttx + 5, (uint8_t*)&graphicData.CMD, 8);

    memcpy(m_uarttx + 13, imageData,
        cnt * sizeof(graphic_data_struct_t));
    dataLength = sizeof(CommunatianData_graphic_t) +
        cnt * sizeof(graphic_data_struct_t);

    AppendCRC16CheckSum(m_uarttx, dataLength);

    m_uart->UARTTransmit(m_uarttx, dataLength);
    UI_seq++;
}

/************************************************UI推送浮点*************************************************/
void Judgement::UI_ReFresh(int cnt, float_data_struct_t* floatdata)
{
    uint8_t dataLength;
    CommunatianData_graphic_t graphicData;
    memset(m_uarttx, 0, DMA_TX_SIZE);

    graphicData.txFrameHeader.sof = UI_SOF;
    graphicData.txFrameHeader.data_length =
        6 + cnt * sizeof(graphic_data_struct_t);
    graphicData.txFrameHeader.seq = UI_seq;

    memcpy(m_uarttx, &graphicData.txFrameHeader, sizeof(frame_header_t));
    AppendCRC8CheckSum(m_uarttx, sizeof(frame_header_t));

    graphicData.CMD = UI_CMD_Robo_Exchange;
    switch (cnt)
    {
    case 1:
        graphicData.txID.data_cmd_id = UI_Data_ID_Draw1;
        break;
    case 2:
        graphicData.txID.data_cmd_id = UI_Data_ID_Draw2;
        break;
    case 5:
        graphicData.txID.data_cmd_id = UI_Data_ID_Draw5;
        break;
    case 7:
        graphicData.txID.data_cmd_id = UI_Data_ID_Draw7;
        break;
    default:
        break;
    }

    graphicData.txID.sender_ID = robotId;
    graphicData.txID.receiver_ID = clientId;

    memcpy(m_uarttx + 5, (uint8_t*)&graphicData.CMD, 8);

    memcpy(m_uarttx + 13, floatdata,
        cnt * sizeof(graphic_data_struct_t));
    dataLength = sizeof(CommunatianData_graphic_t) +
        cnt * sizeof(graphic_data_struct_t);

    AppendCRC16CheckSum(m_uarttx, dataLength);

    m_uart->UARTTransmit(m_uarttx, dataLength);
    UI_seq++;
}

/************************************************UI推送字符*************************************************/
void Judgement::Char_ReFresh(string_data_struct_t* string_Data)
{
    uint8_t dataLength;
    CommunatianData_graphic_t graphicData;
    memset(m_uarttx, 0, DMA_TX_SIZE);

    graphicData.txFrameHeader.sof = UI_SOF;
    graphicData.txFrameHeader.data_length = 51;
    graphicData.txFrameHeader.seq = UI_seq;

    memcpy(m_uarttx, &graphicData.txFrameHeader, sizeof(frame_header_t));
    AppendCRC8CheckSum(m_uarttx, sizeof(frame_header_t));

    graphicData.CMD = UI_CMD_Robo_Exchange;
    graphicData.txID.data_cmd_id = UI_Data_ID_DrawChar;
    graphicData.txID.sender_ID = robotId;
    graphicData.txID.receiver_ID = clientId;

    memcpy(m_uarttx + 5, (uint8_t*)&graphicData.CMD, 8);

    memcpy(m_uarttx + 13, string_Data, sizeof(string_data_struct_t));
    dataLength = sizeof(CommunatianData_graphic_t) +
        sizeof(string_data_struct_t);

    AppendCRC16CheckSum(m_uarttx, dataLength);

    m_uart->UARTTransmit(m_uarttx, dataLength);
    UI_seq++;
}
