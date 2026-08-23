#pragma once

#include <iostream>
#include <libserial/SerialPort.h>
#include <string>
#include <vector>
#include <memory>
#include <cstring>
#include <limits>
#include <cstdint>

using namespace LibSerial;

typedef union
{
    uint8_t U8_Buff[4];
    float Float;
} Bint32_Union;

typedef union
{
    uint8_t U8_Buff[2];
    uint16_t UInt16;
} Bint16_Union;

// 控制标志：fire=0xFF 开火，fire=0x00 不开火
typedef struct {
    uint8_t fire;
} Control_Flag_t;

class Driver
{
private:
    std::shared_ptr<SerialPort> _serial_port = nullptr;
    std::string _port_name;
    BaudRate _bps;
    bool isOpen = false;

    // 串口发送单个字节
    int send(unsigned char byte)
    {
        try
        {
            _serial_port->WriteByte(byte);
        }
        catch (const std::runtime_error &)
        {
            std::cerr << "Serial write runtime_error." << std::endl;
            return -2;
        }
        catch (const NotOpen &)
        {
            std::cerr << "Serial port not open." << std::endl;
            return -1;
        }
        return 0;
    }

    // 串口发送字节数组
    int send(const std::vector<unsigned char> &data)
    {
        try
        {
            _serial_port->Write(data);
        }
        catch (const std::runtime_error &)
        {
            std::cerr << "Serial write runtime_error." << std::endl;
            return -2;
        }
        catch (const NotOpen &)
        {
            std::cerr << "Serial port not open." << std::endl;
            return -1;
        }

        _serial_port->DrainWriteBuffer();
        return 0;
    }

    // CRC8 计算（多项式 0x07）
    static uint8_t calc_crc8(const std::vector<unsigned char> &buf, size_t len)
    {
        uint8_t crc = 0;
        for (size_t i = 0; i < len; i++)
        {
            crc ^= buf[i];
            for (int j = 0; j < 8; j++)
            {
                if (crc & 0x80)
                    crc = (crc << 1) ^ 0x07;
                else
                    crc <<= 1;
            }
        }
        return crc;
    }

public:
    Driver(const std::string &port_name, BaudRate bps)
        : _port_name(port_name), _bps(bps) {}
    ~Driver() { close(); }

    // 打开串口
    int open()
    {
        _serial_port = std::make_shared<SerialPort>();
        try
        {
            _serial_port->Open(_port_name);
            _serial_port->SetBaudRate(_bps);
            _serial_port->SetCharacterSize(CharacterSize::CHAR_SIZE_8);
            _serial_port->SetFlowControl(FlowControl::FLOW_CONTROL_NONE);
            _serial_port->SetParity(Parity::PARITY_NONE);
            _serial_port->SetStopBits(StopBits::STOP_BITS_1);
        }
        catch (...)
        {
            std::cerr << "Failed to open serial port: " << _port_name << std::endl;
            isOpen = false;
            return -1;
        }

        _serial_port->FlushIOBuffers();
        isOpen = true;
        return 0;
    }

    // 关闭串口
    void close()
    {
        if (_serial_port != nullptr)
        {
            _serial_port->Close();
            _serial_port = nullptr;
            isOpen = false;
        }
    }

    // -------------------------------------------------------
    // 上位机 -> 云台，总长 23 字节
    // Byte0:      0x19  帧头
    // Byte1~4:    int32 vx_set x1000（小端）
    // Byte5~8:    int32 vy_set x1000（小端）
    // Byte9~12:   int32 wz_set x1000（小端）
    // Byte13~16:  float yaw_set （小端）
    // Byte17~20:  float pitch_set（小端）
    // Byte21:     uint8 controlFlag (0xFF=开火, 0x00=不开火)
    // Byte22:     uint8 crc8 (对 Byte0~21 计算)
    // -------------------------------------------------------
    void nav_data(float vx, float vy, float wz, float yaw, float pitch,
                  const Control_Flag_t &controlFlag)
    {
        if (!isOpen)
        {
            std::cerr << "Serial port not open!" << std::endl;
            return;
        }

        std::cout << "\033[32m[Driver] Send cmd_vel_chassis: vx=" << vx
                  << " m/s, vy=" << vy
                  << " m/s, wz=" << wz
                  << " rad/s\033[0m" << std::endl;

        // float 速度 x1000 转 int32，小端存储
        int32_t vx_i = static_cast<int32_t>(vx * 1000.0f);
        int32_t vy_i = static_cast<int32_t>(vy * 1000.0f);
        int32_t wz_i = static_cast<int32_t>(wz * 1000.0f);

        Bint32_Union yaw_union, pitch_union;
        yaw_union.Float   = yaw;
        pitch_union.Float = pitch;

        std::vector<unsigned char> buf(23);

        // Byte0: 帧头
        buf[0] = 0x19;

        // Byte1~4: vx int32 小端
        buf[1] = static_cast<uint8_t>((vx_i >>  0) & 0xFF);
        buf[2] = static_cast<uint8_t>((vx_i >>  8) & 0xFF);
        buf[3] = static_cast<uint8_t>((vx_i >> 16) & 0xFF);
        buf[4] = static_cast<uint8_t>((vx_i >> 24) & 0xFF);

        // Byte5~8: vy int32 小端
        buf[5] = static_cast<uint8_t>((vy_i >>  0) & 0xFF);
        buf[6] = static_cast<uint8_t>((vy_i >>  8) & 0xFF);
        buf[7] = static_cast<uint8_t>((vy_i >> 16) & 0xFF);
        buf[8] = static_cast<uint8_t>((vy_i >> 24) & 0xFF);

        // Byte9~12: wz int32 小端
        buf[9]  = static_cast<uint8_t>((wz_i >>  0) & 0xFF);
        buf[10] = static_cast<uint8_t>((wz_i >>  8) & 0xFF);
        buf[11] = static_cast<uint8_t>((wz_i >> 16) & 0xFF);
        buf[12] = static_cast<uint8_t>((wz_i >> 24) & 0xFF);

        // Byte13~16: yaw float 小端
        buf[13] = yaw_union.U8_Buff[0];
        buf[14] = yaw_union.U8_Buff[1];
        buf[15] = yaw_union.U8_Buff[2];
        buf[16] = yaw_union.U8_Buff[3];

        // Byte17~20: pitch float 小端
        buf[17] = pitch_union.U8_Buff[0];
        buf[18] = pitch_union.U8_Buff[1];
        buf[19] = pitch_union.U8_Buff[2];
        buf[20] = pitch_union.U8_Buff[3];

        // Byte21: controlFlag
        buf[21] = controlFlag.fire;

        // Byte22: CRC8（对 Byte0~21 共22字节计算）
        buf[22] = calc_crc8(buf, 22);

        send(buf);
    }

    // 串口接收单字节（可添加超时）
    int recvdata(unsigned char &byte, size_t msTimeout = 0)
    {
        try
        {
            _serial_port->ReadByte(byte, msTimeout);
        }
        catch (const ReadTimeout &)
        {
            return -2;
        }
        catch (const NotOpen &)
        {
            return -1;
        }
        return 0;
    }

    // -------------------------------------------------------
    // 云台 -> 上位机，总长 14 字节
    // Byte0:      0x91  帧头
    // Byte1~4:    float yaw   （小端）
    // Byte5~8:    float pitch （小端）
    // Byte9~10:   uint16 robot_hp（小端）
    // Byte11:     uint8  robot_id
    // Byte12:     uint8  packed = (game_progress << 4) | HP_deduction_reason
    //                    高4位: game_progress  (>> 4) & 0x0F
    //                    低4位: HP_deduction_reason   & 0x0F
    // Byte13:     uint8  crc8 (对 Byte0~12 共13字节计算)
    // -------------------------------------------------------
    int recv_nav_data(float   &yaw,
                      float   &pitch,
                      uint16_t &robot_hp,
                      uint8_t  &robot_id,
                      uint8_t  &game_progress,
                      uint8_t  &hp_deduction_reason,
                      size_t msTimeout = 0)
    {
        if (!isOpen)
        {
            std::cerr << "Serial port not open when receiving nav data!" << std::endl;
            return -1;
        }

        std::vector<unsigned char> recv_buf(14);

        // 先同步帧头 0x91，丢弃无效字节
        while (true)
        {
            int ret = recvdata(recv_buf[0], msTimeout);
            if (ret != 0) return ret; // 超时(-2) 或串口错误(-1)
            if (recv_buf[0] == 0x91) break;
        }

        // 读取剩余 13 字节（Byte1~13）
        for (int i = 1; i < 14; i++)
        {
            int ret = recvdata(recv_buf[i], msTimeout);
            if (ret != 0) return -2;
        }

        // 校验 CRC8（对 Byte0~12 共13字节）
        uint8_t crc = calc_crc8(recv_buf, 13);
        if (crc != recv_buf[13])
        {
            std::cerr << "CRC check failed! Calculated: " << static_cast<int>(crc)
                      << ", Received: " << static_cast<int>(recv_buf[13]) << std::endl;
            return -3;
        }

        // 解析 yaw float（Byte1~4，小端）
        Bint32_Union yaw_union;
        yaw_union.U8_Buff[0] = recv_buf[1];
        yaw_union.U8_Buff[1] = recv_buf[2];
        yaw_union.U8_Buff[2] = recv_buf[3];
        yaw_union.U8_Buff[3] = recv_buf[4];

        // 解析 pitch float（Byte5~8，小端）
        Bint32_Union pitch_union;
        pitch_union.U8_Buff[0] = recv_buf[5];
        pitch_union.U8_Buff[1] = recv_buf[6];
        pitch_union.U8_Buff[2] = recv_buf[7];
        pitch_union.U8_Buff[3] = recv_buf[8];

        // 解析 robot_hp uint16（Byte9~10，小端）
        Bint16_Union robot_hp_union;
        robot_hp_union.U8_Buff[0] = recv_buf[9];
        robot_hp_union.U8_Buff[1] = recv_buf[10];

        // 解析 packed byte（Byte12）：高4位 game_progress，低4位 HP_deduction_reason
        uint8_t packed = recv_buf[12];

        // 赋值输出参数
        yaw                 = yaw_union.Float;
        pitch               = pitch_union.Float;
        robot_hp            = robot_hp_union.UInt16;
        robot_id            = recv_buf[11];
        if(robot_id==107){
            robot_id=1; //zijishi蓝色
        }else if(robot_id==7){
            robot_id=0;
        }
 
        game_progress       = (packed >> 4) & 0x0F;
        //game_progress       = 4 ;
        hp_deduction_reason = packed & 0x0F;

        return 0;
    }
};
