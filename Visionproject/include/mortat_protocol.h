#pragma once
// ============================================================================
//  mortat_protocol.h —— MORTAT 下位机串口协议
//
//  与 ServoConsole.cs 里的 Proto / FrameParser 两个类逐字节一致：
//      0xAA 0x55 LEN CMD DATA... CRC8
//      LEN = CMD(1) + DATA(n)；CRC8 覆盖 LEN..DATA 全部字节，初值 0，多项式 0x07
//  本文件不依赖 OpenCV、不依赖操作系统，方便在 PC 上做单元测试。
// ============================================================================

#include <cstdint>
#include <cstring>
#include <functional>
#include <string>
#include <vector>

namespace mortat
{

    constexpr uint8_t kSync0 = 0xAA;
    constexpr uint8_t kSync1 = 0x55;

    constexpr int kMemNone = 0x7FFF;      // 固件里"没记录过记忆位置"的魔数
    constexpr int kStatusLen = 16;        // 状态帧 LEN
    constexpr int kStopBaseUs = 1535;     // 固件 S360_STOP_US，与 board_config.h 一致
    constexpr int kPosCenterTenths = 900; // 位置中值（90.0°）

    enum Cmd : uint8_t
    {
        CMD_TURN_LEFT = 0x01,  // data: 步数(1)
        CMD_TURN_RIGHT = 0x02, // data: 步数(1)
        CMD_MEM_LEFT = 0x03,
        CMD_MEM_RIGHT = 0x04,
        CMD_GOTO_MEM_LEFT = 0x05,
        CMD_GOTO_MEM_RIGHT = 0x06,
        CMD_FIRE = 0x07, // data: 圈数(1) 速度%(1)
        CMD_STOP = 0x08,
        CMD_CENTER = 0x09,
        CMD_GET_STATUS = 0x0B,
        CMD_SET_STEP = 0x0C,    // data: u16 步长(0.1°)
        CMD_SET_POS = 0x0D,     // data: u16 绝对位置(0.1°)
        CMD_SET_FIRE_MS = 0x0E, // data: u16 每圈毫秒
        CMD_GET_VERSION = 0x0F,
        CMD_SET_SPIN_TRIM = 0x10, // data: i16 µs 偏移（CH7）
        CMD_BOOT_INFO = 0x11,
        CMD_SWEEP_CH7 = 0x12,
        CMD_SET_BALANCE = 0x13,   // data: 平衡%
        CMD_SELECT_TARGET = 0x20, // data: 0=180°(CH0) 1=360°步进(CH1)
        CMD_SET_STEP_MS = 0x21,   // data: u16 步进舵机每圈毫秒
        CMD_SET_STEP_TRIM = 0x22, // data: i16 µs 偏移（CH1）

        RSP_STATUS = 0x81, // LEN=16
        RSP_ACK = 0x82,    // LEN=3, data: 原命令(1) 结果(1)
    };

    // ---------------------------------------------------------------- CRC8
    inline uint8_t crc8(const uint8_t *d, int off, int len)
    {
        uint8_t crc = 0x00;
        for (int i = 0; i < len; ++i)
        {
            crc ^= d[off + i];
            for (int b = 0; b < 8; ++b)
            {
                if (crc & 0x80)
                    crc = (uint8_t)((crc << 1) ^ 0x07);
                else
                    crc = (uint8_t)(crc << 1);
            }
        }
        return crc;
    }

    // ---------------------------------------------------------------- 组帧
    inline std::vector<uint8_t> buildFrame(uint8_t cmd, const uint8_t *data, int n)
    {
        std::vector<uint8_t> f;
        f.reserve((size_t)(4 + n));
        f.push_back(kSync0);
        f.push_back(kSync1);
        f.push_back((uint8_t)(1 + n)); // LEN = CMD + DATA
        f.push_back(cmd);
        for (int i = 0; i < n; ++i)
            f.push_back(data[i]);
        f.push_back(crc8(f.data(), 2, 2 + n)); // CRC 覆盖 LEN + CMD + DATA
        return f;
    }

    inline std::vector<uint8_t> buildFrame(uint8_t cmd)
    {
        return buildFrame(cmd, nullptr, 0);
    }

    inline std::vector<uint8_t> buildFrame1(uint8_t cmd, uint8_t a)
    {
        return buildFrame(cmd, &a, 1);
    }

    inline std::vector<uint8_t> buildFrame2(uint8_t cmd, uint8_t a, uint8_t b)
    {
        const uint8_t d[2] = {a, b};
        return buildFrame(cmd, d, 2);
    }

    // 小端 u16 / i16
    inline std::vector<uint8_t> buildU16(uint8_t cmd, uint16_t v)
    {
        const uint8_t d[2] = {(uint8_t)(v & 0xFF), (uint8_t)(v >> 8)};
        return buildFrame(cmd, d, 2);
    }

    inline std::vector<uint8_t> buildI16(uint8_t cmd, int16_t v)
    {
        return buildU16(cmd, (uint16_t)v);
    }

    inline std::string hex(const std::vector<uint8_t> &v)
    {
        static const char *kHex = "0123456789ABCDEF";
        std::string s;
        s.reserve(v.size() * 3);
        for (size_t i = 0; i < v.size(); ++i)
        {
            if (i)
                s.push_back(' ');
            s.push_back(kHex[(v[i] >> 4) & 0x0F]);
            s.push_back(kHex[v[i] & 0x0F]);
        }
        return s;
    }

    // ---------------------------------------------------------------- 状态帧
    // payload（LEN-1 = 15 字节）：
    //   pos(2) memL(2) memR(2) step(2) state(1) progress(1) trim(2) ms(2) balance(1)
    struct Status
    {
        int pos_tenths = kPosCenterTenths; // 0.1° 为单位，900 = 90.0°
        int mem_left = kMemNone;
        int mem_right = kMemNone;
        int step_tenths = 20; // 步长 2.0°
        int state = 0;        // 0 空闲 1 顺时针 2 换向停顿 3 逆时针 4 完成
        int progress = 0;
        int trim_us = 0;
        int ms_per_turn = 1400;
        int balance = 100;
        bool valid = false;

        bool parse(const std::vector<uint8_t> &p)
        {
            if (p.size() != (size_t)(kStatusLen - 1))
                return false;
            pos_tenths = (int)(int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
            mem_left = (int)(int16_t)((uint16_t)p[2] | ((uint16_t)p[3] << 8));
            mem_right = (int)(int16_t)((uint16_t)p[4] | ((uint16_t)p[5] << 8));
            step_tenths = (int)(uint16_t)((uint16_t)p[6] | ((uint16_t)p[7] << 8));
            state = p[8];
            progress = p[9];
            trim_us = (int)(int16_t)((uint16_t)p[10] | ((uint16_t)p[11] << 8));
            ms_per_turn = (int)(uint16_t)((uint16_t)p[12] | ((uint16_t)p[13] << 8));
            balance = p[14];
            valid = true;
            return true;
        }

        double posDeg() const { return pos_tenths / 10.0; }
        double stepDeg() const { return step_tenths / 10.0; }
        bool firing() const { return state == 1 || state == 2 || state == 3; }
    };

    inline const char *ackText(uint8_t r)
    {
        switch (r)
        {
        case 0x00:
            return "OK";
        case 0x01:
            return "还没记录过记忆位置";
        case 0x02:
            return "参数越界";
        case 0x03:
            return "PCA9685 无应答(查 I2C 接线)";
        case 0x04:
            return "命令参数错误";
        case 0x05:
            return "RAM 记住了，但写 Flash 失败";
        default:
            return "未知结果";
        }
    }

    inline const char *stateText(int s)
    {
        switch (s)
        {
        case 0:
            return "空闲";
        case 1:
            return "顺时针";
        case 2:
            return "换向停顿";
        case 3:
            return "逆时针";
        case 4:
            return "完成";
        default:
            return "?";
        }
    }

    // ---------------------------------------------------------------- 流式解析
    // 与 C# FrameParser 行为一致：容忍文本日志、容忍坏帧（逐字节回溯）。
    class FrameParser
    {
    public:
        std::function<void(uint8_t cmd, const std::vector<uint8_t> &payload)> onFrame;
        std::function<void(const std::string &line)> onText;

        void feed(const uint8_t *data, size_t n)
        {
            for (size_t i = 0; i < n; ++i)
                feedByte(data[i]);
        }

        void feed(const std::vector<uint8_t> &v) { feed(v.data(), v.size()); }

        void reset()
        {
            buf_.clear();
            text_.clear();
        }

    private:
        std::vector<uint8_t> buf_;
        std::string text_;

        void emitText(uint8_t b)
        {
            if (b == 0x0A)
            {
                if (!text_.empty())
                {
                    std::string s = text_;
                    text_.clear();
                    // trim
                    size_t a = s.find_first_not_of(" \t\r");
                    size_t z = s.find_last_not_of(" \t\r");
                    if (a != std::string::npos && onText)
                        onText(s.substr(a, z - a + 1));
                }
                return;
            }
            if (b == 0x0D)
                return;
            if (b >= 0x20 && b <= 0x7E)
                text_.push_back((char)b);
        }

        void feedByte(uint8_t b)
        {
            buf_.push_back(b);
            for (;;)
            {
                if (buf_.size() < 2)
                    return;
                if (buf_[0] != kSync0 || buf_[1] != kSync1)
                {
                    emitText(buf_[0]);
                    buf_.erase(buf_.begin());
                    continue;
                }
                if (buf_.size() < 3)
                    return;
                const int len = (int)buf_[2];
                if (len < 1 || len > 16)
                {
                    emitText(buf_[0]);
                    buf_.erase(buf_.begin());
                    continue;
                }
                if ((int)buf_.size() < 4 + len)
                    return;
                if (crc8(buf_.data(), 2, 1 + len) != buf_[3 + len])
                {
                    emitText(buf_[0]);
                    buf_.erase(buf_.begin());
                    continue;
                }
                const uint8_t cmd = buf_[3];
                std::vector<uint8_t> payload(buf_.begin() + 4, buf_.begin() + 4 + (len - 1));
                buf_.erase(buf_.begin(), buf_.begin() + 4 + len);
                if (onFrame)
                    onFrame(cmd, payload);
            }
        }
    };

} // namespace mortat
