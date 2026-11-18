#pragma once
// ---------------------------------------------------------------------------
//  mortat_link.h —— 下位机链路抽象
//
//  真实链路：SerialLink（/dev/ttyUSB*，115200 8N1，termios）
//  仿真链路：SimLink（没有板子也能把整套瞄准逻辑跑通，单元测试用）
// ---------------------------------------------------------------------------
#include <cstdint>
#include <string>
#include <vector>

#include "mortat_protocol.h"

namespace mortat
{

    class IMortatLink
    {
    public:
        virtual ~IMortatLink() = default;

        virtual bool open(std::string *err)
        {
            (void)err;
            return true;
        }
        virtual void close() {}

        virtual bool connected() const = 0;
        virtual bool send(const std::vector<uint8_t> &frame) = 0;

        // 收数据 -> 解析 -> 更新 status()
        virtual void pump() = 0;

        virtual const Status &status() const = 0;

        // 设备打印的 ASCII 行（开机自检里有 "MORTAT v2.4"）
        virtual std::string takeText() { return std::string(); }
        // ACK 的可读描述
        virtual std::string takeAck() { return std::string(); }
    };

} // namespace mortat
