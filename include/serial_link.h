#pragma once
// ---------------------------------------------------------------------------
//  serial_link.h —— 真实串口链路（Linux / termios，115200 8N1）
//  Windows / 非 POSIX 平台上可以编译，但 open() 会直接失败并给出提示。
// ---------------------------------------------------------------------------
#include <deque>
#include <string>
#include <vector>

#include "mortat_link.h"

namespace mortat
{

    class SerialLink : public IMortatLink
    {
    public:
        SerialLink(const std::string &device, int baud = 115200);
        ~SerialLink() override;

        bool open(std::string *err) override;
        void close() override;

        bool connected() const override;
        bool send(const std::vector<uint8_t> &frame) override;
        void pump() override;

        const Status &status() const override { return st_; }

        std::string takeText() override;
        std::string takeAck() override;

    private:
        bool writeAll(const uint8_t *data, size_t n);

        std::string dev_;
        int baud_ = 115200;
        int fd_ = -1;

        Status st_;
        FrameParser parser_;
        std::deque<std::string> texts_;
        std::deque<std::string> acks_;
    };

} // namespace mortat
