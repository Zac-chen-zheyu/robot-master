// ---------------------------------------------------------------------------
//  serial_link.cpp —— termios 串口实现
// ---------------------------------------------------------------------------
#include "serial_link.h"

#include <cerrno>
#include <cstdio>
#include <cstring>

#if defined(__linux__) || defined(__APPLE__)
#include <fcntl.h>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>
#endif

namespace mortat
{
    namespace
    {

#if defined(__linux__) || defined(__APPLE__)
        speed_t toSpeed(int baud, bool *ok)
        {
            *ok = true;
            switch (baud)
            {
            case 9600:
                return B9600;
            case 19200:
                return B19200;
            case 38400:
                return B38400;
            case 57600:
                return B57600;
            case 115200:
                return B115200;
            case 230400:
                return B230400;
            case 460800:
                return B460800;
#if defined(B921600)
            case 921600:
                return B921600;
#endif
            default:
                *ok = false;
                return B115200;
            }
        }
#endif

    } // namespace

    SerialLink::SerialLink(const std::string &device, int baud) : dev_(device), baud_(baud)
    {
        parser_.onFrame = [this](uint8_t cmd, const std::vector<uint8_t> &p)
        {
            if (cmd == RSP_STATUS)
            {
                Status s;
                if (s.parse(p))
                    st_ = s;
            }
            else if (cmd == RSP_ACK && p.size() >= 2)
            {
                char buf[128];
                std::snprintf(buf, sizeof(buf), "ACK cmd=0x%02X -> %s (%u)",
                              p[0], ackText(p[1]), (unsigned)p[1]);
                acks_.push_back(std::string(buf));
            }
        };
        parser_.onText = [this](const std::string &line)
        { texts_.push_back(line); };
    }

    SerialLink::~SerialLink() { close(); }

    bool SerialLink::open(std::string *err)
    {
        close();

#if defined(__linux__) || defined(__APPLE__)
        fd_ = ::open(dev_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (fd_ < 0)
        {
            if (err)
            {
                *err = "打不开 " + dev_ + "：" + std::strerror(errno) +
                       "。检查：1) 设备名 ls /dev/ttyUSB*  2) 权限 sudo usermod -aG dialout $USER 后重新登录";
            }
            return false;
        }

        termios tio;
        std::memset(&tio, 0, sizeof(tio));
        if (tcgetattr(fd_, &tio) != 0)
        {
            if (err)
                *err = std::string("tcgetattr 失败：") + std::strerror(errno);
            close();
            return false;
        }

        cfmakeraw(&tio); // 关掉回显/行缓冲/特殊字符
        bool ok = true;
        const speed_t sp = toSpeed(baud_, &ok);
        if (!ok)
        {
            if (err)
                *err = "不支持的波特率: " + std::to_string(baud_);
            close();
            return false;
        }
        cfsetispeed(&tio, sp);
        cfsetospeed(&tio, sp);

        tio.c_cflag |= (CLOCAL | CREAD);
        tio.c_cflag &= ~(CSTOPB | PARENB | CRTSCTS);
#if defined(CSIZE)
        tio.c_cflag &= ~CSIZE;
#endif
        tio.c_cflag |= CS8;
        tio.c_cc[VMIN] = 0;
        tio.c_cc[VTIME] = 0;

        if (tcsetattr(fd_, TCSANOW, &tio) != 0)
        {
            if (err)
                *err = std::string("tcsetattr 失败：") + std::strerror(errno);
            close();
            return false;
        }
        tcflush(fd_, TCIOFLUSH);

        st_ = Status();
        st_.valid = false;
        return true;
#else
        if (err)
            *err = "串口模块只支持 Linux/macOS；Windows 上请用 --sim 跑逻辑，或到 Ubuntu 编译。";
        return false;
#endif
    }

    void SerialLink::close()
    {
#if defined(__linux__) || defined(__APPLE__)
        if (fd_ >= 0)
        {
            ::close(fd_);
            fd_ = -1;
        }
#else
        fd_ = -1;
#endif
        parser_.reset();
        texts_.clear();
        acks_.clear();
    }

    bool SerialLink::connected() const { return fd_ >= 0; }

    bool SerialLink::send(const std::vector<uint8_t> &frame)
    {
        if (fd_ < 0 || frame.empty())
            return false;
        return writeAll(frame.data(), frame.size());
    }

    bool SerialLink::writeAll(const uint8_t *data, size_t n)
    {
#if defined(__linux__) || defined(__APPLE__)
        size_t off = 0;
        int guard = 0;
        while (off < n)
        {
            const ssize_t w = ::write(fd_, data + off, n - off);
            if (w > 0)
            {
                off += (size_t)w;
                guard = 0;
                continue;
            }
            if (w < 0 && errno == EINTR)
                continue;
            if (w < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))
            {
                if (++guard > 200)
                    return false; // ~200ms 还写不进去，放弃这一帧
                ::usleep(1000);
                continue;
            }
            return false;
        }
        return true;
#else
        (void)data;
        (void)n;
        return false;
#endif
    }

    void SerialLink::pump()
    {
#if defined(__linux__) || defined(__APPLE__)
        if (fd_ < 0)
            return;

        for (;;)
        {
            fd_set rf;
            FD_ZERO(&rf);
            FD_SET(fd_, &rf);
            timeval tv;
            tv.tv_sec = 0;
            tv.tv_usec = 0;
            const int r = ::select(fd_ + 1, &rf, nullptr, nullptr, &tv);
            if (r <= 0)
                break;

            uint8_t buf[512];
            const ssize_t n = ::read(fd_, buf, sizeof(buf));
            if (n <= 0)
                break;
            parser_.feed(buf, (size_t)n);
        }
#endif
    }

    std::string SerialLink::takeText()
    {
        if (texts_.empty())
            return std::string();
        std::string s = texts_.front();
        texts_.pop_front();
        return s;
    }

    std::string SerialLink::takeAck()
    {
        if (acks_.empty())
            return std::string();
        std::string s = acks_.front();
        acks_.pop_front();
        return s;
    }

} // namespace mortat
