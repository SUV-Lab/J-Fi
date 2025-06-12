#include "serial_comm.hpp"

#include <cstring>
#include <stdexcept>
#include <cerrno>
#include <rclcpp/rclcpp.hpp>
#include <sys/ioctl.h>
#include <chrono>
#include <thread>
#include <vector>

SerialComm::SerialComm()
    : fd_(-1),
      running_(false),
      rx_buffer_{}
{
}

SerialComm::~SerialComm()
{
    // Ensure the receiver thread is stopped and resources are released.
    running_ = false;
    if (mav_recv_thread_.joinable()) {
        mav_recv_thread_.join();
    }
    closePort();
}

bool SerialComm::init(std::function<void(const std::vector<uint8_t> &)> recv_cb,
                      const std::string & port_name, int baud_rate)
{
    receive_callback_ = recv_cb;

    bool ret = openPort(port_name, baud_rate);

    if (ret) {
        running_ = true;
        mav_recv_thread_ = std::thread(&SerialComm::recvLoop, this);
    } else {
        RCLCPP_ERROR(rclcpp::get_logger("SerialComm"), "[init] Failed to open port. Receiver thread not started.");
    }

    return ret;
}

void SerialComm::recvLoop()
{
    while (running_) {
        if (fd_ < 0) {
            RCLCPP_ERROR(rclcpp::get_logger("SerialComm"), "[recvMavLoop] read() failed: %s", strerror(errno));
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            continue;
        }

        ssize_t n = ::read(fd_, rx_buffer_.data(), rx_buffer_.size());
        if (n < 0) {
            RCLCPP_ERROR(rclcpp::get_logger("SerialComm"), "[recvMavLoop] read() failed: %s", strerror(errno));
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            continue;
        } else if (n == 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            RCLCPP_ERROR(rclcpp::get_logger("SerialComm"), "[recvMavLoop] not yet");
            continue;
        } else {
            if (receive_callback_) {
                std::vector<uint8_t> data(rx_buffer_.data(), rx_buffer_.data() + n);
                receive_callback_(data);
            }
        }
    }
}

bool SerialComm::openPort(const std::string & port_name, int baud_rate)
{
    std::lock_guard<std::mutex> lock(fd_mutex_);
    if (fd_ >= 0) {
        return true;
    }

    fd_ = ::open(port_name.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (fd_ < 0) {
        RCLCPP_ERROR(rclcpp::get_logger("SerialComm"), "[openPort] Failed to open %s: %s", port_name.c_str(), strerror(errno));
        return false;
    }

    struct termios tty;
    memset(&tty, 0, sizeof(tty));
    if (tcgetattr(fd_, &tty) != 0) {
        RCLCPP_ERROR(rclcpp::get_logger("SerialComm"), "[openPort] tcgetattr failed: %s", strerror(errno));
        ::close(fd_);
        fd_ = -1;
        return false;
    }

    // Set baud rate.
    speed_t speed = B115200;
    switch (baud_rate) {
        case 9600:
            speed = B9600;
            break;
        case 19200:
            speed = B19200;
            break;
        case 38400:
            speed = B38400;
            break;
        case 57600:
            speed = B57600;
            break;
        case 115200:
            speed = B115200;
            break;
        default:
            RCLCPP_WARN(rclcpp::get_logger("SerialComm"), "[openPort] Unsupported baud rate: %d, defaulting to 115200", baud_rate);
            speed = B115200;
            break;
    }

    cfsetospeed(&tty, speed);
    cfsetispeed(&tty, speed);

    // Configure for 8N1.
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag |= CLOCAL | CREAD;

    // Set raw mode.
    cfmakeraw(&tty);

    if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
        RCLCPP_ERROR(rclcpp::get_logger("SerialComm"), "[openPort] tcsetattr failed: %s", strerror(errno));
        ::close(fd_);
        fd_ = -1;
        return false;
    }

    tcflush(fd_, TCIOFLUSH);

    RCLCPP_INFO(rclcpp::get_logger("SerialComm"), "[openPort] Opened port %s at %d bps", port_name.c_str(), baud_rate);
    return true;
}

void SerialComm::closePort()
{
    std::lock_guard<std::mutex> lock(fd_mutex_);
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
        RCLCPP_INFO(rclcpp::get_logger("SerialComm"), "Port closed");
    }
}


void SerialComm::writeData(const std::vector<uint8_t> & data)
{
    std::lock_guard<std::mutex> lock(fd_mutex_);
    if (fd_ < 0)
        return;

    ssize_t written = ::write(fd_, data.data(), data.size());
    if (written < 0) {
        RCLCPP_ERROR(rclcpp::get_logger("SerialComm"), "[writeData] write() failed: %s", strerror(errno));
    }
}
