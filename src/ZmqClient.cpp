#include "ZmqClient.h"
#include <iostream>

namespace {
std::mutex _mutex;
std::string createAddr(const std::string& ip, uint16_t port)
{
    return "tcp://" + ip + ':' + std::to_string(port);
}
}

bool ZmqClient::setAddr(const std::string& ip, uint16_t port, int rcvTimeout)
{
    if (ip.empty()) return false;

    if (!_addr.empty())
    {
        _socket.disconnect(_addr);
    }
    _socket.setsockopt(ZMQ_RCVTIMEO, rcvTimeout);
    _socket.setsockopt(ZMQ_SNDTIMEO, 1000);
    _socket.setsockopt(ZMQ_LINGER, 0);
    _addr = createAddr(ip, port);
    std::cout << "Connect to " << _addr.c_str() << std::endl;
    _socket.connect(_addr);
    return true;
}

void ZmqClient::disconnect()
{
    _socket.disconnect(_addr);
    _addr.clear();
}

zmq::message_t ZmqClient::sendMsg()
{
    zmq::message_t message(_reqBuf.data(), _reqBuf.size());
    try
    {
        if (!_socket.send(message))
        {
            std::cout << "Send failed!" << _addr.c_str() << std::endl;
        }
        else if (!_socket.recv(&message))
        {
            std::cout << "Recv failed!" << _addr.c_str() << std::endl;
        }
        else
        {
            return message;
        }
    } catch (const zmq::error_t& t)
    {
        std::cout << _addr.c_str() << "socket error: " << t.what() << std::endl;
    }
    return {};
}
