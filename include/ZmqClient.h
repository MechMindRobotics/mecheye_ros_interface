#ifndef ZMQCLIENT_H
#define ZMQCLIENT_H

#include <zmq.hpp>
#include <mutex>
#include <iostream>

class ZmqClient
{
public:
    bool setAddr(const std::string& ip, uint16_t port, int rcvTimeout);
    void disconnect();
    bool empty() const
    {
        return _addr.empty();
    }

protected:
    std::string sendReq(std::string request)
    {
        if (_addr.empty())
            return "";
        // Lock this request
        std::lock_guard<std::mutex> lock(_mutex);
        _reqBuf = request;
        const zmq::message_t received = sendMsg();
        if (received.size() == 0)
            return "";
        std::string reply((char*)received.data(), received.size());
        return reply;
    }

private:
    zmq::message_t sendMsg();

    std::string _addr;
    std::string _reqBuf;
    std::mutex _mutex;
    zmq::context_t _context{ 1 };
    zmq::socket_t _socket{ _context, ZMQ_REQ };
};
#endif  // ZMQCLIENT_H
