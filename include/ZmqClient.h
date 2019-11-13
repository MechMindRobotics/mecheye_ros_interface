#ifndef ZMQCLIENT_H
#define ZMQCLIENT_H

#include <zmq.hpp>
#include <mutex>

class ZmqClient
{
public:
    bool setAddr(const std::string& ip, uint16_t port, int rcvTimeout);
    void disconnect();
    bool empty() const { return _addr.empty(); }

protected:
    template <typename Reply, typename Req>
    Reply sendReq(const Req& request)
    {
        Reply reply;
        if (_addr.empty()) return reply;

        // Lock this request
        std::lock_guard<std::mutex> lock(_mutex);
        request.SerializeToString(&_reqBuf);
        const zmq::message_t received = sendMsg();
        if (received.size() == 0) return reply;

        reply.ParseFromArray(received.data(), received.size());
        return reply;
    }

private:
    zmq::message_t sendMsg();

    std::string _addr;
    std::string _reqBuf;
    std::mutex _mutex;
    zmq::context_t _context{1};
    zmq::socket_t _socket{_context, ZMQ_REQ};
};
#endif // ZMQCLIENT_H
