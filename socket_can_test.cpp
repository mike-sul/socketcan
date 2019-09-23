#include <iostream>

#include <unistd.h>
#include <stdio.h>
#include <string.h>

#include <sys/socket.h>
#include <net/if.h>

#include <linux/can.h>
#include <linux/can/raw.h>

#include <memory>

namespace SocketCAN {

class Socket {
public:
  Socket(const char* ifc) {

    if((_fd = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
      perror("Error while opening socket");
      return;
    }

    auto ifindx = if_nametoindex(ifc);
    if (ifindx <= 0) {
      std::cerr << "Failed to get the SocketCAN interface index: "
                << ifc << ": error" << strerror(errno) << std::endl;

      close(_fd);
      return;
    }

    struct sockaddr_can addr{0};

    addr.can_family  = AF_CAN;
    addr.can_ifindex = static_cast<int>(ifindx);

    if(bind(_fd, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
      std::cerr << "Failed to bind to the can socket: "
                << ifc << ": error" << strerror(errno) << std::endl;

      close(_fd);
      return;
    }

  }

  ~Socket() {
    close(_fd);
  }

public:
  void write(const struct can_frame* frame) {
    ssize_t nwritten = -1;
    size_t nToWrite = sizeof(struct can_frame);
    const __u8* srcDataPtr = reinterpret_cast<const __u8*>(frame);

    do {
      nwritten = ::write(_fd, reinterpret_cast<const void*>(srcDataPtr), nToWrite);
      if (nwritten < 0) {
        throw std::runtime_error(std::string("Failed to write data into the can socket: ") + strerror(errno));
      }

      srcDataPtr += nwritten;
      nToWrite -= static_cast<size_t>(nwritten);

    } while (nToWrite > 0);
  }

  void read(struct can_frame* frame) {

    ssize_t nread = -1;
    __u8* dstDataPtr = reinterpret_cast<__u8*>(frame);
    size_t nToRead = sizeof(struct can_frame);

    do {
      nread = ::read(_fd, dstDataPtr, nToRead);
      if (nread < 0) {
        throw std::runtime_error(std::string("Failed to read data from the can socket: ") + strerror(errno));
      }

      nToRead -= static_cast<size_t>(nread);
      dstDataPtr += nread;

    } while (nToRead > 0);

  }

private:
  int _fd;
};

class Writer {
public:
  Writer(const char* ifc, unsigned int id): _socket{ifc}, _id{id} {}
  ~Writer() {}

public:
  void send(const __u8* data, size_t size) {
    struct can_frame frame{0};
    const size_t     maxDataLen = CAN_MAX_DLEN;

    const size_t fullDataFrameNumb = size/CAN_MAX_DLEN;
    const size_t lastDataFrameSize = size%CAN_MAX_DLEN;
    const __u8* srcDataPtr = data;

    frame.can_id  = _id;
    frame.can_dlc = static_cast<__u8>(maxDataLen);
    for (size_t ii = 0; ii < fullDataFrameNumb; ++ii) {
      memcpy(frame.data, srcDataPtr, maxDataLen);
      _socket.write(&frame);
      srcDataPtr += maxDataLen;
    }

    if (lastDataFrameSize > 0) {
      frame.can_dlc = static_cast<__u8>(lastDataFrameSize);
      memcpy(frame.data, srcDataPtr, lastDataFrameSize);
      _socket.write(&frame);
    }

  }

private:
  Socket  _socket;
  canid_t _id;
};


class Receiver {
public:
  Receiver(const char* ifc, unsigned int id): _socket{ifc}, _id{id} {}
  ~Receiver() {}

public:
  void receive(__u8* data, size_t size) {

    size_t nreceived = 0;
    size_t totalReceived = 0;
    __u8* dstDataPtr = data;

    while (totalReceived < size) {
      struct can_frame frame;

      _socket.read(&frame);
      if (frame.can_id != _id) {
        // TODO: set filtering at the driver/kernel level by using ioctl
        continue;
      }

      nreceived = frame.can_dlc;
      if (nreceived > CAN_MAX_DLEN) {
        throw std::runtime_error(std::string("Received invalid can frame,"
                                             " DLC (data length code must be <= )") + std::to_string(CAN_MAX_DLEN));
      }

      memcpy(dstDataPtr, frame.data, nreceived);
      dstDataPtr += nreceived;
      totalReceived += nreceived;
    }
  }

private:
  Socket  _socket;
  canid_t _id;
};

class MsgChannel {
public:
  MsgChannel(Writer& writer, Receiver& receiver): _writer(writer), _receiver(receiver) {}
  ~MsgChannel() {}

public:
  void send(const std::string& msg) {
    // TODO: check if message size exceeds the maximum possible value of the Msg::len field
    Msg::Header header;
    header.len = htobe32(msg.size());

    _writer.send(reinterpret_cast<const __u8*>(&header), sizeof (header));
    _writer.send(reinterpret_cast<const __u8*>(msg.data()), msg.size());
  }

  std::string recv() {
    Msg::Header header;

    _receiver.receive(reinterpret_cast<__u8*>(&header), sizeof (header));
    const auto msgLen = be32toh(header.len);
    std::string result(msgLen, 0);

    //TODO: think of something less hacky
    _receiver.receive(reinterpret_cast<__u8*>(const_cast<char*>(result.data())), msgLen);

    return result;
  }

private:
  struct Msg {
    struct Header {
      uint32_t  len;
      __u8  pad[CAN_MAX_DLEN - sizeof (len)];
    } header;
    __u8* data;
  };

private:
  Writer&   _writer;
  Receiver& _receiver;
};

} // namespace SocketCAN



#include <iostream>
#include <thread>

int main()
{
  const char canIfc[] = "vcan0";
  const int canChannelID = 0x128;

  SocketCAN::Writer canWriter{canIfc, canChannelID};
  SocketCAN::Receiver canReceiver{canIfc, canChannelID};
  SocketCAN::MsgChannel canChannel{canWriter, canReceiver};

  const std::string dataToSend[] = {
    "Hello, World via SocketCAN. Virtual CAN driver is used.",
    "0123456789 0123456789 0123456789 0123456789 0123456789"
  };

  std::thread recvThread{[&]() {
        for (const auto& msg : dataToSend) {
          auto recMsg = canChannel.recv();
          std::cout << "Received: " << recMsg << std::endl;
          if (msg != recMsg) {
            std::cerr << "!!! Received msg is invalid, should be: " << msg << std::endl;
          }
        }
    }
  };

  std::thread sendThread{[&]() {
      for (const auto& msg : dataToSend) {
        canChannel.send(msg);
      }
    }
  };

  sendThread.join();
  recvThread.join();

  return 0;
}
