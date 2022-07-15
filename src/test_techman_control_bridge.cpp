#include <Poco/Net/SocketAddress.h>
#include <Poco/Net/SocketStream.h>
#include <Poco/Net/StreamSocket.h>
#include <chrono>
#include <iostream>
#include <string>
#include <thread>

int main(int argc, char *argv[])
{
    Poco::Net::SocketAddress server_address("127.0.0.1", 50010);
    Poco::Net::StreamSocket socket(server_address);
    Poco::Net::SocketStream stream(socket);

    while (true)
    {
        std::string command = "VELC0.00,0.00,0.01,0.00,0.00,0.00:";
        std::cout << "sending command : " << command << std::endl;
        stream << command << std::endl;
        //auto send_size = socket.sendBytes(command.c_str(), command.size());
        //std::cout << "sending command size : " << send_size << std::endl;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return 0;
}
