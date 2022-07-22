#include <boost/asio.hpp>
#include <chrono>
#include <iostream>
#include <sstream>
#include <thread>

int main(void)
{
    std::cout << "test techman sensor bridge starts!" << std::endl;
    boost::asio::io_context ioctx;
    boost::asio::ip::tcp::socket socket(ioctx);
    while (true)
    {
        try
        {
            socket.connect(boost::asio::ip::tcp::endpoint(boost::asio::ip::address::from_string("127.0.0.1"), 50012));
            std::cout << "Connection is established" << std::endl;
            break;
        }
        catch (const boost::system::system_error &e)
        {
            std::cout << e.what() << std::endl;
            std::cout << "Try to connect again." << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }

    while (true)
    {
        std::string buff;
        boost::asio::read_until(socket, boost::asio::dynamic_buffer(buff), ":");
        std::cout << "Receive data : " << buff << std::endl;
    }
}