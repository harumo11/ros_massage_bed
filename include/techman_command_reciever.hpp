#include <Poco/Net/ServerSocket.h>
#include <Poco/Net/SocketAddress.h>
#include <Poco/Net/StreamSocket.h>
#include <Poco/String.h>
#include <Poco/StringTokenizer.h>
#include <iostream>
#include <optional>
#include <thread>
#include <vector>

class techman_command_reciever {

public:
    techman_command_reciever(const unsigned int port = 50011);
    ~techman_command_reciever();
    std::optional<std::vector<double>> split_message(const std::string message);
    void try_connect(const unsigned int port);
    void try_disconnect();
    std::string recieve_command_message();
    Poco::Net::StreamSocket socket_from_user_app;

private:
    const std::string header = "VELC";
};

techman_command_reciever::techman_command_reciever(const unsigned int port)
{
    this->try_connect(port);
}

techman_command_reciever::~techman_command_reciever()
{
    this->try_disconnect();
}

void techman_command_reciever::try_connect(const unsigned int port)
{
    std::cout << "[Info] [techman_command_reciever] Wait to connect client" << std::endl;
    Poco::Net::ServerSocket srv(port);
    this->socket_from_user_app = srv.acceptConnection();
    std::cout << "[Info] [techman_command_reciever] Connection established" << std::endl;
    std::cout << "[Info] [techman_command_reciever] Connected to " << srv.peerAddress() << std::endl;
}

void techman_command_reciever::try_disconnect()
{
    this->socket_from_user_app.shutdown();
    this->socket_from_user_app.close();
}

std::optional<std::vector<double>> techman_command_reciever::split_message(const std::string message)
{
    std::string msg_header_removed = message.substr(message.find_first_of(header) + this->header.size());
    std::string msg_footer_removed = msg_header_removed.erase(msg_header_removed.find_first_of(":"));
    Poco::StringTokenizer tokernizer(msg_footer_removed, ",");
    std::vector<double> velocity_commands;
    for (auto command : tokernizer) {
        velocity_commands.push_back(std::stod(command));
    }

    if (velocity_commands.size() != 6) {
        std::cout << "[Warn] [techman_command_reciever] Recieved command size should be six. But acutually data size is " << velocity_commands.size() << ". nullopt will be returned." << std::endl;
        return std::nullopt;
    } else {
        return velocity_commands;
    }
}

std::string techman_command_reciever::recieve_command_message()
{
    char message[4096];
    int recieve_message_size = this->socket_from_user_app.receiveBytes(message, sizeof(message));
    message[recieve_message_size] = '\0';
    std::string recieved_message_data(message);
    return recieved_message_data;
};
