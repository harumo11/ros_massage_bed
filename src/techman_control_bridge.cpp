//このプログラムはテックマンを位置制御＆速度制御するためのものです．
//このプログラムはTCP通信経由でメッセージを受け取りテックマンのスクリプトに
//変換して速度制御を実行します．

#include <iostream>
#include <vector>
#include <chrono>
#include <string>
#include <memory>
#include <thread>
#include <optional>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <tm_msgs/SendScript.h>
#include <ros/ros.h>
#include <boost/asio.hpp>
#include <boost/tokenizer.hpp>

class command_reciever
{
public:
    command_reciever(const unsigned int port);
    void recieve_loop();
    void run();
    std::shared_ptr<std::vector<double>> recieved_commands = std::make_shared<std::vector<double>>();
    bool stop = false;

private:
    boost::asio::io_context io;
    const std::string header = "VELC";
    const std::string footer = ":";
    std::optional<std::vector<double>> parse_message(const std::string recieved_msg);
};

command_reciever::command_reciever(const unsigned int port)
{
    ROS_DEBUG_STREAM("command reciever starts");
    for (size_t i = 0; i < 6; i++)
    {
        this->recieved_commands->push_back(0);
    }
}

std::optional<std::vector<double>> command_reciever::parse_message(const std::string recieved_msg)
{
    ROS_DEBUG_STREAM("parse message starts.");
    std::string msg_header_removed = recieved_msg.substr(recieved_msg.find_first_of(this->header) + this->header.size());
    std::string msg_footer_removed = msg_header_removed.erase(msg_header_removed.find_first_of(":"));

    boost::char_separator<char> comma(",");
    boost::tokenizer<boost::char_separator<char>> tokenizer(msg_footer_removed, comma);
    std::vector<double> velocity_commands;
    for (auto command : tokenizer)
    {
        velocity_commands.push_back(std::stod(command));
    }

    if (velocity_commands.size() != 6)
    {
        ROS_WARN_STREAM("Recieved commands size shoulg be 6. But " << velocity_commands.size() << " is given.");
        return std::nullopt;
    }
    else
    {
        return velocity_commands;
    }
}

void command_reciever::recieve_loop()
{

    while (ros::ok())
    {
        ROS_DEBUG_STREAM("run starts.");
        ROS_INFO_STREAM("Waiting connection from user application.");
        boost::asio::ip::tcp::acceptor acceptor(this->io, boost::asio::ip::tcp::endpoint(boost::asio::ip::tcp::v4(), 50010));
        boost::asio::ip::tcp::socket socket(this->io);
        acceptor.accept(socket);
        ROS_INFO_STREAM(socket.remote_endpoint());
        ROS_INFO_STREAM("Connection estublished with user application.");

        while (ros::ok())
        {
            std::string data;
            try
            {
                boost::asio::read_until(socket, boost::asio::dynamic_buffer(data), "\n");
            }
            catch (boost::system::system_error &e)
            {
                ROS_ERROR_STREAM(e.what());
                break;
            }

            auto parsed_velocity_commands = this->parse_message(data);
            if (parsed_velocity_commands.has_value())
            {
                ROS_DEBUG_STREAM("parse success!");
                this->recieved_commands->at(0) = parsed_velocity_commands.value().at(0);
                this->recieved_commands->at(1) = parsed_velocity_commands.value().at(1);
                this->recieved_commands->at(2) = parsed_velocity_commands.value().at(2);
                this->recieved_commands->at(3) = parsed_velocity_commands.value().at(3);
                this->recieved_commands->at(4) = parsed_velocity_commands.value().at(4);
                this->recieved_commands->at(5) = parsed_velocity_commands.value().at(5);
                ROS_INFO_STREAM(this->recieved_commands->at(0) << "\t" << this->recieved_commands->at(1) << "\t" << this->recieved_commands->at(2) << "\t" << this->recieved_commands->at(3) << "\t" << this->recieved_commands->at(4) << "\t" << this->recieved_commands->at(5));
            }
            else
            {
                ROS_WARN_STREAM("parse NOT success! Velocity commands are set all zero");
                this->recieved_commands->assign({0, 0, 0, 0, 0, 0});
            }
        }

        socket.shutdown(boost::asio::ip::tcp::socket::shutdown_both);
        socket.close();
    }

    ROS_INFO_STREAM("The connection is closed successfully");
}

void command_reciever::run()
{
    std::thread th(&command_reciever::recieve_loop, this);
    th.join();
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "techman_control_bridge_node");
    ROS_INFO_STREAM(ros::master::getHost());
    ROS_INFO_STREAM(ros::master::getURI());
    ros::NodeHandle node_handle;
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
    {
        ros::console::notifyLoggerLevelsChanged();
    }

    ROS_DEBUG_STREAM("Techman control bridge starts");
    command_reciever rec(50010);
    rec.run();

    return 0;
}
