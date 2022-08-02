//このプログラムはテックマンを位置制御＆速度制御するためのものです．
//このプログラムはTCP通信経由でメッセージを受け取りテックマンのスクリプトに
//変換して速度制御を実行します．

#include <boost/asio.hpp>
#include <boost/tokenizer.hpp>
#include <chrono>
#include <glog/logging.h>
#include <iostream>
#include <memory>
#include <optional>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <string>
#include <thread>
#include <tm_msgs/SendScript.h>
#include <vector>

class command_receiver
{
public:
    command_receiver(const unsigned int port);
    std::vector<float> received_commands = {0, 0, 0, 0, 0, 0};
    bool stop = false;
    unsigned int user_socket_port_num;
    std::optional<std::vector<float>> parse_message(const std::string received_msg);
    boost::asio::io_context io;

private:
    const std::string header = "VELC";
    const std::string footer = ":";
};

command_receiver::command_receiver(const unsigned int port)
{
    this->user_socket_port_num = port;
    ROS_DEBUG_STREAM("command receiver starts");
}

std::optional<std::vector<float>> command_receiver::parse_message(const std::string received_msg)
{
    ROS_DEBUG_STREAM("parse message starts.");
    std::string msg_header_removed = received_msg.substr(received_msg.find_first_of(this->header) + this->header.size());
    std::string msg_footer_removed = msg_header_removed.erase(msg_header_removed.find_first_of(":"));

    boost::char_separator<char> comma(",");
    boost::tokenizer<boost::char_separator<char>> tokenizer(msg_footer_removed, comma);
    std::vector<float> velocity_commands;
    for (auto command : tokenizer)
    {
        velocity_commands.push_back(std::stof(command));
    }

    if (velocity_commands.size() != 6)
    {
        ROS_WARN_STREAM("Received commands size should be 6. But " << velocity_commands.size() << " is given.");
        return std::nullopt;
    }
    else
    {
        return velocity_commands;
    }
}

class techman
{
private:
    tm_msgs::SendScript tm_script;
    ros::ServiceClient service_client;

public:
    techman(ros::NodeHandle &node_handle);
    ~techman();
    void send_script(const std::string velocity_script);
    std::string convert_to_tm_script(std::vector<float> vel_commands);
};

techman::techman(ros::NodeHandle &node_handle)
{
    ROS_INFO_STREAM("Connection setting to Techman starts.");

    this->service_client = node_handle.serviceClient<tm_msgs::SendScript>("tm_driver/send_script");
    this->tm_script.request.id = "bridge";
    this->tm_script.request.script = "ContinueVLine(50, 1000)";

    while (ros::ok())
    {
        ros::Duration interval_try_connection(1);

        if (this->service_client.call(this->tm_script))
        {
            if (this->tm_script.response.ok)
            {
                ROS_INFO_STREAM("Sent script to Techman and The connection is established.");
                break;
            }
            else
            {
                ROS_ERROR_STREAM("Sent script to Techman, but response not yet ok. Exit.");
                interval_try_connection.sleep();
            }
        }
        else
        {
            ROS_ERROR_STREAM("Can not send script. Is techman node running? Exit.");
            interval_try_connection.sleep();
        }
    }

    ROS_INFO_STREAM("Connection to Techaman is established");
}

techman::~techman()
{
    ROS_INFO_STREAM("Shutdown process starts");
    this->tm_script.request.script = "StopContinueVmode()";

    if (this->service_client.call(this->tm_script))
    {
        if (tm_script.response.ok)
        {
            ROS_INFO_STREAM("Techman velocity mode shutdown.");
        }
        else
        {
            ROS_WARN_STREAM("Techman velocity mode can not shutdown.");
        }
    }
}

std::string techman::convert_to_tm_script(std::vector<float> vel_commands)
{
    if (vel_commands.size() != 6)
    {
        ROS_WARN_STREAM("The size of given velocity commands is " << vel_commands.size() << "."
                                                                  << "That size must be 6. The tm script that all elements are zero is created.");
        vel_commands = {0, 0, 0, 0, 0, 0};
    }
    std::stringstream tm_msgs;
    tm_msgs << "SetContinueVLine(" << std::fixed << std::setprecision(5)
            << vel_commands.at(0) << "," << vel_commands.at(1) << "," << vel_commands.at(2) << ","
            << vel_commands.at(3) << "," << vel_commands.at(4) << "," << vel_commands.at(5) << ")" << std::endl;

    return tm_msgs.str();
}

void techman::send_script(const std::string velocity_script)
{
    this->tm_script.request.script = velocity_script;

    if (this->service_client.call(this->tm_script))
    {
        if (tm_script.response.ok)
        {
            ROS_INFO_STREAM("Sent velocity command script successfully");
        }
    }
    else
    {
        ROS_WARN_STREAM("Sent velocity command script, but resposen not ok.");
    }
}

int main(int argc, char *argv[])
{

    google::InitGoogleLogging(argv[0]);
    google::InstallFailureSignalHandler();

    ros::init(argc, argv, "techman_control_bridge_node");
    ros::NodeHandle node_handle;
    ros::Rate control_interval(100);
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
    {
        ros::console::notifyLoggerLevelsChanged();
    }
    ROS_DEBUG_STREAM("Techman control bridge starts");

    const int velocity_command_receive_port = 50011;
    command_receiver rec(velocity_command_receive_port);
    techman arm(node_handle);

    while (ros::ok())
    {
        ROS_DEBUG_STREAM("run starts.");
        ROS_INFO_STREAM("Waiting connection from user application.");
        boost::asio::ip::tcp::acceptor acceptor(rec.io, boost::asio::ip::tcp::endpoint(boost::asio::ip::tcp::v4(), rec.user_socket_port_num));
        boost::asio::ip::tcp::socket socket(rec.io);
        acceptor.accept(socket);
        ROS_INFO_STREAM(socket.remote_endpoint());
        ROS_INFO_STREAM("Connection estublished with user application.");

        while (ros::ok())
        {
            std::string data;
            try
            {
                boost::asio::read_until(socket, boost::asio::dynamic_buffer(data), ":");
                ROS_WARN_STREAM("received data: " << data);
                //boost::asio::read_until(socket, boost::asio::dynamic_buffer(data), "\n");
            }
            catch (boost::system::system_error &e)
            {
                ROS_ERROR_STREAM(e.what());
                break;
            }

            auto parsed_velocity_commands = rec.parse_message(data);
            if (parsed_velocity_commands.has_value())
            {
                ROS_DEBUG_STREAM("parse success!");
                rec.received_commands.at(0) = parsed_velocity_commands.value().at(0);
                rec.received_commands.at(1) = parsed_velocity_commands.value().at(1);
                rec.received_commands.at(2) = parsed_velocity_commands.value().at(2);
                rec.received_commands.at(3) = parsed_velocity_commands.value().at(3);
                rec.received_commands.at(4) = parsed_velocity_commands.value().at(4);
                rec.received_commands.at(5) = parsed_velocity_commands.value().at(5);
                ROS_INFO_STREAM(rec.received_commands.at(0) << "\t" << rec.received_commands.at(1) << "\t" << rec.received_commands.at(2) << "\t" << rec.received_commands.at(3) << "\t" << rec.received_commands.at(4) << "\t" << rec.received_commands.at(5));
            }
            else
            {
                ROS_WARN_STREAM("parse NOT success! Velocity commands are set all zero");
                rec.received_commands.assign({0, 0, 0, 0, 0, 0});
            }

            arm.send_script(arm.convert_to_tm_script(rec.received_commands));
        }

        socket.shutdown(boost::asio::ip::tcp::socket::shutdown_both);
        socket.close();
    }
    ROS_INFO_STREAM("The connection is closed successfully");

    return 0;
}