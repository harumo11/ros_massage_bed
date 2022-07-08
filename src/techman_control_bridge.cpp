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
#include <boost/asio.hpp>
#include <boost/tokenizer.hpp>

class command_receiver
{
public:
    command_receiver(const unsigned int port);
    void recieve_loop();
    void run();
    std::vector<float> received_commands = {0, 0, 0, 0, 0, 0};
    bool stop = false;

private:
    boost::asio::io_context io;
    const std::string header = "VELC";
    const std::string footer = ":";
    std::optional<std::vector<float>> parse_message(const std::string recieved_msg);
};

command_receiver::command_receiver(const unsigned int port)
{
    ROS_DEBUG_STREAM("command reciever starts");
}

std::optional<std::vector<float>> command_receiver::parse_message(const std::string recieved_msg)
{
    ROS_DEBUG_STREAM("parse message starts.");
    std::string msg_header_removed = recieved_msg.substr(recieved_msg.find_first_of(this->header) + this->header.size());
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
        ROS_WARN_STREAM("Recieved commands size shoulg be 6. But " << velocity_commands.size() << " is given.");
        return std::nullopt;
    }
    else
    {
        return velocity_commands;
    }
}

void command_receiver::recieve_loop()
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
                this->received_commands.at(0) = parsed_velocity_commands.value().at(0);
                this->received_commands.at(1) = parsed_velocity_commands.value().at(1);
                this->received_commands.at(2) = parsed_velocity_commands.value().at(2);
                this->received_commands.at(3) = parsed_velocity_commands.value().at(3);
                this->received_commands.at(4) = parsed_velocity_commands.value().at(4);
                this->received_commands.at(5) = parsed_velocity_commands.value().at(5);
                ROS_INFO_STREAM(this->received_commands.at(0) << "\t" << this->received_commands.at(1) << "\t" << this->received_commands.at(2) << "\t" << this->received_commands.at(3) << "\t" << this->received_commands.at(4) << "\t" << this->received_commands.at(5));
            }
            else
            {
                ROS_WARN_STREAM("parse NOT success! Velocity commands are set all zero");
                this->received_commands.assign({0, 0, 0, 0, 0, 0});
            }
        }

        socket.shutdown(boost::asio::ip::tcp::socket::shutdown_both);
        socket.close();
    }

    ROS_INFO_STREAM("The connection is closed successfully");
}

void command_receiver::run()
{
    std::thread th(&command_receiver::recieve_loop, this);
    th.join();
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
    std::vector<float> tm_commands;
    if (vel_commands.size() != 6)
    {
        ROS_WARN_STREAM("The size of given velocity commands is " << vel_commands.size() << "."
                                                                  << "That size must be 6. The tm script that all elements are zero is created.");
        tm_commands = {0, 0, 0, 0, 0, 0};
    }
    else
    {
        std::stringstream tm_msgs;
        tm_msgs << "SetContinueVLine(" << std::fixed << std::setprecision(5)
                << tm_commands.at(0) << "," << tm_commands.at(1) << "," << tm_commands.at(2) << ","
                << tm_commands.at(3) << "," << tm_commands.at(4) << "," << tm_commands.at(5) << ")";

        return tm_msgs.str();
    }
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
    ROS_DEBUG_STREAM("Techman control bridge starts");
    ros::init(argc, argv, "techman_control_bridge_node");
    ros::Rate control_interval(100);
    ros::NodeHandle node_handle;
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
    {
        ros::console::notifyLoggerLevelsChanged();
    }

    const int velocity_command_receive_port = 50010;
    command_receiver rec(velocity_command_receive_port);
    rec.run();

    techman arm(node_handle);

    while (ros::ok())
    {
        arm.send_script(arm.convert_to_tm_script(rec.received_commands));

        ros::spinOnce();
        control_interval.sleep();
    }

    return 0;
}
