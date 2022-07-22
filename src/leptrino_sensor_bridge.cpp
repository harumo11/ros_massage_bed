#include <boost/asio.hpp>
#include <geometry_msgs/WrenchStamped.h>
#include <iostream>
#include <ros/ros.h>
#include <sstream>

class LeptrinoListener
{
private:
    geometry_msgs::WrenchStamped data;

public:
    LeptrinoListener()
    {
        this->data.header.stamp = ros::Time::now();
        this->data.wrench.force.x = 0;
        this->data.wrench.force.y = 0;
        this->data.wrench.force.z = 0;
        this->data.wrench.torque.x = 0;
        this->data.wrench.torque.y = 0;
        this->data.wrench.torque.z = 0;
    };

    void callback(const geometry_msgs::WrenchStamped &msg)
    {
        this->data = msg;
    };

    std::string flatten()
    {
        std::stringstream ss;
        ss << "SNSE" << std::fixed << std::setprecision(5) << this->data.wrench.force.x << ","
           << this->data.wrench.force.y << "," << this->data.wrench.force.z << ","
           << this->data.wrench.torque.x << "," << this->data.wrench.torque.y << ","
           << this->data.wrench.torque.z << ":";

        return ss.str();
    };
};

class SensorValueSender
{
private:
    boost::asio::io_context actx;
    boost::asio::ip::tcp::socket socket = boost::asio::ip::tcp::socket(actx);
    unsigned int my_port;

public:
    SensorValueSender(const unsigned int port = 50013)
    {
        this->my_port = port;
    };

    void prepare()
    {
        boost::asio::ip::tcp::acceptor acceptor(this->actx, boost::asio::ip::tcp::endpoint(boost::asio::ip::tcp::v4(), this->my_port));
        acceptor.accept(this->socket);
        ROS_INFO_STREAM("Remote info : " << this->socket.remote_endpoint());
        ROS_INFO_STREAM("Connection estublished with user application.");
    };

    void write(const std::string msg)
    {
        boost::asio::write(this->socket, boost::asio::buffer(msg));
    };
};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "leptrino_sensor_bridge");
    ros::NodeHandle node_handle;
    ros::Rate control_hz(1000);

    LeptrinoListener sensor;
    ros::Subscriber leptrino_subscriber = node_handle.subscribe("/leptrino_force_torque/force_torque", 1, &LeptrinoListener::callback, &sensor);
    SensorValueSender sender;

    while (ros::ok())
    {
        ROS_INFO_STREAM(sensor.flatten());
        sender.write(sensor.flatten());
        ros::spinOnce();
        control_hz.sleep();
    }

    return 0;
}