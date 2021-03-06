#include <boost/asio.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <ros/ros.h>
#include <sstream>

class EEFListener
{
private:
    geometry_msgs::PoseStamped data;

public:
    EEFListener()
    {
        this->data.header.stamp = ros::Time::now();
        this->data.pose.position.x = 0;
        this->data.pose.position.y = 0;
        this->data.pose.position.z = 0;
        this->data.pose.orientation.x = 0;
        this->data.pose.orientation.y = 0;
        this->data.pose.orientation.z = 0;
        this->data.pose.orientation.w = 0;
    };

    void callback(const geometry_msgs::PoseStamped &msg)
    {
        this->data = msg;
    };

    std::string flatten()
    {
        std::stringstream ss;
        ss << "POSE" << std::fixed << std::setprecision(5) << this->data.pose.position.x << ","
           << this->data.pose.position.y << "," << this->data.pose.position.z << ","
           << this->data.pose.orientation.x << "," << this->data.pose.orientation.y << ","
           << this->data.pose.orientation.z << "," << this->data.pose.orientation.w << ":";

        return ss.str();
    };
};

class CommandSender
{
private:
    boost::asio::io_context actx;
    boost::asio::ip::tcp::socket socket = boost::asio::ip::tcp::socket(actx);
    unsigned int my_port;

public:
    CommandSender(const unsigned int port = 50012)
    {
        this->my_port = port;
        this->prepare();
    };

    void prepare()
    {
        this->socket.close();
        std::cout << "||| Waiting for the connection form user application." << std::endl;
        boost::asio::ip::tcp::acceptor acceptor(this->actx, boost::asio::ip::tcp::endpoint(boost::asio::ip::tcp::v4(), this->my_port));
        acceptor.accept(this->socket);
        ROS_INFO_STREAM("Remote info : " << this->socket.remote_endpoint());
        ROS_INFO_STREAM("Connection estublished with user application");
    };

    void write(const std::string msg)
    {
        boost::asio::write(this->socket, boost::asio::buffer(msg));
    };
};

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "test_techman_sensor_bridge");
    ros::NodeHandle node_handle;
    ros::Rate control_hz(100);
    ROS_INFO_STREAM("Waiting for tool_pose message");
    ros::topic::waitForMessage<geometry_msgs::PoseStamped>("tool_pose");
    ROS_INFO_STREAM("Confirmed tool_pose message");

    EEFListener tool_pose;
    ros::Subscriber tool_pose_subscriber = node_handle.subscribe("tool_pose", 1, &EEFListener::callback, &tool_pose);
    CommandSender sender;

    while (ros::ok())
    {
        ROS_INFO_STREAM(tool_pose.flatten());

        try
        {
            sender.write(tool_pose.flatten());
        }
        catch (const boost::system::system_error &e)
        {
            ROS_WARN_STREAM(e.what());
            sender.prepare();
        }

        ros::spinOnce();
        control_hz.sleep();
    }

    return 0;
}
