#include <iostream>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <tm_msgs/SendScript.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/PoseStamped.h>

class EEFListener
{
private:
public:
    EEFListener();
    void callback(const geometry_msgs::PoseStamped &msg);
    geometry_msgs::PoseStamped data;
};

EEFListener::EEFListener()
{
    this->data.header.stamp = ros::Time::now();
    this->data.pose.position.x = 0;
    this->data.pose.position.y = 0;
    this->data.pose.position.z = 0;
    this->data.pose.orientation.x = 0;
    this->data.pose.orientation.y = 0;
    this->data.pose.orientation.z = 0;
    this->data.pose.orientation.w = 0;
}

void EEFListener::callback(const geometry_msgs::PoseStamped &msg)
{
    this->data = msg;
}

class LeptrinoListener
{
public:
    LeptrinoListener();
    void callback(const geometry_msgs::WrenchStamped &msg);
    geometry_msgs::WrenchStamped data;
};

LeptrinoListener::LeptrinoListener()
{
    this->data.header.stamp = ros::Time::now();
    this->data.wrench.force.x = 0;
    this->data.wrench.force.y = 0;
    this->data.wrench.force.z = 0;
    this->data.wrench.torque.x = 0;
    this->data.wrench.torque.y = 0;
    this->data.wrench.torque.z = 0;
}

void LeptrinoListener::callback(const geometry_msgs::WrenchStamped &msg)
{
    this->data = msg;
}

double mean(std::vector<double> data)
{
    double sum = 0;
    for (auto e : data)
    {
        sum += e;
    }

    return sum / (double)data.size();
}

geometry_msgs::WrenchStamped calc_bias(std::vector<geometry_msgs::WrenchStamped> msgs)
{
    std::vector<double> force_x_data;
    std::vector<double> force_y_data;
    std::vector<double> force_z_data;
    std::vector<double> torque_x_data;
    std::vector<double> torque_y_data;
    std::vector<double> torque_z_data;

    for (auto e : msgs)
    {
        force_x_data.push_back(e.wrench.force.x);
        force_y_data.push_back(e.wrench.force.y);
        force_z_data.push_back(e.wrench.force.z);
        torque_x_data.push_back(e.wrench.torque.x);
        torque_y_data.push_back(e.wrench.torque.y);
        torque_z_data.push_back(e.wrench.torque.z);
    }

    geometry_msgs::WrenchStamped bias;
    bias.wrench.force.x = mean(force_x_data);
    bias.wrench.force.y = mean(force_y_data);
    bias.wrench.force.z = mean(force_z_data);
    bias.wrench.torque.x = mean(torque_x_data);
    bias.wrench.torque.y = mean(torque_y_data);
    bias.wrench.torque.z = mean(torque_z_data);

    return bias;
}

geometry_msgs::WrenchStamped remove_bias(const geometry_msgs::WrenchStamped recieved_msg, const geometry_msgs::WrenchStamped bias)
{
    geometry_msgs::WrenchStamped caliblated_msg;
    caliblated_msg.wrench.force.x = recieved_msg.wrench.force.x - bias.wrench.force.x;
    caliblated_msg.wrench.force.y = recieved_msg.wrench.force.y - bias.wrench.force.y;
    caliblated_msg.wrench.force.z = recieved_msg.wrench.force.z - bias.wrench.force.z;
    caliblated_msg.wrench.torque.x = recieved_msg.wrench.torque.x - bias.wrench.torque.x;
    caliblated_msg.wrench.torque.y = recieved_msg.wrench.torque.y - bias.wrench.torque.y;
    caliblated_msg.wrench.torque.z = recieved_msg.wrench.torque.z - bias.wrench.torque.z;

    return caliblated_msg;
}

std::string unpack_wrenchstamped(const geometry_msgs::WrenchStamped &message)
{
    std::stringstream ss;
    ss << message.wrench.force.x << ","
       << message.wrench.force.y << ","
       << message.wrench.force.z << ","
       << message.wrench.torque.x << ","
       << message.wrench.torque.y << ","
       << message.wrench.torque.z << ",";

    return ss.str();
}

std::string unpack_posestamped(const geometry_msgs::PoseStamped &message)
{
    std::stringstream ss;
    ss << message.pose.position.x << ","
       << message.pose.position.y << ","
       << message.pose.position.z << ","
       << message.pose.orientation.x << ","
       << message.pose.orientation.y << ","
       << message.pose.orientation.z << ","
       << message.pose.orientation.w << ",";

    return ss.str();
}

std::string create_tm_vel_message(std::vector<double> vel_commands)
{
    std::vector<float> commands_f;

    if (vel_commands.size() == 6)
    {
        for (size_t i = 0; i < 6; i++)
        {
            commands_f.push_back(static_cast<float>(vel_commands.at(i)));
        }
    }
    else
    {
        commands_f = {0, 0, 0, 0, 0, 0};
    }

    std::stringstream ss;
    ss << "SetContinueVLine(" << std::fixed << std::setprecision(5)
       << commands_f.at(0) << "," << commands_f.at(1) << "," << commands_f.at(2) << ","
       << commands_f.at(3) << "," << commands_f.at(4) << "," << commands_f.at(5) << ")";

    return ss.str();
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "simple_force_massage");
    ros::NodeHandle node_handle;
    ros::Rate calib_timer(500);
    ros::Rate control_timer(100);

    LeptrinoListener sensor;
    EEFListener end_effector;
    ros::Subscriber sub = node_handle.subscribe("leptrino_force_torque/force_torque", 1, &LeptrinoListener::callback, &sensor);
    ros::Subscriber eef_sub = node_handle.subscribe("/tool_pose", 1, &EEFListener::callback, &end_effector);

    // calc bias
    std::vector<geometry_msgs::WrenchStamped> measured_biases;
    for (int i = 0; i < 1000; i++)
    {
        ROS_INFO_STREAM(sensor.data);
        measured_biases.push_back(sensor.data);
        ros::spinOnce();
        calib_timer.sleep();
    }
    auto bias = calc_bias(measured_biases);
    ROS_DEBUG_STREAM_NAMED("bias", bias);

    // initialize techman velocity mode
    ros::ServiceClient techman = node_handle.serviceClient<tm_msgs::SendScript>("tm_driver/send_script");
    tm_msgs::SendScript tm_vel_init_script;
    tm_vel_init_script.request.id = "demo";
    tm_vel_init_script.request.script = "ContinueVLine(50, 1000)";
    if (techman.call(tm_vel_init_script))
    {
        if (tm_vel_init_script.response.ok)
        {
            ROS_INFO_STREAM("Sent script to robot");
        }
        else
        {
            ROS_WARN_STREAM("Sent script to robot, but renspose not yet ok");
        }
    }
    else
    {
        ROS_ERROR_STREAM("Error send script to robot");
    }

    // log preparing
    std::ofstream log("/home/harumo/catkin_ws/src/ros_massage_bed/massage_log.csv");
    if (!log.is_open())
    {
        ROS_ERROR_STREAM("can not open log file");
    }

    log << "Time"
        << ","
        << "tool pose x"
        << ","
        << "tool pose y"
        << ","
        << "tool pose z"
        << ","
        << "tool orientation x"
        << ","
        << "tool orientation y"
        << ","
        << "tool orientation z"
        << ","
        << "tool orientation w"
        << ","
        << "sensor pose x"
        << ","
        << "sensor pose y"
        << ","
        << "sensor pose z"
        << ","
        << "sensor orientation x"
        << ","
        << "sensor orientation y"
        << ","
        << "sensor orientation z" << std::endl;

    // control loop
    unsigned int iter = 0;
    while (ros::ok())
    {
        // recieve force torqe sensor data
        auto force_torque = remove_bias(sensor.data, bias);
        auto measured_force_z = -1 * force_torque.wrench.force.z;

        // calc z-axis velocity command with p-control
        double reference_force_z = 15;
        double error = reference_force_z - measured_force_z;
        ROS_INFO_STREAM("control error : " << error);
        double gain = -0.003;
        double u = gain * error;
        ROS_INFO_STREAM("control u : " << u);

        // calc x-axis sin wave command
        double amplitude = 50.0 / 1000.0; //2cm
        double w = 0.5;                   // 0.5Hz
        double massage_vel = amplitude * std::sin(2 * M_PI * w * ros::Time::now().toSec());

        // send velocity command to robot
        std::vector<double> test_vel_commands = {massage_vel, 0, u, 0, 0, 0};
        std::string vel_script = create_tm_vel_message(test_vel_commands);
        tm_msgs::SendScript tm_vel_script;
        tm_vel_script.request.id = "demo";
        tm_vel_script.request.script = vel_script;
        if (techman.call(tm_vel_script))
        {
            if (tm_vel_init_script.response.ok)
            {
                ROS_INFO_STREAM("Sent vel script to robot");
            }
            else
            {
                ROS_WARN_STREAM("Sent vel script to robot, but response not yet ok");
            }
        }

        log << std::fixed << std::setprecision(20) << ros::Time::now().toSec() << "," << unpack_posestamped(end_effector.data) << unpack_wrenchstamped(sensor.data) << std::endl;

        ros::spinOnce();
        control_timer.sleep();
    }

    // finalize
    tm_msgs::SendScript tm_fin_script;
    tm_fin_script.request.id = "demo";
    tm_fin_script.request.script = "StopContinueVmode()";
    if (techman.call(tm_fin_script))
    {
        if (tm_fin_script.response.ok)
        {
            ROS_INFO_STREAM("velocity mode shutdown");
        }
        else
        {
            ROS_WARN_STREAM("velocity mode is not shutdown");
        }
    }

    ROS_INFO_STREAM("See you!");

    return 0;
}
