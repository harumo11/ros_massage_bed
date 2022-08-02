#include "./third_party/imgui.h"
#include "./third_party/imgui_impl_glfw.h"
#include "./third_party/imgui_impl_opengl3.h"
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <utility>

int main(int argc, char const *argv[])
{
    boost::asio::io_context actx;
    boost::asio::ip::tcp::socket socket(actx);
    boost::asio::ip::tcp::endpoint client_address(boost::asio::ip::make_address("127.0.0.1"), 50011);
    std::cout << "||| Try to connect techman_control_bridge. Did you run techman_control_bridge using following command" << std::endl;
    std::cout << "rosrun ros_massage_bed techman_control_bridge" << std::endl;
    socket.connect(client_address);

    // GL 3.2 + GLSL 150
    const char *glsl_version = "#version 130";
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    // Create window with graphic context
    ImVec2 window_size = {640, 480};
    glfwInit();
    GLFWwindow *window = glfwCreateWindow(window_size.x, window_size.y, "Dashboard for Bed System", nullptr, nullptr);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1); // enable vsync

    // set up dear imgui context
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO &io = ImGui::GetIO();

    // set up dear imgui style
    ImGui::StyleColorsLight();

    // setup platform/renderer bindings
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init(glsl_version);
    // ImGui_ImplOpenGL3_Init();

    // background color
    ImVec4 clear_color = ImVec4(0.3, 0.3, 0.3, 1);

    // main loop
    while (!glfwWindowShouldClose(window))
    {
        // get event
        glfwPollEvents();

        // start the dear imgui frame
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        {
            // widget setting for window2
            ImGui::SetNextWindowPos(ImVec2(0, 0));
            ImGui::SetNextWindowSize(ImVec2(640, 480));
            ImGui::Begin("Control for bed system");
            ImGui::TextColored(ImVec4(0.0, 0.5, 0.2, 1.0), "Techman");
            //techman control widgets
            static float slider_x_vel_value = 0.0;
            static float slider_y_vel_value = 0.0;
            static float slider_z_vel_value = 0.0;
            static float slider_r_vel_value = 0.0;
            static float slider_p_vel_value = 0.0;
            static float slider_w_vel_value = 0.0;
            ImGui::SliderFloat("Velocity x [m/s]", &slider_x_vel_value, -0.03, 0.03);
            ImGui::Dummy(ImVec2(0.0, 20.0));
            ImGui::SliderFloat("Velocity y [m/s]", &slider_y_vel_value, -0.03, 0.03);
            ImGui::Dummy(ImVec2(0.0, 20.0));
            ImGui::SliderFloat("Velocity z [m/s]", &slider_z_vel_value, -0.03, 0.03);
            ImGui::Dummy(ImVec2(0.0, 20.0));
            ImGui::SliderFloat("Angular velocity roll  [rad/s]", &slider_r_vel_value, -0.01, 0.01);
            ImGui::Dummy(ImVec2(0.0, 20.0));
            ImGui::SliderFloat("Angular velocity pitch [rad/s]", &slider_p_vel_value, -0.01, 0.01);
            ImGui::Dummy(ImVec2(0.0, 20.0));
            ImGui::SliderFloat("Angular velocity yow   [rad/s]", &slider_w_vel_value, -0.01, 0.01);
            ImGui::Dummy(ImVec2(0.0, 20.0));
            if (ImGui::Button("Reset"))
            {
                slider_x_vel_value = 0;
                slider_y_vel_value = 0;
                slider_z_vel_value = 0;
                slider_r_vel_value = 0;
                slider_p_vel_value = 0;
                slider_w_vel_value = 0;
            }
            ImGui::Dummy(ImVec2(0.0, 20.0));
            ImGui::End();

            //send command to techman
            slider_r_vel_value *= 0.01;
            slider_p_vel_value *= 0.01;
            slider_w_vel_value *= 0.01;
            std::stringstream tm_msgs;
            tm_msgs << std::fixed << std::setprecision(5) << "VELC" << slider_x_vel_value << "," << slider_y_vel_value << "," << slider_z_vel_value << "," << slider_r_vel_value << "," << slider_p_vel_value << "," << slider_w_vel_value << ":";
            std::cout << tm_msgs.str() << std::endl;
            boost::asio::write(socket, boost::asio::buffer(tm_msgs.str()));
        }

        // rendering
        ImGui::Render();
        int display_w, display_h;
        glfwGetFramebufferSize(window, &display_w, &display_h);
        glViewport(0, 0, display_w, display_h);
        glClearColor(clear_color.x, clear_color.y, clear_color.z, clear_color.w);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        glfwSwapBuffers(window);
    }

    // cleanup
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    glfwDestroyWindow(window);
    glfwTerminate();

    return 0;
}
