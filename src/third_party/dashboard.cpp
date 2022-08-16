#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <iostream>
#include <utility>

int main(int argc, char const* argv[])
{
    // GL 3.2 + GLSL 150
    const char* glsl_version = "#version 130";
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    // Create window with graphic context
    ImVec2 window_size = { 1000, 680 };
    glfwInit();
    GLFWwindow* window = glfwCreateWindow(window_size.x, window_size.y, "Dashboard for Bed System", nullptr, nullptr);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1); // enable vsync

    // set up dear imgui context
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();

    // set up dear imgui style
    ImGui::StyleColorsLight();

    // setup platform/renderer bindings
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init(glsl_version);
    // ImGui_ImplOpenGL3_Init();

    // background color
    ImVec4 clear_color = ImVec4(0.3, 0.3, 0.3, 1);

    // main loop
    while (!glfwWindowShouldClose(window)) {
        // get event
        glfwPollEvents();

        // start the dear imgui frame
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();
        {
            // widget setting for window1
            ImGui::SetNextWindowPos(ImVec2(0, 0));
            ImGui::SetNextWindowSize(ImVec2(300, 680));
            ImGui::Begin("Diagnosis for bed system");
            ImGui::Button("button 1");
            bool is_connected_ros = true;
            ImGui::Checkbox("Connect to ROS", &is_connected_ros);
            if (is_connected_ros) {
                ImGui::Text("connected");
            } else {
                ImGui::Text("Not connected");
            }
            bool is_connected_dualsense = false;
            ImGui::Checkbox("Connect to Dualsense(PS5 controller)", &is_connected_dualsense);
            if (is_connected_dualsense) {
                ImGui::Text("connected");
            } else {
                ImGui::Text("Not connected");
            }
            ImGui::End();
        }

        {
            // widget setting for window2
            ImGui::SetNextWindowPos(ImVec2(300, 0));
            ImGui::SetNextWindowSize(ImVec2(700, 680));
            ImGui::Begin("Control for bed system");
            ImGui::TextColored(ImVec4(0.0, 0.5, 0.2, 1.0), "Techman");
            //techman control widgets
            static float slider_x_vel_value = 0.0;
            static float slider_y_vel_value = 0.0;
            static float slider_z_vel_value = 0.0;
            static float slider_r_vel_value = 0.0;
            static float slider_p_vel_value = 0.0;
            static float slider_w_vel_value = 0.0;
            ImGui::SliderFloat("Velocity x [m/s]", &slider_x_vel_value, -0.1, 0.1);
            ImGui::SliderFloat("Velocity y [m/s]", &slider_y_vel_value, -0.1, 0.1);
            ImGui::SliderFloat("Velocity z [m/s]", &slider_z_vel_value, -0.1, 0.1);
            ImGui::SliderFloat("Angular velocity roll  [rad/s]", &slider_r_vel_value, -0.1, 0.1);
            ImGui::SliderFloat("Angular velocity pitch [rad/s]", &slider_p_vel_value, -0.1, 0.1);
            ImGui::SliderFloat("Angular velocity yow   [rad/s]", &slider_w_vel_value, -0.1, 0.1);
            if (ImGui::Button("Reset")) {
                slider_x_vel_value = 0;
                slider_y_vel_value = 0;
                slider_z_vel_value = 0;
                slider_r_vel_value = 0;
                slider_p_vel_value = 0;
                slider_w_vel_value = 0;
            }

            ImGui::Dummy(ImVec2(0.0, 20.0));
            ImGui::Separator();

            ImGui::TextColored(ImVec4(0.6, 0.0, 0.2, 1.0), "Leptrino");
            //leptrino control widgets

            ImGui::Dummy(ImVec2(0.0, 20.0));
            ImGui::Separator();

            ImGui::TextColored(ImVec4(0.3, 0.1, 0.8, 1.0), "Linear Actuator");
            //linear actuator widgets

            ImGui::Dummy(ImVec2(0.0, 20.0));
            ImGui::End();
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
