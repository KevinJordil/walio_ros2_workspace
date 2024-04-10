#include <iostream>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "opencn_communication_interfaces/srv/pins.hpp"
#include "opencn_communication_interfaces/srv/init.hpp"

class MotionControlNode : public rclcpp::Node
{

public:
    MotionControlNode() : Node("motion_control_node") {
        init_client = this->create_client<opencn_communication_interfaces::srv::Init>("opencn_init");

        pin_client = this->create_client<opencn_communication_interfaces::srv::Pins>("opencn_pins");

        joy_sub = this->create_subscription<sensor_msgs::msg::Joy>("joy", 10, std::bind(&MotionControlNode::joy_callback, this, std::placeholders::_1));
    }

    bool call_init_service()
    {
        auto init_request = std::make_shared<opencn_communication_interfaces::srv::Init::Request>();
        init_request->ip_address = "192.168.53.15";
        init_request->port = 7002;
        init_request->device = "EPOS4";

        while (!init_client->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
                return false;
            }
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Init service not available, waiting again...");
        }

        auto future_init_result = init_client->async_send_request(init_request);
        // Wait for the result.
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future_init_result) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
            // Get result as pointer due to a ROS bug
            auto result = *future_init_result.get();
            // Check if the service call was successful.
            if (result.success) {
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Init service call was successful");
            } else {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Init service call was not successful");
            }
        } else {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call init service");
        }

        // ros sleep 1 seconds
        rclcpp::sleep_for(std::chrono::seconds(1));

        auto reset_request = std::make_shared<opencn_communication_interfaces::srv::Pins::Request>();

        // Array of Pin
        std::vector<opencn_communication_interfaces::msg::Pin> reset_request_pins;

        opencn_communication_interfaces::msg::Pin reset_pin;
        reset_pin.pin_class = opencn_communication_interfaces::msg::Pin::CMPINBIT;
        reset_pin.cmpinbit.value = 1;
        reset_pin.name = "lcec.0.EPOS4.fault-reset";
        reset_pin.transaction_type = opencn_communication_interfaces::msg::Pin::SET;
        reset_request_pins.push_back(reset_pin);

        reset_request->pins = reset_request_pins;

        while (!pin_client->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
                return false;
            }
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Pins service not available, waiting again...");
        }

        auto reset_future_result = pin_client->async_send_request(reset_request);
        // Wait for the result.

        if(reset_future_result.valid()){
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service call was successful");
        } else {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service call error");
        }

        // ros sleep 1 seconds
        rclcpp::sleep_for(std::chrono::seconds(2));

        auto request = std::make_shared<opencn_communication_interfaces::srv::Pins::Request>();

        // Array of Pin
        std::vector<opencn_communication_interfaces::msg::Pin> request_pins;

        opencn_communication_interfaces::msg::Pin request_pin1;
        request_pin1.pin_class = opencn_communication_interfaces::msg::Pin::CMPINU32;
        request_pin1.cmpinu32.value = 1;
        request_pin1.name = "mux.target.selector";
        request_pin1.transaction_type = opencn_communication_interfaces::msg::Pin::SET;
        request_pins.push_back(request_pin1);

        opencn_communication_interfaces::msg::Pin request_pin2;
        request_pin2.pin_class = opencn_communication_interfaces::msg::Pin::CMPINBIT;
        request_pin2.cmpinbit.value = 1;
        request_pin2.name = "simple_pg.enable";
        request_pin2.transaction_type = opencn_communication_interfaces::msg::Pin::SET;
        request_pins.push_back(request_pin2);

        opencn_communication_interfaces::msg::Pin request_pin3;
        request_pin3.pin_class = opencn_communication_interfaces::msg::Pin::CMPINBIT;
        request_pin3.cmpinbit.value = 1;
        request_pin3.name = "lcec.0.EPOS4.set-mode-csv";
        request_pin3.transaction_type = opencn_communication_interfaces::msg::Pin::SET;
        request_pins.push_back(request_pin3);

        request->pins = request_pins;

        while (!pin_client->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
                return false;
            }
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Pins service not available, waiting again...");
        }

        auto future_result = pin_client->async_send_request(request);
        // Wait for the result.

        if(future_result.valid()){
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service call was successful");
        } else {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service call error");
        }


        return true;
    }

    bool call_set_loopback_service(){
        auto request = std::make_shared<opencn_communication_interfaces::srv::Pins::Request>();

        // Array of Pin
        std::vector<opencn_communication_interfaces::msg::Pin> request_pins;

        opencn_communication_interfaces::msg::Pin request_pin1;
        request_pin1.pin_class = opencn_communication_interfaces::msg::Pin::CMPINI32;
        request_pin1.cmpini32.value = -42;
        request_pin1.name = "loopback.0.s32_in.1";
        request_pin1.transaction_type = opencn_communication_interfaces::msg::Pin::SET;
        request_pins.push_back(request_pin1);

        opencn_communication_interfaces::msg::Pin request_pin2;
        request_pin2.pin_class = opencn_communication_interfaces::msg::Pin::CMPINU32;
        request_pin2.cmpinu32.value = 43;
        request_pin2.name = "loopback.0.u32_in.2";
        request_pin2.transaction_type = opencn_communication_interfaces::msg::Pin::SET;
        request_pins.push_back(request_pin2);

        opencn_communication_interfaces::msg::Pin request_pin3;
        request_pin3.pin_class = opencn_communication_interfaces::msg::Pin::CMPINBIT;
        request_pin3.cmpinbit.value = true;
        request_pin3.name = "loopback.0.b_in.3";
        request_pin3.transaction_type = opencn_communication_interfaces::msg::Pin::SET;
        request_pins.push_back(request_pin3);

        opencn_communication_interfaces::msg::Pin request_pin4;
        request_pin4.pin_class = opencn_communication_interfaces::msg::Pin::CMPINFLOAT;
        request_pin4.cmpinfloat.value = 3.14;
        request_pin4.name = "loopback.0.f_in.0";
        request_pin4.transaction_type = opencn_communication_interfaces::msg::Pin::SET;
        request_pins.push_back(request_pin4);


        request->pins = request_pins;

        while (!pin_client->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
                return false;
            }
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "pins service not available, waiting again...");
        }

        auto future_result = pin_client->async_send_request(request);
        // Wait for the result.
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future_result) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
            // Get result as pointer due to a ROS bug (https://github.com/ros2/rclcpp/issues/1968)
            auto result = *future_result.get();
            // Check if the service call was successful.
            if (result.success) {
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Pins service call was successful");
            } else {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Pins service call was not successful");
            }


        } else {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call pins service");
        }

        return true;
    }

    bool cal_get_loopback_service(){
        auto request = std::make_shared<opencn_communication_interfaces::srv::Pins::Request>();
        // Get the result
        std::vector<opencn_communication_interfaces::msg::Pin> result_pins;

        opencn_communication_interfaces::msg::Pin result_pin1;
        result_pin1.pin_class = opencn_communication_interfaces::msg::Pin::CMPINI32;
        result_pin1.name = "loopback.0.s32_out.1";
        result_pin1.transaction_type = opencn_communication_interfaces::msg::Pin::GET;
        result_pins.push_back(result_pin1);

        opencn_communication_interfaces::msg::Pin result_pin2;
        result_pin2.pin_class = opencn_communication_interfaces::msg::Pin::CMPINU32;
        result_pin2.name = "loopback.0.u32_out.2";
        result_pin2.transaction_type = opencn_communication_interfaces::msg::Pin::GET;
        result_pins.push_back(result_pin2);

        opencn_communication_interfaces::msg::Pin result_pin3;
        result_pin3.pin_class = opencn_communication_interfaces::msg::Pin::CMPINBIT;
        result_pin3.name = "loopback.0.b_out.3";
        result_pin3.transaction_type = opencn_communication_interfaces::msg::Pin::GET;
        result_pins.push_back(result_pin3);

        opencn_communication_interfaces::msg::Pin result_pin4;
        result_pin4.pin_class = opencn_communication_interfaces::msg::Pin::CMPINFLOAT;
        result_pin4.name = "loopback.0.f_out.0";
        result_pin4.transaction_type = opencn_communication_interfaces::msg::Pin::GET;
        result_pins.push_back(result_pin4);

        request->pins = result_pins;

        while (!pin_client->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
                return false;
            }
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Pins service not available, waiting again...");
        }

        auto future_result = pin_client->async_send_request(request);
        // Wait for the result.
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future_result) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
            // Get result as pointer due to a ROS bug (https://github.com/ros2/rclcpp/issues/1968)
            auto result = *future_result.get();
            // Check if the service call was successful.
            if (result.success) {
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service call was successful");
                // For each pin in the result, print the value
                for (auto pin : result.pins) {
                    if (pin.pin_class == opencn_communication_interfaces::msg::Pin::CMPINI32) {
                        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Pin %s: %d", pin.name.c_str(), pin.cmpini32.value);
                    } else if (pin.pin_class == opencn_communication_interfaces::msg::Pin::CMPINU32) {
                        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Pin %s: %u", pin.name.c_str(), pin.cmpinu32.value);
                    } else if (pin.pin_class == opencn_communication_interfaces::msg::Pin::CMPINBIT) {
                        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Pin %s: %s", pin.name.c_str(), pin.cmpinbit.value ? "true" : "false");
                    } else if (pin.pin_class == opencn_communication_interfaces::msg::Pin::CMPINFLOAT) {
                        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Pin %s: %f", pin.name.c_str(), pin.cmpinfloat.value);
                    }
                }


            } else {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Pins service call was not successful");
            }


        } else {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call pins service");
        }

        return true;
    }


private:
    // Joy callback
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        if(old_button_status != msg->buttons[0]){
            auto request = std::make_shared<opencn_communication_interfaces::srv::Pins::Request>();

            // Array of Pin
            std::vector<opencn_communication_interfaces::msg::Pin> request_pins;

            opencn_communication_interfaces::msg::Pin request_pin1;
            request_pin1.pin_class = opencn_communication_interfaces::msg::Pin::CMPINFLOAT;
            request_pin1.cmpinfloat.value = msg->buttons[0] * 50;
            request_pin1.name = "simple_pg.joint1.cmd-pos";
            request_pin1.transaction_type = opencn_communication_interfaces::msg::Pin::SET;
            request_pins.push_back(request_pin1);

            request->pins = request_pins;

            while (!pin_client->wait_for_service(std::chrono::seconds(1))) {
                if (!rclcpp::ok()) {
                    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
                    return;
                }
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Pins service not available, waiting again...");
            }

            auto future_result = pin_client->async_send_request(request);
            // Wait for the result.

            if(future_result.valid()){
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service call was successful");
            } else {
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service call error");
            }


            old_button_status = msg->buttons[0];


        }
        /*
        for (size_t i = 0; i < msg->axes.size(); i++) {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Axis %ld: %f", i, msg->axes[i]);
        }
        for (size_t i = 0; i < msg->buttons.size(); i++) {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Button %ld: %d", i, msg->buttons[i]);
        }
        */
    }
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub;

    int old_button_status = 0;

    // Init service
    rclcpp::Client<opencn_communication_interfaces::srv::Init>::SharedPtr init_client;

    // Pins service
    rclcpp::Client<opencn_communication_interfaces::srv::Pins>::SharedPtr pin_client;


};


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Initializing MotionControlNode");

    auto node = std::make_shared<MotionControlNode>();

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Call init");
    node->call_init_service();

    rclcpp::spin(node);

    // ros sleep 2 seconds
    //rclcpp::sleep_for(std::chrono::seconds(2));

    //node->call_set_loopback_service();

    //node->cal_get_loopback_service();

    rclcpp::shutdown();

    return 0;
}
