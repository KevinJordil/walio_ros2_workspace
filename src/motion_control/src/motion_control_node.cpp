#include <iostream>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "opencn_communication_interfaces/srv/pins.hpp"
#include "opencn_communication_interfaces/srv/init.hpp"

using Pin = opencn_communication_interfaces::msg::Pin;

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

        // Wait for the service to be available
        while (!init_client->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
                return false;
            }
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Init service not available, waiting again...");
        }

        // Call the service
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
                return false;
            }
        } else {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call init service");
            return false;
        }

        // Wait 1 second to be sure that the device is ready
        rclcpp::sleep_for(std::chrono::seconds(1));

        // Create a reset request if the device is in fault state
        auto reset_request = std::make_shared<opencn_communication_interfaces::srv::Pins::Request>();

        // Array of Pin
        std::vector<Pin> reset_request_pins;

        // Create a pin to reset the fault
        Pin reset_pin;
        reset_pin.pin_class = Pin::CMPINBIT;
        reset_pin.cmpinbit.value = 1;
        reset_pin.name = "lcec.0.EPOS4.fault-reset";
        reset_pin.transaction_type = Pin::SET;
        reset_request_pins.push_back(reset_pin);

        reset_request->pins = reset_request_pins;

        // Wait for the service to be available
        while (!pin_client->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
                return false;
            }
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Pins service not available, waiting again...");
        }

        // Call the service
        auto reset_future_result = pin_client->async_send_request(reset_request);
        
        // Wait for the result.
        if(reset_future_result.valid()){
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Reset driver service call was successful");
        } else {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Reset driver service call error");
            return false;
        }

        // Wait 2 seconds to be sure that the device is in a right state
        rclcpp::sleep_for(std::chrono::seconds(2));

        // Create a request to set the parameters
        auto first_request = std::make_shared<opencn_communication_interfaces::srv::Pins::Request>();

        // Array of Pin
        std::vector<Pin> first_request_pins;

        Pin first_request_pin1;
        first_request_pin1.pin_class = Pin::CMPINFLOAT;
        first_request_pin1.cmpinfloat.value = 3000;
        first_request_pin1.name = "basic_pg.best_pg.joint0.max_speed";
        first_request_pin1.transaction_type = Pin::SET;
        first_request_pins.push_back(first_request_pin1);

        Pin first_request_pin2;
        first_request_pin2.pin_class = Pin::CMPINFLOAT;
        first_request_pin2.cmpinfloat.value = 100;
        first_request_pin2.name = "basic_pg.best_pg.joint0.max_accel";
        first_request_pin2.transaction_type = Pin::SET;
        first_request_pins.push_back(first_request_pin2);

        Pin first_request_pin3;
        first_request_pin3.pin_class = Pin::CMPINBIT;
        first_request_pin3.cmpinbit.value = 1;
        first_request_pin3.name = "basic_pg.best_pg.enable";
        first_request_pin3.transaction_type = Pin::SET;
        first_request_pins.push_back(first_request_pin3);

        first_request->pins = first_request_pins;

        // Wait for the service to be available
        while (!pin_client->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
                return false;
            }
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Pins service not available, waiting again...");
        }

        // Call the service
        auto first_future_result = pin_client->async_send_request(first_request);
        
        // Wait for the result.
        if(first_future_result.valid()){
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Parameters service call was successful");
        } else {
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Parameters service call error");
            return false;
        }

        return true;
    }

    // Function to test the loopback service with all types of pins with the SET transaction
    bool call_set_loopback_service(){
        auto request = std::make_shared<opencn_communication_interfaces::srv::Pins::Request>();

        // Array of Pin
        std::vector<Pin> request_pins;

        Pin request_pin1;
        request_pin1.pin_class = Pin::CMPINI32;
        request_pin1.cmpini32.value = -42;
        request_pin1.name = "loopback.0.s32_in.1";
        request_pin1.transaction_type = Pin::SET;
        request_pins.push_back(request_pin1);

        Pin request_pin2;
        request_pin2.pin_class = Pin::CMPINU32;
        request_pin2.cmpinu32.value = 43;
        request_pin2.name = "loopback.0.u32_in.2";
        request_pin2.transaction_type = Pin::SET;
        request_pins.push_back(request_pin2);

        Pin request_pin3;
        request_pin3.pin_class = Pin::CMPINBIT;
        request_pin3.cmpinbit.value = true;
        request_pin3.name = "loopback.0.b_in.3";
        request_pin3.transaction_type = Pin::SET;
        request_pins.push_back(request_pin3);

        Pin request_pin4;
        request_pin4.pin_class = Pin::CMPINFLOAT;
        request_pin4.cmpinfloat.value = 3.14;
        request_pin4.name = "loopback.0.f_in.0";
        request_pin4.transaction_type = Pin::SET;
        request_pins.push_back(request_pin4);

        request->pins = request_pins;

        // Wait for the service to be available
        while (!pin_client->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
                return false;
            }
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "pins service not available, waiting again...");
        }

        // Call the service
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

    // Function to test the loopback service with all types of pins with the GET transaction
    bool cal_get_loopback_service(){
        auto request = std::make_shared<opencn_communication_interfaces::srv::Pins::Request>();
        // Get the result
        std::vector<Pin> result_pins;

        Pin result_pin1;
        result_pin1.pin_class = Pin::CMPINI32;
        result_pin1.name = "loopback.0.s32_out.1";
        result_pin1.transaction_type = Pin::GET;
        result_pins.push_back(result_pin1);

        Pin result_pin2;
        result_pin2.pin_class = Pin::CMPINU32;
        result_pin2.name = "loopback.0.u32_out.2";
        result_pin2.transaction_type = Pin::GET;
        result_pins.push_back(result_pin2);

        Pin result_pin3;
        result_pin3.pin_class = Pin::CMPINBIT;
        result_pin3.name = "loopback.0.b_out.3";
        result_pin3.transaction_type = Pin::GET;
        result_pins.push_back(result_pin3);

        Pin result_pin4;
        result_pin4.pin_class = Pin::CMPINFLOAT;
        result_pin4.name = "loopback.0.f_out.0";
        result_pin4.transaction_type = Pin::GET;
        result_pins.push_back(result_pin4);

        request->pins = result_pins;

        // Wait for the service to be available
        while (!pin_client->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
                return false;
            }
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Pins service not available, waiting again...");
        }

        // Call the service
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
                    if (pin.pin_class == Pin::CMPINI32) {
                        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Pin %s: %d", pin.name.c_str(), pin.cmpini32.value);
                    } else if (pin.pin_class == Pin::CMPINU32) {
                        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Pin %s: %u", pin.name.c_str(), pin.cmpinu32.value);
                    } else if (pin.pin_class == Pin::CMPINBIT) {
                        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Pin %s: %s", pin.name.c_str(), pin.cmpinbit.value ? "true" : "false");
                    } else if (pin.pin_class == Pin::CMPINFLOAT) {
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
        //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Axis %d: %f", 1, msg->axes[1]);   
        float future_speed = msg->axes[1] * 1000;
        if(current_speed != future_speed && ((current_speed + 10 < future_speed) || (current_speed - 10 > future_speed) || (future_speed < 10 && future_speed > -10))){
            current_speed = future_speed;
            auto first_request = std::make_shared<opencn_communication_interfaces::srv::Pins::Request>();

            // Array of Pin
            std::vector<Pin> first_request_pins;

            Pin first_request_pin1;
            first_request_pin1.pin_class = Pin::CMPINFLOAT;
            first_request_pin1.cmpinfloat.value = current_speed;
            first_request_pin1.name = "basic_pg.best_pg.joint0.cmd_in";
            first_request_pin1.transaction_type = Pin::SET;
            first_request_pins.push_back(first_request_pin1);

            first_request->pins = first_request_pins;

            // Wait for the service to be available
            while (!pin_client->wait_for_service(std::chrono::seconds(1))) {
                if (!rclcpp::ok()) {
                    RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
                    return;
                }
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Pins service not available, waiting again...");
            }

            // Call the service
            auto first_future_result = pin_client->async_send_request(first_request);
            
            // Wait for the result.
            if(first_future_result.valid()){
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service call was successful");
            } else {
                RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service call error");
            }

            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "speed set: %f", current_speed);   


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

    float current_speed = 0;

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

    //node->call_set_loopback_service();

    //node->cal_get_loopback_service();

    rclcpp::shutdown();

    return 0;
}
