#include <array>
#include <chrono>
#include <functional>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "quanser/quanser_messages.h"
#include "quanser/quanser_types.h"
#include "quanser/quanser_led.h"

#define LED_STRIP_SIZE 33

using namespace std::chrono_literals;

class QCar2LedTrip : public rclcpp::Node
{
public:
    QCar2LedTrip()
    : Node("qcar2_led_trip")
    {
        auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};

        param_desc.description = "Device type for the LED strip.";
        param_desc.additional_constraints = "Valid values are 'physical' and 'virtual'.";
        this->declare_parameter("device_type", std::string("physical"), param_desc);

        param_desc.description = "LED strip color ID.";
        param_desc.additional_constraints = "Valid values: 0=red, 1=green, 2=blue, 3=yellow, 4=cyan, 5=magenta.";
        this->declare_parameter("led_color_id", 0, param_desc);

        const std::string device_type = this->get_parameter("device_type").as_string();
        const std::string led_uri = get_led_uri(device_type);
        if (led_uri.empty()) {
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Opening LED strip on %s", led_uri.c_str());

        const t_int result = aaaf5050_mc_k12_open(led_uri.c_str(), LED_STRIP_SIZE, &led_strip_);
        if (result < 0) {
            msg_get_error_messageA(nullptr, result, error_message_, sizeof(error_message_));
            RCLCPP_ERROR(this->get_logger(), "Failed to open LED strip: %s", error_message_);
            return;
        }

        led_ready_ = true;
        last_led_color_id_ = static_cast<int>(this->get_parameter("led_color_id").as_int());
        apply_led_color(last_led_color_id_);

        parameter_cb_ = this->add_on_set_parameters_callback(
            std::bind(&QCar2LedTrip::set_parameters_callback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(200ms, std::bind(&QCar2LedTrip::sync_led_color, this));
    }

    ~QCar2LedTrip() override
    {
        if (!led_ready_) {
            return;
        }

        write_color(make_color(0, 0, 0));

        const t_int result = aaaf5050_mc_k12_close(led_strip_);
        if (result < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to close LED strip: %d", result);
        }
    }

private:
    std::string get_led_uri(const std::string & device_type)
    {
        if (device_type == "physical") {
            return "spi://localhost:1?memsize=420,word=8,baud=3333333,lsb=off,frame=1";
        }

        if (device_type == "virtual") {
            return "tcpip://localhost:18969";
        }

        RCLCPP_ERROR(
            this->get_logger(),
            "Invalid device_type '%s'. Use 'physical' or 'virtual'.",
            device_type.c_str());
        return "";
    }

    rcl_interfaces::msg::SetParametersResult set_parameters_callback(
        const std::vector<rclcpp::Parameter> & parameters)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;

        for (const auto & parameter : parameters) {
            if (parameter.get_name() == "device_type") {
                result.successful = false;
                result.reason = "device_type cannot be changed while the node is running.";
                return result;
            }

            if (parameter.get_name() == "led_color_id") {
                const int led_color_id = static_cast<int>(parameter.as_int());
                if (led_color_id < 0 || led_color_id > 5) {
                    result.successful = false;
                    result.reason = "led_color_id must be between 0 and 5.";
                    return result;
                }
            }
        }

        return result;
    }

    void sync_led_color()
    {
        if (!led_ready_) {
            return;
        }

        const int led_color_id = static_cast<int>(this->get_parameter("led_color_id").as_int());
        if (led_color_id == last_led_color_id_) {
            return;
        }

        last_led_color_id_ = led_color_id;
        apply_led_color(led_color_id);
    }

    void apply_led_color(int led_color_id)
    {
        static const std::array<t_led_color, 6> kColors = {{
            make_color(255, 0, 0),
            make_color(0, 255, 0),
            make_color(0, 0, 255),
            make_color(255, 255, 0),
            make_color(0, 255, 255),
            make_color(255, 0, 255),
        }};

        if (led_color_id < 0 || led_color_id >= static_cast<int>(kColors.size())) {
            RCLCPP_WARN(this->get_logger(), "Ignoring invalid led_color_id %d", led_color_id);
            return;
        }

        write_color(kColors[led_color_id]);
        RCLCPP_INFO(this->get_logger(), "LED strip color set to id %d", led_color_id);
    }

    static t_led_color make_color(t_uint8 red, t_uint8 green, t_uint8 blue)
    {
        t_led_color color{};
        color.red = red;
        color.green = green;
        color.blue = blue;
        return color;
    }

    void write_color(const t_led_color & color_value)
    {
        t_led_color color[LED_STRIP_SIZE];
        for (int i = 0; i < LED_STRIP_SIZE; ++i) {
            color[i] = color_value;
        }

        const t_int result = aaaf5050_mc_k12_write(led_strip_, color, LED_STRIP_SIZE);
        if (result < 0) {
            msg_get_error_messageA(nullptr, result, error_message_, sizeof(error_message_));
            RCLCPP_ERROR(this->get_logger(), "Failed to write LED strip: %s", error_message_);
        }
    }

    t_aaaf5050_mc_k12 led_strip_{};
    bool led_ready_ = false;
    int last_led_color_id_ = -1;
    char error_message_[512]{};

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr parameter_cb_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<QCar2LedTrip>());
    rclcpp::shutdown();
    return 0;
}
