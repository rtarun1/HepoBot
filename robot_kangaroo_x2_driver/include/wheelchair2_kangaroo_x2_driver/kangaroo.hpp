#ifndef WHEELCHAIR2_KANGAROO_X2_DRIVER__KANGAROO_HPP_
#define WHEELCHAIR2_KANGAROO_X2_DRIVER__KANGAROO_HPP_

#include <memory>
#include <string>
#include <vector>
#include <boost/thread.hpp>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace wheelchair2_kangaroo_x2_driver
{
    class Kangaroo : public hardware_interface::SystemInterface
    {

        struct Configuration
        {
            std::string port;
            bool mixed_mode;
            int hz;
            double wheel_center_distance;
            int encoder_lines_per_revolution;
            double wheel_diameter;
            std::string left_wheel_name;
            std::string right_wheel_name;
        };

        struct Wheel
        {
            double command_velocity;
            double velocity;
            double position;
        };

    public:
        RCLCPP_SHARED_PTR_DEFINITIONS(Kangaroo)

        hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo &info) override;

        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

        hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State &previous_state) override;

        hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &previous_state) override;

        hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;

        hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;

        hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;

        hardware_interface::return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

        bool open_connection();
        void close_connection();

    private:
        bool is_connection_open() const;
        bool send_start_signals(unsigned char address);
        int get_parameter(unsigned char address, char channel, unsigned char desired_parameter);
        bool send_get_request(unsigned char address, char channel, unsigned char desired_parameter);
        int read_message(unsigned char address, bool &ok);
        unsigned char read_one_byte(bool &ok);
        int evaluate_kangaroo_response(unsigned char address, unsigned char *header, unsigned char *data, unsigned char *crc, bool &ok);
        void handle_errors(unsigned char address, int error_code);
        double encoder_lines_to_drive(int encoder_lines); // Convert encoder lines to meters
        int drive_to_encoder_lines(double linear);        // Convert meters to encoder lines
        double encoder_lines_to_turn(int encoder_lines);  // Convert encoder lines to radians
        int turn_to_encoder_lines(double angular);        // Convert radians to encoder lines
        double encoder_lines_to_radians(int encoder_lines);
        int radians_to_encoder_lines(double radians);
        bool set_channel_speed(double speed, unsigned char address, char channel);

        char channel_1_ = '1';
        char channel_2_ = '2';

        boost::mutex output_mutex_;
        boost::mutex input_mutex_;

        int termios_file_descriptor_;
        Configuration config_;

        Wheel left_wheel_;
        Wheel right_wheel_;
    };
} // namespace wheelchair2_kangaroo_x2_driver

#endif // WHEELCHAIR2_KANGAROO_X2_DRIVER__KANGAROO_HPP_