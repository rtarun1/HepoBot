#include "robot_kangaroo_x2_driver/kangaroo.hpp"
#include "robot_kangaroo_x2_driver/kangaroo_library.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>
#include <termios.h>
#include <fcntl.h>
#include <cerrno>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

#define RAD_2_DEG 180.0 / 3.14159265359
#define DEG_2_RAD 3.14159265359 / 180.0

namespace robot_kangaroo_x2_driver
{
    /* ---------------------------------------- */
    /* --------- KANGAROO IO FUNCTIONS -------- */
    /* ---------------------------------------- */

    bool Kangaroo::is_connection_open() const
    {
        return termios_file_descriptor_ >= 0;
    }

    bool Kangaroo::open_connection()
    {
        RCLCPP_INFO(rclcpp::get_logger("Kangaroo"), "Opening connection...");
        struct termios port_options;

        if (is_connection_open())
        {
            RCLCPP_INFO(rclcpp::get_logger("Kangaroo"), "Port is already open, closing it first");
            close_connection();
        }

        termios_file_descriptor_ = ::open(config_.port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);

        if (termios_file_descriptor_ < 0)
        {
            RCLCPP_FATAL(rclcpp::get_logger("Kangaroo"), "Failed to open port %s", strerror(errno));
            // RCLCPP_FATAL(rclcpp::get_logger("Kangaroo"), "Failed to open port %s", config_.port.c_str());
            return false;
        }

        if (fcntl(termios_file_descriptor_, F_SETFL, 0) < 0)
        {
            RCLCPP_FATAL(rclcpp::get_logger("Kangaroo"), "Failed to set port descriptor");
            return false;
        }

        if (tcgetattr(termios_file_descriptor_, &port_options) < 0)
        {
            RCLCPP_FATAL(rclcpp::get_logger("Kangaroo"), "Failed to get port attributes");
            return false;
        }

        if (cfsetispeed(&port_options, B9600) < 0)
        {
            RCLCPP_FATAL(rclcpp::get_logger("Kangaroo"), "Failed to set input baud rate");
            return false;
        }

        if (cfsetospeed(&port_options, B9600) < 0)
        {
            RCLCPP_FATAL(rclcpp::get_logger("Kangaroo"), "Failed to set output baud rate");
            return false;
        }

        port_options.c_cflag |= (CREAD | CLOCAL | CS8);
        port_options.c_cflag &= ~(PARODD | CRTSCTS | CSTOPB | PARENB);
        // port_options.c_iflag |= ( );
        port_options.c_iflag &= ~(IUCLC | IXANY | IMAXBEL | IXON | IXOFF | IUTF8 | ICRNL | INPCK);
        port_options.c_oflag |= (NL0 | CR0 | TAB0 | BS0 | VT0 | FF0);
        port_options.c_oflag &= ~(OPOST | ONLCR | OLCUC | OCRNL | ONOCR | ONLRET | OFILL | OFDEL | NL1 | CR1 | CR2 | TAB3 | BS1 | VT1 | FF1);
        port_options.c_lflag |= (NOFLSH);
        port_options.c_lflag &= ~(ICANON | IEXTEN | TOSTOP | ISIG | ECHOPRT | ECHO | ECHOE | ECHOK | ECHOCTL | ECHOKE);
        port_options.c_cc[VINTR] = 0x03;
        port_options.c_cc[VQUIT] = 0x1C;
        port_options.c_cc[VERASE] = 0x7F;
        port_options.c_cc[VKILL] = 0x15;
        port_options.c_cc[VEOF] = 0x04;
        port_options.c_cc[VTIME] = 0x01;
        port_options.c_cc[VMIN] = 0x00;
        port_options.c_cc[VSWTC] = 0x00;
        port_options.c_cc[VSTART] = 0x11;
        port_options.c_cc[VSTOP] = 0x13;
        port_options.c_cc[VSUSP] = 0x1A;
        port_options.c_cc[VEOL] = 0x00;
        port_options.c_cc[VREPRINT] = 0x12;
        port_options.c_cc[VDISCARD] = 0x0F;
        port_options.c_cc[VWERASE] = 0x17;
        port_options.c_cc[VLNEXT] = 0x16;
        port_options.c_cc[VEOL2] = 0x00;

        if (tcsetattr(termios_file_descriptor_, TCSANOW, &port_options) < 0)
        {
            RCLCPP_FATAL(rclcpp::get_logger("Kangaroo"), "Failed to set port attributes");
            return false;
        }

        RCLCPP_INFO(rclcpp::get_logger("Kangaroo"), "Connection opened.");
        return true;
    }

    void Kangaroo::close_connection()
    {
        RCLCPP_INFO(rclcpp::get_logger("Kangaroo"), "Closing connection...");
        ::close(termios_file_descriptor_);
        RCLCPP_INFO(rclcpp::get_logger("Kangaroo"), "Connection closed.");
    }

    bool Kangaroo::send_start_signals(unsigned char address)
    {
        unsigned char buffer[7];

        size_t bytes_written_to_channel_3 = write_kangaroo_start_command(address, channel_1_, buffer);
        if (::write(termios_file_descriptor_, buffer, bytes_written_to_channel_3) < 0)
        {
            RCLCPP_ERROR(rclcpp::get_logger("Kangaroo"), "Failed to send start signal to channel 1 or 3/D");
            close_connection();
            return false;
        }

        size_t bytes_written_to_channel_4 = write_kangaroo_start_command(address, channel_2_, buffer);
        if (::write(termios_file_descriptor_, buffer, bytes_written_to_channel_4) < 0)
        {
            RCLCPP_ERROR(rclcpp::get_logger("Kangaroo"), "Failed to send start signal to channel 2 or 4/T");
            close_connection();
            return false;
        }

        return true;
    }

    int Kangaroo::get_parameter(unsigned char address, char channel, unsigned char desired_parameter)
    {
        output_mutex_.lock();
        input_mutex_.lock();

        if (!send_get_request(address, channel, desired_parameter))
        {
            output_mutex_.unlock();
            input_mutex_.unlock();

            throw std::runtime_error("Failed to send get request");
        }

        output_mutex_.unlock();

        bool ok = true;
        int result = read_message(address, ok);

        input_mutex_.unlock();

        if (!ok)
        {
            throw std::runtime_error("Failed to read message");
        }

        return result;
    }

    bool Kangaroo::send_get_request(unsigned char address, char channel, unsigned char desired_parameter)
    {
        if (!is_connection_open())
        {
            return false;
        }

        unsigned char buffer[18];

        int num_of_bytes = write_kangaroo_get_command(address, channel, desired_parameter, buffer);

        if (::write(termios_file_descriptor_, buffer, num_of_bytes) < 0)
        {
            RCLCPP_ERROR(rclcpp::get_logger("Kangaroo"), "Failed to send get request");
            return false;
        }

        return true;
    }

    int Kangaroo::read_message(unsigned char address, bool &ok)
    {
        if (!is_connection_open())
        {
            ok = false;
            return 0;
        }

        unsigned char header[3] = {0};
        unsigned char data[8] = {0};
        unsigned char crc[2] = {0};

        for (size_t i = 0; i < 3; i++)
        {
            header[i] = read_one_byte(ok);
            if (!ok)
            {
                RCLCPP_ERROR(rclcpp::get_logger("Kangaroo"), "Failed to get data");
                return 0;
            }
        }

        for (size_t i = 0; i < header[2]; i++)
        {
            data[i] = read_one_byte(ok);
            // bail if reading one byte fails
            if (!ok)
            {
                RCLCPP_ERROR(rclcpp::get_logger("Kangaroo"), "Failed to get data");
                return 0;
            }
            //std::cout << (int)data[i] << ", ";
        }

        for (size_t i = 0; i < 2; i++)
        {
            crc[i] = read_one_byte(ok);
            if (!ok)
            {
                RCLCPP_ERROR(rclcpp::get_logger("Kangaroo"), "Failed to read CRC");
                return 0;
            }
        }

        return evaluate_kangaroo_response(address, header, data, crc, ok);
    }

    unsigned char Kangaroo::read_one_byte(bool &ok)
    {
        unsigned char buffer;

        int bits_read = ::read(termios_file_descriptor_, &buffer, 1);

        if (bits_read == 0)
        {
            // try once more to read from the serial communication
            bits_read = ::read(termios_file_descriptor_, &buffer, 1);
            if (bits_read < 0)
            {
                RCLCPP_ERROR(rclcpp::get_logger("Kangaroo"), "Reading from serial failed: %s", strerror(errno));
                // close();
                ok = false;
                return 0;
            }
            if (bits_read == 0)
            {
                RCLCPP_ERROR(rclcpp::get_logger("Kangaroo"), "Reading from serial failed: %s", strerror(errno));
                ok = false;
                return 0;
            }
        }

        if (bits_read > 0)
            return buffer;
        if (bits_read < 0)
        {
            RCLCPP_ERROR(rclcpp::get_logger("Kangaroo"), "Reading from serial failed: %s", strerror(errno));
            close_connection();
            ok = false;
            return 0;
        }

        return buffer;
    }

    int Kangaroo::evaluate_kangaroo_response(unsigned char address, unsigned char *header, unsigned char *data, unsigned char *, bool &ok)
    {
        size_t data_size = header[2];
        size_t value_size = data_size - 3; // there are 3 bits that aren't the value
        size_t value_offset = 3;           // offset of the value in data

        if (data[1] & (1 << 1))
        {
            // ROS_INFO("The joint is under acceleration.");
        }

        if (data[1] & (1 << 4)) // testing the flag to see if there is an echo code
        {
            value_size--; // if there is an echo code, then there will be 1 extra byte
            value_offset++;
            // echo_code = true;
            RCLCPP_INFO(rclcpp::get_logger("Kangaroo"), "There was an echo code.");
        }

        if (data[1] & (1 << 6))
        {
            value_size--;
            value_offset++;
            RCLCPP_INFO(rclcpp::get_logger("Kangaroo"), "There was a sequence code.");
        }

        int value = un_bitpack_number(&data[value_offset], value_size);

        // std::cout << "The value is: " << value << std::endl;
        if (data[1] & 1) // testing the flag to see if there is an error
        {
            handle_errors(address, value);
            ok = false;
            return 0;
        }

        return value;
    }

    void Kangaroo::handle_errors(unsigned char address, int error_code)
    {
        switch (error_code)
        {
        case 1:
            send_start_signals(address);
            RCLCPP_ERROR(rclcpp::get_logger("Kangaroo"), "Channels have not been started, attempted to start them.");
            break;
        case 2:
            RCLCPP_ERROR(rclcpp::get_logger("Kangaroo"), "Channel needs to be homed.");
            break;
        case 3:
            send_start_signals(address);
            RCLCPP_ERROR(rclcpp::get_logger("Kangaroo"), "Control error occurred. Sent start signals to clear.");
            break;
        case 4:
            RCLCPP_ERROR(rclcpp::get_logger("Kangaroo"), "Controller is in the wrong mode. Change the switch.");
            break;
        case 5:
            RCLCPP_ERROR(rclcpp::get_logger("Kangaroo"), "The given parameter is unknown.");
            break;
        case 6:
            RCLCPP_ERROR(rclcpp::get_logger("Kangaroo"), "Serial timeout occurred.");
            break;
        default:
            RCLCPP_ERROR(rclcpp::get_logger("Kangaroo"), "Something terrible has occurred. (ERROR)");
        }
        return;
    }

    inline double Kangaroo::encoder_lines_to_radians(int encoder_lines)
    {
        return (encoder_lines * 2 * M_PI / config_.encoder_lines_per_revolution);
    }

    inline int Kangaroo::radians_to_encoder_lines(double radians)
    {
        return (radians * config_.encoder_lines_per_revolution / (2 * M_PI));
    }

    double Kangaroo::encoder_lines_to_drive(int encoder_lines)
    {
        return (encoder_lines * ((config_.wheel_diameter * M_PI) / config_.encoder_lines_per_revolution)) / 2;
    }

    int Kangaroo::drive_to_encoder_lines(double linear)
    {
        return 2 * linear * (config_.encoder_lines_per_revolution / (config_.wheel_diameter * M_PI));
    }

    double Kangaroo::encoder_lines_to_turn(int encoder_lines)
    {
        return ((encoder_lines * (360.0 / ((config_.encoder_lines_per_revolution * config_.wheel_center_distance) / config_.wheel_diameter))) / 2) * DEG_2_RAD;
    }

    int Kangaroo::turn_to_encoder_lines(double angular)
    {
        return 2 * (angular * RAD_2_DEG) * (((config_.encoder_lines_per_revolution * config_.wheel_center_distance) / config_.wheel_diameter) / 360.0);
    }

    bool Kangaroo::set_channel_speed(double speed, unsigned char address, char channel)
    {
        // std::cout << "Sending " << speed << " to Channel " << channel << std::endl;

        if (!is_connection_open())
        {
            return false;
        }

        output_mutex_.lock();

        // send
        unsigned char buffer[18];
        int num_of_bytes = write_kangaroo_speed_command(address, channel, speed, buffer);

        output_mutex_.unlock();

        if (::write(termios_file_descriptor_, buffer, num_of_bytes) < 0)
        {
            RCLCPP_ERROR(rclcpp::get_logger("Kangaroo"), "Failed to update channel %c: %s", channel, strerror(errno));
            close_connection();
            return false;
        }
        return true;
    }

    /* ---------------------------------------- */
    /* ---------- HARDWARE INTERFACE ---------- */
    /* ---------------------------------------- */

    hardware_interface::CallbackReturn Kangaroo::on_init(const hardware_interface::HardwareInfo &info)
    {
        RCLCPP_INFO(rclcpp::get_logger("Kangaroo"), "Initializing...");
        // Check if the requested resources are available
        if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
        {
            return hardware_interface::CallbackReturn::ERROR;
        }

        // Get the parameters
        config_.port = (std::string)info_.hardware_parameters["port"];
        config_.hz = std::stoi(info_.hardware_parameters["hz"]);
        config_.encoder_lines_per_revolution = std::stoi(info_.hardware_parameters["encoder_lines_per_revolution"]);
        config_.wheel_center_distance = std::stod(info_.hardware_parameters["wheel_center_distance"]);
        config_.wheel_diameter = std::stod(info_.hardware_parameters["wheel_diameter"]);
        config_.left_wheel_name = info_.hardware_parameters["left_wheel_name"];
        config_.right_wheel_name = info_.hardware_parameters["right_wheel_name"];
        config_.mixed_mode = info_.hardware_parameters["mixed_mode"] == "true";

        if (config_.mixed_mode)
        {
            RCLCPP_INFO(rclcpp::get_logger("Kangaroo"), "Mixed Mode is on.");
            channel_1_ = '3';
            channel_2_ = '4';
        }

        termios_file_descriptor_ = -1;

        RCLCPP_INFO(rclcpp::get_logger("Kangaroo"), "Initialized");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    std::vector<hardware_interface::StateInterface> Kangaroo::export_state_interfaces()
    {
        std::vector<hardware_interface::StateInterface> state_interfaces;
        state_interfaces.emplace_back(hardware_interface::StateInterface(config_.left_wheel_name, hardware_interface::HW_IF_POSITION, &left_wheel_.position));
        state_interfaces.emplace_back(hardware_interface::StateInterface(config_.right_wheel_name, hardware_interface::HW_IF_POSITION, &right_wheel_.position));

        state_interfaces.emplace_back(hardware_interface::StateInterface(config_.left_wheel_name, hardware_interface::HW_IF_VELOCITY, &left_wheel_.velocity));
        state_interfaces.emplace_back(hardware_interface::StateInterface(config_.right_wheel_name, hardware_interface::HW_IF_VELOCITY, &right_wheel_.velocity));
        return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> Kangaroo::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        command_interfaces.emplace_back(hardware_interface::CommandInterface(config_.left_wheel_name, hardware_interface::HW_IF_VELOCITY, &left_wheel_.command_velocity));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(config_.right_wheel_name, hardware_interface::HW_IF_VELOCITY, &right_wheel_.command_velocity));
        return command_interfaces;
    }

    hardware_interface::CallbackReturn Kangaroo::on_configure(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(rclcpp::get_logger("Kangaroo"), "Configuring...");
        if (!is_connection_open() && !open_connection())
        {
            return hardware_interface::CallbackReturn::ERROR;
        }
        RCLCPP_INFO(rclcpp::get_logger("Kangaroo"), "Configured");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn Kangaroo::on_cleanup(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(rclcpp::get_logger("Kangaroo"), "Cleaning up...");
        close_connection();
        RCLCPP_INFO(rclcpp::get_logger("Kangaroo"), "Cleaned up");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn Kangaroo::on_activate(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(rclcpp::get_logger("Kangaroo"), "Activating...");

        if (!is_connection_open())
        {
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (!send_start_signals((unsigned char)128))
        {
            return hardware_interface::CallbackReturn::ERROR;
        }

        RCLCPP_INFO(rclcpp::get_logger("Kangaroo"), "Activated");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn Kangaroo::on_deactivate(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(rclcpp::get_logger("Kangaroo"), "Deactivating...");
        // close_connection(); Already closed in cleanup
        RCLCPP_INFO(rclcpp::get_logger("Kangaroo"), "Deactivated");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::return_type Kangaroo::read(const rclcpp::Time &, const rclcpp::Duration &)
    {
        // RCLCPP_INFO(rclcpp::get_logger("Kangaroo"), "Reading...");

        if (!is_connection_open())
        {
            return hardware_interface::return_type::ERROR;
        }

        try{
            if (config_.mixed_mode)
            {
                double linear_position = encoder_lines_to_drive(get_parameter((unsigned char)128, channel_1_, (unsigned char)1));
                double angular_position = encoder_lines_to_turn(get_parameter((unsigned char)128, channel_2_, (unsigned char)1));
                
                // Convert to individual wheel positions
                left_wheel_.position = (linear_position - (angular_position * config_.wheel_center_distance / 2)) / (config_.wheel_diameter / 2);
                right_wheel_.position = (linear_position + (angular_position * config_.wheel_center_distance / 2)) / (config_.wheel_diameter / 2);

                double linear_velocity = encoder_lines_to_drive(get_parameter((unsigned char)128, channel_1_, (unsigned char)2));
                double angular_velocity = encoder_lines_to_turn(get_parameter((unsigned char)128, channel_2_, (unsigned char)2));
                
                // Convert to individual wheel velocities
                left_wheel_.velocity = (linear_velocity - (angular_velocity * config_.wheel_center_distance / 2)) / (config_.wheel_diameter / 2);
                right_wheel_.velocity = (linear_velocity + (angular_velocity * config_.wheel_center_distance / 2)) / (config_.wheel_diameter / 2);
            }
            else
            {
                left_wheel_.position = encoder_lines_to_radians(get_parameter((unsigned char)128, channel_1_, (unsigned char)1));
                right_wheel_.position = encoder_lines_to_radians(get_parameter((unsigned char)128, channel_2_, (unsigned char)1));

                left_wheel_.velocity = encoder_lines_to_radians(get_parameter((unsigned char)128, channel_1_, (unsigned char)2));
                right_wheel_.velocity = encoder_lines_to_radians(get_parameter((unsigned char)128, channel_2_, (unsigned char)2));
            }
        }
        catch (std::string error)
        {
            RCLCPP_ERROR(rclcpp::get_logger("Kangaroo"), "Error: %s", error.c_str());
            return hardware_interface::return_type::ERROR;
        }

        // RCLCPP_INFO(rclcpp::get_logger("Kangaroo"), "Read: %s %s %s %s", std::to_string(left_wheel_.position).c_str(), std::to_string(left_wheel_.velocity).c_str(), std::to_string(right_wheel_.position).c_str(), std::to_string(right_wheel_.velocity).c_str());
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type Kangaroo::write(const rclcpp::Time &, const rclcpp::Duration &)
    {
        // RCLCPP_INFO(rclcpp::get_logger("Kangaroo"), "Writing %s %s", std::to_string(left_wheel_.command_velocity).c_str(), std::to_string(right_wheel_.command_velocity).c_str());

        if (!is_connection_open())
        {
            return hardware_interface::return_type::ERROR;
        }

        tcflush(termios_file_descriptor_, TCOFLUSH);

        if (config_.mixed_mode)
        {
            // Convert individual wheel velocities to linear and angular velocities
            double left_wheel_velocity = left_wheel_.command_velocity * (config_.wheel_diameter / 2);
            double right_wheel_velocity = right_wheel_.command_velocity * (config_.wheel_diameter / 2);

            double linear_velocity_command = (left_wheel_velocity + right_wheel_velocity) / 2;
            double angular_velocity_command = (right_wheel_velocity - left_wheel_velocity) / config_.wheel_center_distance;

            linear_velocity_command = drive_to_encoder_lines(linear_velocity_command);
            angular_velocity_command = turn_to_encoder_lines(angular_velocity_command);

            set_channel_speed(linear_velocity_command, (unsigned char)128, channel_1_);
            set_channel_speed(angular_velocity_command, (unsigned char)128, channel_2_);
        }
        else
        {
            double left_wheel_velocity = radians_to_encoder_lines(left_wheel_.command_velocity);
            double right_wheel_velocity = radians_to_encoder_lines(right_wheel_.command_velocity);

            set_channel_speed(left_wheel_velocity, (unsigned char)128, channel_1_);
            set_channel_speed(right_wheel_velocity, (unsigned char)128, channel_2_);
        }

        return hardware_interface::return_type::OK;
    }
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  robot_kangaroo_x2_driver::Kangaroo, hardware_interface::SystemInterface)