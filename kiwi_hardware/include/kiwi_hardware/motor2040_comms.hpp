#ifndef MOTOR2040_COMMS_HPP_
#define MOTOR2040_COMMS_HPP_

#include <array>
#include <cstring>
#include <libserial/SerialPort.h>


class Motor2040Comms
{
public:
  Motor2040Comms() = default;
  ~Motor2040Comms();

  // Serial port management
  void connect(const std::string& serial_device, int timeout_ms);
  void disconnect();
  bool is_connected() const;

  // Velocity control
  template <typename T>
  void set_velocities(std::array<T, 4> vel_targets)
  {
    // Convert input array to an array of floats
    std::array<float, 4> vel_targets_float;
    for (uint i = 0; i < NUM_MOTORS_; i++) {
      vel_targets_float[i] = static_cast<float>(vel_targets[i] / (2 * 3.1415927));
    }

    // Build a 16 byte string from an array of floats
    std::string output_buffer = std::string(MESSAGE_LENGTH_, 0);
    for (uint i = 0; i < NUM_MOTORS_; i++) {
      const char* cptr = reinterpret_cast<const char*>(&vel_targets_float[i]);
      memcpy(&output_buffer[i * sizeof(float)], cptr, sizeof(float));
    }

    send_and_receive_velocities_(output_buffer);
  }

  // Feedback
  std::array<double, 4> get_positions();
  std::array<double, 4> get_velocities();

private:
  // Serial port communication
  LibSerial::SerialPort serial_conn_;
  int serial_timeout_ms_;
  void send_and_receive_velocities_(std::string output);

  // Constants
  static const uint8_t NUM_MOTORS_ = 4;
  static const uint8_t MESSAGE_LENGTH_ = 4 * NUM_MOTORS_;
  
  // Feedback
  std::array<int8_t, NUM_MOTORS_> positions_measured_;
  std::array<float, 4> velocities_measured_;
};

#endif // MOTOR2040_COMMS_HPP_
