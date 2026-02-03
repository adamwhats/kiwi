#include "motor2040_comms.hpp"

#include <iostream>

Motor2040Comms::~Motor2040Comms()
{
  if (serial_conn_.IsOpen())
  {
    Motor2040Comms::disconnect();
  }
}

void Motor2040Comms::connect(const std::string& serial_device, int timeout_ms)
{
    serial_timeout_ms_ = timeout_ms;
    serial_conn_.Open(serial_device);
    serial_conn_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
}

void Motor2040Comms::disconnect()
{
  std::array<float, 4> zeros = {0.0f, 0.0f, 0.0f, 0.0f};
  Motor2040Comms::set_velocities(zeros),
  serial_conn_.Close();
}

bool Motor2040Comms::is_connected() const 
{
  std::string output_buffer = std::string(16, 0);
  return serial_conn_.IsOpen();
}

void Motor2040Comms::send_and_receive_velocities_(std::string output)
{
  // Send the byte string
  serial_conn_.FlushIOBuffers();
  serial_conn_.Write(output);

  // Wait for a response
  std::string input_buffer(16, 0);
  try
  {
    serial_conn_.Read(input_buffer, MESSAGE_LENGTH_, serial_timeout_ms_);
  }
  catch (const LibSerial::ReadTimeout&)
  {
      std::cerr << "The LibSerial::Read() call has timed out - potentially caused by high current draw" << std::endl ;
  }

  // Parse the response
  for (uint i = 0; i < NUM_MOTORS_; i++) {
    const float* fptr = reinterpret_cast<const float*>(&input_buffer[i * sizeof(float)]);
    velocities_measured_[i] = *fptr * (2 * 3.1415927);
  }
}

std::array<double, 4> Motor2040Comms::get_velocities()
{
  std::array<double, 4> velocities;
  for (uint i = 0; i < NUM_MOTORS_; i++) {
    velocities[i] = static_cast<double>(velocities_measured_[i]);
  }
  return velocities;
}

std::array<double, 4> Motor2040Comms::get_positions()
{
  std::array<double, 4> positions;
  for (uint i = 0; i < NUM_MOTORS_; i++) {
    positions[i] = static_cast<double>(positions_measured_[i]);
  }
  return positions;
}
