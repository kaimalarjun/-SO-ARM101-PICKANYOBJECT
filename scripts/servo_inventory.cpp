// Read-only SO-101 servo inventory using the pinned Feetech driver library.
// Sends register READ requests only; never changes torque, EEPROM or position.
#include <feetech_driver/common.hpp>
#include <feetech_driver/communication_protocol.hpp>

#include <cstdint>
#include <iostream>
#include <memory>

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "usage: servo_inventory /dev/serial/by-id/DEVICE\n";
    return 2;
  }

  auto serial = std::make_unique<feetech_driver::SerialPort>(argv[1]);
  if (auto result = serial->configure(); !result) {
    std::cerr << "serial configure: " << result.error() << '\n';
    return 3;
  }
  if (auto result = serial->open(); !result) {
    std::cerr << "serial open: " << result.error() << '\n';
    return 3;
  }
  feetech_driver::CommunicationProtocol bus(std::move(serial));

  std::cout << "id model position_ticks homing_offset_ticks range_min range_max\n";
  for (int id = 1; id <= 6; ++id) {
    auto model = bus.read_model_number(static_cast<uint8_t>(id));
    auto position = bus.read_position(static_cast<uint8_t>(id));
    auto offset = bus.read_word(static_cast<uint8_t>(id), SMS_STS_OFS_L);
    auto range_min = bus.read_word(static_cast<uint8_t>(id), SMS_STS_MIN_ANGLE_LIMIT_L);
    auto range_max = bus.read_word(static_cast<uint8_t>(id), SMS_STS_MAX_ANGLE_LIMIT_L);
    if (!model || !position || !offset || !range_min || !range_max) {
      std::cerr << "could not read all registers for servo " << id << '\n';
      return 4;
    }
    std::cout << id << ' ' << model.value() << ' ' << position.value() << ' '
              << feetech_driver::decode_sign_magnitude(
                     offset.value(), SMS_STS_SIGN_BIT_HOMING_OFFSET)
              << ' ' << range_min.value() << ' ' << range_max.value() << '\n';
  }
}
