#pragma once

#include "oculus_bridge/oculus_protocol.hpp"

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace oculus_bridge
{

struct DecodedPing
{
  uint16_t version;
  uint16_t src_device_id;
  uint32_t ping_id;
  uint32_t status;
  double frequency;
  double temperature;
  double pressure;
  double heading;
  double pitch;
  double roll;
  double range_resolution;
  uint16_t n_ranges;
  uint16_t n_beams;
  DataSizeType data_size;
  std::vector<int16_t> bearings;
  std::vector<uint8_t> image;
  std::vector<uint8_t> raw_packet;
  std::string source;
};

struct DecodedStatus
{
  uint32_t device_id;
  uint16_t device_type;
  uint16_t part_number;
  uint32_t status;
  uint32_t ip_addr;
  uint32_t ip_mask;
  uint32_t connected_ip_addr;
  std::vector<uint8_t> mac_address;
  double pressure;
  double temperatures[8];
  std::string source;
};

class OculusPacketDecoder
{
public:
  std::optional<DecodedPing> decode_ping_packet(
    const std::vector<uint8_t> & packet,
    const std::string & source,
    std::string & error) const;

  std::optional<DecodedStatus> decode_status_packet(
    const std::vector<uint8_t> & packet,
    const std::string & source,
    std::string & error) const;

private:
  static size_t bytes_per_sample(DataSizeType data_size);
};

}  // namespace oculus_bridge
