#include "oculus_bridge/oculus_packet_decoder.hpp"

#include <cstring>

namespace oculus_bridge
{

namespace
{

template<typename T>
bool copy_struct(const std::vector<uint8_t> & packet, T & out)
{
  if (packet.size() < sizeof(T)) {
    return false;
  }
  std::memcpy(&out, packet.data(), sizeof(T));
  return true;
}

}  // namespace

size_t OculusPacketDecoder::bytes_per_sample(DataSizeType data_size)
{
  switch (data_size) {
    case DataSizeType::k8Bit:
      return 1;
    case DataSizeType::k16Bit:
      return 2;
    case DataSizeType::k24Bit:
      return 3;
    case DataSizeType::k32Bit:
      return 4;
    default:
      return 0;
  }
}

std::optional<DecodedPing> OculusPacketDecoder::decode_ping_packet(
  const std::vector<uint8_t> & packet,
  const std::string & source,
  std::string & error) const
{
  OculusMessageHeader header{};
  if (!copy_struct(packet, header)) {
    error = "Packet too small for Oculus header.";
    return std::nullopt;
  }

  if (header.oculus_id != kOculusCheckId) {
    error = "Invalid Oculus check id.";
    return std::nullopt;
  }

  if (header.msg_id != static_cast<uint16_t>(OculusMessageType::kSimplePingResult)) {
    error = "Packet is not a simple ping result.";
    return std::nullopt;
  }

  DecodedPing decoded{};
  decoded.version = header.msg_version;
  decoded.src_device_id = header.src_device_id;
  decoded.source = source;
  decoded.raw_packet = packet;

  uint32_t image_offset = 0;
  uint32_t image_size = 0;
  uint16_t n_beams = 0;
  uint16_t n_ranges = 0;
  size_t struct_size = 0;

  if (header.msg_version == 2) {
    OculusSimplePingResult2 ping{};
    if (!copy_struct(packet, ping)) {
      error = "Packet too small for OculusSimplePingResult2.";
      return std::nullopt;
    }

    decoded.ping_id = ping.ping_id;
    decoded.status = ping.status;
    decoded.frequency = ping.frequency;
    decoded.temperature = ping.temperature;
    decoded.pressure = ping.pressure;
    decoded.heading = ping.heading;
    decoded.pitch = ping.pitch;
    decoded.roll = ping.roll;
    decoded.range_resolution = ping.range_resolution;
    decoded.n_ranges = ping.n_ranges;
    decoded.n_beams = ping.n_beams;
    decoded.data_size = ping.data_size;

    image_offset = ping.image_offset;
    image_size = ping.image_size;
    n_beams = ping.n_beams;
    n_ranges = ping.n_ranges;
    struct_size = sizeof(OculusSimplePingResult2);
  } else {
    OculusSimplePingResult ping{};
    if (!copy_struct(packet, ping)) {
      error = "Packet too small for OculusSimplePingResult.";
      return std::nullopt;
    }

    decoded.ping_id = ping.ping_id;
    decoded.status = ping.status;
    decoded.frequency = ping.frequency;
    decoded.temperature = ping.temperature;
    decoded.pressure = ping.pressure;
    decoded.heading = 0.0;
    decoded.pitch = 0.0;
    decoded.roll = 0.0;
    decoded.range_resolution = ping.range_resolution;
    decoded.n_ranges = ping.n_ranges;
    decoded.n_beams = ping.n_beams;
    decoded.data_size = ping.data_size;

    image_offset = ping.image_offset;
    image_size = ping.image_size;
    n_beams = ping.n_beams;
    n_ranges = ping.n_ranges;
    struct_size = sizeof(OculusSimplePingResult);
  }

  const size_t expected_packet_size = sizeof(OculusMessageHeader) + header.payload_size;
  if (packet.size() < expected_packet_size) {
    error = "Packet shorter than declared payload size.";
    return std::nullopt;
  }

  if (struct_size + static_cast<size_t>(n_beams) * sizeof(int16_t) > packet.size()) {
    error = "Packet too small for bearing table.";
    return std::nullopt;
  }

  if (static_cast<size_t>(image_offset) + static_cast<size_t>(image_size) > packet.size()) {
    error = "Packet too small for image payload.";
    return std::nullopt;
  }

  const size_t sample_bytes = bytes_per_sample(decoded.data_size);
  if (sample_bytes == 0) {
    error = "Unsupported data size.";
    return std::nullopt;
  }

  const size_t minimum_image_size =
    static_cast<size_t>(n_beams) * static_cast<size_t>(n_ranges) * sample_bytes;
  if (image_size < minimum_image_size) {
    error = "Image payload smaller than beam/range dimensions imply.";
    return std::nullopt;
  }

  decoded.bearings.resize(n_beams);
  std::memcpy(decoded.bearings.data(), packet.data() + struct_size, n_beams * sizeof(int16_t));

  decoded.image.resize(image_size);
  std::memcpy(decoded.image.data(), packet.data() + image_offset, image_size);

  return decoded;
}

std::optional<DecodedStatus> OculusPacketDecoder::decode_status_packet(
  const std::vector<uint8_t> & packet,
  const std::string & source,
  std::string & error) const
{
  OculusStatusMsg status{};
  if (!copy_struct(packet, status)) {
    error = "Packet too small for OculusStatusMsg.";
    return std::nullopt;
  }

  if (status.hdr.oculus_id != kOculusCheckId) {
    error = "Invalid Oculus status check id.";
    return std::nullopt;
  }

  DecodedStatus decoded{};
  decoded.device_id = status.device_id;
  decoded.device_type = status.device_type;
  decoded.part_number = status.part_number;
  decoded.status = status.status;
  decoded.ip_addr = status.ip_addr;
  decoded.ip_mask = status.ip_mask;
  decoded.connected_ip_addr = status.connected_ip_addr;
  decoded.mac_address = {
    status.mac_addr0, status.mac_addr1, status.mac_addr2,
    status.mac_addr3, status.mac_addr4, status.mac_addr5};
  decoded.pressure = status.pressure;
  decoded.temperatures[0] = status.temperature0;
  decoded.temperatures[1] = status.temperature1;
  decoded.temperatures[2] = status.temperature2;
  decoded.temperatures[3] = status.temperature3;
  decoded.temperatures[4] = status.temperature4;
  decoded.temperatures[5] = status.temperature5;
  decoded.temperatures[6] = status.temperature6;
  decoded.temperatures[7] = status.temperature7;
  decoded.source = source;

  return decoded;
}

}  // namespace oculus_bridge
