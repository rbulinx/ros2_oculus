#pragma once

#include <cstdint>

namespace oculus_bridge
{

constexpr uint16_t kOculusCheckId = 0x4f53;

enum class OculusMessageType : uint16_t
{
  kSimpleFire = 0x15,
  kPingResult = 0x22,
  kSimplePingResult = 0x23,
  kUserConfig = 0x55,
  kDummy = 0xff,
};

enum class DataSizeType : uint8_t
{
  k8Bit = 0,
  k16Bit = 1,
  k24Bit = 2,
  k32Bit = 3,
};

#pragma pack(push, 1)

struct OculusMessageHeader
{
  uint16_t oculus_id;
  uint16_t src_device_id;
  uint16_t dst_device_id;
  uint16_t msg_id;
  uint16_t msg_version;
  uint32_t payload_size;
  uint16_t spare2;
};

struct OculusSimpleFireMessage
{
  OculusMessageHeader head;
  uint8_t master_mode;
  uint8_t ping_rate;
  uint8_t network_speed;
  uint8_t gamma_correction;
  uint8_t flags;
  double range;
  double gain_percent;
  double speed_of_sound;
  double salinity;
};

struct OculusSimpleFireMessage2
{
  OculusMessageHeader head;
  uint8_t master_mode;
  uint8_t ping_rate;
  uint8_t network_speed;
  uint8_t gamma_correction;
  uint8_t flags;
  double range_percent;
  double gain_percent;
  double speed_of_sound;
  double salinity;
  uint32_t ext_flags;
  uint32_t reserved0[2];
  uint32_t beacon_locator_frequency;
  uint32_t reserved1[5];
};

struct OculusSimplePingResult
{
  OculusSimpleFireMessage fire_message;
  uint32_t ping_id;
  uint32_t status;
  double frequency;
  double temperature;
  double pressure;
  double speed_of_sound_used;
  uint32_t ping_start_time;
  DataSizeType data_size;
  double range_resolution;
  uint16_t n_ranges;
  uint16_t n_beams;
  uint32_t image_offset;
  uint32_t image_size;
  uint32_t message_size;
};

struct OculusSimplePingResult2
{
  OculusSimpleFireMessage2 fire_message;
  uint32_t ping_id;
  uint32_t status;
  double frequency;
  double temperature;
  double pressure;
  double heading;
  double pitch;
  double roll;
  double speed_of_sound_used;
  double ping_start_time;
  DataSizeType data_size;
  double range_resolution;
  uint16_t n_ranges;
  uint16_t n_beams;
  uint32_t spare0;
  uint32_t spare1;
  uint32_t spare2;
  uint32_t spare3;
  uint32_t image_offset;
  uint32_t image_size;
  uint32_t message_size;
};

struct OculusVersionInfo
{
  uint32_t firmware_version0;
  uint32_t firmware_date0;
  uint32_t firmware_version1;
  uint32_t firmware_date1;
  uint32_t firmware_version2;
  uint32_t firmware_date2;
};

struct OculusStatusMsg
{
  OculusMessageHeader hdr;
  uint32_t device_id;
  uint16_t device_type;
  uint16_t part_number;
  uint32_t status;
  OculusVersionInfo version_info;
  uint32_t ip_addr;
  uint32_t ip_mask;
  uint32_t connected_ip_addr;
  uint8_t mac_addr0;
  uint8_t mac_addr1;
  uint8_t mac_addr2;
  uint8_t mac_addr3;
  uint8_t mac_addr4;
  uint8_t mac_addr5;
  double temperature0;
  double temperature1;
  double temperature2;
  double temperature3;
  double temperature4;
  double temperature5;
  double temperature6;
  double temperature7;
  double pressure;
};

#pragma pack(pop)

}  // namespace oculus_bridge
