#include <Arduino.h>
#include <SPI.h>
#include <cmath>

#include <DW1000Ng.hpp>
#include <DW1000NgConstants.hpp>

#include <HardwareSerial.h>

// Tag mode selection (set via build_flags in platformio.ini)
#ifndef TAG_MODE_TDOA3
  #ifndef TAG_MODE_TWR
    #define TAG_MODE_TDOA3 1  // Default mode
  #endif
#endif

#if !TAG_MODE_TDOA3
  #include "twr_tag.hpp"
#endif

namespace {
constexpr uint8_t kPinDwCs = 7;   // TODO: update to your wiring
constexpr uint8_t kPinDwIrq = 2;  // TODO: update to your wiring
constexpr uint8_t kPinDwRst = 3;  // TODO: update to your wiring

constexpr uint16_t kNetworkId = 0xBEEF;
constexpr uint8_t kDeviceId = 0x08;  // tag id
constexpr uint8_t kFirstAnchorId = 0x01;
constexpr uint8_t kAnchorCount = 4;

constexpr bool kEnableDebug = true;
constexpr uint32_t kDebugIntervalMs = 1000;
constexpr uint32_t kHeartbeatIntervalMs = 1000;
constexpr uint32_t kDistanceSensorIntervalMs = 100;

constexpr int8_t kMavUartRxPin = 20;
constexpr int8_t kMavUartTxPin = 21;
constexpr uint32_t kMavUartBaud = 57600;

constexpr uint8_t kMavSystemId = 1;
constexpr uint8_t kMavComponentId = 191;  // MAV_COMP_ID_ONBOARD_COMPUTER
constexpr uint8_t kMavType = 18;          // MAV_TYPE_ONBOARD_CONTROLLER
constexpr uint8_t kMavAutopilot = 8;      // MAV_AUTOPILOT_INVALID
constexpr uint8_t kMavBaseMode = 0;
constexpr uint8_t kMavSystemStatus = 3;   // MAV_STATE_STANDBY
constexpr uint16_t kDistanceMinCm = 10;
constexpr uint16_t kDistanceMaxCm = 5000;
constexpr uint8_t kDistanceTypeLaser = 0;       // MAV_DISTANCE_SENSOR_LASER
constexpr uint8_t kDistanceOrientationNone = 0; // MAV_SENSOR_ROTATION_NONE
constexpr uint8_t kDistanceCovarianceCm = 0;
constexpr uint8_t kDistanceCovarianceMin = 1;
constexpr bool kDistanceUsePhysicalAnchorIds = false;

#if TAG_MODE_TDOA3
  constexpr bool kUartBinaryOutput = false;
#else
  constexpr bool kUartBinaryOutput = false;  // Enable binary output for TWR
#endif

constexpr size_t kMaxFrameSize = 127;

#if TAG_MODE_TDOA3
  // TDOA3 constants
  constexpr uint8_t kPacketType = 0x30;
  constexpr uint8_t kSeqHasDistanceMask = 0x80;
  constexpr uint8_t kPositionBlockType = 0xA1;
  constexpr size_t kMacHeaderSize = 21;
  constexpr size_t kMacDestAddrOffset = 5;
  constexpr size_t kMacSrcAddrOffset = 13;
#endif

device_configuration_t kRadioConfig = {
    .extendedFrameLength = false,
    .receiverAutoReenable = true,
    .smartPower = true,
    .frameCheck = true,
    .nlos = false,
    .sfd = SFDMode::STANDARD_SFD,
    .channel = Channel::CHANNEL_2,
    .dataRate = DataRate::RATE_6800KBPS,
    .pulseFreq = PulseFrequency::FREQ_64MHZ,
    .preambleLen = PreambleLength::LEN_128,
    .preaCode = PreambleCode::CODE_9,
};

interrupt_configuration_t kInterruptConfig = {
    .interruptOnSent = false,
    .interruptOnReceived = false,
    .interruptOnReceiveFailed = false,
    .interruptOnReceiveTimeout = false,
    .interruptOnReceiveTimestampAvailable = false,
    .interruptOnAutomaticAcknowledgeTrigger = false,
};

#if TAG_MODE_TDOA3
  struct __attribute__((packed)) RangeHeader {
    uint8_t type;
    uint8_t seq;
    uint32_t txTimeStamp;
    uint8_t remoteCount;
  };

  struct __attribute__((packed)) RemoteAnchorDataShort {
    uint8_t id;
    uint8_t seq;
    uint32_t rxTimeStamp;
  };

  struct __attribute__((packed)) RemoteAnchorDataFull {
    uint8_t id;
    uint8_t seq;
    uint32_t rxTimeStamp;
    uint16_t distance;
  };
#else
  // TWR mode
  twr_tag::Tag gTag;
  bool gTxInFlight = false;
#endif

uint8_t gRxBuffer[kMaxFrameSize] = {};
uint32_t gLastDebugMs = 0;
uint32_t gRxCount = 0;
uint32_t gTxCount = 0;
uint32_t gLastHeartbeatMs = 0;
uint32_t gLastDistanceSensorMs = 0;
uint8_t gMavSequence = 0;
uint8_t gNextDistanceAnchorIndex = 0;

HardwareSerial gMavUart(1);

uint16_t mavCrcAccumulate(uint8_t data, uint16_t crc) {
  uint8_t tmp = data ^ static_cast<uint8_t>(crc & 0xFF);
  tmp ^= static_cast<uint8_t>(tmp << 4);
  return static_cast<uint16_t>((crc >> 8) ^
                               (static_cast<uint16_t>(tmp) << 8) ^
                               (static_cast<uint16_t>(tmp) << 3) ^
                               (static_cast<uint16_t>(tmp) >> 4));
}

uint16_t mavX25Crc(const uint8_t* data, size_t len, uint8_t crcExtra) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; ++i) {
    crc = mavCrcAccumulate(data[i], crc);
  }
  crc = mavCrcAccumulate(crcExtra, crc);
  return crc;
}

void sendMavlinkV2Message(uint32_t msgId,
                          const uint8_t* payload,
                          uint8_t payloadLen,
                          uint8_t crcExtra) {
  constexpr uint8_t kStx = 0xFD;
  constexpr uint8_t kIncompatFlags = 0x00;
  constexpr uint8_t kCompatFlags = 0x00;
  uint8_t packet[10 + 32 + 2] = {};
  if (payloadLen > 32) {
    return;
  }

  packet[0] = kStx;
  packet[1] = payloadLen;
  packet[2] = kIncompatFlags;
  packet[3] = kCompatFlags;
  packet[4] = gMavSequence++;
  packet[5] = kMavSystemId;
  packet[6] = kMavComponentId;
  packet[7] = static_cast<uint8_t>(msgId & 0xFF);
  packet[8] = static_cast<uint8_t>((msgId >> 8) & 0xFF);
  packet[9] = static_cast<uint8_t>((msgId >> 16) & 0xFF);
  memcpy(packet + 10, payload, payloadLen);

  uint16_t crc = mavX25Crc(packet + 1, 9 + payloadLen, crcExtra);
  packet[10 + payloadLen] = static_cast<uint8_t>(crc & 0xFF);
  packet[10 + payloadLen + 1] = static_cast<uint8_t>((crc >> 8) & 0xFF);

  gMavUart.write(packet, 10 + payloadLen + 2);
}

void sendMavlinkHeartbeat() {
  constexpr uint8_t kMsgIdHeartbeat = 0;
  constexpr uint8_t kPayloadLen = 9;
  constexpr uint8_t kCrcExtraHeartbeat = 50;

  uint8_t payload[kPayloadLen] = {};

  uint32_t customMode = 0;
  memcpy(payload, &customMode, sizeof(customMode));
  payload[4] = kMavType;
  payload[5] = kMavAutopilot;
  payload[6] = kMavBaseMode;
  payload[7] = kMavSystemStatus;
  payload[8] = 3;  // MAVLink v1.0

  sendMavlinkV2Message(kMsgIdHeartbeat, payload, kPayloadLen, kCrcExtraHeartbeat);
}

#if !TAG_MODE_TDOA3
void sendMavlinkDistanceSensor(uint8_t anchorIndex, uint32_t nowMs) {
  constexpr uint8_t kMsgIdDistanceSensor = 132;
  constexpr uint8_t kPayloadLen = 14;
  constexpr uint8_t kCrcExtraDistanceSensor = 85;

  uint16_t currentDistanceCm = 0;
  uint8_t covariance = kDistanceCovarianceCm;

  float distanceMeters = gTag.getDistance(anchorIndex);
  if (std::isfinite(distanceMeters) && distanceMeters > 0.0f) {
    long roundedCm = lroundf(distanceMeters * 100.0f);
    if (roundedCm < kDistanceMinCm) {
      roundedCm = kDistanceMinCm;
    }
    if (roundedCm > kDistanceMaxCm) {
      roundedCm = kDistanceMaxCm;
    }
    currentDistanceCm = static_cast<uint16_t>(roundedCm);
  } else {
    covariance = 255;
  }

  if (covariance == 0) {
    covariance = kDistanceCovarianceMin;
  }

  uint8_t payload[kPayloadLen] = {};
  memcpy(payload + 0, &nowMs, sizeof(nowMs));
  memcpy(payload + 4, &kDistanceMinCm, sizeof(kDistanceMinCm));
  memcpy(payload + 6, &kDistanceMaxCm, sizeof(kDistanceMaxCm));
  memcpy(payload + 8, &currentDistanceCm, sizeof(currentDistanceCm));
  payload[10] = kDistanceTypeLaser;
  payload[11] = static_cast<uint8_t>(0xA0 + anchorIndex);
  payload[12] = anchorIndex;
  payload[13] = covariance;

  if (kEnableDebug && !kUartBinaryOutput) {
    Serial.printf("MAV DS id=%u idx=%u dist=%ucm cov=%u\n",
            static_cast<unsigned>(payload[11]),
                  static_cast<unsigned>(anchorIndex),
                  static_cast<unsigned>(currentDistanceCm),
                  static_cast<unsigned>(covariance));
  }

  sendMavlinkV2Message(kMsgIdDistanceSensor,
                       payload,
                       kPayloadLen,
                       kCrcExtraDistanceSensor);
}
#endif

void startReceive() {
  DW1000Ng::startReceive(ReceiveMode::IMMEDIATE);
}

#if TAG_MODE_TDOA3

void printPositionBlock(const uint8_t* payload, size_t remaining) {
  if (remaining < 1 + 3 * sizeof(float)) {
    return;
  }
  if (payload[0] != kPositionBlockType) {
    return;
  }
  float pos[3];
  memcpy(pos, payload + 1, sizeof(pos));
  Serial.printf("POS x=%.2f y=%.2f z=%.2f\n", pos[0], pos[1], pos[2]);
}

size_t writeBytes(uint8_t* buffer, size_t offset, const void* data, size_t len) {
  memcpy(buffer + offset, data, len);
  return offset + len;
}

void emitBinaryFrame(uint8_t sourceId,
                     const RangeHeader* rangeHeader,
                     const uint8_t* payload,
                     size_t payloadLength,
                     uint64_t rxTimestamp) {
  // Frame format (little-endian):
  // [0xAA 0x55] [len u16] [src u8] [seq u8] [remoteCount u8]
  // [txTs u32] [rxTs u64] [remote entries...] [optional pos block]
  uint8_t out[256] = {};
  size_t offset = 0;
  const uint8_t start[2] = {0xAA, 0x55};
  offset = writeBytes(out, offset, start, sizeof(start));

  uint16_t lenPlaceholder = 0;
  size_t lenOffset = offset;
  offset = writeBytes(out, offset, &lenPlaceholder, sizeof(lenPlaceholder));

  offset = writeBytes(out, offset, &sourceId, sizeof(sourceId));
  offset = writeBytes(out, offset, &rangeHeader->seq, sizeof(rangeHeader->seq));
  offset = writeBytes(out, offset, &rangeHeader->remoteCount, sizeof(rangeHeader->remoteCount));
  offset = writeBytes(out, offset, &rangeHeader->txTimeStamp, sizeof(rangeHeader->txTimeStamp));
  offset = writeBytes(out, offset, &rxTimestamp, sizeof(rxTimestamp));

  const uint8_t* ptr = payload;
  size_t remaining = payloadLength;
  for (uint8_t i = 0; i < rangeHeader->remoteCount; i++) {
    if (remaining < sizeof(RemoteAnchorDataShort)) {
      break;
    }
    auto remoteShort = reinterpret_cast<const RemoteAnchorDataShort*>(ptr);
    bool hasDistance = (remoteShort->seq & kSeqHasDistanceMask) != 0;
    uint8_t seq = static_cast<uint8_t>(remoteShort->seq & ~kSeqHasDistanceMask);

    offset = writeBytes(out, offset, &remoteShort->id, sizeof(remoteShort->id));
    offset = writeBytes(out, offset, &seq, sizeof(seq));
    offset = writeBytes(out, offset, &remoteShort->rxTimeStamp, sizeof(remoteShort->rxTimeStamp));

    if (hasDistance && remaining >= sizeof(RemoteAnchorDataFull)) {
      auto remoteFull = reinterpret_cast<const RemoteAnchorDataFull*>(ptr);
      offset = writeBytes(out, offset, &remoteFull->distance, sizeof(remoteFull->distance));
      ptr += sizeof(RemoteAnchorDataFull);
      remaining -= sizeof(RemoteAnchorDataFull);
    } else {
      uint16_t zero = 0;
      offset = writeBytes(out, offset, &zero, sizeof(zero));
      ptr += sizeof(RemoteAnchorDataShort);
      remaining -= sizeof(RemoteAnchorDataShort);
    }
  }

  if (remaining >= 1 + 3 * sizeof(float) && ptr[0] == kPositionBlockType) {
    offset = writeBytes(out, offset, ptr, 1 + 3 * sizeof(float));
  }

  uint16_t payloadLen = static_cast<uint16_t>(offset - sizeof(start) - sizeof(lenPlaceholder));
  memcpy(out + lenOffset, &payloadLen, sizeof(payloadLen));

  Serial.write(out, offset);
}

void handlePacket(const uint8_t* data, uint16_t len, uint64_t rxTimestamp) {
  if (len < kMacHeaderSize + sizeof(RangeHeader)) {
    return;
  }

  uint8_t destId = data[kMacDestAddrOffset];
  uint8_t sourceId = data[kMacSrcAddrOffset];
  if (destId != 0xFF) {
    return;
  }

  auto rangeHeader = reinterpret_cast<const RangeHeader*>(data + kMacHeaderSize);
  if (rangeHeader->type != kPacketType) {
    return;
  }

  if (kEnableDebug && !kUartBinaryOutput) {
    Serial.printf("TDOA3 rx src=%u seq=%u txTs=%lu rxTs=%llu remote=%u\n",
                  sourceId,
                  rangeHeader->seq,
                  static_cast<unsigned long>(rangeHeader->txTimeStamp),
                  static_cast<unsigned long long>(rxTimestamp),
                  rangeHeader->remoteCount);
  }

  const uint8_t* payload = data + kMacHeaderSize + sizeof(RangeHeader);
  size_t payloadLength = len - kMacHeaderSize - sizeof(RangeHeader);
  const uint8_t* ptr = payload;
  size_t remaining = payloadLength;

  for (uint8_t i = 0; i < rangeHeader->remoteCount; i++) {
    if (remaining < sizeof(RemoteAnchorDataShort)) {
      break;
    }
    auto remoteShort = reinterpret_cast<const RemoteAnchorDataShort*>(ptr);
    bool hasDistance = (remoteShort->seq & kSeqHasDistanceMask) != 0;
    uint8_t seq = static_cast<uint8_t>(remoteShort->seq & ~kSeqHasDistanceMask);

    if (hasDistance && remaining >= sizeof(RemoteAnchorDataFull)) {
      auto remoteFull = reinterpret_cast<const RemoteAnchorDataFull*>(ptr);
      if (kEnableDebug && !kUartBinaryOutput) {
        Serial.printf("  REM id=%u seq=%u rxTs=%lu distTicks=%u\n",
                      remoteFull->id,
                      seq,
                      static_cast<unsigned long>(remoteFull->rxTimeStamp),
                      remoteFull->distance);
      }
      ptr += sizeof(RemoteAnchorDataFull);
      remaining -= sizeof(RemoteAnchorDataFull);
    } else {
      if (kEnableDebug && !kUartBinaryOutput) {
        Serial.printf("  REM id=%u seq=%u rxTs=%lu\n",
                      remoteShort->id,
                      seq,
                      static_cast<unsigned long>(remoteShort->rxTimeStamp));
      }
      ptr += sizeof(RemoteAnchorDataShort);
      remaining -= sizeof(RemoteAnchorDataShort);
    }
  }

  if (!kUartBinaryOutput) {
    printPositionBlock(ptr, remaining);
  }

  if (kUartBinaryOutput) {
    emitBinaryFrame(sourceId, rangeHeader, payload, payloadLength, rxTimestamp);
  }
}

#else  // TAG_MODE_TWR

size_t writeBytesTWR(uint8_t* buffer, size_t offset, const void* data, size_t len) {
  memcpy(buffer + offset, data, len);
  return offset + len;
}

void emitBinaryFrameTWR(uint32_t nowMs) {
  // Binary frame format:
  // [0xAA 0x55] [len u16] [timestamp u32] [anchorCount u8]
  // For each anchor: [anchorId u8] [distance float] [positionValid u8] [x float] [y float] [z float]
  
  uint8_t out[256] = {};
  size_t offset = 0;
  
  const uint8_t start[2] = {0xAA, 0x55};
  offset = writeBytesTWR(out, offset, start, sizeof(start));
  
  uint16_t lenPlaceholder = 0;
  size_t lenOffset = offset;
  offset = writeBytesTWR(out, offset, &lenPlaceholder, sizeof(lenPlaceholder));
  
  offset = writeBytesTWR(out, offset, &nowMs, sizeof(nowMs));
  offset = writeBytesTWR(out, offset, &kAnchorCount, sizeof(kAnchorCount));
  
  for (uint8_t i = 0; i < kAnchorCount; i++) {
    uint8_t anchorId = static_cast<uint8_t>(kFirstAnchorId + i);
    offset = writeBytesTWR(out, offset, &anchorId, sizeof(anchorId));
    
    float distance = gTag.getDistance(i);
    offset = writeBytesTWR(out, offset, &distance, sizeof(distance));
    
    twr_tag::AnchorPosition pos = gTag.getAnchorPosition(i);
    uint8_t posValid = pos.timestamp > 0 ? 1 : 0;
    offset = writeBytesTWR(out, offset, &posValid, sizeof(posValid));
    offset = writeBytesTWR(out, offset, &pos.x, sizeof(pos.x));
    offset = writeBytesTWR(out, offset, &pos.y, sizeof(pos.y));
    offset = writeBytesTWR(out, offset, &pos.z, sizeof(pos.z));
  }
  
  uint16_t payloadLen = static_cast<uint16_t>(offset - sizeof(start) - sizeof(lenPlaceholder));
  memcpy(out + lenOffset, &payloadLen, sizeof(payloadLen));
  
  Serial.write(out, offset);
}

void handlePacketTWR(const uint8_t* data, uint16_t len, uint64_t rxTimestamp) {
  if (len < sizeof(twr_tag::MAC80215Header) + 2) {
    return;
  }

  const twr_tag::MAC80215Header* macHeader = 
    reinterpret_cast<const twr_tag::MAC80215Header*>(data);
  
  uint8_t srcAddr = macHeader->srcAddr[0];
  uint8_t packetType = data[sizeof(twr_tag::MAC80215Header)];
  
  // Pass payload to tag handler
  const uint8_t* payload = data + sizeof(twr_tag::MAC80215Header);
  size_t payloadLen = len - sizeof(twr_tag::MAC80215Header);
  
  gTag.onReceive(payload, payloadLen, static_cast<uint32_t>(rxTimestamp));
  
  if (kEnableDebug && !kUartBinaryOutput) {
    Serial.printf("TWR rx src=%u len=%u\n", srcAddr, len);

    if (packetType == twr_tag::kPacketTypeReport &&
        srcAddr >= kFirstAnchorId &&
        srcAddr < static_cast<uint8_t>(kFirstAnchorId + kAnchorCount)) {
      uint8_t anchorIndex = static_cast<uint8_t>(srcAddr - kFirstAnchorId);
      float distance = gTag.getDistance(anchorIndex);
      Serial.printf("TWR distance anchor=%u (idx=%u): %.3f m\n",
                    srcAddr,
                    anchorIndex,
                    static_cast<double>(distance));
    }
  }
  
  // Emit binary frame after receiving REPORT packet
  if (kUartBinaryOutput && packetType == twr_tag::kPacketTypeReport) {
    emitBinaryFrameTWR(millis());
  }
}

#endif  // TAG_MODE_TDOA3

}  // namespace

void setup() {
  Serial.begin(115200);
  delay(200);
  gMavUart.begin(kMavUartBaud, SERIAL_8N1, kMavUartRxPin, kMavUartTxPin);

  SPI.begin(4, 5, 6, 7);

  DW1000Ng::initialize(kPinDwCs, kPinDwIrq, kPinDwRst);
  DW1000Ng::applyConfiguration(kRadioConfig);
  DW1000Ng::applyInterruptConfiguration(kInterruptConfig);
  DW1000Ng::disableFrameFiltering();
  DW1000Ng::setAntennaDelay(16384);

  DW1000Ng::setNetworkId(kNetworkId);
  DW1000Ng::setDeviceAddress(kDeviceId);

  byte eui[8] = {0xDE, 0xCA, 0xF0, 0x00, 0x00, 0x00, 0x00, 0xf1};
  DW1000Ng::setEUI(eui);

#if TAG_MODE_TDOA3
  // TDOA3 initialization
  startReceive();
#else
  // TWR initialization
  twr_tag::Config twrConfig;
  twrConfig.tagId = kDeviceId;
  twrConfig.firstAnchorId = kFirstAnchorId;
  twrConfig.maxAnchors = kAnchorCount;
  twrConfig.debug = kEnableDebug;
  gTag.begin(twrConfig);
  
  startReceive();
#endif

  if (kEnableDebug) {
    Serial.printf("Mode: %s\n", 
#if TAG_MODE_TDOA3
      "TDOA3"
#else
      "TWR"
#endif
    );
  }
}

void loop() {
  uint32_t nowMs = millis();

  if (nowMs - gLastHeartbeatMs >= kHeartbeatIntervalMs) {
    sendMavlinkHeartbeat();
    gLastHeartbeatMs = nowMs;
  }
  
#if !TAG_MODE_TDOA3
  gTag.tick(nowMs);

  if (nowMs - gLastDistanceSensorMs >= kDistanceSensorIntervalMs) {
    sendMavlinkDistanceSensor(gNextDistanceAnchorIndex, nowMs);
    gNextDistanceAnchorIndex = static_cast<uint8_t>((gNextDistanceAnchorIndex + 1) %
                                                     kAnchorCount);
    gLastDistanceSensorMs = nowMs;
  }
#endif

  if (DW1000Ng::isReceiveDone()) {
    DW1000Ng::clearReceiveStatus();
    uint16_t len = DW1000Ng::getReceivedDataLength();
    if (len > 0 && len <= kMaxFrameSize) {
      DW1000Ng::getReceivedData(gRxBuffer, len);
      uint64_t rxTs = DW1000Ng::getReceiveTimestamp();
      
#if TAG_MODE_TDOA3
      handlePacket(gRxBuffer, len, rxTs);
#else
      handlePacketTWR(gRxBuffer, len, rxTs);
#endif
      
      gRxCount++;
    }
    startReceive();
  }

  if (DW1000Ng::isReceiveFailed()) {
    DW1000Ng::clearReceiveFailedStatus();
    startReceive();
  }

  if (DW1000Ng::isReceiveTimeout()) {
    DW1000Ng::clearReceiveTimeoutStatus();
    startReceive();
  }

#if !TAG_MODE_TDOA3
  // TWR: Handle transmission
  if (!gTxInFlight && gTag.shouldTransmit(nowMs)) {
    size_t txLen = gTag.buildTxFrame(gRxBuffer, sizeof(gRxBuffer));
    
    if (txLen > 0) {
      DW1000Ng::forceTRxOff();
      DW1000Ng::clearReceiveStatus();
      DW1000Ng::clearReceiveFailedStatus();
      DW1000Ng::clearReceiveTimeoutStatus();
      
      DW1000Ng::setTransmitData(gRxBuffer, static_cast<uint16_t>(txLen));
      DW1000Ng::startTransmit(TransmitMode::IMMEDIATE);
      gTxInFlight = true;
      gTxCount++;
      
      if (kEnableDebug && !kUartBinaryOutput) {
        // Extract destination and packet type from built frame
        uint8_t destAddr = 0xFF;
        uint8_t packetType = 0xFF;
        if (txLen >= sizeof(twr_tag::MAC80215Header) + 1) {
          auto macHeader = reinterpret_cast<const twr_tag::MAC80215Header*>(gRxBuffer);
          destAddr = macHeader->destAddr[0];
          packetType = gRxBuffer[sizeof(twr_tag::MAC80215Header)];
        }
        Serial.printf("TWR tx len=%u dest=0x%02X type=0x%02X\n", 
                      static_cast<unsigned>(txLen), destAddr, packetType);
      }
    }
  }

  if (DW1000Ng::isTransmitDone()) {
    DW1000Ng::clearTransmitStatus();

    if (gTxInFlight) {
      uint64_t txTs = DW1000Ng::getTransmitTimestamp();
      gTag.markTransmit(static_cast<uint32_t>(txTs));
      gTxInFlight = false;
    }

    startReceive();
  }
#endif

  if (kEnableDebug && !kUartBinaryOutput) {
    uint32_t now = millis();
    if (now - gLastDebugMs >= kDebugIntervalMs) {
      Serial.printf("DBG ms=%lu rx=%lu tx=%lu\n",
                    static_cast<unsigned long>(now),
                    static_cast<unsigned long>(gRxCount),
                    static_cast<unsigned long>(gTxCount));
      gLastDebugMs = now;
    }
  }

  delay(1);
}