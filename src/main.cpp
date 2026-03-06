#include <Arduino.h>
#include <SPI.h>

#include <DW1000Ng.hpp>
#include <DW1000NgConstants.hpp>

namespace {
constexpr uint8_t kPinDwCs = 7;   // TODO: update to your wiring
constexpr uint8_t kPinDwIrq = 2;  // TODO: update to your wiring
constexpr uint8_t kPinDwRst = 3;  // TODO: update to your wiring

constexpr uint16_t kNetworkId = 0xBEEF;
constexpr uint16_t kDeviceId = 0x0004;  // tag id

constexpr bool kEnableDebug = false;
constexpr uint32_t kDebugIntervalMs = 1000;
constexpr bool kUartBinaryOutput = true;

constexpr size_t kMaxFrameSize = 127;
constexpr uint8_t kPacketType = 0x30;
constexpr uint8_t kSeqHasDistanceMask = 0x80;
constexpr uint8_t kPositionBlockType = 0xA1;
constexpr size_t kMacHeaderSize = 21;
constexpr size_t kMacDestAddrOffset = 5;
constexpr size_t kMacSrcAddrOffset = 13;

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

uint8_t gRxBuffer[kMaxFrameSize] = {};
uint32_t gLastDebugMs = 0;
uint32_t gRxCount = 0;

void startReceive() {
  DW1000Ng::startReceive(ReceiveMode::IMMEDIATE);
}

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

}  // namespace

void setup() {
  Serial.begin(115200);
  delay(200);

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

  startReceive();
}

void loop() {
  if (DW1000Ng::isReceiveDone()) {
    DW1000Ng::clearReceiveStatus();
    uint16_t len = DW1000Ng::getReceivedDataLength();
    if (len > 0 && len <= kMaxFrameSize) {
      DW1000Ng::getReceivedData(gRxBuffer, len);
      uint64_t rxTs = DW1000Ng::getReceiveTimestamp();
      handlePacket(gRxBuffer, len, rxTs);
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

  if (kEnableDebug) {
    uint32_t now = millis();
    if (now - gLastDebugMs >= kDebugIntervalMs) {
      Serial.printf("DBG ms=%lu rx=%lu\n",
                    static_cast<unsigned long>(now),
                    static_cast<unsigned long>(gRxCount));
      gLastDebugMs = now;
    }
  }

  delay(1);
}