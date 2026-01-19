#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>
#include <FastCRC.h>
#include <SPI.h>

#define USE_CRC_8

#define REMOVE_ESCAPING
#define HIGHLIGHT_ESCAPED_CHARS

// SPI pins
#define PIN_SS 10

// Simulation parameters
#define BER_PROBABILITY 0
#define REQ_DELAY 100

#define MAX_PAYLOAD_LEN 64
#define SOF_BYTE 0xAA
#define EOF_BYTE 0x55
#define ESC_BYTE 0x7D
#define XOR_FLAG 0x20

#define TX_ADDR 0x01
#define RX_ADDR 0x02

#define FLAG_DATA 0x01
#define FLAG_ACK 0x02
#define FLAG_NACK 0x04
#define FLAG_EOT 0x08

#if defined(USE_CRC_32)
typedef uint32_t crc_t;
typedef FastCRC32 crcEngine_t;
#elif defined(USE_CRC_16)
typedef uint16_t crc_t;
typedef FastCRC16 crcEngine_t;
#elif defined(USE_CRC_8)
typedef uint8_t crc_t;
typedef FastCRC8 crcEngine_t;
#endif

struct __attribute__((packed)) Frame
{
  uint8_t sof; // Start of Frame
  uint8_t len; // payload length
  uint8_t srcAddr;
  uint8_t destAddr;
  uint8_t flags;
  uint8_t seqNum; // Depending on a flag this field serves different purpose
  uint8_t payload[MAX_PAYLOAD_LEN];
  crc_t crc;
  uint8_t eof; // End of Frame
};

/**
 * @brief Calculate CRC for a given frame
 * @param frame frame to calculate CRC for
 * @param crcEngine crc object
 * @return crc_t checksum value
 */
crc_t calculateCRC(Frame &frame, crcEngine_t &crcEngine)
{
  // Checksum is calculated from len to the end of payload (based on length).
  size_t dataSize = sizeof(frame.len) + sizeof(frame.srcAddr) + sizeof(frame.destAddr) + sizeof(frame.flags) + sizeof(frame.seqNum) + frame.len;

  uint8_t *startPtr = (uint8_t *)&frame.len;

#if defined(USE_CRC_32)
  return crcEngine.crc32(startPtr, dataSize);
#elif defined(USE_CRC_16)
  return crcEngine.ccitt(startPtr, dataSize);
#elif defined(USE_CRC_8)
  return crcEngine.smbus(startPtr, dataSize);
#endif
}

/**
 * @brief Inject random bit errors into the frame (BSC Model)
 * @param frame Frame to inject errors into
 */
void injectErrors(Frame &frame)
{
  uint8_t *rawBytes = (uint8_t *)&frame;
  size_t frameSize = sizeof(Frame);

  for (size_t i = 0; i < frameSize; i++)
  {
    for (int bit = 0; bit < 8; bit++)
    {
      if ((random(0, 10000) / 10000.0) < BER_PROBABILITY)
      {
        bitWrite(rawBytes[i], bit, !bitRead(rawBytes[i], bit));
      }
    }
  }
}

/**
 * @brief Print frame log to Serial
 * @param dir Direction string
 * @param f Frame to log
 */
void printLog(const char *dir, Frame &f)
{
  Serial.print(dir);
  Serial.print("; 0x");
  Serial.print(f.flags, HEX);
  Serial.print("; ");
  Serial.print(f.seqNum);
  Serial.print("; ");

  for (int i = 0; i < f.len; i++)
  {
    char c = (char)f.payload[i];
#ifdef REMOVE_ESCAPING
    if (c == ESC_BYTE)
    {
      i++;
      char escapedChar = (char)(f.payload[i] ^ XOR_FLAG);
#ifdef HIGHLIGHT_ESCAPED_CHARS
      Serial.print("[");
      Serial.print((uint8_t)escapedChar, HEX);
      Serial.print("]");
#else
      Serial.print((uint8_t)escapedChar, HEX);
#endif
      continue;
    }
#endif
    if (c >= 32 && c <= 126)
      Serial.print(c);
    else
      Serial.print(".");
  }
  Serial.println();
}

#endif