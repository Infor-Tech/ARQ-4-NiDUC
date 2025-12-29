#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>
#include <FastCRC.h>
#include <SPI.h>

#define USE_CRC_16

// SPI pins
#define PIN_SS 10

// Simulation parameters
#define BER_PROBABILITY 0.0001f
#define REQ_DELAY 100

#define MAX_PAYLOAD_LEN 64
#define SOF_BYTE 0xAA
#define EOF_BYTE 0x55

#define FLAG_DATA 0x01
#define FLAG_ACK 0x02
#define FLAG_NACK 0x04
#define FLAG_EOT 0x08

#if defined(USE_CRC_32)
typedef uint32_t crc_t;
#elif defined(USE_CRC_16)
typedef uint16_t crc_t;
#elif defined(USE_CRC_8)
typedef uint8_t crc_t;
#endif

struct __attribute__((packed)) Frame
{
  uint8_t sof; // Start of Frame
  uint8_t len; // payload length
  uint8_t flags;
  uint8_t seqNum;
  uint8_t payload[MAX_PAYLOAD_LEN];
  crc_t crc;   // Suma kontrolna
  uint8_t eof; // End of Frame
};

/**
 * @brief Calculate CRC for a given frame
 * @param frame frame to calculate CRC for
 * @param crcEngine crc object
 * @return crc_t checksum value
 */
crc_t calculateCRC(Frame &frame, FastCRC16 &crcEngine)
{
  // Obliczamy CRC od pola len do końca payloadu (pomijamy SOF i samo CRC)
  size_t dataSize = sizeof(uint8_t) * 2 + sizeof(uint8_t) + frame.len;
  // len(1) + flags(1) + seq(2) + payload(len)

  uint8_t *startPtr = (uint8_t *)&frame.len;

  return crcEngine.ccitt(startPtr, dataSize);
}

// Funkcja symulująca błędy (BSC Model)
void injectErrors(Frame &frame)
{
  uint8_t *rawBytes = (uint8_t *)&frame;
  size_t frameSize = sizeof(Frame);
  bool corrupted = false;

  for (size_t i = 0; i < frameSize; i++)
  {
    for (int bit = 0; bit < 8; bit++)
    {
      if ((random(0, 10000) / 10000.0) < BER_PROBABILITY)
      {
        bitWrite(rawBytes[i], bit, !bitRead(rawBytes[i], bit));
        corrupted = true;
      }
    }
  }
}

// Funkcja logowania zgodna z wymaganiami
void printLog(const char *dir, Frame &f)
{
  Serial.print(dir);
  Serial.print("; 0x");
  Serial.print(f.flags, HEX);
  Serial.print("; ");
  Serial.print(f.seqNum);
  Serial.print("; ");

  // Wypisz tylko faktyczny payload (bez śmieci z bufora)
  for (int i = 0; i < f.len; i++)
  {
    char c = (char)f.payload[i];
    if (c >= 32 && c <= 126)
      Serial.print(c);
    else
      Serial.print(".");
  }
  Serial.println();
}

#endif