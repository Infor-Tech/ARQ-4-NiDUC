#include "config.h"
#include "payload.h"

FastCRC16 CRC;

uint32_t fileOffset = 0;
uint8_t currentSeqNum = 1;
bool transmissionFinished = false;

// Stats
uint32_t sentFrames = 0;
uint32_t retransmissions = 0;

/**
 * @brief Sents a frame via SPI with error injection
 * @param txFrame Prepared frame to send
 */
void sendFrameSPI(Frame &txFrame)
{
  // 1. Zrób kopię do wysłania (żeby nie psuć oryginału w pamięci logiki)
  Frame frameToSend = txFrame;

  // 2. Symulacja błędów (przekłamanie w "kanale" w stronę odbiornika)
  injectErrors(frameToSend);

  // 3. Transmisja
  digitalWrite(PIN_SS, LOW);

  uint8_t *ptr = (uint8_t *)&frameToSend;
  for (size_t i = 0; i < sizeof(Frame); i++)
  {
    SPI.transfer(ptr[i]);
  }

  digitalWrite(PIN_SS, HIGH);
}

/**
 * @brief Receives an ACK/NACK frame via SPI
 * @param rxFrame Reference to store the received frame
 * @return true if a frame with correct SOF/EOF was received, false otherwise
 */
bool receiveAckSPI(Frame &rxFrame)
{
  memset(&rxFrame, 0, sizeof(Frame));

  digitalWrite(PIN_SS, LOW);

  // delay before starting reception
  delayMicroseconds(50);

  uint8_t *ptr = (uint8_t *)&rxFrame;

  for (size_t i = 0; i < sizeof(Frame); i++)
  {
    // Send dummy byte to generate clock and receive data
    ptr[i] = SPI.transfer(0x00);

    // Delay between bytes for Slave to prepare data
    delayMicroseconds(20);
  }

  digitalWrite(PIN_SS, HIGH);

  if (rxFrame.sof == SOF_BYTE && rxFrame.eof == EOF_BYTE)
  {
    return true;
  }
  return false;
}

void setup()
{
  Serial.begin(115200);
  SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV16);
  pinMode(PIN_SS, OUTPUT);
  digitalWrite(PIN_SS, HIGH);

  randomSeed(analogRead(0));
  delay(1000);
}

void loop()
{
  if (transmissionFinished)
    return;

  // 1. Prepate frame to send
  Frame txFrame;
  memset(&txFrame, 0, sizeof(Frame)); // empty frame
  txFrame.sof = SOF_BYTE;
  txFrame.eof = EOF_BYTE;
  txFrame.seqNum = currentSeqNum;

  if (fileOffset >= PAYLOAD_DATA_LEN)
  {
    txFrame.flags = FLAG_EOT;
    txFrame.len = 0;
  }
  else
  {
    txFrame.flags = FLAG_DATA;
    size_t bytesToSend = min((size_t)MAX_PAYLOAD_LEN, (size_t)(PAYLOAD_DATA_LEN - fileOffset));
    txFrame.len = (uint8_t)bytesToSend;
    // Get payload from PROGMEM
    memcpy_P(txFrame.payload, PAYLOAD_DATA + fileOffset, bytesToSend);
  }

  txFrame.crc = calculateCRC(txFrame, CRC);

  // log transmission attempt
  printLog("TX", txFrame);

  // 2. generate errors and send frame
  sendFrameSPI(txFrame);
  sentFrames++;

  // 3. wait for slave to proccess frame
  delay(REQ_DELAY);

  // 4. Receive response
  Frame ackFrame;
  bool received = receiveAckSPI(ackFrame);

  bool success = false;
  if (received)
  {
    crc_t calcCRC = calculateCRC(ackFrame, CRC);

    printLog("RX", ackFrame); // Log received frame

    if (calcCRC == ackFrame.crc)
    {
      if (ackFrame.flags & FLAG_ACK)
      {
        if (ackFrame.seqNum == currentSeqNum)
        {
          success = true;
          if (txFrame.flags & FLAG_EOT)
          {
            transmissionFinished = true;
          }
          else
          {
            fileOffset += txFrame.len;
            currentSeqNum++;
          }
        }
        else
        {
          // Serial.println(F("BŁĄD: Zły numer sekwencji w ACK"));
        }
      }
      else if (ackFrame.flags & FLAG_NACK)
      {
        // Serial.println(F("INFO: Otrzymano NACK -> Retransmisja"));
      }
    }
    else
    {
      // Serial.println(F("BŁĄD: CRC odpowiedzi niepoprawne -> Traktuj jako utratę -> Retransmisja"));
    }
  }
  else
  {
    // Serial.println(F("BŁĄD: Brak poprawnej struktury ramki od Slave (Timeout/Noise)"));
  }

  if (!success && !transmissionFinished)
  {
    retransmissions++;
    delay(100);
  }

  delay(50); // delay between cycles
}