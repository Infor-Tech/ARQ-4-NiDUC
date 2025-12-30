#include "config.h"

FastCRC16 CRC;

volatile uint8_t rxBuffer[sizeof(Frame)];
volatile uint8_t rxIndex = 0;

Frame responseFrame;
volatile bool responseReady = false;
volatile uint8_t txIndex = 0;

void processReceivedFrame(Frame rxFrame);

void setup()
{
  Serial.begin(115200);

  // SPI Slave config
  pinMode(MISO, OUTPUT);
  SPCR |= _BV(SPE);      // Enable SPI
  SPI.attachInterrupt(); // Enable interrupts

  // Initialization of response frame
  memset(&responseFrame, 0, sizeof(Frame));
}

ISR(SPI_STC_vect)
{
  uint8_t receivedByte = SPDR;

  // 1. Receiving Handling
  if (rxIndex < sizeof(Frame))
  {
    rxBuffer[rxIndex++] = receivedByte;
  }

  // 2. Transmission Handling (prepare SPDR for NEXT clock cycle)
  if (responseReady && txIndex < sizeof(Frame))
  {
    SPDR = ((uint8_t *)&responseFrame)[txIndex++];
  }
  else
  {
    // Nothing to send, send dummy byte
    SPDR = 0x00;
  }
}

void loop()
{
  // Detect end of transaction (SS goes HIGH)
  if (digitalRead(PIN_SS) == HIGH)
  {

    // If a full frame (or more bytes) has been received
    if (rxIndex >= sizeof(Frame))
    {
      Frame receivedFrame;
      memcpy(&receivedFrame, (void *)rxBuffer, sizeof(Frame));

      rxIndex = 0;

      // Check if this is a valid data frame (starts with SOF),
      if (receivedFrame.sof == SOF_BYTE)
      {
        processReceivedFrame(receivedFrame);
      }
      else
      {
        responseReady = false;
        SPDR = 0x00;
      }
    }
    else if (rxIndex > 0)
    {
      // Incomplete frame received, reset index
      rxIndex = 0;
    }
  }
}

void processReceivedFrame(Frame rxFrame)
{
  crc_t calcCRC = calculateCRC(rxFrame, CRC);

  memset(&responseFrame, 0, sizeof(Frame));
  responseFrame.sof = SOF_BYTE;
  responseFrame.eof = EOF_BYTE;

  if (calcCRC == rxFrame.crc && rxFrame.eof == EOF_BYTE)
  {
    responseFrame.flags = FLAG_ACK;
    responseFrame.seqNum = rxFrame.seqNum;
  }
  else
  {
    responseFrame.flags = FLAG_NACK;
    responseFrame.seqNum = rxFrame.seqNum;
  }

  responseFrame.len = 0;
  responseFrame.crc = calculateCRC(responseFrame, CRC);

  // 2. Prepare data for transmission

  // A. Copy to log
  Frame logFrame = responseFrame;

  // B. Error Simulation
  injectErrors(responseFrame);

  // C. Set flags for SPI interrupt
  responseReady = true;                  // Interrupt will now take data from responseFrame
  SPDR = ((uint8_t *)&responseFrame)[0]; // Load FIRST byte manually
  txIndex = 1;                           // Interrupt will now load from SECOND byte

  // Log the received and transmitted frames
  printLog("RX", rxFrame);
  printLog("TX", logFrame);
}