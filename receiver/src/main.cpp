#include "config.h"

FastCRC16 CRC;

// Bufory i indeksy
volatile uint8_t rxBuffer[sizeof(Frame)];
volatile uint8_t rxIndex = 0;

// Zmienne do wysyłania odpowiedzi (TX)
Frame responseFrame;                 // Tu przechowujemy przygotowaną odpowiedź
volatile bool responseReady = false; // Czy mamy coś do wysłania?
volatile uint8_t txIndex = 0;        // Który bajt teraz wysyłamy?

void setup()
{
  Serial.begin(115200);

  // Konfiguracja SPI Slave
  pinMode(MISO, OUTPUT);
  SPCR |= _BV(SPE);      // Włącz SPI
  SPI.attachInterrupt(); // Włącz przerwania

  // Inicjalizacja
  memset(&responseFrame, 0, sizeof(Frame));
}

ISR(SPI_STC_vect)
{
  uint8_t receivedByte = SPDR; // Odczyt kasuje flagę przerwania

  // 1. Obsługa Odbioru (zawsze zapisujemy to, co Master wysyła)
  if (rxIndex < sizeof(Frame))
  {
    rxBuffer[rxIndex++] = receivedByte;
  }

  // 2. Obsługa Nadawania (przygotowanie SPDR na NASTĘPNY cykl zegara)
  if (responseReady && txIndex < sizeof(Frame))
  {
    // Mamy gotową odpowiedź, ładujemy kolejny bajt
    SPDR = ((uint8_t *)&responseFrame)[txIndex++];
  }
  else
  {
    // Nie mamy nic do wysłania lub skończyliśmy -> wyślij 0 (padding)
    SPDR = 0x00;
  }
}

void loop()
{
  // Wykrywanie końca transakcji (SS przechodzi w stan HIGH)
  if (digitalRead(PIN_SS) == HIGH)
  {

    // Jeśli odebrano pełną ramkę (lub więcej bajtów)
    if (rxIndex >= sizeof(Frame))
    {
      Frame receivedFrame;
      // Kopiujemy z bufora volatile do lokalnej zmiennej (atomowo by było idealnie, ale tu wystarczy)
      memcpy(&receivedFrame, (void *)rxBuffer, sizeof(Frame));

      // Zerujemy indeks odbioru na przyszłość
      rxIndex = 0;

      // Sprawdzamy, czy to faktyczna ramka danych (zaczyna się od SOF),
      // czy Master tylko "pompował" puste bajty odbierając ACK.
      if (receivedFrame.sof == SOF_BYTE)
      {
        processReceivedFrame(receivedFrame);
      }
      else
      {
        // To były puste bajty od Mastera (reading phase).
        // Ważne: Po zakończeniu czytania przez Mastera, musimy przestać wysyłać ACK
        // żeby nie zakłócić kolejnej ramki danych.
        responseReady = false;
        SPDR = 0x00;
      }
    }
    else if (rxIndex > 0)
    {
      // Odebrano śmieci lub niepełną ramkę -> resetuj
      rxIndex = 0;
    }
  }
}

void processReceivedFrame(Frame rxFrame)
{
  // 1. Obliczamy matematykę (CRC)
  crc_t calcCRC = calculateCRC(rxFrame, CRC);

  // Czyścimy ramkę odpowiedzi
  memset(&responseFrame, 0, sizeof(Frame));
  responseFrame.sof = SOF_BYTE;
  responseFrame.eof = EOF_BYTE;

  // Decyzja ACK/NACK
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

  // 2. Przygotowanie danych do wysyłki (Critical Section)

  // A. Kopia do logów
  Frame logFrame = responseFrame;

  // B. Psujemy oryginał (Symulacja błędów)
  injectErrors(responseFrame);

  // C. Ustawiamy flagi dla przerwania SPI
  responseReady = true;                  // Przerwanie ma teraz brać dane z responseFrame
  SPDR = ((uint8_t *)&responseFrame)[0]; // Ładujemy PIERWSZY bajt ręcznie
  txIndex = 1;                           // Przerwanie ma ładować od DRUGIEGO bajtu (indeks 1)

  // 3. Logowanie na Serial (teraz bezpieczne, bo SPI działa na przerwaniach w tle)
  printLog("RX", rxFrame);
  printLog("TX", logFrame);
}