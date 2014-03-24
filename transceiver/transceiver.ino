// transmitter.pde
//
// Simple example of how to use VirtualWire to transmit messages
// Implements a simplex (one-way) transmitter with an TX-C1 module
//
// See VirtualWire.h for detailed API docs
// Author: Mike McCauley (mikem@airspayce.com)
// Copyright (C) 2008 Mike McCauley
// $Id: transmitter.pde,v 1.3 2009/03/30 00:07:24 mikem Exp $

#include <VirtualWire.h>

const int led_pin = 13;

const int receive_en_pin = 2;
const int receive_pin = 3;

const int transmit_en_pin = 4;
const int transmit_pin = 5;

const int RSSI_PIN = 1;

boolean cmdReceived = false;

char msg[30];


void setup()
{
  Serial.begin(115200);
  
  pinMode(led_pin, OUTPUT);
  pinMode(receive_en_pin, OUTPUT);
  
  // Initialise the IO and ISR
  vw_set_tx_pin(transmit_pin);
  vw_set_rx_pin(receive_pin);
  vw_set_ptt_pin(transmit_en_pin);
  vw_set_ptt_inverted(true); // Required for DR3100
  vw_setup(2000);       // Bits per sec
  
  vw_rx_start();       // Start the receiver PLL running
}

byte count = 0;

void loop()
{
  uint16_t tmpRSSI;
  char buf[VW_MAX_MESSAGE_LEN];
  uint8_t buflen = VW_MAX_MESSAGE_LEN;

  if (vw_have_message())
  {
    if (vw_get_message((uint8_t*)buf, &buflen, &tmpRSSI)) // Non-blocking
    { 
      buf[buflen] = '\0';

      Serial.print(buf);
      if (strstr(buf, "RSSI"))
      {
        Serial.print("Local RSSI: ");
        Serial.println(tmpRSSI);
      }
    }
  }
  
  if (Serial.available())
  {
    msg[count] = Serial.read();
    count++;
    if (msg[count-1] == '\n')
    {
      msg[count-1] = '/0';
      cmdReceived = true;
    }
    else if (count >= 30)
    {
      count = 0;
    }
  }

  if (cmdReceived)
  {
    digitalWrite(led_pin, HIGH); // Flash a light to show transmitting
    digitalWrite(receive_en_pin, HIGH);
    vw_send((uint8_t *)msg, count);
    vw_wait_tx(); // Wait until the whole message is gone
    digitalWrite(led_pin, LOW);
    digitalWrite(receive_en_pin, LOW);
    cmdReceived = false;
    count = 0;
  }
}
