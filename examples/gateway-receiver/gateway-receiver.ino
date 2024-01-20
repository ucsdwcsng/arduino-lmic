/*

  Module:  raw-halconfig.ino

  Function:
  Auto-configured raw test example, for Adafruit Feather M0 LoRa

  Copyright notice and License:
  See LICENSE file accompanying this project.

  Author:
    Matthijs Kooijman  2015
    Terry Moore, MCCI Corporation  2018

*/

/*******************************************************************************
   Copyright (c) 2015 Matthijs Kooijman

   Permission is hereby granted, free of charge, to anyone
   obtaining a copy of this document and accompanying files,
   to do whatever they want with them without any restriction,
   including, but not limited to, copying, modification and redistribution.
   NO WARRANTY OF ANY KIND IS PROVIDED.

   This example transmits data on hardcoded channel and receives data
   when not transmitting. Running this sketch on two nodes should allow
   them to communicate.
 *******************************************************************************/

#include <lmic.h>
#include <hal/hal.h>
#include <arduino_lmic_hal_boards.h>

#include <SPI.h>

#include <stdarg.h>
#include <stdio.h>
#include <rp2040_flash_datalogger.h>
// we formerly would check this configuration; but now there is a flag,
// in the LMIC, LMIC.noRXIQinversion;
// if we set that during init, we get the same effect.  If
// DISABLE_INVERT_IQ_ON_RX is defined, it means that LMIC.noRXIQinversion is
// treated as always set.
//
// #if !defined(DISABLE_INVERT_IQ_ON_RX)
// #error This example requires DISABLE_INVERT_IQ_ON_RX to be set. Update \
//        lmic_project_config.h in arduino-lmic/project_config to set it.
// #endif

// How often to send a packet. Note that this sketch bypasses the normal
// LMIC duty cycle limiting, so when you change anything in this sketch
// (payload length, frequency, spreading factor), be sure to check if
// this interval should not also be increased.
// See this spreadsheet for an easy airtime and duty cycle calculator:
// https://docs.google.com/spreadsheets/d/1voGAtQAjC1qBmaVuP1ApNKs1ekgUjavHuVQIXyYSvNc

#define RSSI_RESET_VAL 128
#define SCHEDULE_LEN 10
#define SNR_FACTOR 4
#define RSSI_OFFSET 64
#define FREQ_EXPT 920000000
#define FREQ_CNFG 922000000
#define SF_TYPE SF11
#define CR_TYPE CR_4_8
#define PRINT_TO_SERIAL 0 // 1 prints on serial, else in memory
#define ADAFRUIT_FEATHER 2
// check LMIC_DEBUG_LEVEL

// Pin mapping
#if (ADAFRUIT_FEATHER == 2)  // Pin mapping for Adafruit Feather RP2040 LoRa, etc.
const lmic_pinmap lmic_pins = {
    .nss = 16,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 17,
    .dio = {21, 6, LMIC_UNUSED_PIN},
    .rxtx_rx_active = 0,
    .rssi_cal = 8,
    .spi_freq = 8000000,
};

// Pin mapping Adafruit feather M0
#elif (ADAFRUIT_FEATHER == 1) // Pin mapping for Adafruit Feather M0 LoRa, etc.
const lmic_pinmap lmic_pins = {
    .nss = 8,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 4,
    .dio = {3, 6, LMIC_UNUSED_PIN},
    .rxtx_rx_active = 0,
    .rssi_cal = 8, // LBT cal for the Adafruit Feather M0 LoRa, in dB
    .spi_freq = 8000000,
};
#define VBATPIN A7

#else
// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = D10,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = A0,
    .dio = {2, 3, 4},
};
#endif

// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in arduino-lmoc/project_config/lmic_project_config.h,
// otherwise the linker will complain).
void os_getArtEui(u1_t *buf) {}
void os_getDevEui(u1_t *buf) {}
void os_getDevKey(u1_t *buf) {}

// this gets callled by the library but we choose not to display any info;
// and no action is required.
void onEvent(ev_t ev)
{
}

osjob_t arbiter_job, backhaul_job, timeout_job;
FlashWriter flash_writer;

u1_t experiment_status = 0, experiment_type = 0;
int32_t experiment_time;
byte reg_array[200];

// Enable rx mode and call func when a packet is received
void rx(osjobcb_t func)
{
  LMIC.osjob.func = func;
  LMIC.rxtime = os_getTime(); // RX _now_
  // Enable "continuous" RX (e.g. without a timeout, still stops after
  // receiving a packet)

  os_radio(RADIO_RXON);

  //  Serial.print("RxStart: ");
  //  Serial.print(os_getTime());
  //  Serial.print("\n");
}

static void backhaul_data(osjob_t *job)
{
  // char buffer[10];
  // sprintf(buffer, "%d", millis()/1000);  
  // Serial.print(buffer);
  // Serial.print(", ");
  // Asynchronous backhaul job
  for (u2_t ind = 0; ind < LMIC.dataLen; ind++)
  {
    if (LMIC.frame[ind] > 15)
      Serial.print(LMIC.frame[ind], HEX);
    else
    {
      Serial.print("0");
      Serial.print(LMIC.frame[ind], HEX);
    }
  }
  Serial.print(", ");
  Serial.print(LMIC.rssi);
  Serial.print(", ");
  Serial.print(LMIC.snr);
  Serial.print(", ");
  Serial.print(LMIC.sysname_crc_err);
  Serial.print("\n");
}

static void backhaul_data_flash(osjob_t *job)
{
  // Asynchronous backhaul job
  for (u2_t ind = 0; ind < LMIC.dataLen; ind++)
  {
    flash_writer.printf("%02X", LMIC.frame[ind]);
  }
  flash_writer.print(", ");
  flash_writer.printf("%d", LMIC.rssi);
  flash_writer.print(", ");
  flash_writer.printf("%d", LMIC.snr);
  flash_writer.print(", ");
  flash_writer.printf("%d", LMIC.sysname_crc_err);
  flash_writer.print(", ");
  flash_writer.printf("%d", millis()/1000);
  flash_writer.print("\n");
}

static void rxdone_func(osjob_t *job)
{
  // Arbiter
  os_setCallback(job, rx_func);
  // Backhaul
  if (PRINT_TO_SERIAL == 1){
    os_setTimedCallback(&backhaul_job, os_getTime() + 100, backhaul_data);
  }
  else {
    os_setTimedCallback(&backhaul_job, os_getTime() + 100, backhaul_data_flash);
  }
}

static void rx_func(osjob_t *job)
{
  rx(rxdone_func);
}

void wait_for_input_and_print()
{
  unsigned long startTime;
  const unsigned long timeout = 10000; // 10 seconds in milliseconds
  bool inputReceived = false;
  unsigned long cur_time;
  startTime = millis(); // Record the start time
  while (true)
  {
    digitalWrite(LED_BUILTIN, LOW); // turn OFF
    delay(100);
    digitalWrite(LED_BUILTIN, HIGH); // turn ON
    delay(100);
    cur_time = millis() - startTime;
    if (Serial.available() > 0 && cur_time < timeout)
    {
      String input = Serial.readStringUntil('\n');
      input.trim(); // Remove any trailing newline or carriage return characters
      inputReceived = true;
      Serial.println("<Flash Read Begin>");
      FlashReader flash_reader = FlashReader();
      while (flash_reader.current_sector <= flash_reader.max_sector)
      {
        Serial.print(flash_reader.read());
      }
      Serial.println("<Flash Read End>");
      break;
    }
    else if (cur_time >= timeout && !inputReceived)
    {
      Serial.println("No input received within 10 seconds. Moving on...");
      inputReceived = false;
      break;
    }
  }
}

// application entry point
void setup()
{
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  // blink to show reset
  for (int i = 0; i < 10; i++)
  {
    digitalWrite(LED_BUILTIN, LOW); // turn OFF
    delay(500);
    digitalWrite(LED_BUILTIN, HIGH); // turn ON
    delay(500);
  }

  // initialize runtime env
  os_init();

  // disable RX IQ inversion
  LMIC.noRXIQinversion = true;
  LMIC.freq = FREQ_EXPT; // WCSNG
  // MAKERPS(SF8 , BW500, CR_TYPE, 0, 0)
  // MAKERPS(SF7 , BW500, CR_TYPE, 0, 0)
  LMIC.rps = MAKERPS(SF_TYPE, BW125, CR_TYPE, 0, 0); // WCSNG
  LMIC.sysname_tx_rps = MAKERPS(SF_TYPE, BW125, CR_TYPE, 0, 0);
  LMIC.txpow = 21;
  LMIC.radio_txpow = 21; // WCSNG

#if (ADAFRUIT_FEATHER == 1) // Pin mapping for Adafruit Feather M0 LoRa, etc.
  float measuredvbat = analogRead(VBATPIN);
  measuredvbat *= 2;    // we divided by 2, so multiply back
  measuredvbat *= 3.3;  // Multiply by 3.3V, our reference voltage
  measuredvbat /= 1024; // convert to voltage
  Serial.print("VBat: ");
  Serial.println(measuredvbat);
  float vbatPercent = (measuredvbat - 3.2) * 100; // battery ranges from 3.2v to 4.2v
  Serial.print("VBat Percentage: ");
  Serial.println(vbatPercent);
#endif

  wait_for_input_and_print();
  Serial.println("Gateway rx initialized");
  Serial.flush();

#if (PRINT_TO_SERIAL == 1)
  Serial.println("Printing to serial monitor!");
#else
  Serial.println("Printing to memory!");
#endif

  // Set up flash writer
  flash_writer = FlashWriter();

  // setup initial job
  os_setCallback(&arbiter_job, rx_func);
}

void loop()
{
  // execute scheduled jobs and events
  os_runloop_once();
}
