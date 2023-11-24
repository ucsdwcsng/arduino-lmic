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

// Pin mapping Adafruit feather RP2040
#if (defined(ADAFRUIT_FEATHER_RP2040) && (ADAFRUIT_FEATHER_RP2040 == 1))  // Pin mapping for Adafruit Feather M0 LoRa, etc.
const lmic_pinmap lmic_pins = {
  .nss = 16,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 17,
  .dio = { 21, 6, LMIC_UNUSED_PIN },
  .rxtx_rx_active = 0,
  .rssi_cal = 8,
  .spi_freq = 8000000,
};

// Pin mapping Adafruit feather M0
#elif (defined(ADAFRUIT_FEATHER_M0) && (ADAFRUIT_FEATHER_M0 == 1))  // Pin mapping for Adafruit Feather M0 LoRa, etc.
const lmic_pinmap lmic_pins = {
  .nss = 8,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 4,
  .dio = { 3, 6, LMIC_UNUSED_PIN },
  .rxtx_rx_active = 0,
  .rssi_cal = 8,  // LBT cal for the Adafruit Feather M0 LoRa, in dB
  .spi_freq = 8000000,
};
#define VBATPIN A7

#else
// Pin mapping
const lmic_pinmap lmic_pins = {
  .nss = D10,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = A0,
  .dio = { 2, 3, 4 },
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
u1_t experiment_status = 0, experiment_type = 0;
int32_t experiment_time;
byte reg_array[64];

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
  Serial.print(LMIC.rssi-RSSI_OFFSET);
  Serial.print(", ");
  Serial.print(LMIC.snr/SNR_FACTOR);
  Serial.print(", ");
  Serial.print(LMIC.sysname_crc_err);
  Serial.print("\n");
}

static void experiment_rxdone_func(osjob_t *job)
{
  // Arbiter
  os_setCallback(job, rx_func);
  // Backhaul
  os_setTimedCallback(&backhaul_job, os_getTime() + 100, backhaul_data);
}

static void control_rxdone_func(osjob_t *job)
{
  // updated register values
  if ((LMIC.frame[1] == 10) && (LMIC.frame[2] == 0)) {
    // set variables for experiment
    experiment_status = 1;
    LMIC.freq = FREQ_EXPT;
    LMIC.rps = MAKERPS(reg_array[17] >> 4, reg_array[18] >> 4, reg_array[19] >> 4, 0, 0);  // WCSNG (Reverse of Node)

    //set timeout callback to stop sending free beacons
    experiment_time = reg_array[2] * reg_array[3];
    experiment_type = reg_array[4] * reg_array[49];
    // Serial.print("Starting experiment, type:");
    // Serial.print(experiment_type);  
    // Serial.print(", time: ");
    // Serial.print(experiment_time);
    // Serial.println("s + 10s");
    os_setTimedCallback(&timeout_job, os_getTime() + ms2osticks((experiment_time + 10) * 1000), experiment_timeout_func);
  } 
  else if ((LMIC.frame[1] == 2) && (LMIC.frame[2] == 2 || LMIC.frame[2] == 3 || LMIC.frame[2] == 17 || LMIC.frame[2] == 18 || LMIC.frame[2] == 19 ||  LMIC.frame[2] == 4  ||  LMIC.frame[2] == 49 )) {
    reg_array[LMIC.frame[2]] = LMIC.frame[3];
    // Serial.print("Writing Reg: ");
    // Serial.print(LMIC.frame[2], HEX);
    // Serial.print(", Val: ");
    // Serial.print(LMIC.frame[3], HEX);
    // Serial.print("\n");
  } 

  // Arbiter
  os_setCallback(job, rx_func);
}

static void rx_func(osjob_t *job)
{
  // GET BUF_OUT
  if (experiment_status == 1) {
    rx(experiment_rxdone_func);
  }
  else {
    rx(control_rxdone_func);
  }
}

static void experiment_timeout_func(osjob_t *job) {
  // reset variables after experiment
  experiment_status = 0;
  LMIC.freq = FREQ_CNFG;
  LMIC.rps = MAKERPS(SF8, BW125, CR_4_8, 0, 0); // WCSNG

  // Serial.println("Experiment timedout ...");
  radio_init();
  os_radio(RADIO_RST);
  os_setCallback(&timeout_job, rx_func);
}

// application entry point
void setup()
{
  Serial.begin(2000000);
  pinMode(LED_BUILTIN, OUTPUT);
  // blink to show reset
  for (int i = 0; i < 5; i++) {
    digitalWrite(LED_BUILTIN, LOW);  // turn OFF
    delay(200);
    digitalWrite(LED_BUILTIN, HIGH);  // turn ON
    delay(200);
  }

  // initialize runtime env
  os_init();

  // disable RX IQ inversion
  LMIC.noRXIQinversion = true;
  LMIC.freq = FREQ_CNFG; // WCSNG
  // MAKERPS(SF8 , BW500, CR_4_8, 0, 0)
  // MAKERPS(SF7 , BW500, CR_4_5, 0, 0)
  LMIC.rps = MAKERPS(SF8, BW125, CR_4_8, 0, 0); // WCSNG
  LMIC.sysname_tx_rps = MAKERPS(SF8, BW125, CR_4_8, 0, 0);
  LMIC.txpow = 21;
  LMIC.radio_txpow = 21; // WCSNG
  
  // setting default values in registers
  reg_array[2] = 1;    // Experiment run length in seconds
  reg_array[3] = 1;    // Time multiplier for expt time
  reg_array[17] = 34;  // 17: {4bits txsf, 4bits rxsf}
  reg_array[18] = 34;  // 18: {4bits txbw, 4bits rxbw}
  reg_array[19] = 51;  // 19: {4bits txcr, 4bits txcr}


  Serial.flush();
  Serial.println("Hi, this is gateway rx");

#if (defined(ADAFRUIT_FEATHER_M0) && (ADAFRUIT_FEATHER_M0 == 1))  // Pin mapping for Adafruit Feather M0 LoRa, etc.
  float measuredvbat = analogRead(VBATPIN);
  measuredvbat *= 2;     // we divided by 2, so multiply back
  measuredvbat *= 3.3;   // Multiply by 3.3V, our reference voltage
  measuredvbat /= 1024;  // convert to voltage
  Serial.print("VBat: ");
  Serial.println(measuredvbat);
  float vbatPercent = (measuredvbat - 3.2) * 100;  // battery ranges from 3.2v to 4.2v
  Serial.print("VBat Percentage: ");
  Serial.println(vbatPercent);
#endif

  // setup initial job
  os_setCallback(&arbiter_job, rx_func);
}

void loop()
{
  // execute scheduled jobs and events
  os_runloop_once();
}
