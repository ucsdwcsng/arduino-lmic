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
// debug level: LMIC_DEBUG_LEVEL

#define NODE_IDX 127

#define RSSI_RESET_VAL 128
#define SCHEDULE_LEN 100
#define RB_LEN 65

#define FREQ_CAD_INDEX 18
#define FREQ_EXPT_INDEX 20
#define FREQ_CNFG_INDEX 22
#define ADAFRUIT_FEATHER_TYPE 2

#define PRINT_ONLY_PACKET 1
#define TRANSMIT_SAME_PACKET 1
#define BACKHAUL_PRINT_DELAY 100

// Pin mapping
#if (ADAFRUIT_FEATHER_TYPE == 2)  // Pin mapping for Adafruit Feather RP2040.
const lmic_pinmap lmic_pins = {
  .nss = 16,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 17,
  .dio = { 21, 6, LMIC_UNUSED_PIN },
  .rxtx_rx_active = 0,
  .rssi_cal = 8,              // LBT cal for the Adafruit Feather M0 LoRa, in dB
  .spi_freq = 8000000,
};

#elif (ADAFRUIT_FEATHER_TYPE == 1)  // Pin mapping for Adafruit Feather M0 LoRa, etc.
const lmic_pinmap lmic_pins = {
  .nss = 8,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 4,
  .dio = { 3, 6, LMIC_UNUSED_PIN },
  .rxtx_rx_active = 0,
  .rssi_cal = 8,              // LBT cal for the Adafruit Feather M0 LoRa, in dB
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

osjob_t arbiter_job, backhaul_job, loop_job, timeoutjob;
static void tx_func(osjob_t *job);

// WCSNG

// Serial Buffer(s)
byte buf_in[4], buf_out[4], buf_tx[16], buf_print[4];
// 0: tx_interval_global (ms)
// 1: packet_size_bytes
// 2: Experiment Time in seconds
// 3: Experiment Time Multiplier
// 4: Enable CAD
// 5: DIFS as number of CADs
// 6: Backoff CFG1 - Backoff Unit Length in ms
// 7: Backoff CFG2 - Max backoff multiplier
// 8: tx_interval_multiplier
// 9: scheduler_interval_mode (0: Periodic, 1: Poisson, 2: Periodic with Variance);
//---------------------------------

// 10: Result - Counter Byte 0
// 11: Result - Counter Byte 1
// 12: Result - Counter Byte 2
// 13: Result - Backoff Counter Byte 0
// 14: Result - Backoff Counter Byte 1
// 15: Result - Backoff Counter Byte 2
//---------------------------------

// 17: {4bits txsf, 4bits rxsf}
// 18: {4bits txbw, 4bits rxbw}
// 19: {4bits txcr, 4bits txcr}
// 20: CAD Config Register {bit 0: Fixed DIFS Size, bit 1: LMAC CSMA}
// 21: Listen before talk ticks (x16)
// 22: Listen before talk max RSSI s1_t
// 23: Kill CAD Wait time (0 or 1)
//---------------------------------

// 24: Transmit power range - 4 to 21 dBm
// 25: Change experiment frequency
// 26: Change CAD frequency
// 27: max start delay - lower byte (range 0 to 255)
// 28: max start delay - higher byte (range 0 to 255)
// 29: custom start delay - lower byte (range 0 to 255)
// 30: custom start delay - higher byte (range 0 to 255)
// 31: enable random propagation delays

// 24--44 - Node Idx (changed)
// 54--64 - Node Idx (changed)

//101-200 - Node Idx
//---------------------------------

// 45: periodic_tx_variance (x10 ms)

// 46: Result - LBT Counter Byte 0
// 47: Result - LBT Counter Byte 1
// 48: Result - LBT Counter Byte 2

// 52: Result - LBT Counter Byte 0
// 53: Result - LBT Counter Byte 1
// 54: Result - LBT Counter Byte 2

//-------------- FSMA --------------
// 49: Enable FSMA
// 50: Listen before talk min RSSI s1_t
// 51: Enable Exponential backoff
// 55: Enable inband CAD / false positive check (0-None, 1-false positive, 2-inband CAD)
// 57: FSMA beacon symbols

// For Ref
// enum _cr_t { CR_4_8=0, CR_4_6, CR_4_7, CR_4_8 };
// enum _sf_t { FSK=0, SF7, SF8, SF9, SF10, SF11, SF12, SFrfu };
// enum _bw_t { BW125=0, BW250, BW500, BWrfu };

byte reg_array[201];
ostime_t interarrival_array[2048];
u4_t interarrival_ind, expt_start_millis;
ostime_t expt_start_time, expt_stop_time; // 1ms is 62.5 os ticks
int experiment_time;
int arbiter_state;
int experiment_tx_power;
ostime_t scheduler_list_ms[SCHEDULE_LEN];

u1_t freq_expt_ind, freq_cad_ind, freq_cnfg_ind;
u2_t expt_start_delay, max_start_delay, custom_start_delay, propagation_delay;
// float random_decimal;
u4_t trx_freq_vec[24];

u4_t multi_tx_packet_ctr;
u4_t global_cad_counter;
u4_t global_cad_detect_counter;
u4_t global_lbt_counter;
u4_t global_wait_time_counter;
u4_t packet_start_time;
//

// Transmit the given string and call the given function afterwards
void tx(osjobcb_t func)
{
  // set transmit power
  LMIC.txpow = 30;
  LMIC.radio_txpow = 30; // WCSNG

  // the radio is probably in RX mode; stop it.
  os_radio(RADIO_RST);
  // wait a bit so the radio can come out of RX mode
  delay(1);
  // prepare data
  LMIC.dataLen = 4;
  LMIC.frame[0] = buf_out[0];
  LMIC.frame[1] = buf_out[1];
  LMIC.frame[2] = buf_out[2];
  LMIC.frame[3] = buf_out[3];
  // set completion function.
  LMIC.osjob.func = func;
  // start the transmission
  os_radio(RADIO_TX);
}

void tx_multi(osjobcb_t func)
{
  //enable propagation delays
  if (reg_array[31] > 0){
    // generates 3000 to 10000 us (3ms to 7ms)
    propagation_delay = 3000 + floor(7000*(((float)os_getRndU1()) / 255)); 
    delayMicroseconds(propagation_delay);
    // Serial.print("Propagation delay: ");
    // Serial.print(propagation_delay);
    // Serial.print("\n");
  }

  // set transmit power
  LMIC.txpow = experiment_tx_power;
  LMIC.radio_txpow = experiment_tx_power; // WCSNG

  // the radio is probably in RX mode; stop it.
  os_radio(RADIO_RST);
  // wait a bit so the radio can come out of RX mode
  delay(1);
  // prepare data
  LMIC.dataLen = reg_array[1];
  LMIC.frame[0] = NODE_IDX;
  if (TRANSMIT_SAME_PACKET == 1) {
    LMIC.frame[1] = 1;
    LMIC.frame[2] = 0;
    LMIC.frame[3] = 0;

  }
  else {
    LMIC.frame[1] = get_nth_byte(multi_tx_packet_ctr, 0);
    LMIC.frame[2] = get_nth_byte(multi_tx_packet_ctr, 1);
    LMIC.frame[3] = get_nth_byte(multi_tx_packet_ctr, 2);
  }
  // set completion function.
  LMIC.osjob.func = func;
  // start the transmission
  packet_start_time = os_getTime();
  os_radio(RADIO_TX);
}

void tx_gen(osjobcb_t func, u1_t pload_len)
{
  // the radio is probably in RX mode; stop it.
  os_radio(RADIO_RST);
  // wait a bit so the radio can come out of RX mode
  delay(1);
  // prepare data
  LMIC.dataLen = pload_len;
  LMIC.frame[0] = NODE_IDX;
  // set completion function.
  LMIC.osjob.func = func;
  // start the transmission
  os_radio(RADIO_TX);
}

// Enable rx mode and call func when a packet is received
void rx(osjobcb_t func)
{
  LMIC.osjob.func = func;
  LMIC.rxtime = os_getTime(); // RX _now_
  // Enable "continuous" RX (e.g. without a timeout, still stops after
  // receiving a packet)

  os_radio(RADIO_RXON);
}

static void experiment_timeout_func(osjob_t *job) {
  Serial.println("Experiment timeout function triggered");
  radio_init();
  os_radio(RADIO_RST);

  // Immediately trigger  timed_executor
  global_cad_counter = global_cad_counter + LMIC.sysname_cad_counter;
  global_lbt_counter = global_lbt_counter + LMIC.sysname_lbt_counter;
  global_cad_detect_counter = global_cad_detect_counter + LMIC.sysname_cad_detect_counter;
  global_wait_time_counter = global_wait_time_counter + (os_getTime() - packet_start_time);
  os_setCallback(&arbiter_job, timed_executor);
}


static void rxtimeout_func(osjob_t *job)
{
  digitalWrite(LED_BUILTIN, LOW); // off
}

static void rxdone_func(osjob_t *job) {
  // // Blink once to confirm reception and then keep the led on
  // digitalWrite(LED_BUILTIN, LOW); // off
  // delay(10);
  // digitalWrite(LED_BUILTIN, HIGH); // on

  buf_in[0] = LMIC.frame[0];
  buf_in[1] = LMIC.frame[1];
  buf_in[2] = LMIC.frame[2];
  buf_in[3] = LMIC.frame[3];

  buf_print[0] = LMIC.frame[0];
  buf_print[1] = LMIC.frame[1];
  buf_print[2] = LMIC.frame[2];
  buf_print[3] = LMIC.frame[3];

  if (PRINT_ONLY_PACKET != 1 ){
    os_setTimedCallback(&backhaul_job, os_getTime() + BACKHAUL_PRINT_DELAY, rx_backhaul_data);
  }
  // elif
  // Serial.print("[");
  // Serial.print(buf_in[0]);
  // Serial.print(",");
  // Serial.print(buf_in[1]);
  // Serial.print(",");
  // Serial.print(buf_in[2]);
  // Serial.print(",");
  // Serial.print(buf_in[3]);
  // Serial.print("]");
  // Serial.print(";");

  // Serial.print("RSSI: ");
  // Serial.print(LMIC.rssi);
  // Serial.print("; ");

  // Serial.print("CRC: ");
  // Serial.print(LMIC.sysname_crc_err);
  // Serial.print("\n");

  // Arbiter
  os_setCallback(job, arbiter_fn);
}

static void rx_func(osjob_t *job)
{
  // GET BUF_OUT
  rx(rxdone_func);
}

static void txdone_func(osjob_t *job)
{
  // Immediately go to arbiter
  os_setCallback(job, arbiter_fn);
}

static void tx_gen_done(osjob_t *job)
{
  // Wait for 10 ms and call .
  os_setTimedCallback(job, os_getTime() + ms2osticks(50), timestamp_readback);
}

static void txmultidone_func(osjob_t *job)
{
  // Immediately go to timed_executor
  multi_tx_packet_ctr = multi_tx_packet_ctr + 1;
  global_cad_counter = global_cad_counter + LMIC.sysname_cad_counter;
  global_lbt_counter = global_lbt_counter + LMIC.sysname_lbt_counter;
  global_cad_detect_counter = global_cad_detect_counter + LMIC.sysname_cad_detect_counter;
  global_wait_time_counter = global_wait_time_counter + (os_getTime() - packet_start_time);

  interarrival_array[interarrival_ind] = os_getTime();
  interarrival_ind++;

  os_setTimedCallback(&backhaul_job, os_getTime() + BACKHAUL_PRINT_DELAY, tx_backhaul_data);
  os_setCallback(job, timed_executor);
}

static void tx_func(osjob_t *job)
{
  // Send BUF OUT
  tx(txdone_func);
}

static void tx_func_multi(osjob_t *job)
{
  // Send BUF OUT
  tx_multi(txmultidone_func);
}

// print transmit packet info
static void tx_backhaul_data(osjob_t *job) {
  // Asynchronous backhaul job
  for (u2_t ind = 0; ind < LMIC.dataLen; ind++) {
    if (LMIC.frame[ind] > 15)
      Serial.print(LMIC.frame[ind], HEX);
    else {
      Serial.print("0");
      Serial.print(LMIC.frame[ind], HEX);
    }
  }
  Serial.print(",");
  Serial.printf("%d", (millis()-expt_start_millis));
  Serial.print("\n");
}

static void rx_backhaul_data(osjob_t *job) {
  // Asynchronous backhaul job
  // for (u2_t ind = 0; ind < LMIC.dataLen; ind++)
  // {
  //   Serial.print(LMIC.frame[ind]);
  //   Serial.print(",");
  // }
  Serial.print("[");
  Serial.print(buf_print[0]);
  Serial.print(",");
  Serial.print(buf_print[1]);
  Serial.print(",");
  Serial.print(buf_print[2]);
  Serial.print(",");
  Serial.print(buf_print[3]);
  Serial.print("],");
  Serial.print(LMIC.rssi);
  Serial.print(",");
  Serial.print(LMIC.snr);
  Serial.print(",");
  Serial.print(LMIC.sysname_crc_err);
  Serial.print(",");
  Serial.printf("%d", millis() / 1000);
  Serial.print("\n");
}

// This function schedules inter-arrival times
static u4_t get_wait_time_ms()
{
  u4_t interval_mean = (u4_t)reg_array[0] * (u4_t)reg_array[8];
  float cur_rand = 0, cur_interval = 0, cur_var = 0;
  switch (reg_array[9])
  {
  case 0:
    // Periodic
    return interval_mean;
    break;
  case 1:
    // Poisson
    // Typecast and add 0.1 to prevent Inf.
    cur_rand = -log(((float)os_getRndU1() + 0.1) / 255.1);
    cur_interval = (float)interval_mean * cur_rand;
    return (u4_t)floor(cur_interval);
    break;
  case 2:
    // Periodic with Variance
    cur_var = 10 * ((float)os_getRndU1() - 127.5) / 255 * ((float)reg_array[45]);
    cur_var = floor((float)interval_mean + cur_var);
    if (cur_var < 0)
      cur_var = 0;
    return (u4_t)cur_var;
    break;
  default:
    return interval_mean;
  }
}

static void fill_schedule()
{
  scheduler_list_ms[0] = os_getTime();
  for (byte ind = 1; ind < SCHEDULE_LEN; ind++)
  {
    scheduler_list_ms[ind] = scheduler_list_ms[ind-1] + ms2osticks(get_wait_time_ms());
  }
}

static void prepare_multi_tx()
{
  // This function prepares for multi tx transmission

  // Preparing buffer
  for (byte ind = 0; ind < reg_array[1]; ind++)
  {
    LMIC.frame[LMIC.dataLen++] = 0;
  }

  // Reset Packet Counter
  multi_tx_packet_ctr = 0;
  // Reset Backoff Counter
  global_cad_counter = 0;
  global_cad_detect_counter = 0;
  global_lbt_counter = 0;
  global_wait_time_counter= 0;
  // Load CAD State
  LMIC.sysname_enable_cad = reg_array[4];
  // Load DIFS number
  LMIC.sysname_cad_difs = reg_array[5];
  // Load Backoff CNFGs
  LMIC.sysname_backoff_cfg1 = reg_array[6];
  LMIC.sysname_backoff_cfg2 = reg_array[7];
  // Load other cad regs:
  //  reg_array[20];{bit 0: Fixed DIFS Size, bit 1: LMAC CSMA}
  LMIC.sysname_use_fixed_difs = reg_array[20] % 2;
  LMIC.sysname_csma_algo = (reg_array[20] >> 1) % 2;

  // Load the LBT CNFGS
  LMIC.lbt_ticks = ((ostime_t)reg_array[21]) << 4; // Listen before talk ticks (x16)
  LMIC.lbt_dbmax = reg_array[22];                  // Listen before talk max RSSI s1_t
  LMIC.sysname_kill_cad_delay = reg_array[23];     // Kill CAD Wait time (0 or 1)

  // FSMA params
  LMIC.sysname_enable_FSMA = reg_array[49];
  LMIC.sysname_lbt_dbmin = reg_array[50]; // > -188 (-118 and below are decting as false positive for cad)
  LMIC.sysname_enable_exponential_backoff = reg_array[51];

  // Change the center frequency
  freq_expt_ind = reg_array[25];
  freq_cad_ind = reg_array[26];
  LMIC.freq = trx_freq_vec[freq_expt_ind]; // WCSNG
  LMIC.sysname_cad_freq_vec[0] = trx_freq_vec[freq_expt_ind];
  LMIC.sysname_cad_freq_vec[1] = trx_freq_vec[freq_cad_ind];
  
  // Set the RPS
  LMIC.sysname_tx_rps = MAKERPS(reg_array[17] >> 4, reg_array[18] >> 4, reg_array[19] >> 4, 0, 0);  // WCSNG
  LMIC.sysname_cad_rps = MAKERPS(reg_array[17] % 16, reg_array[18] % 16, reg_array[19] % 16, 0, 0); // WCSNG

  LMIC.sysname_enable_inband_cad = reg_array[55];
  LMIC.sysname_FSMA_beacon_symbols = reg_array[57];

  if (experiment_tx_power != (int8_t) reg_array[24]) {
    if (PRINT_ONLY_PACKET != 1 ){
      Serial.print("Transmitting power (in dBm) is changed from ");
      Serial.print(experiment_tx_power);
    }
    experiment_tx_power = (int8_t) reg_array[24];
    if (PRINT_ONLY_PACKET != 1 ){
      Serial.print(", to ");
      Serial.println(experiment_tx_power);
    }
  }

  // Clearing the interrarival index counter
  interarrival_ind = 0;
}

static byte get_nth_byte(int number_in, byte idx)
{
  return (number_in >> (idx * 8));
}

static void store_multitx_results()
{
  // Store results and re-configure node for CNFG operations
  reg_array[10] = get_nth_byte(multi_tx_packet_ctr, 0);
  reg_array[11] = get_nth_byte(multi_tx_packet_ctr, 1);
  reg_array[12] = get_nth_byte(multi_tx_packet_ctr, 2);

  reg_array[13] = get_nth_byte(global_cad_counter, 0);
  reg_array[14] = get_nth_byte(global_cad_counter, 1);
  reg_array[15] = get_nth_byte(global_cad_counter, 2);

  reg_array[46] = get_nth_byte(global_lbt_counter, 0);
  reg_array[47] = get_nth_byte(global_lbt_counter, 1);
  reg_array[48] = get_nth_byte(global_lbt_counter, 2);

  reg_array[52] = get_nth_byte(global_wait_time_counter, 0);
  reg_array[53] = get_nth_byte(global_wait_time_counter, 1);
  reg_array[54] = get_nth_byte(global_wait_time_counter, 2);
  //  reg_array[46] = get_nth_byte(global_cad_detect_counter, 0);
  //  reg_array[47] = get_nth_byte(global_cad_detect_counter, 1);
  //  reg_array[48] = get_nth_byte(global_cad_detect_counter, 2);

  // Resetting Freq and RPS
  LMIC.freq = trx_freq_vec[freq_cnfg_ind];      // WCSNG
  LMIC.rps = MAKERPS(SF8, BW125, CR_4_8, 0, 0); // WCSNG
  LMIC.sysname_tx_rps = MAKERPS(SF8, BW125, CR_4_8, 0, 0);
  LMIC.sysname_cad_rps = MAKERPS(SF8, BW125, CR_4_8, 0, 0);

  // Resetting CAD State
  LMIC.sysname_enable_cad = 0;
  LMIC.sysname_use_fixed_difs = 0;
  LMIC.sysname_csma_algo = 0;

  LMIC.lbt_ticks = 0;              // Listen before talk ticks (x16)
  LMIC.lbt_dbmax = 0;              // Listen before talk max RSSI s1_t
  LMIC.sysname_kill_cad_delay = 0; // Kill CAD Wait time (0 or 1)

  // Resetting FSMA params
  LMIC.sysname_enable_FSMA = 0;
  LMIC.sysname_lbt_dbmin = -116; // > -188 (-118 and below are decting as false positive for cad)
  LMIC.sysname_enable_exponential_backoff = 0;
}

static void timestamp_readback(osjob_t *job)
{

  if (4 * interarrival_ind > (RB_LEN - 1))
  {

    ostime_t tosend_val = 0;

    for (u1_t osfet = 0; osfet < (RB_LEN - 1); osfet += 4)
    {
      tosend_val = interarrival_array[interarrival_ind];
      LMIC.frame[1 + osfet] = get_nth_byte(tosend_val, 0);
      LMIC.frame[2 + osfet] = get_nth_byte(tosend_val, 1);
      LMIC.frame[3 + osfet] = get_nth_byte(tosend_val, 2);
      LMIC.frame[4 + osfet] = get_nth_byte(tosend_val, 3);
      if (PRINT_ONLY_PACKET != 1 ){
        Serial.println(LMIC.frame[1 + osfet]);
      }
      interarrival_ind--;
    }

    tx_gen(tx_gen_done, RB_LEN);
  }
  else if (interarrival_ind > 0)
  {

    ostime_t tosend_val = 0;

    tosend_val = interarrival_array[interarrival_ind];
    for (u1_t osfet = 0; osfet < (RB_LEN - 1); osfet += 4)
    {
      LMIC.frame[1 + osfet] = get_nth_byte(tosend_val, 0);
      LMIC.frame[2 + osfet] = get_nth_byte(tosend_val, 1);
      LMIC.frame[3 + osfet] = get_nth_byte(tosend_val, 2);
      LMIC.frame[4 + osfet] = get_nth_byte(tosend_val, 3);
      if (PRINT_ONLY_PACKET != 1 ){
        Serial.println(LMIC.frame[1 + osfet]);
      }
      if (interarrival_ind == 0)
        tosend_val = 0;
      else
      {
        interarrival_ind--;
        tosend_val = interarrival_array[interarrival_ind];
      }
    }
    arbiter_state = 0;
    tx_gen(tx_gen_done, RB_LEN);
  }
  else
  {
    os_clearCallback(job);
    os_setCallback(&arbiter_job, arbiter_fn);
  }
}

static void timed_executor(osjob_t *job)
{

  static int executor_flag = 0;
  static byte scheduler_idx = 0;
  static ostime_t prev_time = 0;
  static byte dirt_bit = 0;

  if (!dirt_bit)
  {
    prev_time = expt_start_time;
    dirt_bit = 1;
  }

  switch (arbiter_state)
  {
  case 10:
    // Continuous Tx
    executor_flag = (os_getTime() > expt_stop_time) ? 0 : 1;
    if (executor_flag)
    {
      // If schedule list is complete, schedule more
      if (scheduler_idx == 0)
        fill_schedule();

      // Schedule Tx
      prev_time = scheduler_list_ms[scheduler_idx];

      // Check if time passed, if not schedule. If passed, do immediately
      if (prev_time > os_getTime())
      {

        os_setTimedCallback(job, prev_time, tx_func_multi);
      }
      else
        os_setCallback(job, tx_func_multi);

      scheduler_idx = (scheduler_idx + 1) % SCHEDULE_LEN;
    }
    else
    {
      // Stop, reset static vars
      // and hand over control to arbiter
      executor_flag = 0;
      scheduler_idx = 0;
      prev_time = 0;
      dirt_bit = 0;
      os_clearCallback(job);
      arbiter_state = 0;
      store_multitx_results();
      os_setCallback(&arbiter_job, arbiter_fn);
    }
    break;
  default:
    executor_flag = 0;
    executor_flag = 0;
    scheduler_idx = 0;
    prev_time = 0;
    dirt_bit = 0;
    os_clearCallback(job);
    arbiter_state = 0; // Clearing arbiter state
    os_setCallback(&arbiter_job, arbiter_fn);
  }
}

static void arbiter_fn(osjob_t *job)
{

  LMIC.freq = trx_freq_vec[freq_cnfg_ind];
  // Three bytes for buf_in
  // byte 0    : Node IDX/Address
  // byte 1    : Command to Node
  // byte 2    : Register
  // byte 3    : Value
  if (buf_in[0] == 255) {
    delay(5); // delay 5ms for broadcast messages
  }

  if ((buf_in[0] == NODE_IDX || buf_in[0] == 255) && !LMIC.sysname_crc_err)
  {
    switch (buf_in[1])
    {
    case 0:
      // Signal Alive
      buf_out[0] = NODE_IDX;
      buf_out[1] = 0;
      buf_out[2] = 0;
      buf_out[3] = 0;
      // Save the receive RSSI in the reg_array
      reg_array[NODE_IDX] = LMIC.rssi;
      if (PRINT_ONLY_PACKET != 1 ){
        Serial.println("Signalling Alive");
      }
      arbiter_state = 0;
      os_setCallback(job, tx_func);
      break;
    case 1:
      // Read Reg buf_in[2]
      buf_out[0] = NODE_IDX;
      buf_out[1] = 1;
      buf_out[2] = buf_in[2];
      buf_out[3] = reg_array[buf_in[2]];
      if (PRINT_ONLY_PACKET != 1 ){
        Serial.print("Reading Reg: ");
        Serial.print(buf_in[2], HEX);
        Serial.print(", value: ");
        Serial.print(buf_out[3]);
        Serial.print("\n");
      }
      arbiter_state = 0;
      os_setCallback(job, tx_func);
      break;
    case 2:
      // Write Reg buf_in[2] with buf_in[3];
      reg_array[buf_in[2]] = buf_in[3];
      buf_out[0] = NODE_IDX;
      buf_out[1] = 2;
      buf_out[2] = buf_in[2];
      buf_out[3] = buf_in[3];
      if (PRINT_ONLY_PACKET != 1 ){
        Serial.print("Writing Reg: ");
        Serial.print(buf_in[2], HEX);
        Serial.print(", Val: ");
        Serial.print(buf_in[3], HEX);
        Serial.print("\n");
      }
      arbiter_state = 0;
      os_setCallback(job, tx_func);
      break;
    case 3:
      // Retune the CNFG or EXPT Frequency
      buf_out[0] = NODE_IDX;
      buf_out[1] = 3;
      buf_out[2] = buf_in[2];
      buf_out[3] = buf_in[3];

      if (buf_in[2] == 0)
      {
        // Changing CNFG Frequency
        freq_cnfg_ind = buf_in[3];
        if (PRINT_ONLY_PACKET != 1 ){
          Serial.print("Switching CNFG Frequncy to: ");
          Serial.print(trx_freq_vec[freq_cnfg_ind]);
          Serial.print("\n");
        }
      }
      else
      {
        // Changing EXPT Frequency
        freq_expt_ind = buf_in[3];
        if (PRINT_ONLY_PACKET != 1 ){
          Serial.print("Switching EXPT Frequncy to: ");
          Serial.print(trx_freq_vec[freq_expt_ind]);
          Serial.print("\n");
        }
      }

      arbiter_state = 0;
      os_setCallback(job, tx_func);

      break;
    case 4:
      // Signal Alive With the RSSI
      buf_out[0] = NODE_IDX;
      buf_out[1] = 4;
      buf_out[2] = buf_in[2];
      buf_out[3] = LMIC.rssi;
      // Save the receive RSSI in the reg_array
      reg_array[NODE_IDX] = LMIC.rssi;
      if (PRINT_ONLY_PACKET != 1 ){
        Serial.println("Signalling Alive with RSSI");
      }
      arbiter_state = 0;
      os_setCallback(job, tx_func);
      break;
    case 5:
      // Read Register and Replace Value with Request
      buf_out[0] = NODE_IDX;
      buf_out[1] = 5;
      buf_out[2] = buf_in[2];
      buf_out[3] = reg_array[buf_in[2]];
      // Write Reg buf_in[2] with buf_in[3];
      reg_array[buf_in[2]] = buf_in[3];
      if (PRINT_ONLY_PACKET != 1 ){
        Serial.print("Writing Reg: ");
        Serial.print(buf_in[2], HEX);
        Serial.print(", Val: ");
        Serial.print(buf_in[3], HEX);
        Serial.print("\n");
      }
      arbiter_state = 0;
      os_setCallback(job, tx_func);
      break;
    case 20:
      // This thing triggers time-stamp readback
      buf_out[0] = NODE_IDX;
      buf_out[1] = 20;
      buf_out[2] = buf_in[2];
      buf_out[3] = 255;
      if (PRINT_ONLY_PACKET != 1 ){
        Serial.println("Timestamp Readback!");
      }
      arbiter_state = 20;
      os_setCallback(job, tx_func);
      break;
    case 10:
      // Start Continuous Tx
      buf_out[0] = NODE_IDX;
      buf_out[1] = 10;
      buf_out[2] = buf_in[2];     
      buf_out[3] = 255;
      arbiter_state = 10;
      Serial.println("Continuous Tx Trigger!");
      os_setCallback(job, tx_func);
      break;
    case 254:
      // BROADCAST RX Message Handler
      reg_array[buf_in[2]] = LMIC.rssi;
      if (PRINT_ONLY_PACKET != 1 ){
        Serial.print("Got Broadcast from Node: ");
        Serial.print(buf_in[2]);
        Serial.print(", RSSI: ");
        Serial.print(LMIC.rssi);
        Serial.print("\n");
      }

      // Reset buf_in to prevent retrigger
      buf_in[0] = 0;
      buf_in[1] = 0;
      buf_in[2] = 0;
      buf_in[3] = 0;
      os_setCallback(job, arbiter_fn);
      break;
    case 255:
      // Broadcast
      buf_out[0] = 255;
      buf_out[1] = 254; // Broadcast Rx Action
      buf_out[2] = NODE_IDX;
      buf_out[3] = 0;
      // Save the receive RSSI in the reg_array
      reg_array[NODE_IDX] = LMIC.rssi;
      if (PRINT_ONLY_PACKET != 1 ){
        Serial.println("Broadcasting Now!");
      }
      arbiter_state = 0;
      os_setCallback(job, tx_func);
      break;
    default:
      buf_out[0] = NODE_IDX;
      buf_out[1] = 0;
      buf_out[2] = 0;
      buf_out[3] = 0;
      arbiter_state = 0;
      os_setCallback(job, tx_func);
    }
    // Reset buf_in to prevent retrigger
    buf_in[0] = 0;
    buf_in[1] = 0;
    buf_in[2] = 0;
    buf_in[3] = 0;
  }
  else
  {

    switch (arbiter_state)
    {
    case 0:
      // Go to resting listen mode
      os_setCallback(job, rx_func);
      break;
    case 10:
      // generate a random delay
      max_start_delay = reg_array[28]*256 + reg_array[27];
      custom_start_delay = reg_array[30]*256 + reg_array[29];
      if (max_start_delay > 1) {
        expt_start_delay = (radio_rand1()*radio_rand1())%(max_start_delay); // generates delay between 0 to max_start_delay (default: 1s)
        delay(expt_start_delay);
      
        Serial.print("Added random start delay: ");
        Serial.print(expt_start_delay);
        Serial.print(" ms, max start delay: ");
        Serial.print(max_start_delay);
        Serial.println(" ms");
      } else {
        expt_start_delay = 0;
      }

      if (custom_start_delay > 0) {
        delay(custom_start_delay);
        Serial.print("Added custom start delay: ");
        Serial.print(custom_start_delay);
        Serial.println(" ms");
      }

      // Start Continuous Transmission
      prepare_multi_tx();
      expt_start_millis = millis();
      expt_start_time = os_getTime();
      experiment_time = reg_array[2] * reg_array[3];
      expt_stop_time = expt_start_time + ms2osticks(experiment_time * 1000);
      LMIC.sysname_experiment_timeout = expt_stop_time;
      interarrival_array[interarrival_ind] = expt_start_time;
      interarrival_ind++;

      //set timeout callback to bring out CAD/TX mode and end experiment
      if (PRINT_ONLY_PACKET != 1 ){
        Serial.print("Setting experiment timeout after: ");
        Serial.print(experiment_time);
        Serial.println(" s");
      }
      os_setTimedCallback(&timeoutjob, expt_stop_time, experiment_timeout_func);
      os_setCallback(job, timed_executor);
      break;
    case 11:
      // Start Continuous Reception
      os_setCallback(job, timed_executor);
      break;
    case 20:
      // Start Continuous Timestamp readback
      interarrival_ind = multi_tx_packet_ctr + 16;
      os_setCallback(job, timestamp_readback);
      break;
    default:
      os_setCallback(job, rx_func);
    }
  }
}

// application entry point
void setup()
{
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  // blink to show reset
  for(int i=0; i<5; i++) {
    digitalWrite(LED_BUILTIN, LOW); // turn OFF
    delay(200);
    digitalWrite(LED_BUILTIN, HIGH); // turn ON
    delay(200);
  }

  // initialize runtime env
  os_init();

  // reset radio
  radio_init();

  // disable RX IQ inversion
  LMIC.noRXIQinversion = true;

  // Say Hi
  Serial.flush();
  Serial.print("Hi I am Node ");
  Serial.print(NODE_IDX);
  Serial.print("\n");

  //  LMIC.rps = MAKERPS(SF8 , BW500, CR_4_8, 0, 0); // WCSNG
  //  LMIC.sysname_tx_rps =  MAKERPS(SF8 , BW500, CR_4_8, 0, 0); // WCSNG
  //  LMIC.sysname_cad_rps =  MAKERPS(SF8 , BW500, CR_4_8, 0, 0); // WCSNG
  LMIC.rps = MAKERPS(SF8, BW125, CR_4_8, 0, 0);             // WCSNG
  LMIC.sysname_tx_rps = MAKERPS(SF8, BW125, CR_4_8, 0, 0);  // WCSNG
  LMIC.sysname_cad_rps = MAKERPS(SF8, BW125, CR_4_8, 0, 0); // WCSNG
  LMIC.txpow = -2;
  LMIC.radio_txpow = -5; // WCSNG
  LMIC.adrTxPow = -3;

  // Set the generic TRX frequencies:
  for (byte idx = 0; idx < 24; idx++)
  {
    trx_freq_vec[idx] = 900000000 + ((u4_t)idx) * 1000000;
  }

  freq_cad_ind = FREQ_CAD_INDEX;
  freq_expt_ind = FREQ_EXPT_INDEX;
  freq_cnfg_ind = FREQ_CNFG_INDEX; // 13
  // Set the LMIC CAD Frequencies
  LMIC.freq = trx_freq_vec[freq_cnfg_ind]; // WCSNG
  LMIC.sysname_cad_freq_vec[0] = trx_freq_vec[freq_expt_ind];
  LMIC.sysname_cad_freq_vec[1] = trx_freq_vec[freq_cad_ind];
  // LMIC.sysname_cad_freq_vec[2] = trx_freq_vec[freq_expt_ind] - 2000000;
  // LMIC.sysname_cad_freq_vec[3] = trx_freq_vec[freq_expt_ind] - 4000000;

  // Setup Registers
  interarrival_ind = 0;
  arbiter_state = 0;       // Resting state
  multi_tx_packet_ctr = 0; // Resetting counter

  reg_array[0] = 90; // tx_interval
  reg_array[1] = 20; // Packet Size Bytes
  reg_array[2] = 1; // Experiment run length in seconds
  reg_array[3] = 1;  // Time multiplier for expt time
  reg_array[4] = 0;  // Enable CAD - OFF by default
  reg_array[5] = 9;  // DIFS as number of CADs
  reg_array[6] = 5; // Backoff Unit in ms (backoff cnfg 1)
  reg_array[7] = 100;  // Max Backoff Unit Multiplier length (backoff cnfg 2)
  reg_array[8] = 1;  // tx_interval_multiplier
  reg_array[9] = 1;  // scheduler_interval_mode (0: Periodic, 1: Poisson, 2: Periodic with Variance);

  reg_array[17] = 66; // 17: {4bits txsf, 4bits rxsf}
  reg_array[18] = 0; // 18: {4bits txbw, 4bits rxbw}
  // reg_array[19] = 51; // 19: {4bits txcr, 4bits rxcr}
  reg_array[19] = 0; // 19: {4bits txcr, 4bits rxcr}

  reg_array[20] = 0;   // CAD Type and Config Reg
  reg_array[21] = 8;   // Listen before talk ticks (x16)
  reg_array[22] = -90; // Listen before talk max RSSI s1_t
  reg_array[23] = 1;   // Kill CAD Wait time (0 or 1)

  reg_array[24] = 21;
  reg_array[25] = freq_expt_ind;
  reg_array[26] = freq_cad_ind;
  reg_array[27] = 0;
  reg_array[28] = 0;
  reg_array[29] = 0;
  reg_array[30] = 0;
  reg_array[31] = 1;
  
  reg_array[45] = 10; // Variance if using periodic scheduling
  LMIC.sysname_kill_cad_delay = 0; // Kill CAD Wait time (0 or 1)
  
  // FSMA
  reg_array[49] = 0; // Diasble FSMA  
  reg_array[50] = -150; // Listen before talk min RSSI s1_T
  reg_array[51] = 0; // Disable exponential backoff
  reg_array[55] = 1; // enable inband CAD
  reg_array[56] = 2; // RPS of inband CAD
  reg_array[57] = 1; // beacon symbols

  LMIC.sysname_is_FSMA_node = 1;
  LMIC.sysname_enable_variable_cad_difs = 0;
  LMIC.sysname_waittime_between_cads = 2; // in ms
  LMIC.sysname_FSMA_beacon_symbols = 1;
  LMIC.sysname_enable_inband_cad = 1;

  for (byte idx = 0; idx < 100; idx++)
    reg_array[101 + idx] = RSSI_RESET_VAL;

  buf_in[0] == 0;
  buf_in[1] == 0;
  buf_in[2] == 0;
  buf_in[3] == 0;

  buf_print[0] == 0;
  buf_print[1] == 0;
  buf_print[2] == 0;
  buf_print[3] == 0;

#if (ADAFRUIT_FEATHER_TYPE == 1)  // Pin mapping for Adafruit Feather M0 LoRa, etc.
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
  expt_start_time = os_getTime();
  expt_stop_time = expt_start_time + ms2osticks(reg_array[2] * reg_array[3] * 1000);
  os_setCallback(&arbiter_job, rx_func);
}

void loop()
{
  // execute scheduled jobs and events
  os_runloop_once();
}

// float sum_var = 0;
// float ind_var = 1;
// trash:
//   u4_t rand_var = get_wait_time_ms();
//   sum_var += rand_var;
//   Serial.print(ind_var);
//   Serial.print(": ");
//   Serial.print(rand_var);
//   Serial.print(", ");
//   Serial.print(sum_var/ind_var);
//   Serial.print("\n");
////  delay(10);
//  ind_var++;