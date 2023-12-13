#include <lmic.h>
#include <hal/hal.h>
#include <arduino_lmic_hal_boards.h>

#include <SPI.h>

#include <stdarg.h>
#include <stdio.h>

#define FREQ_CAD2 916000000
#define FREQ_EXPT 920000000
#define FREQ_CNFG 922000000
#define INTERRUPT_CAD 1
#define BEACON_SYMBOLS 10
#define VBATPIN A7
#define TOTAL_TRANSMISSIONS 1000
#define DELAY_TIME 20

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

int32_t interrupt_timer = us2osticks(10000 + 2048 * 1);  //SF-8, DIFS-1
// int32_t interrupt_timer = us2osticks(5600 + 2048 * 1);  //SF-8, DIFS-1

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

int32_t interrupt_timer = us2osticks(16800 + 2048 * BEACON_SYMBOLS);
int32_t transmit_symbols;

// int32_t interrupt_timer = us2osticks(1000 + 2048);

#else
// Pin mapping
const lmic_pinmap lmic_pins = {
  .nss = D10,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = A0,
  .dio = { 2, 3, 4 },
};

// int32_t interrupt_timer = us2osticks(7000 + 2048*2);
// int32_t interrupt_timer = us2osticks(35678 + 2048*2); //SF-10, DIFS-2
int32_t interrupt_timer = us2osticks(21000 + 2048 * 2);  //SF-10, DIFS-2
#endif

// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in arduino-lmoc/project_config/lmic_project_config.h,
// otherwise the linker will complain).
void os_getArtEui(u1_t *buf) {}
void os_getDevEui(u1_t *buf) {}
void os_getDevKey(u1_t *buf) {}

osjob_t arbiter_job, interrupt_job, sleep_job, timeoutjob;
byte reg_array[64];
ostime_t expt_start_time, expt_stop_time;  // 1ms is 62.5 os ticks
int32_t experiment_time;

#if (defined(INTERRUPT_CAD) && (INTERRUPT_CAD == 1))
int32_t interrupt_cad_timer = us2osticks(1100 + 2048 * BEACON_SYMBOLS);
#endif

void tx(osjobcb_t func) {
  // Serial.println("Setting timed callback and transmitting..");
  LMIC.freq = FREQ_CAD2;
  // set trigger
#if (defined(INTERRUPT_CAD) && (INTERRUPT_CAD == 1))
  os_setTimedCallback(&interrupt_job, os_getTime() + interrupt_cad_timer, interrupt_CAD_func);  // FSMA
#else
  os_setTimedCallback(&interrupt_job, os_getTime() + interrupt_timer, interrupt_func);  // FSMA
#endif

  // start the transmission
  os_radio(RADIO_TX);
  // os_setCallback(&sleep_job, sleep);
}

static void tx_func(osjob_t *job) {
  // Send BUF OUT
  tx(sleep);
}

static void interrupt_func(osjob_t *job) {
  // Serial.println("Inside interrupt fn ...");
  // hal_pin_rst(0);                                 // drive RST pin low
  // hal_waitUntil(os_getTime() + us2osticks(200));  // wait >100us
  // hal_pin_rst(2);                                 // configure RST pin floating!
  // hal_waitUntil(os_getTime() + us2osticks(200));

  os_setCallback(job, tx_func);
}

static void interrupt_CAD_func(osjob_t *job) {
  // LMIC.sysname_enable_cad = 0;  //FSMA is sub category of CAD
  // LMIC.sysname_enable_FSMA = 1;

  // u1_t isChanneFree = 0;
  // while (isChanneFree == 0) {
  //   isChanneFree = cadlora_customSensing();
  // }

  // Serial.println("Inside interrupt fn ...");
  hal_pin_rst(0);                                 // drive RST pin low
  hal_waitUntil(os_getTime() + us2osticks(200));  // wait >100us
  hal_pin_rst(2);                                 // configure RST pin floating!
  hal_waitUntil(os_getTime() + us2osticks(200));

  if (transmit_symbols < TOTAL_TRANSMISSIONS) {
    transmit_symbols = transmit_symbols + 1;
    // LMIC.radio_txpow = -4 + (transmit_symbols%25);  // WCSNG

    Serial.print("Transmission: number = ");
    Serial.print(transmit_symbols);
    Serial.print(", power = ");
    Serial.println(LMIC.radio_txpow);
    delay(DELAY_TIME);

    os_setCallback(job, tx_func);
  } else {
    radio_init();
    Serial.print("Transmission done went to sleep");
  }
}

static void sleep(osjob_t *job) {
  Serial.println("Sleeping ...");
  // delay(250);
  delay(1);
  os_setCallback(job, tx_func);
}

static void rxdone_func(osjob_t *job) {
  Serial.print("[");
  Serial.print(LMIC.frame[0]);
  Serial.print(",");
  Serial.print(LMIC.frame[1]);
  Serial.print(",");
  Serial.print(LMIC.frame[2]);
  Serial.print(",");
  Serial.print(LMIC.frame[3]);
  Serial.print("]");
  Serial.print(";");

  Serial.print("RSSI: ");
  Serial.print(LMIC.rssi - RSSI_OFF);
  Serial.print("; ");

  Serial.print("SNR: ");
  Serial.print(LMIC.snr / 4);
  Serial.print("; ");

  Serial.print("CRC: ");
  Serial.print(LMIC.sysname_crc_err);
  Serial.print("\n");

  // Arbiter
  if ((LMIC.frame[1] == 10) && (LMIC.frame[2] == 0)) {
    // setting experiment variables
    LMIC.freq = FREQ_CAD2;
    LMIC.sysname_cad_rps = MAKERPS(reg_array[17] >> 4, reg_array[18] >> 4, reg_array[19] >> 4, 0, 0);  // WCSNG (Reverse of Node)
    LMIC.sysname_tx_rps = MAKERPS(reg_array[17] % 16, reg_array[18] % 16, reg_array[19] % 16, 0, 0);   // WCSNG (Reverse of Node)

    //set timeout callback to stop sending free beacons
    experiment_time = reg_array[2] * reg_array[3];
    Serial.print("Setting experiment timeout after: ");
    Serial.print(experiment_time);
    Serial.println("s + 10s");
    os_setTimedCallback(&timeoutjob, os_getTime() + ms2osticks((experiment_time + 10) * 1000), experiment_timeout_func);
    os_setCallback(&arbiter_job, tx_func);
  } else if ((LMIC.frame[1] == 2) && (LMIC.frame[2] == 2 || LMIC.frame[2] == 3 || LMIC.frame[2] == 17 || LMIC.frame[2] == 18 || LMIC.frame[2] == 19)) {
    reg_array[LMIC.frame[2]] = LMIC.frame[3];
    Serial.print("Writing Reg: ");
    Serial.print(LMIC.frame[2], HEX);
    Serial.print(", Val: ");
    Serial.print(LMIC.frame[3], HEX);
    Serial.print("\n");
    os_setCallback(&arbiter_job, rx_func);
  } else {
    os_setCallback(&arbiter_job, rx_func);
  }
}

// Enable rx mode and call func when a packet is received
void rx(osjobcb_t func) {
  LMIC.osjob.func = func;
  LMIC.rxtime = os_getTime();  // RX _now_
  // Enable "continuous" RX (e.g. without a timeout, still stops after
  // receiving a packet)

  os_radio(RADIO_RXON);
}

static void rx_func(osjob_t *job) {
  // GET BUF_OUT
  rx(rxdone_func);
}

static void experiment_timeout_func(osjob_t *job) {
  // resetting to control params
  LMIC.freq = FREQ_CNFG;
  LMIC.rps = MAKERPS(SF8, BW125, CR_4_8, 0, 0);  // WCSNG

  Serial.println("Experiment timeout function triggered...");
  radio_init();
  os_radio(RADIO_RST);
  intialize();
  os_setCallback(&interrupt_job, rx_func);
}

static void intialize() {

  // disable RX IQ inversion
  LMIC.noRXIQinversion = true;

  //  LMIC.rps = MAKERPS(SF8 , BW500, CR_4_8, 0, 0); // WCSNG
  //  LMIC.sysname_tx_rps =  MAKERPS(SF8 , BW500, CR_4_8, 0, 0); // WCSNG
  //  LMIC.sysname_cad_rps =  MAKERPS(SF8 , BW500, CR_4_8, 0, 0); // WCSNG
  LMIC.rps = MAKERPS(SF8, BW125, CR_4_8, 0, 0);               // WCSNG
  LMIC.sysname_tx_rps = MAKERPS(SF8, BW125, CR_4_8, 0, 0);    // WCSNG
  LMIC.sysname_cad_rps = MAKERPS(SF10, BW125, CR_4_8, 0, 0);  // WCSNG
  LMIC.txpow = -4;
  LMIC.radio_txpow = -4;  // WCSNG

  // Set the LMIC CAD Frequencies
  LMIC.freq = 922000000;                     // WCSNG
  LMIC.sysname_cad_freq_vec[0] = 918000000;  // reverse for gateway
  LMIC.sysname_cad_freq_vec[1] = 920000000;  // reverse for gateway

// LMIC.sysname_cad_freq_vec[2] = 920000000 - 2000000;
// LMIC.sysname_cad_freq_vec[3] = 920000000 - 4000000;

// FSMA
#if (defined(INTERRUPT_CAD) && (INTERRUPT_CAD == 1))
  LMIC.sysname_enable_cad = 0;    //FSMA is sub category of CAD
  LMIC.sysname_is_FSMA_node = 0;  //FSMA is sub category of CAD
#else
  LMIC.sysname_enable_cad = 0;                                                          //FSMA is sub category of CAD
  LMIC.sysname_is_FSMA_node = 0;                                                        //FSMA is sub category of CAD
#endif

  LMIC.sysname_kill_cad_delay = 1;
  LMIC.sysname_is_FSMA_node = 0;
  LMIC.sysname_enable_exponential_backoff = 0;
  LMIC.sysname_enable_variable_cad_difs = 0;

  LMIC.lbt_ticks = 8;
  LMIC.sysname_cad_difs = 1;
  LMIC.sysname_lbt_dbmin = -115;
  LMIC.sysname_lbt_counter = 0;
  LMIC.sysname_cad_counter = 0;
  LMIC.sysname_cad_detect_counter = 0;
  LMIC.lbt_dbmax = -90;
  // LMIC.sysname_backoff_cfg1 = 12;
  // LMIC.sysname_backoff_cfg2 = 64;

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
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  // blink to show reset
  for (int i = 0; i < 5; i++) {
    digitalWrite(LED_BUILTIN, LOW);  // turn OFF
    delay(200);
    digitalWrite(LED_BUILTIN, HIGH);  // turn ON
    delay(200);
  }

  Serial.flush();
  // Say Hi
  Serial.print("Hi, I will sense channel and transmit beacons");
  Serial.print("\n");

  // prepare data
  LMIC.dataLen = 0;
  //LMIC.frame[0] = 'H';
  //LMIC.frame[1] = 'i';
  // set completion function.
  LMIC.osjob.func = sleep;

  // setting default values in registers
  reg_array[2] = 1;    // Experiment run length in seconds
  reg_array[3] = 1;    // Time multiplier for expt time
  reg_array[17] = 34;  // 17: {4bits txsf, 4bits rxsf}
  reg_array[18] = 34;  // 18: {4bits txbw, 4bits rxbw}
  reg_array[19] = 51;  // 19: {4bits txcr, 4bits txcr}

  // initialize runtime env
  os_init();

  intialize();

  transmit_symbols = 0;
  // os_setCallback(&arbiter_job, rx_func);  // this will only sense and transmit when experiment starts and end after that
  os_setCallback(&arbiter_job, tx_func);  // this will always sense and transmit
}

void loop() {
  // put your main code here, to run repeatedly:
  os_runloop_once();
}
