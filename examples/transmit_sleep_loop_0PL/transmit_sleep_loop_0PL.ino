#include <lmic.h>
#include <hal/hal.h>
#include <arduino_lmic_hal_boards.h>

#include <SPI.h>

#include <stdarg.h>
#include <stdio.h>

#if defined(ADAFRUIT_FEATHER_M0) && (ADAFRUIT_FEATHER_M0 == 1)  // Pin mapping for Adafruit Feather M0 LoRa, etc.
const lmic_pinmap lmic_pins = {
  .nss = 8,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 4,
  .dio = { 3, 6, LMIC_UNUSED_PIN },
  .rxtx_rx_active = 0,
  .rssi_cal = 8,              // LBT cal for the Adafruit Feather M0 LoRa, in dB
  .spi_freq = 8000000,
};

int32_t interrupt_timer = us2osticks(4500 + 2048*2);

#else
// Pin mapping
const lmic_pinmap lmic_pins = {
  .nss = D10,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = A0,
  .dio = { 2, 3, 4 },
};

int32_t interrupt_timer = us2osticks(7000 + 2048*2);
#endif

// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in arduino-lmoc/project_config/lmic_project_config.h,
// otherwise the linker will complain).
void os_getArtEui(u1_t *buf) {}
void os_getDevEui(u1_t *buf) {}
void os_getDevKey(u1_t *buf) {}

osjob_t arbiter_job;
osjob_t interrupt_job;
osjob_t sleep_job;

void tx(osjobcb_t func) {
  // the radio is probably in RX mode; stop it.
  os_radio(RADIO_RST);
  // wait a bit so the radio can come out of RX mode
  delay(1);

  // prepare data
  LMIC.dataLen = 0;
  //LMIC.frame[0] = 'H';
  //LMIC.frame[1] = 'i';
  // set completion function.
  LMIC.osjob.func = func;

  // start the transmission
  // Serial.println("Setting timed callback and transmitting..");
  // os_setTimedCallback(&interrupt_job,  os_getTime() + us2osticks(2250+2048*7), interrupt_func);
  // os_setTimedCallback(&interrupt_job, os_getTime() + us2osticks(12800 + 2048*2), interrupt_func);  // FSMA
  // os_setTimedCallback(&interrupt_job, os_getTime() + interrupt_timer, interrupt_func);  // FSMA

  os_radio(RADIO_TX);
  Serial.println("Transmit done ..");
  os_setCallback(&sleep_job, sleep);
}

static void interrupt_func(osjob_t *job) {
  //  Serial.println("Inside interrupt fn ...");
  radio_init();
  os_radio(RADIO_RST);
  Serial.println("Radio RESET is done ...");

  // delay(1000);
  os_setCallback(job, tx_func);
}

static void tx_func(osjob_t *job) {
  // Send BUF OUT
  tx(sleep);
}

static void sleep(osjob_t *job) {
  Serial.println("Sleeping ...");
  delay(250);
  os_setCallback(job, tx_func);
}

static void intialize() {
  // initialize runtime env
  os_init();

  // disable RX IQ inversion
  LMIC.noRXIQinversion = true;

  //  LMIC.rps = MAKERPS(SF8 , BW500, CR_4_8, 0, 0); // WCSNG
  //  LMIC.sysname_tx_rps =  MAKERPS(SF8 , BW500, CR_4_8, 0, 0); // WCSNG
  //  LMIC.sysname_cad_rps =  MAKERPS(SF8 , BW500, CR_4_8, 0, 0); // WCSNG
  LMIC.rps = MAKERPS(SF8, BW125, CR_4_8, 0, 0);              // WCSNG
  LMIC.sysname_tx_rps = MAKERPS(SF8, BW125, CR_4_8, 0, 0);   // WCSNG
  LMIC.sysname_cad_rps = MAKERPS(SF10, BW125, CR_4_8, 0, 0);  // WCSNG
  LMIC.txpow = 21;
  LMIC.radio_txpow = 21;  // WCSNG

  // Set the LMIC CAD Frequencies
  LMIC.freq = 918000000;  // WCSNG
  LMIC.sysname_cad_freq_vec[0] = 918000000; // reverse for gateway
  LMIC.sysname_cad_freq_vec[1] = 920000000;// reverse for gateway
  // LMIC.sysname_cad_freq_vec[2] = 920000000 - 2000000;
  // LMIC.sysname_cad_freq_vec[3] = 920000000 - 4000000;

  // FSMA
  LMIC.sysname_enable_cad = 1; //FSMA is sub category of CAD
  LMIC.sysname_kill_cad_delay = 1;
  LMIC.sysname_enable_FSMA = 1;
  LMIC.sysname_is_FSMA_node = 0;
  LMIC.sysname_enable_exponential_backoff = 0;
  LMIC.sysname_enable_variable_cad_difs = 0;

  LMIC.lbt_ticks = 0;
  LMIC.sysname_cad_difs = 2;
  LMIC.sysname_lbt_dbmin = -115;
  // LMIC.sysname_backoff_cfg1 = 12;
  // LMIC.sysname_backoff_cfg2 = 64;

  Serial.flush();
  // Say Hi
  Serial.print("Hi I am Node ");
  Serial.print("\n");
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  intialize();
  os_setCallback(&arbiter_job, tx_func);
}

void loop() {
  // put your main code here, to run repeatedly:
  os_runloop_once();
}
