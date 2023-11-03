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

static void cad_func(osjob_t *job) {
  LMIC.rps = LMIC.sysname_cad_rps;
  LMIC.freq = LMIC.sysname_cad_freq_vec[1];

  u1_t isChanneFree;
  isChanneFree = cadlora_fixedDIFS();
  if (isChanneFree == 0) {
    Serial.print("sensing in channel: ");
    Serial.println(LMIC.freq);
    Serial.print("CAD done, status: ");
    Serial.println(!isChanneFree);
    Serial.print("LMIC RSSI Mean and Max: ");
    Serial.print(LMIC.sysname_lbt_rssi_mean);
    Serial.print(", ");
    Serial.println(LMIC.sysname_lbt_rssi_max);
  }
  //delay(500);
  os_setCallback(job, cad_func);
}

static void intialize() {
  // initialize runtime env
  os_init();

  // disable RX IQ inversion
  LMIC.noRXIQinversion = true;

  //  LMIC.rps = MAKERPS(SF8 , BW500, CR_4_8, 0, 0); // WCSNG
  //  LMIC.sysname_tx_rps =  MAKERPS(SF8 , BW500, CR_4_8, 0, 0); // WCSNG
  //  LMIC.sysname_cad_rps =  MAKERPS(SF8 , BW500, CR_4_8, 0, 0); // WCSNG
  LMIC.rps = MAKERPS(SF10, BW125, CR_4_8, 0, 0);              // WCSNG
  LMIC.sysname_tx_rps = MAKERPS(SF10, BW125, CR_4_8, 0, 0);   // WCSNG
  LMIC.sysname_cad_rps = MAKERPS(SF8, BW125, CR_4_8, 0, 0);  // WCSNG
  LMIC.txpow = 21;
  LMIC.radio_txpow = 21;  // WCSNG

  // Set the LMIC CAD Frequencies
  LMIC.freq = 922000000;  // WCSNG
  LMIC.sysname_cad_freq_vec[0] = 922000000;
  LMIC.sysname_cad_freq_vec[1] = 920000000;
//  LMIC.sysname_cad_freq_vec[2] = 920000000 - 2000000;
//  LMIC.sysname_cad_freq_vec[3] = 920000000 - 4000000;

  // FSMA 
  LMIC.sysname_enable_cad = 1; //FSMA is sub category of CAD
  LMIC.sysname_kill_cad_delay = 1;
  LMIC.sysname_enable_FSMA = 1;
  LMIC.sysname_is_FSMA_node = 0;
  LMIC.sysname_enable_exponential_backoff = 0;
  LMIC.sysname_enable_variable_cad_difs = 0;

  LMIC.lbt_ticks = 8;
  LMIC.sysname_cad_difs = 2;
  LMIC.sysname_lbt_dbmin = -115;
  // LMIC.sysname_backoff_cfg1 = 12;
  // LMIC.sysname_backoff_cfg2 = 64;

  Serial.flush();
  // Say Hi
  Serial.print("--- Hi I am Node ---");
  Serial.print("\n");
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  intialize();
  LMIC.lbt_ticks = 1;
  LMIC.sysname_cad_difs = 2;
  os_setCallback(&arbiter_job, cad_func);
}

void loop() {
  // put your main code here, to run repeatedly:
  os_runloop_once();
}
