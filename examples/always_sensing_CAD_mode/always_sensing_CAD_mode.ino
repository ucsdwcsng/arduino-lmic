#include <lmic.h>
#include <hal/hal.h>
#include <arduino_lmic_hal_boards.h>

#include <SPI.h>

#include <stdarg.h>
#include <stdio.h>

#define ENABLE_CAD_SF 8
#define DELAY_TIME 20
#define TOTAL_CADS 2000

// Pin mapping
#if (defined(ADAFRUIT_FEATHER_RP2040) && (ADAFRUIT_FEATHER_RP2040 == 1))  // Pin mapping for Adafruit Feather M0 LoRa, etc.
const lmic_pinmap lmic_pins = {
  .nss = 16,
  .rxtx = LMIC_UNUSED_PIN,
  .rst = 17,
  .dio = { 21, 6, LMIC_UNUSED_PIN },
  .rxtx_rx_active = 0,
  .rssi_cal = 8,              // LBT cal for the Adafruit Feather M0 LoRa, in dB
  .spi_freq = 8000000,
};

#elif (defined(ADAFRUIT_FEATHER_M0) && (ADAFRUIT_FEATHER_M0 == 1))  // Pin mapping for Adafruit Feather M0 LoRa, etc.
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

osjob_t arbiter_job;
osjob_t interrupt_job;
osjob_t sleep_job;
uint32_t cad_count;

static void cad_func(osjob_t *job) {
  LMIC.rps = LMIC.sysname_cad_rps;
  LMIC.freq = LMIC.sysname_cad_freq_vec[1];
  LMIC.sysname_enable_cad_analysis = 1;
  cad_count++;

  u1_t isChanneFree;
  isChanneFree = cadlora_customSensing();
  if (isChanneFree == 0) {
    
    Serial.print(os_getTime());
    Serial.print(", ");

    Serial.print(LMIC.sysname_lbt_rssi_mean);
    Serial.print(", ");

    Serial.print(LMIC.sysname_lbt_rssi_max);
    Serial.print(", ");

    Serial.print(LMIC.freq);
    Serial.print(", ");

    Serial.println(cad_count);
  }
  delay(DELAY_TIME);
  if (cad_count < TOTAL_CADS) {
    os_setCallback(job, cad_func);
  } 
  else {
    radio_init();
    Serial.print("CAD done went to sleep");
  }
}

static void intialize() {

  // disable RX IQ inversion
  LMIC.noRXIQinversion = true;

  //  LMIC.rps = MAKERPS(SF8 , BW500, CR_4_8, 0, 0); // WCSNG
  //  LMIC.sysname_tx_rps =  MAKERPS(SF8 , BW500, CR_4_8, 0, 0); // WCSNG
  //  LMIC.sysname_cad_rps =  MAKERPS(SF8 , BW500, CR_4_8, 0, 0); // WCSNG
  LMIC.rps = MAKERPS(SF10, BW125, CR_4_8, 0, 0);              // WCSNG
  LMIC.sysname_tx_rps = MAKERPS(SF10, BW125, CR_4_8, 0, 0);   // WCSNG
  #if (defined(ENABLE_CAD_SF) && (ENABLE_CAD_SF == 8))  // Pin mapping for Adafruit Feather M0 LoRa, etc.
    LMIC.sysname_cad_rps = MAKERPS(SF8, BW125, CR_4_8, 0, 0);  // WCSNG
  #elif (defined(ENABLE_CAD_SF) && (ENABLE_CAD_SF == 9)) 
    LMIC.sysname_cad_rps = MAKERPS(SF9, BW125, CR_4_8, 0, 0);  // WCSNG
  #elif (defined(ENABLE_CAD_SF) && (ENABLE_CAD_SF == 10)) 
    LMIC.sysname_cad_rps = MAKERPS(SF10, BW125, CR_4_8, 0, 0);  // WCSNG
  #elif (defined(ENABLE_CAD_SF) && (ENABLE_CAD_SF == 11)) 
    LMIC.sysname_cad_rps = MAKERPS(SF11, BW125, CR_4_8, 0, 0);  // WCSNG
  #elif (defined(ENABLE_CAD_SF) && (ENABLE_CAD_SF == 12)) 
    LMIC.sysname_cad_rps = MAKERPS(SF12, BW125, CR_4_8, 0, 0);  // WCSNG
  #endif

  LMIC.txpow = 21;
  LMIC.radio_txpow = 21;  // WCSNG

  // Set the LMIC CAD Frequencies
  LMIC.freq = 916000000;  // WCSNG
  LMIC.sysname_cad_freq_vec[0] = 922000000;
  LMIC.sysname_cad_freq_vec[1] = 916000000;

//  LMIC.sysname_cad_freq_vec[2] = 920000000 - 2000000;
//  LMIC.sysname_cad_freq_vec[3] = 920000000 - 4000000;

  // FSMA 
  LMIC.sysname_enable_cad = 1; //FSMA is sub category of CAD
  LMIC.sysname_kill_cad_delay = 1;
  LMIC.sysname_enable_FSMA = 1;
  LMIC.sysname_is_FSMA_node = 0;
  LMIC.sysname_enable_exponential_backoff = 0;
  LMIC.sysname_enable_variable_cad_difs = 0;

  LMIC.lbt_ticks = 100;
  LMIC.sysname_cad_difs = 2;
  LMIC.sysname_lbt_dbmin = -115;
  // LMIC.sysname_backoff_cfg1 = 12;
  // LMIC.sysname_backoff_cfg2 = 64;


  Serial.print("CAD Frequency: ");
  Serial.print(LMIC.sysname_cad_freq_vec[1]);
  Serial.print(", CAD RPS: ");
  Serial.println(ENABLE_CAD_SF);

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
  Serial.print("--- Hi, I always sense channel!");
  Serial.print("\n");

  // initialize runtime env
  os_init();

  cad_count = 0;
  intialize();
  os_setCallback(&arbiter_job, cad_func);
}

void loop() {
  // put your main code here, to run repeatedly:
  os_runloop_once();
}
