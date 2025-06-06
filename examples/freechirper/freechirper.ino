// From https://github.com/ucsdwcsng/fsma-sx127x-rp2040/tree/main/freechirper
#include <SPI.h>
#include <SX127XLT.h>

SX127XLT LT;

#define LED 13
#define NSS 16
#define NRESET 17
#define DIO0 21
#define IRQ_IN 25
#define LORA_DEVICE DEVICE_SX1276

#define TX_FREQ 918000000
#define SF 9
#define BW_NUM_kHz 125
#define BW LORA_BW_ ## BW_NUM_kHz
#define TX_POWER 20
// Number of consecutive chirps in a group
#define NUMBER_OF_CHIRPS 1
// For computing the delay between groups
#define DATA_SF 10
#define DATA_BW_kHz 125
#define DELAY_NDCHIRP_BETWEEN_FREE_CHIRPS 5

static uint32_t CHIRP_LEN_us;
static uint32_t DELAY_BETWEEN_CHIRPS_ms;

void setup(void)
{
    Serial.begin(9600);
    Serial.println(F("Chirp Transmitter"));
    SPI.begin();
    pinMode(LED, OUTPUT);
    if (!LT.begin(NSS, NRESET, DIO0, LORA_DEVICE))
    {
      Serial.println(F("No device responding"));
      return;
    }
    Serial.println(F("LoRa Device found"));
    LT.setupLoRa(TX_FREQ, 0, SF, LORA_BW_125, LORA_CR_4_8, LDRO_AUTO);
    CHIRP_LEN_us = 1000 * (1 << SF) / BW_NUM_kHz;
    DELAY_BETWEEN_CHIRPS_ms = (1 << DATA_SF) / DATA_BW_kHz * DELAY_NDCHIRP_BETWEEN_FREE_CHIRPS;
}

uint32_t last;

void loop(void)
{
    LT.setMode(MODE_STDBY_RC);
    uint8_t ptr = LT.readRegister(REG_FIFOTXBASEADDR);
    LT.writeRegister(REG_FIFOADDRPTR, ptr);
    // Set up to transmit a zero length packet
    LT.writeRegister(REG_PAYLOADLENGTH, 0);
    LT.setTxParams(TX_POWER, RADIO_RAMP_DEFAULT);

    // Check again if we should transmit
    // HIGH is free for FSMA and busy for BSMA
recheck:
    while (digitalRead(IRQ_IN) != HIGH);
    if (millis() - last < DELAY_BETWEEN_CHIRPS_ms)
      goto recheck;
    digitalWrite(LED, HIGH);
    // Start of tx time
    int start = micros();
    // Inform SX127x to start transmitting
    LT.writeRegister(REG_OPMODE, (MODE_TX + 0x88));
    // Wait in the MCU for the specified time
    while ((micros() - start) < NUMBER_OF_CHIRPS*CHIRP_LEN_us);
    // Inform SX127x to abort the transmission
    LT.setMode(MODE_STDBY_RC);
    // Record the time of the last chirp
    last = millis();
    digitalWrite(LED, LOW);
}
