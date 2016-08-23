/*******************************************************************************
 * Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman
 *
 * Permission is hereby granted, free of charge, to anyone
 * obtaining a copy of this document and accompanying files,
 * to do whatever they want with them without any restriction,
 * including, but not limited to, copying, modification and redistribution.
 * NO WARRANTY OF ANY KIND IS PROVIDED.
 *
 * This example sends a valid LoRaWAN packet with payload "Hello,
 * world!", using frequency and encryption settings matching those of
 * the The Things Network.
 *
 * This uses OTAA (Over-the-air activation), where where a DevEUI and
 * application key is configured, which are used in an over-the-air
 * activation procedure where a DevAddr and session keys are
 * assigned/generated for use with all further communication.
 *
 * Note: LoRaWAN per sub-band duty-cycle limitation is enforced (1% in
 * g1, 0.1% in g2), but not the TTN fair usage policy (which is probably
 * violated by this sketch when left running for longer)!

 * To use this sketch, first register your application and device with
 * the things network, to set or generate an AppEUI, DevEUI and AppKey.
 * Multiple devices can use the same AppEUI, but each device has its own
 * DevEUI and AppKey.
 *
 * Do not forget to define the radio type correctly in config.h.
 *
 *******************************************************************************/

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085_U.h>

Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);


// This EUI must be in little-endian format, so least-significant-byte
// first. When copying an EUI from ttnctl output, this means to reverse
// the bytes. For TTN issued EUIs the last bytes should be 0xD5, 0xB3,
// 0x70.
static const u1_t PROGMEM APPEUI[8]={ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}

// This should also be in little endian format, see above.
static const u1_t PROGMEM DEVEUI[8]={ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}

// This key should be in big endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). In
// practice, a key taken from ttnctl can be copied as-is.
// The key shown here is the semtech default key.
static const u1_t PROGMEM APPKEY[16] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
void os_getDevKey (u1_t* buf) {  memcpy_P(buf, APPKEY, 16);}

static uint8_t mydata[32];
static osjob_t sendjob;

const unsigned TX_INTERVAL = 5;

const lmic_pinmap lmic_pins = {
    .nss = 8,                       // chip select on feather (rf95module) CS
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 4,                       // reset pin
    .dio = {3, 6, LMIC_UNUSED_PIN}, // assumes external jumpers [feather_lora_jumper]
                                    // DIO0 is hardwired to GPIO3
                                    // DIO1 is jumpers to GPIO6
};


unsigned long convertSec(long time) {
    return (time * US_PER_OSTICK) / 1000;
}

void onEvent (ev_t ev) {
    Serial.print(convertSec(os_getTime()));
    Serial.print(": ");
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            Serial.println(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_BEACON_FOUND:
            Serial.println(F("EV_BEACON_FOUND"));
            break;
        case EV_BEACON_MISSED:
            Serial.println(F("EV_BEACON_MISSED"));
            break;
        case EV_BEACON_TRACKED:
            Serial.println(F("EV_BEACON_TRACKED"));
            break;
        case EV_JOINING:
            Serial.println(F("EV_JOINING"));
            break;
        case EV_JOINED:
            Serial.println(F("EV_JOINED"));
            break;
        case EV_RFU1:
            Serial.println(F("EV_RFU1"));
            break;
        case EV_JOIN_FAILED:
            Serial.println(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            Serial.println(F("EV_REJOIN_FAILED"));
            break;
            break;
        case EV_TXCOMPLETE:
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if(LMIC.dataLen) {
                // data received in rx slot after tx
                Serial.print(convertSec(os_getTime()));
                Serial.print(F(": Data Received: "));
                Serial.write(LMIC.frame+LMIC.dataBeg, LMIC.dataLen);
                Serial.println();
            }
            // Schedule next transmission
            Serial.print(convertSec(os_getTime()));
            Serial.println(": Scheduling next transmition");
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            Serial.print(convertSec(os_getTime()));
            Serial.println(": Next transmition complete");
            break;
        case EV_LOST_TSYNC:
            Serial.println(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            Serial.println(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            Serial.println(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            Serial.println(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            Serial.println(F("EV_LINK_ALIVE"));
            break;
         default:
            Serial.println(F("Unknown event"));
            break;
    }
}

void do_send(osjob_t* j){
    Serial.print(convertSec(os_getTime()));
    Serial.println(": Do Send" + LMIC_getSeqnoUp());

    /* Get a new sensor event */
    sensors_event_t event;
    bmp.getEvent(&event);
    float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;
    float pressure, temperature, altitude;

    /* Display the results (barometric pressure is measure in hPa) */
    if (event.pressure)
    {
      /* Display atmospheric pressure in hPa */
      pressure = event.pressure;
      bmp.getTemperature(&temperature);
      altitude = bmp.pressureToAltitude(seaLevelPressure, event.pressure, temperature);

//      Serial.print("Pressure: "); Serial.print(pressure); Serial.println(" hPa");
//      Serial.print("Temperature: "); Serial.print(temperature); Serial.println(" C");
//      Serial.print("Altitude: "); Serial.print(altitude); Serial.println(" m");
//      Serial.println("");
    }
    else
    {
      Serial.println("Sensor error");
    }


    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
        // Prepare upstream data transmission at the next possible time.
         ssize_t const nData = snprintf((char *)mydata, sizeof(mydata), "P: %d T: %d A: %d", (int)pressure, (int)temperature, (int)altitude );
         Serial.println(nData);
        LMIC_setTxData2(1, mydata, nData, 0);
        Serial.print(convertSec(os_getTime()));
        Serial.println(F(": Packet queued"));
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

void setup() {
    pinMode(13, OUTPUT);
    digitalWrite(13, LOW);
    while (!Serial); // wait for Serial to be initialized
    Serial.begin(9600);
    delay(100);     // per sample code on RF_95 test
    Serial.println(F("Starting"));

    /* Initialise the sensor */
    if(!bmp.begin()) {
      /* There was a problem detecting the BMP085 ... check your connections */
      Serial.print("Ooops, no BMP085 detected ... Check your wiring or I2C ADDR!");
      // while(1);
    }


    #ifdef VCC_ENABLE
    // For Pinoccio Scout boards
    pinMode(VCC_ENABLE, OUTPUT);
    digitalWrite(VCC_ENABLE, HIGH);
    delay(1000);
    #endif

    // LMIC init
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();


    // Disable link check validation
    LMIC_setLinkCheckMode(0);

    // Set data rate and transmit power (note: txpow seems to be ignored by the library)
    LMIC_setDrTxpow(DR_SF7,14);

    // Select SubBand
    LMIC_selectSubBand(1); // must align with subband on gateway. Zero origin

//    pinMode(3, INPUT_PULLUP);
    // Start job
    do_send(&sendjob);
}

void loop() {
    unsigned long now;
    now = millis();
    if ((now & 512) != 0) {
      digitalWrite(13, HIGH);
    }
    else {
      digitalWrite(13, LOW);

    }

    os_runloop_once();
}
