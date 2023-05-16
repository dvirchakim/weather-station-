
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Adafruit_BMP280.h>


///////////////////////////////define otaa join keys deveui ,appey, *appeui-optional/////////////
static const u1_t PROGMEM APPEUI[8] = { 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00 };
void os_getArtEui(u1_t* buf) {
  memcpy_P(buf, APPEUI, 8);
}
static const u1_t PROGMEM DEVEUI[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
void os_getDevEui(u1_t* buf) {
  memcpy_P(buf, DEVEUI, 8);
}
static const u1_t PROGMEM APPKEY[16] = { 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
void os_getDevKey(u1_t* buf) {
  memcpy_P(buf, APPKEY, 16);
}
/////////////////////////////////////////////////////

static char data[256];  // Allocate buffer for data string
static osjob_t sendjob;


const char* sensor_names[] = { "outdoor temp", "indoor temp", "humidity", "wind speed", "wind direction", "uv level", "lux level" }; //define sensors names
float windSpeed;                    // variable to store the calculated wind speed
const int ldrPin = 34;              // LDR pin connected to analog pin 34
const int oneWireBus = 16;          // variable for outdoor temp sensors library 
const int wind_direction_pin = 35;  // select the analog input pin
int UVOUT = 15;                     //Output from the sensor
int REF_3V3 = 4;                    //3.3V power on the Arduino board
int ldrValue;                       // variable to store the LDR value
int sensorPin = 27;                 // define the analog input pin for wind speed
int wind_direction;                 //define the analog input pin for wind direction




OneWire oneWire(oneWireBus);         // Setup a oneWire instance to communicate with any OneWire devices

DallasTemperature sensors(&oneWire); // Pass our oneWire reference to Dallas Temperature sensor
Adafruit_BMP280 bmp;                 // I2C
///////////////////////////////////////////////////////


// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations, minor diviation).
const unsigned TX_INTERVAL = 60;         // setting transmitting intervals

const lmic_pinmap lmic_pins = {         // spi pins for lora transmitter
  .nss = 5,                             // chip select pin
  .rxtx = LMIC_UNUSED_PIN,       
  .rst = 17,                            // chip reset
  .dio = { 2, 14, LMIC_UNUSED_PIN },   // dio0: IRQ , dio1: send confirm 
};


void printHex2(unsigned v) {           // takes an unsigned integer and
  v &= 0xff;                           // extracts the least significant byte
  if (v < 16)
    Serial.print('0');
  Serial.print(v, HEX);                //prints its hexadecimal representation
}   

void onEvent(ev_t ev) {               // handle join evet session 
  Serial.print(os_getTime());
  Serial.print(": ");
  switch (ev) {
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
      {
        u4_t netid = 0;
        devaddr_t devaddr = 0;
        u1_t nwkKey[16];
        u1_t artKey[16];
        LMIC_getSessionKeys(&netid, &devaddr, nwkKey, artKey);
        Serial.print("netid: ");
        Serial.println(netid, DEC);
        Serial.print("devaddr: ");
        Serial.println(devaddr, HEX);
        Serial.print("AppSKey: ");
        for (size_t i = 0; i < sizeof(artKey); ++i) {
          if (i != 0)
            Serial.print("-");
          printHex2(artKey[i]);
        }
        Serial.println("");
        Serial.print("NwkSKey: ");
        for (size_t i = 0; i < sizeof(nwkKey); ++i) {
          if (i != 0)
            Serial.print("-");
          printHex2(nwkKey[i]);
        }
        Serial.println();
      }
      // Disable link check validation (automatically enabled
      // during join, but because slow data rates change max TX
      // size, we don't use it in this example.
      LMIC_setLinkCheckMode(0);
      break;
    /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_RFU1:
        ||     Serial.println(F("EV_RFU1"));
        ||     break;
        */
    case EV_JOIN_FAILED:
      Serial.println(F("EV_JOIN_FAILED"));
      break;
    case EV_REJOIN_FAILED:
      Serial.println(F("EV_REJOIN_FAILED"));
      break;
    case EV_TXCOMPLETE:
      Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
      if (LMIC.txrxFlags & TXRX_ACK)
        Serial.println(F("Received ack"));
      if (LMIC.dataLen) {
        Serial.print(F("Received "));
        Serial.print(LMIC.dataLen);
        Serial.println(F(" bytes of payload"));
      }
      // Schedule next transmission
      os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
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
    /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_SCAN_FOUND:
        ||    Serial.println(F("EV_SCAN_FOUND"));
        ||    break;
        */
    case EV_TXSTART:
      Serial.println(F("EV_TXSTART"));
      break;
    case EV_TXCANCELED:
      Serial.println(F("EV_TXCANCELED"));
      break;
    case EV_RXSTART:
      /* do not print anything -- it wrecks timing */
      break;
    case EV_JOIN_TXCOMPLETE:
      Serial.println(F("EV_JOIN_TXCOMPLETE: no JoinAccept"));
      break;

    default:
      Serial.print(F("Unknown event: "));
      Serial.println((unsigned)ev);
      break;
  }
}

void do_send(osjob_t* j) {
  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND) {
    Serial.println(F("OP_TXRXPEND, not sending"));

  } else {
    pinMode(UVOUT, INPUT);
    pinMode(REF_3V3, INPUT);
    Serial.println(F("Starting"));
    sensors.requestTemperatures();
    /////////////////////outdoortemp ///////////////////////
    delay(150);
    /////////////////////////////////////////////////////////
    wind_direction = analogRead(wind_direction_pin);  // read the value from the analog pin
    delay(100);                                       // wait for 100 milliseconds before reading again

    //////////////////////wind speed////////////////////////
    windSpeed = analogRead(sensorPin);  // read the analog sensor value
    delay(150);

    /////////////uv and bmp///////////////////////////////
    //Use the 3.3V power pin as a reference to get a very accurate output value from sensor
    Serial.println(("ML8511,BMP280 test"));
    unsigned status;
    status = bmp.begin(0x76);
    /* Default settings from datasheet. */
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                    Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                    Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                    Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                    Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

    sensors.begin();
    delay(150);

    //////////////////////////////////////////////////////
    float in_temp = bmp.readTemperature();
    float out_temp = sensors.getTempCByIndex(0);
    int uvLevel = averageAnalogRead(UVOUT);
    int refLevel = averageAnalogRead(REF_3V3);
    float outputVoltage = 3.3 / refLevel * uvLevel;
    ldrValue = analogRead(ldrPin);

    /////////////////////////////////////////////////////
    const int num_sensors = sizeof(sensor_names) / sizeof(sensor_names[0]);
    const float values[] = { out_temp, in_temp, 60, windSpeed, wind_direction, uvLevel, ldrValue };

    int data_len = 0;
    for (int i = 0; i < num_sensors; i++) {
      int n = snprintf(data + data_len, sizeof(data) - data_len, "%s=%.2f,", sensor_names[i], values[i]);
      if (n >= 0 && data_len + n < sizeof(data)) {
        data_len += n;
      } else {
        // Error: not enough space in buffer
        break;
      }
    }

    // Prepare upstream data transmission at the next possible time.
    LMIC_setTxData2(1, (uint8_t*)data, strlen(data), 0);
    Serial.print(F("Packet queued: "));
    Serial.println(data);
  }
}
// Next TX is scheduled after TX_COMPLETE event.


int averageAnalogRead(int pinToRead) {
  byte numberOfReadings = 8;
  unsigned int runningValue = 0;

  for (int x = 0; x < numberOfReadings; x++)
    runningValue += analogRead(pinToRead);
  runningValue /= numberOfReadings;

  return (runningValue);
}

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


void setup() {
  Serial.begin(9600);
  Serial.println(F("Starting"));

  // LMIC init
  os_init();
  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();

  // Start job (sending automatically starts OTAA too)
  do_send(&sendjob);
}

void loop() {
  os_runloop_once();
}
