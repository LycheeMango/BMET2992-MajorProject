
//#include "scheduler.h"
#include "switch.h"
#include "ppg.h"
#include "led.h"

// Bluetooth settings:
#include "BluetoothSerial.h"
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif
#include "BluetoothSerial.h"
#include <Esp.h>

// pin:
#define SWITCH_PIN_1 33
#define SWITCH_PIN_2 34

#define WARNING_LED_PIN 32 // red
#define BT_LED_PIN 13 // blue
#define STATUS_LED_PIN 26 // green

//#define SENSOR_PIN 25 // defined in ppg.h


// const:
#define DEBOUNCE_CNT 5 
#define BAUD 115200 
#define BUFFER_LENGTH 50 // two seconds
#define FAST_FLASH_TIME 40000 //40ms
#define SLOW_FLASH_TIME 600000 // 240ms
#define TICK_20MSEC 20000
#define DEVICE_ON true
#define DEVICE_OFF false

// variables:
int threshold = 1600;
uint8_t low_pulse_threshold = 50;
uint8_t high_pulse_threshold = 100;
float pulse_rate = 0.0;
uint16_t sensor_reading = 0; 
unsigned long led_interval = 0;
unsigned long last_flash_time = 0;

// flags
bool device_status = false;
bool ppg_trigger = false;
uint8_t pulse_warning = NORMAL; // defined in ppg.h
bool pulse_train = false;
bool bt_connect = false;
bool status_led_state = false;


// classes:
//Scheduler scheduler(20); // Create a scheduler with a 20ms interval

  // two switches:
Button device_switch(1, DEBOUNCE_CNT, SWITCH_PIN_1);
Button ppg_switch(2, DEBOUNCE_CNT, SWITCH_PIN_2);

  // three LEDs:
LED warning_led(WARNING_LED_PIN);
LED bt_led(BT_LED_PIN);
LED status_led(STATUS_LED_PIN);

  // one PPG sensor:
PPG ppg_1(threshold, low_pulse_threshold, high_pulse_threshold);

BluetoothSerial SerialBT;

// variables for Bluetooth
String btName = "ESP32USleepYouDie"; // CHANGE NAME ACCORDINGLY!!!
bool connected; // checks if bluetooth is connected
const int packetSize = 50; // Number of data points in each packet
uint16_t sensorpacket[packetSize]; // Array to store raw sensor data
const unsigned long packetInterval = 1000; // 1000 ms = 1 second
unsigned long lastPacketTime = 0;
int packetsequence = 0;
int sensorindexvalue = 0; // index for each sensor value in the pack

// variables for 20ms ticks:
unsigned long last_tick_time;

void inputParsing() {
//  inputParsing from switch
  device_switch.inputParse();
  ppg_switch.inputParse();
  device_status = device_switch.getLatch();
  ppg_trigger = ppg_switch.getLatch();

  status_led.LEDTrigger(device_status); // change led to device_status
  if (device_status) {
    digitalWrite(STATUS_LED_PIN, HIGH); // if device on, turn on device_status LED
  }
}// end of inputParsing()


bool LEDTrigger(bool trigger, int pin){
  bool led_status = false;
  if (trigger) {
    digitalWrite(pin, HIGH);
    led_status = true;
  }
  else {
    digitalWrite(pin, LOW);
    led_status = false;  
  }  
  return led_status;
}

void ppgTask() {
  if (device_status) {
//    Serial.println("\n Device is on. \n");
    
    if (ppg_trigger) {
//        Serial.println("\n Measuring PPG. \n");
        ppg_1.run();  //measure the ppg
        pulse_rate = ppg_1.getPulseRate(); //get the last calculated pulse_rate from ppg
        pulse_train = ppg_1.getPulseTrain(); //get the pulse_train from ppg
        
        status_led.LEDTrigger(pulse_train); 
        if (pulse_train) {
          digitalWrite(STATUS_LED_PIN, HIGH);
          status_led.setState(DEVICE_ON);
        }
        else {
          digitalWrite(STATUS_LED_PIN, LOW);
          status_led.setState(DEVICE_OFF);
        }
       
        // now check pulse_rate to see if warning is needed:
        pulse_warning = ppg_1.getPulseWarning();
        bool state = false;
        unsigned long current_time = micros();
        if (pulse_warning == LOW_PULSE_WARNING) {
          Serial.print("low pulse warning!!!!\n");
          led_interval = SLOW_FLASH_TIME;
          if (current_time - last_flash_time >= led_interval) {
            last_flash_time = current_time;
            state = LEDTrigger(!warning_led.getState(), WARNING_LED_PIN);
            warning_led.setState(state);
          } 
        }
        else if (pulse_warning == HIGH_PULSE_WARNING) {
          Serial.print("high pulse warning!!!!\n");
          led_interval = FAST_FLASH_TIME;
          if (current_time - last_flash_time >= led_interval) {
            last_flash_time = current_time;
            state = LEDTrigger(!warning_led.getState(), WARNING_LED_PIN);
            warning_led.setState(state);
          } 
        }
        else {
          // normal
          Serial.print("normal!!!!\n");
          warning_led.setState(false);
          digitalWrite(WARNING_LED_PIN, LOW);
        }
        
    }
    
  }
  else {
//    Serial.println("\n Device is turned off. \n");
    ppg_trigger = false;
  }

}// end of ppgTask()




//--------------------------------------------------------------------------------
//-------------------------------------SET UP ()-------------------------------------


void setup() {
  // SET pin modes:
  pinMode(SWITCH_PIN_1, INPUT);
  pinMode(SWITCH_PIN_2, INPUT);
  pinMode(SENSOR_PIN, INPUT);
  pinMode(WARNING_LED_PIN, OUTPUT);
  pinMode(BT_LED_PIN, OUTPUT);
  pinMode(STATUS_LED_PIN, OUTPUT);
  
  // Other setup code
  Serial.begin(BAUD);
  Serial.println("\n Major Project \n");

  //***************bluetooth set up for initialization**********
  Serial.println("****************************");
  Serial.println("* ESP Bluetooth SLAVE DEMO *");
  Serial.println("****************************");
  Serial.println("\n Now run the host program...");

  //SerialBT.register_callback(btCallback);
  SerialBT.begin(btName); //Bluetooth device name
  if (!SerialBT.begin(btName))
  {
    Serial.println("An error occurred initializing Bluetooth");
  }
  else
  {
    Serial.println("Bluetooth initialized");
    Serial.println(btName);
  }
}// end of setup ()

//--------------------------------------------------------------------------------------
//-------------------------------------MAIN LOOP ()-------------------------------------

void loop() {

  unsigned long current_time_20tick = micros();

  // ********************* 20 ms tick***************
  if (current_time_20tick - last_tick_time > TICK_20MSEC)
  {
    last_tick_time = current_time_20tick;

    // 20 msec tasks:
    // 1. Input Parsing:
    inputParsing();
    // 2. run PPG;
    ppgTask();
       
    bt_connect = SerialBT.connected(); // updates the bluetooth connection status
    // change bt_led status
    if (bt_connect==true) {
      digitalWrite(BT_LED_PIN, HIGH);
      bt_led.setState(DEVICE_ON);
    }
    else {
      digitalWrite(BT_LED_PIN, LOW);
      bt_led.setState(DEVICE_OFF);
    }

    sensor_reading = ppg_1.getSensorReading();
    sensorpacket[sensorindexvalue] = sensor_reading;
    sensorindexvalue ++;
    if (sensorindexvalue >= packetSize) {
      sensorindexvalue = 0;

      SerialBT.print("Packet Sequence:");
      SerialBT.print(packetsequence);
      SerialBT.println();
      packetsequence++;
      
      SerialBT.print("RAW:");
      for (int i = 0; i < packetSize; i++) {
        SerialBT.print(sensorpacket[i]);
        if (i < packetSize - 1) {
          SerialBT.print(",");
        }
      }
      SerialBT.println(); // Send a newline character to indicate the end of the packet
    }
  } // 20ms tick finished


  //******************************************************************bluetooth set up for BPM************************>>>>>>>>>>>>>>>>>>>>>

  // 1 second tick for BPM
  if ((millis() - lastPacketTime) >= packetInterval)
  {
    lastPacketTime = millis();
    
    //**********sending BPM 
    SerialBT.print("BPM:");
    SerialBT.print(pulse_rate); // heart_rate is calculated within ppgTask()
    SerialBT.println(); // Send a newline character to indicate the end of the packet

    Serial.printf("BPM: %1.f\n", pulse_rate);
  }

} // loop finished
