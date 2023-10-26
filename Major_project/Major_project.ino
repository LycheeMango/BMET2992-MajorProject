
#include "scheduler.h"
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

// variables:
int threshold = 1600;
uint8_t low_pulse_threshold = 50;
uint8_t high_pulse_threshold = 100;
float pulse_rate = 0.0;
uint16_t sensor_reading = 0; 

// flags
bool device_status = false;
bool ppg_trigger = false;
uint8_t pulse_warning = NORMAL; // defined in ppg.h
bool pulse_train = false;


// classes:
Scheduler scheduler(20); // Create a scheduler with a 20ms interval

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

//*****************bluetooth set up for variable declarations*************
void inputParsing() {
//  inputParsing from switch
  device_switch.inputParse();
  ppg_switch.inputParse();
  device_status = device_switch.getLatch();
  ppg_trigger = ppg_switch.getLatch();

  status_led.LEDTrigger(device_status); // change led to device_status
  if (device_status) {
//    Serial.print("device on!");
//    digitalWrite(STATUS_LED_PIN, HIGH);
//    digitalWrite(BT_LED_PIN, HIGH);
//    digitalWrite(WARNING_LED_PIN, HIGH);
    
  }
}

void ppg_task() {
  if (device_status) {
//    Serial.println("\n Device is on. \n");
    
    if (ppg_trigger) {
//        Serial.println("\n Measuring PPG. \n");
        ppg_1.run();  //measure the ppg
        pulse_rate = ppg_1.getPulseRate(); //get the last calculated pulse_rate from ppg
        pulse_train = ppg_1.getPulseTrain(); //get the pulse_train from ppg
        
        status_led.LEDTrigger(pulse_train); // overwrite the status_led, use pulse_train to set status_led, 
        // so it flashes in sync with pulse

        // now check pulse_rate to see if warning is needed:
        pulse_warning = ppg_1.getPulseWarning();

        // testing: 
        pulse_warning == LOW_PULSE_WARNING;
        
        if (pulse_warning == LOW_PULSE_WARNING) {
          warning_led.flashSlow(); //high pulse detected
        }
        else if (pulse_warning == HIGH_PULSE_WARNING) {
          warning_led.flashFast(); //high pulse detected
        }
        
    }
    
  }
  else {
//    Serial.println("\n Device is turned off. \n");
    ppg_trigger = false;
  }

}

void BT_task() {
  sensor_reading = ppg_1.getSensorReading(); // get the latest sensor reading
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
}

void setup() {
  /*
** Add task to the scheduler
** 20 ms tasks:
** 1. inputParsing() with switches >> output flags >> device_status, ppg_trigger (True: measure ppg
** 2. ppg_task() >> if ppg_trigger then measure ppg and flash LEDs accordingly
*/
  scheduler.addTask(inputParsing); // do input parsing with switches; 
  scheduler.addTask(ppg_task); // do ppg measuring according to the flag set by input
  scheduler.addTask(BT_task); 

  // all 1sec tasks:
  
  
  scheduler.initLastTick();

  

  pinMode(SWITCH_PIN_1, INPUT);
  pinMode(SWITCH_PIN_2, INPUT);
  pinMode(SENSOR_PIN, INPUT);


  pinMode(WARNING_LED_PIN, OUTPUT);
  pinMode(BT_LED_PIN, OUTPUT);
  pinMode(STATUS_LED_PIN, OUTPUT);
  
  // Other setup code
  Serial.begin(BAUD);
  Serial.println("\nTestScheduler\n");


  //***************bluetooth set up for initialization**********
  Serial.println("****************************");
  Serial.println("* ESP Bluetooth SLAVE DEMO *");
  Serial.println("****************************");
  Serial.println("\nNow run the host program...");

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
}

void loop() {
  // Run the scheduler
  scheduler.run();
  // Other loop code
  
}
