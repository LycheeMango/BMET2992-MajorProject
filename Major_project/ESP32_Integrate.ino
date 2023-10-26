#include "scheduler.h"
#include "switch.h"
#include "ppg.h"
//*************************************************************bluetooth set up for variable declarations********************************** >>>>>>>>>>>>
#include "BluetoothSerial.h"
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif
#include "BluetoothSerial.h"
#include <Esp.h>
//#include "btSupport.h"
String btName = "ESP32XX0H32"; // CHANGE NAME ACCORDINGLY!!!
bool connected; // checks if bluetooth is connected
BluetoothSerial SerialBT;

const int packetSize = 50; // Number of data points in each packet
uint16_t sensorpacket[packetSize]; // Array to store raw sensor data
const unsigned long packetInterval = 1000; // 1000 ms = 1 second
unsigned long lastPacketTime = 0;

int packetsequence = 0;
int sensorindexvalue = 0; // index for each sensor value in the pack
//*************************************************************LED set up for variable declarations********************************** >>>>>>>>>>>>
#include "Arduino.h"

//define constant
//PIN
#define SWITCH_PIN_1 33
#define SWITCH_PIN_2 34
//LEDs
#define WARNING_LED_PIN 32 
#define BT_CONNECTED_LED_PIN 13
#define HEARTBEAT_LED_PIN 26
//define other 
#define SENSOR_PIN 25 
#define BAUD 115200
//define LED section constants
#define BUFFER_LENGTH 50
#define TICK_20MSEC 20000
#define BASELINE_VALUE 1000 
#define HEARTBEAT_THRESHOLD 1600
#define HIGH_WARNING_FLASH_DURATION 100 
#define LOW_WARNING_FLASH_DURATION 600
#define DEBOUNCE_CNT 5  

//***********************************************************************set up variables***************************************** >>>>>>>>>>>>
//***********************************************************************Bluetooth set up variables***************************************** >>>>>>>>>>>>
uint16_t sensor_reading; 
unsigned long last_tick_time = 0;

// adaptive threshold variables
int threshold = 1600; // first tamporarily define the threshold, its value will change later accordingly
uint16_t adaptive_buffer[BUFFER_LENGTH];
uint8_t buffer_idx = 0;
uint16_t max_val;
uint16_t min_val;

// Pulse Train State variables
bool old_PT_state = true;     // The previous pulse train state
bool current_PT_state = true; // The current pulse train state

// Pulse period measurement variables
unsigned long pulseStartTime = 0; // Time when the pulse started
unsigned long pulsePeriod = 0;    // Pulse period in milliseconds

// Heart rate calculation variables
float heart_rate = 0.0; // Heart rate in BPM

//***********************************************************************LEDs set up variables***************************************** >>>>>>>>>>>>
// Flags
bool device_status = false;
bool high_p_warning = false;
bool low_p_warning = false;
bool bluetooth_connected = false;
// variables
uint8_t LOW_THRESHOLD = 50;
uint8_t HIGH_THRESHOLD = 100;
//***********************************************************************Switch set up variables***************************************** >>>>>>>>>>>>
uint8_t low_pulse_threshold = 50;
uint8_t high_pulse_threshold = 100;
float pulse_rate = 0.0;

// Flags
bool ppg_trigger = false;
uint8_t pulse_warning = NORMAL; // defined in ppg.h
bool pulse_train = false;

// classes:
Scheduler scheduler(20); // Create a scheduler with a 20ms interval
Button device_switch(1, DEBOUNCE_CNT, SWITCH_PIN_1);
Button ppg_switch(2, DEBOUNCE_CNT, SWITCH_PIN_2);
PPG ppg_1(threshold, low_pulse_threshold, high_pulse_threshold);

//***********************************************************************Switch set up variables end***************************************** >>>>>>>>>>>>
//*********************************************************************** set up variables end***************************************** >>>>>>>>>>>>

//***********************************************************************Switch tasks set up***************************************** >>>>>>>>>>>>
void task1() {
  device_switch.inputParse();
  ppg_switch.inputParse();
  device_status = device_switch.getLatch();
  ppg_trigger = ppg_switch.getLatch();
}

void task2() {
  if (device_status) {
    
//    Serial.println("\n Device is on. \n");
    
    if (ppg_trigger) {
//        Serial.println("\n Measuring PPG. \n");
        ppg_1.run();
        pulse_rate = ppg_1.getPulseRate();
        pulse_train = ppg_1.getPulseTrain();
        // if (pulse_train) >>> blink LED
      }
    }
  else {
//    Serial.println("\n Device is turned off. \n");
    ppg_trigger = false;
    }
  }
//***********************************************************************Switch tasks set up end***************************************** >>>>>>>>>>>>
//***********************************************************************void set up***************************************** >>>>>>>>>>>>
void setup() {
  // initialize pin modes
  pinMode(WARNING_LED_PIN, OUTPUT);
  pinMode(BT_CONNECTED_LED_PIN, OUTPUT);
  pinMode(HEARTBEAT_LED_PIN, OUTPUT);
  pinMode(SENSOR_PIN, INPUT);
  pinMode(SWITCH_PIN_1, INPUT);
  pinMode(SWITCH_PIN_2, INPUT);
  // start the Serial interface
  Serial.begin(BAUD);
  // Other Set up
  Serial.println("\nTestScheduler\n");
  // Add tasks to the scheduler
  scheduler.addTask(task1);
  scheduler.addTask(task2);
  scheduler.initLastTick();
  //******************************************************************bluetooth set up for initialization************************>>>>>>>>>>>>>>>>>>>>>
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
  //***********************************************************************bluetooth set up for initialization finished********************<<<<<<<<<<<<<<<<<<<<<

}

void loop() {
  // Run the scheduler
  scheduler.run();
  // Other loop code
  unsigned long current_time_20tick = micros(); 
  //***********************************************************************LEDs main task blocks ********************<<<<<<<<<<<<<<<<<<<<<
  if (current_time_20tick - last_tick_time > TICK_20MSEC) {
    last_tick_time = current_time_20tick;

    // 20 msec tasks:
    uint16_t sensor_reading = analogRead(SENSOR_PIN);
    sensorpacket[sensorindexvalue] = sensor_reading;
    sensorindexvalue++;
    
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
    //************************************************************bluetooth set up for raw data packets finished********************<<<<<<<<<<<<<<<<<<<<<
    //***************variable threshold *********************
    // calculate threshold
    adaptive_buffer[buffer_idx++] = sensor_reading; // increments the buffer index each time after this line in the loop runs
    if (buffer_idx >= BUFFER_LENGTH)
    {
      buffer_idx = 0; // if the index reaches over 100 (set buffer length), the index goes back to 0 and values are overwritten again starting from 0
    }

    // find max and min by comparing new values with the old ones, whichever one's bigger/smaller stays
    max_val = 0;
    min_val = 4095; // 12-bit ADC (10^12 -1)????????????????????

    for (int idx = 0; idx < BUFFER_LENGTH; idx++) // loops through every value in the buffer and compares with max/min
    {
      max_val = max(max_val, adaptive_buffer[idx]); // compares the two values and takes the bigger one
      min_val = min(min_val, adaptive_buffer[idx]);
      // apply and define the threshold: 80% betweenmax and min
      threshold = ((max_val - min_val) * 0.8 + min_val);
    }

    // print data
    //Serial.printf("inputvalue:%d\n ,thresholdval: %d\n", sensor_reading, threshold);

    //***************** pulse range & rate **************
    // Create a boolean signal based on the threshold
    bool threshold_exceeded = (sensor_reading > threshold);

    // Update pulse train states
    old_PT_state = current_PT_state;
    current_PT_state = threshold_exceeded; //update both based on threshold , in case where it's decreasing:
    //threshold exceeds = false: old PT state becomes 1 but current PT state becomes 0


    // Detect when threshold is exceeded (0 to 1 transition)
    if (current_PT_state == 1 && old_PT_state == 0)
    {
      // Calculate the pulse period
      unsigned long currentTime = millis(); // takes the time when the threshold is exceeded
      pulsePeriod = currentTime - pulseStartTime; // calculates how long it is between each detected pulse
      pulseStartTime = currentTime; //update pulse start time so it becomes a time marker for the last pulse

      // Calculate heart rate in BPM
      if (pulsePeriod > 0)
      {
        heart_rate = 60000.0 / (float)pulsePeriod;
        //Serial.printf("Heart rate: %.1f BPM\n", heart_rate);  // prints the BPM (don't need to print this in arduino?)

      } else
      {
        heart_rate = 0.0; // To avoid division by zero
      }
    }

  } // 20ms tick finished
  //******************************************************************bluetooth set up for BPM************************>>>>>>>>>>>>>>>>>>>>>

  // 1 second tick for BPM
  if ((millis() - lastPacketTime) >= packetInterval)
  {
    lastPacketTime = millis();
    
    //**********sending BPM 

    SerialBT.print("BPM:");
    SerialBT.print(heart_rate);
    SerialBT.println(); // Send a newline character to indicate the end of the packet

    Serial.printf("BPM: %1.f\n", heart_rate);
  }
  //**********************************************************************bluetooth set up for BPM finished********************<<<<<<<<<<<<<<<<<<<<<

  //***********************************************************************Heart Rate updating, Threshold, and device status blocks ********************<<<<<<<<<<<<<<<<<<<<<
  device_status = heart_rate > BASELINE_VALUE; 
  high_p_warning = heart_rate > HIGH_THRESHOLD; 
  low_p_warning = heart_rate < LOW_THRESHOLD; 

  if (device_status) {
    if (high_p_warning) {
      digitalWrite(WARNING_LED_PIN, HIGH_WARNING_FLASH_DURATION); 
    } else if (low_p_warning) {
      digitalWrite(WARNING_LED_PIN, LOW_WARNING_FLASH_DURATION); 
    } else {
      digitalWrite(WARNING_LED_PIN, HIGH); 
    }
  } else {
    digitalWrite(WARNING_LED_PIN, LOW); 
  }
  //***********************************************************************Bluetooth block ********************<<<<<<<<<<<<<<<<<<<<<
  if (bluetooth_connected) {
    digitalWrite(BT_CONNECTED_LED_PIN, HIGH); 
  } else {
    digitalWrite(BT_CONNECTED_LED_PIN, LOW); 
  }

  if (sensor_reading) {
    digitalWrite(HEARTBEAT_LED_PIN, HIGH);
    
    digitalWrite(HEARTBEAT_LED_PIN, LOW);
  }

  //***********************************************************************Bluetooth communication block ********************<<<<<<<<<<<<<<<<<<<<<
  SerialBT.print("Packet Sequence:");
  SerialBT.print(packetsequence);
  SerialBT.println();
  packetsequence++;

  SerialBT.print("RAW:");
  for (int i = 0; i < packetSize; i++) {
    SerialBT.print(sensorpacket[i]);
    if (i < packetSize - 1) {
      SerialBT.print(", ");
    }
  }
  SerialBT.println(); 
}


bool Heartbeatdata(uint16_t sensorData) { //note that its a placeholder!!
  return sensorData > HEARTBEAT_THRESHOLD;

}

void flashLED(int pin, unsigned int pulseDuration) {
  digitalWrite(pin, HIGH);
  digitalWrite(pin, LOW);
}
//******************************************************8****************bluetooth set up********************************** >>>>>>>>>>>>

int openEvt = 0;

void btCallback(esp_spp_cb_event_t event, esp_spp_cb_param_t *param)
//
// This function displays SPP events when they occur. This provides
// information on what is hapening on the bluetooth link.
//
//
{
  if (event == ESP_SPP_SRV_OPEN_EVT) {
    char buf[50];
    openEvt++;
    sprintf(buf, "Client Connected:%d", openEvt);
    Serial.println(buf);
    Serial.print("  Address = ");

    for (int i = 0; i < 6; i++)
    {
      sprintf(&(buf[i * 3]), "%02X:", param->srv_open.rem_bda[i]);
    }
    buf[17] = 0;
    SerialBT.println(buf); //6.3 and 4.4 graph, 4.1
  }


  if (event == ESP_SPP_INIT_EVT)
    Serial.println("ESP_SPP_INIT_EVT");
  else if (event == ESP_SPP_UNINIT_EVT)
    Serial.println("ESP_SPP_INIT_EVT");
  else if (event == ESP_SPP_DISCOVERY_COMP_EVT )
    Serial.println("ESP_SPP_DISCOVERY_COMP_EVT");
  else if (event == ESP_SPP_OPEN_EVT )
    Serial.println("ESP_SPP_OPEN_EVT");
  else if (event == ESP_SPP_CLOSE_EVT )
    Serial.println("ESP_SPP_CLOSE_EVT");
  else if (event == ESP_SPP_START_EVT )
    Serial.println("ESP_SPP_START_EVT");
  else if (event == ESP_SPP_CL_INIT_EVT )
    Serial.println("ESP_SPP_CL_INIT_EVT");
  else if (event == ESP_SPP_DATA_IND_EVT )
    Serial.println("ESP_SPP_DATA_IND_EVT");
  else if (event == ESP_SPP_CONG_EVT )
    Serial.println("ESP_SPP_CONG_EVT");
  else if (event == ESP_SPP_WRITE_EVT )
    Serial.println("ESP_SPP_WRITE_EVT");
  else if (event == ESP_SPP_SRV_OPEN_EVT )
    Serial.println("ESP_SPP_SRV_OPEN_EVT");
  else if (event == ESP_SPP_SRV_STOP_EVT )
    Serial.println("ESP_SPP_SRV_STOP_EVT");
  else
  {
    Serial.print("EV: ");
    Serial.println(event);
  };
}
