

//************************************Pulse rate module********************************
// first import raw signal from sensor
// draw raw graph & set up threshold
// if pulse higher than threshold then use these pulse to calculate BPM

// things from my part to send to python:
//1. raw input signals? 2. threshold values 3. BPM values (easier to just calculate those in arduino using pulse train state)?
// use 1 and 2 to draw graphs again in python


//*************************************************************bluetooth set up for variable declarations********************************** >>>>>>>>>>>>

#include "BluetoothSerial.h"
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif
#include "BluetoothSerial.h"
#include <Esp.h>
//#include "btSupport.h"

String btName = "ESP32XX0H32"; // CHANGE NAME ACCORDINGLY!!!

BluetoothSerial SerialBT;

bool btconnect = false; // checks if bluetooth is connected

const int packetSize = 50; // Number of data points in each packet
uint16_t sensorpacket[packetSize]; // Array to store raw sensor data
const unsigned long packetInterval = 1000; // 1000 ms = 1 second
unsigned long lastPacketTime = 0;

int packetsequence = 0;
int sensorindexvalue = 0; // index for each sensor value in the pack


//**********************************************************bluetooth set up varaiable declaration finished********************<<<<<<<<<<<<<<<<<<<<<


// define constant
#define LED_PIN 32
#define SWITCH_PIN 33
#define SENSOR_PIN 25
#define BAUD 115200
#define BUFFER_LENGTH 50 // two seconds
#define TICK_20MSEC 20000 // 20 milliseconds = 0.02 seconds

// define variables
uint16_t sensor_reading;

// holds tick time counts
unsigned long last_tick_time;


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


//************************************************


void setup() {
  // initialize pin modes
  pinMode(LED_PIN, OUTPUT);
  pinMode(SWITCH_PIN, INPUT);
  pinMode(SENSOR_PIN, INPUT);

  // start the Serial interface
  Serial.begin(BAUD);




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
}
//***********************************************************************bluetooth set up for initialization finished********************<<<<<<<<<<<<<<<<<<<<<

void loop()
{

  unsigned long current_time_20tick = micros();

  // ********************* 20 ms tick***************
  if (current_time_20tick - last_tick_time > TICK_20MSEC)
  {
    last_tick_time = current_time_20tick;

    // 20 msec tasks:

    bool btconnect = SerialBT.connected(); // updates the bluetooth connection status
    
    sensor_reading = analogRead(SENSOR_PIN);


    //******************************************************************bluetooth set up for raw data packets****************>>>>>>>>>>>>>>>>>>>

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

} // loop finished



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
