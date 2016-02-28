

/**
* The MySensors Arduino library handles the wireless radio link and protocol
* between your home built sensors/actuators and HA controller of choice.
* The sensors forms a self healing radio network with optional repeaters. Each
* repeater and gateway builds a routing tables in EEPROM which keeps track of the
* network topology allowing messages to be routed to nodes.
*
* Created by Henrik Ekblad <henrik.ekblad@mysensors.org>
* Copyright (C) 2013-2015 Sensnology AB
* Full contributor list: https://github.com/mysensors/Arduino/graphs/contributors
*
* Documentation: http://www.mysensors.org
* Support Forum: http://forum.mysensors.org
*
* This program is free software; you can redistribute it and/or
* modify it under the terms of the GNU General Public License
* version 2 as published by the Free Software Foundation.
*
*******************************
*
* REVISION HISTORY
* Version 1.0 - Henrik Ekblad
*
* DESCRIPTION
* The W5100 MQTT gateway sends radio network (or locally attached sensors) data to your MQTT broker.
* The node also listens to MY_MQTT_TOPIC_PREFIX and sends out those messages to the radio network
*
* LED purposes:
* - To use the feature, uncomment WITH_LEDS_BLINKING in MyConfig.h
* - RX (green) - blink fast on radio message recieved. In inclusion mode will blink fast only on presentation recieved
* - TX (yellow) - blink fast on radio message transmitted. In inclusion mode will blink slowly
* - ERR (red) - fast blink on error during transmission error or recieve crc error
*
* See http://www.mysensors.org/build/esp8266_gateway for wiring instructions.
* nRF24L01+  ESP8266
* VCC        VCC
* CE         GPIO4
* CSN/CS     GPIO15
* SCK        GPIO14
* MISO       GPIO12
* MOSI       GPIO13
*
* Not all ESP8266 modules have all pins available on their external interface.
* This code has been tested on an ESP-12 module.
* The ESP8266 requires a certain pin configuration to download code, and another one to run code:
* - Connect REST (reset) via 10K pullup resistor to VCC, and via switch to GND ('reset switch')
* - Connect GPIO15 via 10K pulldown resistor to GND
* - Connect CH_PD via 10K resistor to VCC
* - Connect GPIO2 via 10K resistor to VCC
* - Connect GPIO0 via 10K resistor to VCC, and via switch to GND ('bootload switch')
*
* Inclusion mode button:
* - Connect GPIO5 via switch to GND ('inclusion switch')
*
* Hardware SHA204 signing is currently not supported!
*
* Make sure to fill in your ssid and WiFi password below for ssid & pass.
*/

#include <EEPROM.h>
#include <SPI.h>
#include <OneWire.h>    //für Temperatursensoren DS18B20 see here
                        //http://technobabble.prithvitech.com/simplifying-multiple-onewire-buses-on-arduino/
                        //http://www.hacktronics.com/code/OneWire.zip
#include <DallasTemperature.h>

// Enable debug prints to serial monitor
#define MY_DEBUG 

// Enables and select radio type (if attached)
//#define MY_RADIO_NRF24
//#define MY_RADIO_RFM69

#define MY_GATEWAY_MQTT_CLIENT

// Set this nodes subscripe and publish topic prefix
#define MY_MQTT_PUBLISH_TOPIC_PREFIX "DOino<>MQTT" //
#define MY_MQTT_SUBSCRIBE_TOPIC_PREFIX "DOino<>MQTT" //

// Set MQTT client id
#define MY_MQTT_CLIENT_ID "DOino_1"

// W5100 Ethernet module SPI enable (optional if using a shield/module that manages SPI_EN signal)
//#define MY_W5100_SPI_EN 4  

// Enable Soft SPI for NRF radio (note different radio wiring is required)
// The W5100 ethernet module seems to have a hard time co-operate with 
// radio on the same spi bus.
#if !defined(MY_W5100_SPI_EN)
#define MY_SOFTSPI
#define MY_SOFT_SPI_SCK_PIN 14
#define MY_SOFT_SPI_MISO_PIN 16
#define MY_SOFT_SPI_MOSI_PIN 15
#endif  

// When W5100 is connected we have to move CE/CSN pins for NRF radio
#define MY_RF24_CE_PIN 5
#define MY_RF24_CS_PIN 6

// Enable these if your MQTT broker requires usenrame/password
//#define MY_MQTT_USER "username"
//#define MY_MQTT_PASSWORD "password"

// Enable MY_IP_ADDRESS here if you want a static ip address (no DHCP)
#define MY_IP_ADDRESS 192,168,111,99

// If using static ip you need to define Gateway and Subnet address as well
#define MY_IP_GATEWAY_ADDRESS 192,168,111,1
#define MY_IP_SUBNET_ADDRESS 255,255,255,0


// MQTT broker ip address.  
#define MY_CONTROLLER_IP_ADDRESS 192,168,111,22

// The MQTT broker port to to open 
#define MY_PORT 1883


//#############################################################################################
//#############################################################################################
//I/O-Kennung: hier wird die Funktion aller verwendbaren IO�s mit einer Kennziffer festgelegt
//dabei haben alle IO�s die Standardfunktionen plus spez. Sonderfunktionen
//     Standardfunktionen sind:
//      =andere Nutzg; '1' =dig_in; '2' =dig_out; '3' =1wire '4' =DHTxx; '5' =U_Schall

#include <stddef.h>
#include <stdint.h>
#include <MAX31855.h>
#include <Ethernet.h>
#include <MySensor.h>
#include <Bounce2.h>

typedef struct s_iomodus {
  mysensor_sensor sensorType;
  mysensor_data variableType;
  const char* description;
  const char* sensorVersion;
} iomodus_t;

iomodus_t iomodus[] = {
  /*Please fill in the RIGH Sensor Type and Variable Type from the SERIAL API
  * http://www.mysensors.org/download/
  {presentation, set/req, description}
  */
  { S_UNUSED,V_UNKNOWN,"RX","MY_SystemPin" },//D0 DO NOT TOUCH
  { S_UNUSED,V_UNKNOWN,"TX","MY_SystemPin" },//D1 DO NOT TOUCH
  { S_UNUSED,V_UNKNOWN,"free pin","tbd" }, //D2 : PWM?      = IR_Rx??  '=IMPULSEcount??; =tft?????;  
  { S_UNUSED,V_UNKNOWN,"free pin","tbd" }, //D3 : PWM?        = 433_Rx??   =IMPULSEcount;  =tft??;   
  { S_UNUSED,V_UNKNOWN,"free pin","tbd" }, //D4 : PWM?        = 433_Tx??   =lcd;      =tft??;   
  { S_UNUSED,V_UNKNOWN,"free pin","tbd" }, //D5 : PWM?                        =lcd;      =tft??;   
  { S_UNUSED,V_UNKNOWN,"free pin","tbd" }, //D6 : PWM?        = buzzer     =lcd;      =tft??;   
  { S_DIMMER,V_PERCENTAGE,"Test PWM","PWM" }, //D7 : PWM?                        =lcd;      =tft??;   
  { S_BINARY,V_STATUS,"RELAY D8","RELAY_1S" }, //D8 : PWM?                        =lcd;      =tft??;   
  { S_UNUSED,V_UNKNOWN,"free pin","tbd" }, //D9 : PWM?        = IR_Tx??   =lcd;      =tft??;   
  { S_UNUSED,V_UNKNOWN,"W5100","MY_SystemPin" }, //D10 :       = W5100 SS-Pin;
  { S_DOOR,V_ARMED,"Digital Ein","Schalter" }, //D11 : PWM?     
  { S_UNUSED,V_UNKNOWN,"free pin","tbd" }, //D12 : PWM?     
  { S_DIMMER,V_PERCENTAGE,"Test PWM","PWM" }, //D13 : PWM?     
  { S_UNUSED,V_UNKNOWN,"free pin","tbd" }, //D14/TX3 :   =ESP8266;    = rfid3;                
  { S_UNUSED,V_UNKNOWN,"free pin","tbd" }, //D15/RX3 :   =ESP8266;    = rfid3;                
  { S_UNUSED,V_UNKNOWN,"LEDS_BLINKING_FEATURE","MY_SystemPin" }, //D16/TX2 :   =ESP8266;    = rfid2;                
  { S_UNUSED,V_UNKNOWN,"free pin","tbd" }, //D17/RX2 :   =ESP8266;    = rfid2;                
  { S_DIMMER,V_VOLTAGE,"Pumpe1_T5","MOTORspeed" }, //D18/TX1 :   =IMPULSEcount;                 
  { S_UNUSED,V_UNKNOWN,"free pin","tbd" }, //D19/RX1 :   =IMPULSEcount;                               
  { S_DIMMER,V_VOLTAGE,"Pumpe2_T3","MOTORspeed" }, //D20/SDA :   =IMPULSEcount;   =I2C;                    
  { S_UNUSED,V_UNKNOWN,"free pin","tbd" }, //D21/SCL :   =IMPULSEcount;   =I2C;                    
  { S_BINARY,V_STATUS,"RELAY D8","RELAY_1S" }, //D22 : 
  { S_BINARY,V_STATUS,"RELAY D8","RELAY_1S" }, //D23 :
  { S_BINARY,V_STATUS,"RELAY D8","RELAY_1S" }, //D24 :       
  { S_BINARY,V_STATUS,"RELAY D8","RELAY_1S" }, //D25 :
  { S_BINARY,V_STATUS,"RELAY D8","RELAY_1S" }, //D26 :                                                   
  { S_BINARY,V_STATUS,"RELAY D8","RELAY_1S" }, //D27 :                                                   
  { S_BINARY,V_STATUS,"RELAY D8","RELAY_1S" }, //D28 :                                                   
  { S_BINARY,V_STATUS,"RELAY D8","RELAY_1S" }, //D29 :                                                   
  { S_BINARY,V_STATUS,"RELAY D8","RELAY_1S" }, //D30 : 
  { S_BINARY,V_STATUS,"RELAY D8","RELAY_1S" }, //D31 : 
  { S_BINARY,V_STATUS,"RELAY D8","RELAY_1S" }, //D32 : 
  { S_BINARY,V_STATUS,"RELAY D8","RELAY_1S" }, //D33 :
  { S_BINARY,V_STATUS,"RELAY D8","RELAY_1S" }, //D34
  { S_BINARY,V_STATUS,"RELAY D8","RELAY_1S" }, //D35
  { S_BINARY,V_STATUS,"RELAY D8","RELAY_1S" }, //D36
  { S_BINARY,V_STATUS,"RELAY D8","RELAY_1S" }, //D37
  { S_UNUSED,V_UNKNOWN,"MAX31855","CL_Ktype" }, //D38 CL KType CHECK for "MAX31855 NOT connected"
  { S_UNUSED,V_UNKNOWN,"MAX31855","SO_Ktype" }, //D39 SO KType CHECK for "MAX31855 NOT connected"
  { S_UNUSED,V_UNKNOWN,"free pin","tbd" }, //D40 NEU SOLL CS1 Read Sensor Nr 1 Ktype
  { S_UNUSED,V_UNKNOWN,"free pin","tbd" }, //D41 NEU SOLL CS2 Read Sensor Nr 2 Ktype
  { S_TEMP,V_TEMP,"dallas","DS18B20" }, //D42
  { S_UNUSED,V_UNKNOWN,"free pin","tbd" }, //D43
  { S_UNUSED,V_UNKNOWN,"free pin","tbd" }, //D44
  { S_UNUSED,V_UNKNOWN,"free pin","tbd" }, //D45
  { S_UNUSED,V_UNKNOWN,"free pin","tbd" }, //D46
  { S_UNUSED,V_UNKNOWN,"free pin","tbd" }, //D47
  { S_UNUSED,V_UNKNOWN,"free pin","tbd" }, //D48
  { S_UNUSED,V_UNKNOWN,"free pin","tbd" }, //D49
  { S_UNUSED,V_UNKNOWN,"W5100","MY_SystemPin" }, //D50 MISO       =W5100;   =CC3000;   ICSP-Stecker
  { S_UNUSED,V_UNKNOWN,"W5100","MY_SystemPin" }, //D51 MOSI       =W5100;   =CC3000;   ICSP-Stecker
  { S_UNUSED,V_UNKNOWN,"W5100","MY_SystemPin" }, //D52 SCK        =W5100;   =CC3000;   ICSP-Stecker
  { S_UNUSED,V_UNKNOWN,"free pin","tbd" }, //D53SS                       =CC3000;
  { S_UNUSED,V_UNKNOWN,"free pin","tbd" }, //D54 A0 : PWM_5V    =analog;  =NTC;  =tft??;  =lcd;
  { S_UNUSED,V_UNKNOWN,"free pin","tbd" }, //D55 A1 : PWM_5V    =analog;  =NTC;  =tft??;         
  { S_UNUSED,V_UNKNOWN,"free pin","tbd" }, //D56 A2 : PWM_5V    =analog;  =NTC;  =tft??;         
  { S_UNUSED,V_UNKNOWN,"free pin","tbd" }, //D57 A3 : PWM_5V    =analog;  =NTC;  =tft??;         
  { S_TEMP,V_TEMP,"Test NTC","NTC" }, //D58 A4 : PWM_5V    =analog;  =NTC;  =tft??;         
  { S_DIMMER,V_PERCENTAGE,"Test7 PWM 5V","PWM_5V" },    //D59 A5 : PWM_5V    =analog;  =NTC;                    
  { S_DIMMER,V_PERCENTAGE,"Test PWM 5V","PWM_5V" }, //D60 A6 : PWM_5V
  { S_DIMMER,V_PERCENTAGE,"Test7 PWM 5V","PWM_5V" }, //D61 A7 : PWM_5V
  { S_UNUSED,V_UNKNOWN,"free pin","tbd" }, //D62 A8 : PWM_5V    =analog;  =NTC;                    
  { S_UNUSED,V_UNKNOWN,"free pin","tbd" }, //D63 A9 : PWM_5V    =analog;  =NTC;                    
  { S_UNUSED,V_UNKNOWN,"free pin","tbd" }, //D64 A10 : PWM_5V   =analog;  =NTC;                    
  { S_UNUSED,V_UNKNOWN,"free pin","tbd" }, //D65 A11 : PWM_5V   =analog;  =NTC;                    
  { S_UNUSED,V_UNKNOWN,"free pin","tbd" }, //D66 A12 : PWM_5V   =analog;  =NTC;                    
  { S_UNUSED,V_UNKNOWN,"free pin","tbd" }, //D67 A13 : PWM_5V   =analog;  =NTC;                    
  { S_UNUSED,V_UNKNOWN,"free pin","tbd" }, //D68 A14 : PWM_5V   =analog;  =NTC;                    
  { S_UNUSED,V_UNKNOWN,"free pin","tbd" }, //D69 A15 : PWM_5V   =analog;  =NTC;                    
  { S_UNUSED,V_UNKNOWN,"free pin","tbd" }, //  =andere Nutzg;     =I2C;                               
  { S_UNUSED,V_UNKNOWN,"free pin","tbd" }, //  =andere Nutzg;     =I2C;                               
  { S_UNUSED,V_UNKNOWN,"free pin","tbd" }, //  =andere Nutzg;     =I2C;                               
  { S_UNUSED,V_UNKNOWN,"free pin","tbd" }, //  =andere Nutzg;     =I2C;                               
  { S_UNUSED,V_UNKNOWN,"free pin","tbd" }, //  =andere Nutzg;                                            
  { S_UNUSED,V_UNKNOWN,"free pin","tbd" }, //  =andere Nutzg;                                            
  { S_UNUSED,V_UNKNOWN,"free pin","tbd" }, //  =andere Nutzg;                                            
  { S_UNUSED,V_UNKNOWN,"free pin","tbd" }, //  =andere Nutzg;                                            
  { S_UNUSED,V_UNKNOWN,"free pin","tbd" }, //  =andere Nutzg;                                            
  { S_UNUSED,V_UNKNOWN,"free pin","tbd" } //  =andere Nutzg;                                            
};
const size_t iomodus_count = sizeof(iomodus) / sizeof(*iomodus);
//#############################################################################################
uint8_t relay_scenes[][2] = { 
  {0b00001111, 0b01111100}, //Digital Pins  29,28,27,26,25,24,23,22, 30,31,32,33,34,35,36,37
  {0b00000000, 0b00000000}, //here set your scenes
  {0b00000000, 0b00000000}, //
  {0b00000000, 0b00000000}, //
  {0b00000000, 0b00000000}, //
  {0b00000000, 0b00000000}, //
  {0b00000000, 0b00000000}, //
  {0b00000000, 0b00000000}, //
  {0b00000000, 0b00000000}, //
  {0b00000000, 0b00000000}, //
  {0b00000000, 0b00000000}, //
  {0b00000000, 0b00000000}, //
  {0b00000000, 0b00000000}, //
  {0b00000000, 0b00000000}, //
  {0b00000000, 0b00000000}, //
  {0b00000000, 0b00000000} //
};
const size_t relay_scenes_count = sizeof(relay_scenes) / sizeof(*relay_scenes);

//''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
#define RELAY_ON 0  // GPIO value to write to turn on attached RELAY (SSR RELAY)
#define RELAY_OFF 1 // GPIO value to write to turn off attached RELAY (SSR RELAY)
#define RELAY_STD 0 // Value for NORMAL (no power) Position of attached RELAY (Blue Relay)
#define RELAY_ACT 1 // Value for ACTIVE (powered) Position of attached RELAY (Blue Relay)
#define PORT_A_MASK 0xFF // Mask for Port A all if 0-Pin will NOT be affected whith PORTA=...function
#define PORT_C_MASK 0xFF // Mask for Port C all if 0-Pin will NOT be affected whith PORTC=...function
uint8_t last_relstate[relay_scenes_count][2]={};

//''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
// Flash leds on rx/tx/err
/*
#define MY_LEDS_BLINKING_FEATURE
// Set blinking period
#define MY_DEFAULT_LED_BLINK_PERIOD 300

// Enable inclusion mode
#define MY_INCLUSION_MODE_FEATURE
// Enable Inclusion mode button on gateway
#define MY_INCLUSION_BUTTON_FEATURE
// Set inclusion mode duration (in seconds)
#define MY_INCLUSION_MODE_DURATION 60 
// Digital pin used for inclusion mode button
#define MY_INCLUSION_MODE_BUTTON_PIN  3 

#define MY_DEFAULT_ERR_LED_PIN 16  // Error led pin
#define MY_DEFAULT_RX_LED_PIN  16  // Receive led pin
#define MY_DEFAULT_TX_LED_PIN  16  // the PCB, on board LED
*/
//''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
#define lcd_display


//*********************************************************************************************
//LAST_Variables
bool last_bool_value[iomodus_count];
int last_int_value[iomodus_count];
float last_float_value[iomodus_count];
double last_double_value[iomodus_count];
unsigned long last_ulong_value[6];

//*********************************************************************************************
//Variables for Loop
unsigned long next_full_loop = 0;
unsigned long delta_time = 3600000; // jede Stunde werden alle Inputs aktualisiert
boolean complete_loop = 1; // wenn 1, dann einmal komplett durchlaufen
unsigned long next_Time[iomodus_count];
unsigned long next_uTime[iomodus_count];

//*********************************************************************************************
//Variables for Display
/*
int x, x_alt;
byte zeile_pointer[6];
String zeile_data[6] = { "     ","     ","     ","     ","     ","     " };
String display_zeile_alt[6], display_zeile[6];
String taster;

char zeichen, buffer[50];
boolean fob_da = 0;
String zeich, fob_hex, fob_dec, Name, lcd_rfid_message, oeffner, Value;
unsigned long fob_zahl, time_rfid3 = 0, time_rfid2 = 0;
byte zeichen_zahl;
*/
//*********************************************************************************************
//+++++++++++++++++++++++Variables for Temp sensors+++++++++++++++++++++++
//+++++++++++++++++++++++Variables for KType
float Temp_offset[iomodus_count]={9999};
int ktype_LPF_rate[iomodus_count]={9999};
float LPF_ktype;
int clPin, doPin;
#define delta_Ktype 0.0////DELTA_in �C
//+++++++++++++++++++++++Variables for NTC
//float tempNTC and ANLOG;
const float B_wert = 3950; //aus dem Datenblatt des NTC //<<user-eingabe<<
const float Tn = 298.15; //25�Celsius in �Kelvin
const float Rv = 10000; //Vorwiderstand
const float Rn = 10000; //NTC-Widerstand bei 25�C
float Rt;
const int delta_analog = 2;
//+++++++++++++++++++++++Variables for Onewire DS18XXX
//const int oneWirePinsCount = iomodus_count;
const int oneWirePinsCount=iomodus_count;
OneWire ds18x20[oneWirePinsCount];
DallasTemperature sensor[oneWirePinsCount];
const float delta_onewire = 0.2; //Deltas f�r Sendeausl�sung
const float delta_ntc = 0.5; //in �C

//*********************************************************************************************
//Variables for MOTOR
#include <TimerFive.h>
#include <TimerThree.h>

volatile int T3;               // Variable to use as a counter of dimming steps. It is volatile since it is passed between interrupts
volatile int T5;
//int AC_pin_T5 = 18;              // Output to Opto Triac HARDCODED
//int AC_pin_T3 = 20;              // Output to Opto Triac HARDCODED           
int mot_speed_T3 = 100;           // in % Speed level (0-100)  0 = off, 100 = full speed
int mot_speed_T5 = 100;           // in % Speed level (0-100)  0 = off, 100 = full speed
                                  //1 Second = 1.000.000
int freqStep = 20000;             // Means 20ms ONE FULL wave. Every 20ms we check if we should switch SSR off to get the right speed
                                  //if speed is 50 (50%) we set Pin HIGH every 200ms, but after 5x calling Timer (i=50)Time = 100ms we set PIN low for next 5x calling of timer)
                                  //if speed is 10 (10%) we set Pin HIGH every 200ms, but after 1x calling Timer (i=10)Time = 20ms we set PIN low for next 9x calling of timer)
//*********************************************************************************************
//Variables for Flow Sensor /IMPULSE COUNTER
//PULSEvalue = PULSEcounter[offset - i ] / DEVIDEfactor[offset - i ];
// f.e. f=4.8* q(l/min)-> q(l/min)=f/4.8->DEVIDEfactor=4.8
const float delta_counter = 5; //in counter inkrement
volatile unsigned long PULSEcounter[6] = 
{ 
 0, //Count status for D2 -Impulse Input after RESET 
 0, //Count status for D3 -Impulse Input after RESET
 0, //Count status for D21-Impulse Input after RESET
 0, //Count status for D20-Impulse Input after RESET
 0, //Count status for D19-Impulse Input after RESET
 0 //Count status for D18-Impulse Input after RESET
}; 
//hier wird der DEVIDEfactor für die Impulszaehler festgelegt
const int PULSEdivider[6] = 
{6, //DEVIDEfactor D2 :                                                 <<user INPUT
 1, //DEVIDEfactor D3 :                                                 <<user INPUT
 1, //DEVIDEfactor D21 :                                                <<user INPUT
 1, //DEVIDEfactor D20 :                                                <<user INPUT
 1, //DEVIDEfactor D19 :                                                <<user INPUT
 1, //DEVIDEfactor D18 :                                                <<user INPUT
}; 
//#############################################################################################

/*
double ppl = ((double)PULSE_FACTOR)/1000;        // Pulses per liter

volatile unsigned long pulseCount = 0;   
volatile unsigned long lastBlink = 0;
volatile double flow = 0;  
boolean pcReceived = false;
unsigned long oldPulseCount = 0;
unsigned long newBlink = 0;   
double oldflow = 0;
double volume =0;                     
double oldvolume =0;
unsigned long lastSend =0;
unsigned long lastPulse =0;`*/
//*********************************************************************************************
//Variables for 

//*********************************************************************************************
//Variables for Digital Input
Bounce debouncer = Bounce();


//*************************************************************************************




void presentation()
{
  // Send the sketch version information to the gateway and Controller
  sendSketchInfo("MQTTduino by JR", "1.3");
  //all Ports
  for (int i = 0; i<iomodus_count; i++)
  { 
    MyMessage msg(i, iomodus[i].variableType);
//    Serial.println(iomodus[i].sensorType);
//    Serial.println(i);
    if (iomodus[i].description != "free pin") 
    {
      present(i, iomodus[i].sensorType, iomodus[i].description, false);
      //Send a et message which shows in controller where Warnings will come
      send(msg.setType(77).set("for Warnings"), false);
    }
  }
}
//*********************************************************************************************

void receive(const MyMessage &message)
{
  //Serial.println(message.getInt());
  //Serial.println(message.getFloat());
  //Serial.println(message.getBool());
    if (message.isAck()) 
    {
    Serial.println("This is an ack from gateway");
    }

  int childPin = message.sensor;
  //  Serial.println(message.getBool());


  for (int i = 0; i<iomodus_count; i++) ////search All Sensor Variables and set them
  {
    MyMessage msg(i, iomodus[i].variableType);
    while (iomodus[i].sensorType == S_UNUSED) { i++; }  //do not waste time to check UNUSED pins

                              //********************************IF VAR1 = Temp OFFSET komming from Server*************************************** 
    if (message.type == V_VAR1)
    {
      if ((childPin == i) && (iomodus[i].sensorType == S_TEMP) && (iomodus[i].sensorVersion == "MAX31855"))
      {
        Temp_offset[i] = message.getFloat();
      }
       
    }
    if (message.type == V_VAR1)
    {
      if ((childPin == i) && (iomodus[i].sensorType == S_TEMP) && (iomodus[i].sensorVersion == "NTC"))
      {
        Temp_offset[i] = message.getFloat();
      }
    }
    //********************************IF VAR2 = LPF (Low Pass Filter) comming************************************** 
    if (message.type == V_VAR2)
    {
      //***************Low Pass Filter**************
      if ((childPin == i) && (iomodus[i].sensorType == S_TEMP) && (iomodus[i].sensorVersion == "MAX31855") && (message.getFloat() <= 1.0))
      {
        ktype_LPF_rate[i] = message.getFloat();
      }
    }
    //********************************IF V_STATUS or V_LIGHT existst*************************************** 
    if (message.type == V_STATUS)
    {
    //********************************IF RELAY**************************************
      if ((childPin == i) && (iomodus[i].sensorType == S_BINARY) && (iomodus[i].sensorVersion == "RELAY"))
      {
        if (loadState(i) != message.getBool())
        {
          saveState(i, message.getBool());
          digitalWrite(i, message.getBool() ? RELAY_ON : RELAY_OFF);
          // Store state in eeprom
          Serial.println("Current State of Relay on PIN: ");
          Serial.println(i);
          Serial.println("changes to:");
          Serial.println(message.getBool());
          Serial.println();// Print the Value to serial
          send(msg.setSensor(i).set(message.getBool()==HIGH ? 1 : 0),false);
         }
      }
    }
    //********************************IF V_PERCENTAGE or V_DIMMER existst*************************************** 
    if ((childPin == i) && (message.type == V_PERCENTAGE) && (message.getFloat() != last_int_value[i])&&(message.getInt()<=100))
    {
//     Serial.println("im pwm");
//     Serial.println(message.getInt());
      //***************PWM Rate 0 to 100 (%)**************  
      if ((childPin == i) && (iomodus[i].sensorVersion == "PWM") && (i <= 13))
      {
        pinMode(i, OUTPUT);
        analogWrite(i, message.getInt()*2.55); // Set Value 0-255
        last_int_value[i] = message.getInt();
      }
      //http://www.instructables.com/id/Analog-Output-Convert-PWM-to-Voltage/
      if ((childPin == i) && (iomodus[i].sensorVersion == "PWM_5V") && (i >= 54) && (i <= 69))
      {
        pinMode(i, OUTPUT);
        analogWrite(i, message.getInt()*2.55); // Set Value 0-255
        last_int_value[i] = message.getInt();
      }

    }
  }

  //*********************************************************************************************
  //*************************************************************************************
}
void setup()
{
//   DDRA = 0b11111111; // set digital  22- 28 to output
//   DDRC = 0b11111111; // set digital  29-37 to output
  for (int i = 0; i<iomodus_count; i++) //search All Sensor system Ports and set them in the RIGHT way
  {

    while (iomodus[i].sensorVersion == "MY_SystemPin") { i++; }  //do not waste time to check MY_SystemPin's
    MyMessage msg(i, iomodus[i].variableType);
                                   //*************************************************************************************  
                                   //*************************************************************************************  
                                   //********************************Setup MAX31855*************************************** 
    if ((iomodus[i].description == "MAX31855") && (iomodus[i].sensorVersion == "CL_Ktype"))
    {
      clPin = i;
    }
    if ((iomodus[i].description == "MAX31855") && (iomodus[i].sensorVersion == "SO_Ktype"))
    {
      doPin = i;
    }

    //********************************Setup RELAY***************************************     
    if ((iomodus[i].sensorType == S_BINARY) && (iomodus[i].sensorVersion == "RELAY"))
    {

      pinMode(i, OUTPUT);
      // Set RELAY to last known state (using eeprom storage) 
      digitalWrite(i, loadState(i) ? RELAY_ON : RELAY_OFF);
//      send(msg.set(relstate ? RELAY_ON : RELAY_OFF), false);
      send(msg.setSensor(i).set(loadState(i)==HIGH ? 1 : 0),false);
    }
    //********************************Setup FLOWmeter***************************************   
 
    //f=4.8* q( l/min) Fehler:& plusmn; 2% Spannung: 3.5-24vdc, Strom nicht mehr als 10ma, 450 Ausgang Impulse/Liter,
    if ((iomodus[i].sensorType == S_WATER) && (iomodus[i].sensorVersion == "IMPULSEcount"))
    {
      if ((PULSEdivider[0] > 0) && (i == 2)) 
      {pinMode(2, INPUT_PULLUP); attachInterrupt(0, ISR_0, FALLING);}
      if ((PULSEdivider[1] > 0) && (i == 3)) 
      {pinMode(3, INPUT_PULLUP); attachInterrupt(1, ISR_1, FALLING);} 
      if ((PULSEdivider[2] > 0) && (i == 21)) 
      {pinMode(21, INPUT_PULLUP); attachInterrupt(2, ISR_2, FALLING);}
//      if ((PULSEdivider[3] > 0) && (i == 20)) 
//      {pinMode(20, INPUT_PULLUP); attachInterrupt(3, ISR_3, FALLING);}
      if ((PULSEdivider[4] > 0) && (i == 19)) 
      {pinMode(19, INPUT_PULLUP); attachInterrupt(4, ISR_4, FALLING);}
//      if ((PULSEdivider[5] > 0) && (i == 18)) 
//      {pinMode(18, INPUT_PULLUP); attachInterrupt(5, ISR_5, FALLING);}
    }

    //********************************Setup MOTOR-Speed_Control***************************************   
 
    if ((iomodus[i].sensorType == S_WATER) && (iomodus[i].sensorVersion == "MOTORspeed"))
    {
      if (i == 18)
      {
        pinMode(18, OUTPUT);                          // Set the Triac pin as output
        Timer5.initialize(freqStep);                      // Initialize TimerOne library for the freq we need
        Timer5.attachInterrupt(off_check_T5, freqStep);      // 
      }
      if (i == 20)
      {
        pinMode(20, OUTPUT);                          // Set the Triac pin as output
        Timer3.initialize(freqStep);                      // Initialize TimerOne library for the freq we need
        Timer3.attachInterrupt(off_check_T3, freqStep);      // 
      }  

    }
        
    //********************************Setup ONEWire***************************************
    if ((iomodus[i].sensorType == S_TEMP) && (iomodus[i].sensorVersion == "DS18B20"))
    { 
      DeviceAddress deviceAddress;
      ds18x20[i].setPin(i);
      sensor[i].setOneWire(&ds18x20[i]);
      // Startup up the OneWire library
      sensor[i].begin();
      if (sensor[i].getAddress(deviceAddress, 0)) sensor[i].setResolution(deviceAddress, 9);

    }
    //********************************Setup ANALOG INPUTS for READING VOLTAGE***************************************      
     if ((iomodus[i].sensorType == S_MULTIMETER) && (iomodus[i].sensorVersion == "VOLTAGE") && (i >= 54) && (i <= 69))//handling NTC Sensor
     {pinMode(55, INPUT_PULLUP);}
    
    //********************************Setup ???*************************************** 
    /*  for (int RELAY=1, pin=RELAY_1; RELAY<=NUMBER_OF_RELAYS;RELAY++, pin++) {
    // Then set RELAY pins in output mode
    pinMode(pin, OUTPUT);
    // Set RELAY to last known state (using eeprom storage)
    digitalWrite(pin, loadState(RELAY)?RELAY_ON:RELAY_OFF);
    }
    /*
    for (int ktype=1, pin=RELAY_1; ktype<=NUMBER_OF_RELAYS;ktype++, pin++) {
    // Then set RELAY pins in output mode
    pinMode(pin, OUTPUT);
    // Set RELAY to last known state (using eeprom storage)
    digitalWrite(pin, loadState(ktype)?RELAY_ON:RELAY_OFF);
    }
    */
//  apply_configuration(0);
  }

}
//*********************************************************************************************

void loop()
{
  complete_loop = 0;
  if (millis() > next_full_loop) //mindestens jede Stunde eine komplette Aktualisierung
  {
    complete_loop = 1; next_full_loop = millis() + delta_time;
    if (next_full_loop < millis()) { complete_loop = 0; } //wichtig wegen Zahlensprung
                                //von millis() alle 50 Tage

  }
  //*********************************************************************************************
  //*********************************************************************************************

  for (int i = 0; i < iomodus_count; i++) //behandlung aller Ports D2 bis D69
  {

    while (iomodus[i].sensorType == S_UNUSED) { i++; }  // unbenutzte pins �berspringen

    MyMessage msg(i, iomodus[i].variableType);

                              //datenempfang(); //nach jeder Messung auf Datenempfang schalten
                              //display_data(); //display ausgeben und abfragen

                              //********************************************************************************************
                              // DIGITAL INPUTS ON THE BOARD 
    if (
      (iomodus[i].sensorType == S_DOOR) ||
      (iomodus[i].sensorType == S_MOTION) ||
      (iomodus[i].sensorType == S_WATER_LEAK)
      )
    {

      if (millis() > next_Time[i])
      {
        next_Time[i] = next_Time[i] + 1000;  //digitaleing�nge nicht h�ufiger als alle 1000ms abfragen

        pinMode(i, INPUT_PULLUP);
        digitalWrite(i, HIGH);

        // After setting up the button, setup debouncer
        debouncer.attach(i);
        debouncer.interval(5);
        debouncer.update();
        // Get the update value
        int value = debouncer.read();
        
        if ((value != last_bool_value[i]) || complete_loop)
        {
          Serial.print("Current State of PIN: ");
          Serial.print(i);
          Serial.print(" changes to: ");
          Serial.print(value);
          Serial.println();// Print the Value to serial
                   //           send(msg.setSensor(i).set(value, 1));value==HIGH ? 1 : 0
          //MyMessage msg(i, iomodus[i].variableType);
          send(msg.set(value == HIGH ? 1 : 0),false);
          last_bool_value[i] = value;
        }

      }

    }

    //*********************************************************************************************
    // MAX31855 INPUTS ON THE BOARD 
    //https://github.com/RobTillaart/Arduino/tree/master/libraries/MAX31855/
    if ((iomodus[i].sensorType == S_TEMP) && (iomodus[i].sensorVersion == "MAX31855"))
    { int status;
      MAX31855 tc(clPin, i, doPin);
      if (millis() > next_Time[i])
      {
        next_Time[i] = next_Time[i] + 200;  //5 Readings per second

        MAX31855 tc(clPin, i, doPin);
        tc.begin();
        status = tc.read();
        //        Serial.println(status);

        if ((status == 0) ||
          (status == 2) ||
          (status == 4)) //Ktype NOK 0 = OK, 2 und 4 Kurzschluss, 8=NICHT VORHANDEN/NICHTS ANGESCHLOSSEN  
        {
          if (status == 2) //Short to GND
          {
            //        Serial.println("KType Short to GND");
            //MyMessage msg(i, iomodus[i].variableType);
            send(msg.setType(77).set("KType_SHORT_TO GND"), false);
          }
          if (status == 4) //Short to VCC
          {
            //        Serial.println("KType Short to VCC");
            //MyMessage msg(i, iomodus[i].variableType);
            send(msg.setType(77).set("KType_SHORT_TO VCC"), false);
          }
          tc.read();
          double ktype_int = tc.getInternal();
          if (ktype_int == 0.000)

          {
            Serial.println("MAX31855 NOT connected");
            //MyMessage msg(i, iomodus[i].variableType);
            send(msg.setType(77).set("MAX31855 NOT connected"), false);
          }

          if (ktype_int == 0) //Thermocouple not connected
          {
            //MyMessage msg(i, iomodus[i].variableType);
            Serial.println("Thermocouple NOT connected");
            send(msg.setType(77).set("KType_SHORT_TO GND"), false);
          }
          //********checking if Offset was set***********
          if (Temp_offset[i]==9999)
          {
           send(msg.setType(V_VAR1).set("OFFSET NOT SET"), false);
           send(msg.setType(V_VAR2).set("LPF_Rate 0___1 NOTE SET"), false);
          }
          //********if Offset was set/ Set the Offset***********          
          if (Temp_offset[i]!=9999)
          {
          tc.setOffset(Temp_offset[i]);
          }
          //********Read Temp***********    
          tc.read();
          float new_T_ktype = tc.getTemperature();
          LPF_ktype = ((LPF_ktype * ktype_LPF_rate[i]) + new_T_ktype) / (ktype_LPF_rate[i] + 1.0);
          //          Serial.println("KType �C");
          //          Serial.println(LPF_ktype);
        }
      }

      if (millis() > next_uTime[i])   //Ktype 1 sec (max 14 mal /sec)
      {
        next_uTime[i] = next_uTime[i] + 1000;  //1 Readings per second
      //*****************************************************************************

        tc.begin();
//        int status = tc.read();


        tc.read();
        float offset = tc.getOffset();

        
        if ((LPF_ktype > (last_float_value[i] + delta_Ktype)) || (LPF_ktype < (last_float_value[i] - delta_Ktype))
          || complete_loop)
        {
          //MyMessage msg(i, iomodus[i].variableType);
          send(msg.setSensor(i).set(LPF_ktype, 2),false);

          last_float_value[i] = LPF_ktype;
         }
        //****************************ALARM" STATUS von dem Ktype (0 = OK, 2 und 4 Kurzschluss, 8=NICHT VORHANDEN/NICHTS ANGESCHLOSSEN)
         
        else
        {
          if (status == 8) //Thermocouple not connected
          {
            //MyMessage msg(i, iomodus[i].variableType);
            Serial.println("Thermocouple NOT connected");
            send(msg.setType(77).set("KType_SHORT_TO GND"), false);
          }
          else {
            //MyMessage msg(i, iomodus[i].variableType);
            Serial.println("Something WRONG with MAX31855");
            send(msg.setType(77).set("!MAX31855 NOK!"), false);
          }
        }
      }
    }
    //*********************************************************************************************
    // NTC (Analog ONLY) INPUTS ON THE BOARD 

    if ((iomodus[i].sensorType == S_TEMP) && (iomodus[i].sensorVersion == "NTC") && (i >= 54) && (i <= 69))//handling NTC Sensor
    {
      if (millis() > next_Time[i])
      {
        next_Time[i] = next_Time[i] + 10000;  //Update Value every 10s 
        Rt = Rv / ((1024.0 / analogRead(i)) - 1.0);
        float tempNTC = (B_wert * Tn / (B_wert + (Tn * log(Rt / Rn)))) - Tn + 25.0 + Temp_offset[i];// Here Offset if needed
        
        if ((tempNTC > (last_float_value[i] + delta_ntc)) || (tempNTC < (last_float_value[i] - delta_ntc))
          || complete_loop)
        {
          //MyMessage msg(i, iomodus[i].variableType);
          Serial.print("tempNTC of Pin ");
          Serial.print(i);
          Serial.print(" is ");
          Serial.println(tempNTC);
          send(msg.setSensor(i).set(tempNTC, 2), false);

          last_float_value[i] = tempNTC;

        }
      }
    }
    //*********************************************************************************************
    // IMPULSEcounter (D2, D3, D18, D19, D20,D21  ONLY) INPUTS ON THE BOARD 
    
    if ((iomodus[i].sensorType == S_WATER) && (iomodus[i].sensorVersion == "IMPULSEcount")&&(i==2||i==3||(i>=18&&i<=21)))
    {byte offset =23; if (i ==2) {offset = 4;} if (i ==3) {offset = 6;}
     unsigned long PULSEvalue;
      PULSEvalue = PULSEcounter[offset - i ] / PULSEdivider[offset - i ];
       
       if ((PULSEdivider[offset -i] > 0) && ((PULSEvalue > (last_ulong_value[offset - i]+ delta_counter) || complete_loop))) 
         {
          Serial.print("Impulsvalue of Pin ");
          Serial.print(i);
          Serial.print(" is ");
          Serial.println(PULSEvalue);
          send(msg.setSensor(i).set(PULSEvalue),false);
          last_ulong_value[offset - i] = PULSEvalue;
          Serial.print("Impulscounter of Pin ");
          Serial.print(i);
          Serial.print(" is ");
          Serial.println(PULSEcounter[offset - i ]);
          send(msg.setType(77).set(PULSEcounter[offset - i ]), false);
         } 
    } 
    //*********************************************************************************************
      // ONEWire
      /*
      if ((iomodus[i].sensorType == S_TEMP) && (iomodus[i].sensorVersion == "DS18B20"))
      { if (millis() > next_Time[i])
        {next_Time[i] = next_Time[i] +10000;  //onewire nicht häufiger als alle 10s abfragen 
        pinMode(i, INPUT_PULLUP);
        digitalWrite(i,HIGH);
        OneWire ds(i); 
        #define DS18S20_ID 0x10
        #define DS18B20_ID 0x28 
        byte present = 0;   byte data[12];    byte addr[8];
        float temp_tur = 1000.0;
        if (!ds.search(addr)) { ds.reset_search(); temp_tur = -1000.0; } //find a device
        if ((OneWire::crc8( addr, 7) != addr[7]) && (temp_tur > -1000.0)) {temp_tur = -1000.0; }
        if ((addr[0] != DS18S20_ID && addr[0] != DS18B20_ID)&& (temp_tur > -1000.0)) 
         {temp_tur = -1000.0;}
        if (temp_tur > -1000.0) 
         {ds.reset(); 
          ds.select(addr); 
          ds.write(0x44, 1); // Start conversion
          //delay(850); // Wait some time...
          time_wait = millis() +850;                       //wait 2s , then get data
          present = ds.reset(); 
          ds.select(addr);
          ds.write(0xBE); // Issue Read scratchpad command
          for ( int k = 0; k < 9; k++) { data[k] = ds.read(); } // Receive 9 bytes
          temp_tur = ( (data[1] << 8) + data[0] )*0.0625; // Calculate temperature value 18B20
          //temp_tur = ( (data[1] << 8) + data[0] )*0.5 // Calculate temperature value 18S20
         }
        for (int m=0; m < zeilenzahl; m++)
         {if (zeile_pointer[m] == i) {zeile_data[m] = String(temp_tur,1);}
         }
        if ((temp_tur > (last_float_value[i] + delta_onewire)) 
                            || (temp_tur < (last_float_value[i] - delta_onewire)) || complete_loop) 
         {
          send(msg.setSensor(i).set(temp_tur, 2),false);
          last_float_value[i]=temp_tur;
         }
      }   
  }
  */
    //*********************************************************************************************
    // ONEWire with DALLAS
      if ((iomodus[i].sensorType == S_TEMP) && (iomodus[i].sensorVersion == "DS18B20"))
      { if (millis() > next_Time[i])
        {next_Time[i] = next_Time[i] +10000;  //onewire updates every 10s (10s is MINIMUM)
        sensor[i].requestTemperatures();
      // query conversion time and sleep until conversion completed
      //  int16_t conversionTime = sensor[i].millisToWaitForConversion(sensor[i].getResolution());
      // sleep() call can be replaced by wait() call if node need to process incoming messages (or if node is repeater)
      wait(500);
      float temperature=sensor[i].getTempCByIndex(0);
    Serial.print("Temperature for the sensor ");
    Serial.print(i);
    Serial.print(" is ");
    Serial.println(temperature);
      
      send(msg.setSensor(i).set(temperature, 2),false);
      last_float_value[i]=temperature;
        }}
    //*********************************************************************************************
      // ANALOG INPUTS
     if ((iomodus[i].sensorType == S_TEMP) && (iomodus[i].sensorVersion == "ANALOGinput") && (i >= 54) && (i <= 69))//handling NTC Sensor
     {if (millis() > next_Time[i])
      {next_Time[i] = next_Time[i] +1000;  //analogeingänge nicht häufiger als alle 1000ms abfragen 
       int ANALOGinput =analogRead(i);
       if ((ANALOGinput > (last_int_value[i] + delta_analog)) 
                || (ANALOGinput < (last_int_value[i] - delta_analog)) || complete_loop) 
         {Serial.print("ANALOGinput ");
          Serial.print(i);
          Serial.println(ANALOGinput);
          send(msg.setSensor(i).set(ANALOGinput), false);

          last_int_value[i] = ANALOGinput;
         }
      }
   }
    //*********************************************************************************************
      // ANALOG reading VOLTAGE
     if ((iomodus[i].sensorType == S_MULTIMETER) && (iomodus[i].sensorVersion == "VOLTAGE") && (i >= 54) && (i <= 69))//handling NTC Sensor
     {if (millis() > next_Time[i])
      {next_Time[i] = next_Time[i] +1000;  //analogeingänge nicht häufiger als alle 1000ms abfragen 
       int ANALOGinput =analogRead(i);
       if ((ANALOGinput > (last_int_value[i] + delta_analog)) 
                || (ANALOGinput < (last_int_value[i] - delta_analog)) || complete_loop) 
         {Serial.print("ANALOGinput Voltage on Pin ");
          Serial.print(i);
          Serial.print(" is ");
          Serial.println(ANALOGinput);
          send(msg.setSensor(i).set(ANALOGinput), false);

          last_int_value[i] = ANALOGinput;
         }
      }
   } 
    
   //*********************************************************************************************
      // ???



      

   //*********************************************************************************************
      // ???
      //**************************   ende loop  *****************************************************
    }
  }



  //#############################################################################################
  //#############################################################################################
  //#############################  Unterprogramme   #############################################
void ISR_0() //Interrupt D2
{PULSEcounter[0]++;}
void ISR_1() //Interrupt D3
{PULSEcounter[1]++;}
void ISR_2() //Interrupt D21
{PULSEcounter[2]++;}
void ISR_3() //Interrupt D20
{PULSEcounter[3]++;}
void ISR_4() //Interrupt D19
{PULSEcounter[4]++;}
void ISR_5() //Interrupt D18
{PULSEcounter[5]++;}

//SSR für den Motor am PIN 20*********************************************************************************************
void off_check_T3() {
    if(T3>=mot_speed_T3) 
    {                     
      digitalWrite(20, LOW);  // turn off SSR 
//      Serial.println("OFF");
//      Serial.println(T3);                 
    } 
    else 
    {
      digitalWrite(20, HIGH);
//      Serial.println("ON");
//      Serial.println(T3);           
    } 
    T3+=5;  
    if (T3>=100) {T3=0;}   
}
//SSR für den Motor am PIN 18*********************************************************************************************
void off_check_T5() {
    if(T5>=mot_speed_T5) 
    {                     
      digitalWrite(18, LOW);  // turn off SSR 
//      Serial.println("OFF");
//      Serial.println(T5);                 
    } 
    else 
    {
      digitalWrite(18, HIGH);
//      Serial.println("ON");
//      Serial.println(T5);           
    } 
    T5+=5;  
    if (T5>=100) {T5=0;}   
}
//*********************************************************************************************

/*
void apply_configuration(uint8_t pin)
{   
    uint8_t index = pin-22; // Offset for DigitalPins 0=Pin22   
    write_with_mask(&PORTA,PORT_A_MASK,relay_scenes[index][0]);
    last_relstate[index][0];
    write_with_mask(&PORTC,PORT_C_MASK,relay_scenes[index][1]);
    last_relstate[index][1];

    for (int i=22;i<38;i++)
    {     MyMessage msg(i, iomodus[i].variableType);
          int state=digitalRead (i);   
//          int state=bitRead(PORTA,3);
//        Serial.println(bitRead(PORTA,i));// not tested
          Serial.print("Realy Status for Pin ");
          Serial.print(i);
          Serial.print("on PortA ");
          Serial.println(PORTA);
          Serial.print("on PortC ");
          Serial.println(PORTC);
          Serial.print(" is ");
          Serial.println(state);

          send(msg.setSensor(i).set(state ==HIGH ? 1 : 0,false));
    }
}
//*********************************************************************************************
void write_with_mask(volatile uint8_t *p_register, uint8_t mask, uint8_t value)
{
    *p_register = (*p_register | (value & mask)) & (value | ~mask);
} 
*/
