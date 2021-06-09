#include <Arduino.h>

//include the library code:
#include <LiquidCrystal.h>

// LCD Screen Setup
const int rs = 2, en = 3, d4 = 5, d5 = 6, d6 = 9, d7 = 10;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

// Joystick Setup
#define joyX A0
#define joyY A1
#define joySelect 4
boolean yes = false;
boolean no = false;
boolean clicked = false;

// Symptom tracking info
char symptoms[][16] = {{'H', 'e', 'a', 'd', 'a', 'c', 'h', 'e'}, {'M', 'u', 's', 'c', 'l', 'e', ' ', 'A', 'c', 'h', 'e', 's'}, {'S', 'o', 'r', 'e', ' ', 'T', 'h', 'r', 'o', 'a', 't'}, {'L', 'o', 's', 's', ' ', 'T', 'a', 's', 't', 'e', '/', 'S', 'm', 'e', 'l', 'l'}, {'S', 'h', 'o', 'r', 't', ' ', 'O', 'f', ' ', 'B', 'r', 'e', 'a', 't', 'h'}, {'C', 'h', 'e', 's', 't', ' ', 'P', 'a', 'i', 'n'}, {'C', 'h', 'i', 'l', 'l', 's'}, {'D', 'r', 'y', ' ', 'C', 'o', 'u', 'g', 'h'}};
int idx = 0;
int responses[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};

// Thermometer Setup
#define ADDR      0x5A
 
//EEPROM 32x16
#define TO_MAX    0x00
#define TO_MIN    0x01
#define PWM_CTRL  0x02
 
//RAM 32x16
#define RAW_IR_1  0x04
#define RAW_IR_2  0x05
#define TA        0x06
#define TOBJ_1    0x07
#define TOBJ_2    0x08
 
#define SYNC_PIN  2
 
static const uint32_t TWI_CLOCK = 100000;
static const uint32_t RECV_TIMEOUT = 100000;
static const uint32_t XMIT_TIMEOUT = 100000;

Twi *pTwi = WIRE_INTERFACE;
 
static void Wire_Init(void) {
  pmc_enable_periph_clk(WIRE_INTERFACE_ID);
  PIO_Configure(
  g_APinDescription[PIN_WIRE_SDA].pPort,
  g_APinDescription[PIN_WIRE_SDA].ulPinType,
  g_APinDescription[PIN_WIRE_SDA].ulPin,
  g_APinDescription[PIN_WIRE_SDA].ulPinConfiguration);
  PIO_Configure(
  g_APinDescription[PIN_WIRE_SCL].pPort,
  g_APinDescription[PIN_WIRE_SCL].ulPinType,
  g_APinDescription[PIN_WIRE_SCL].ulPin,
  g_APinDescription[PIN_WIRE_SCL].ulPinConfiguration);
 
  NVIC_DisableIRQ(TWI1_IRQn);
  NVIC_ClearPendingIRQ(TWI1_IRQn);
  NVIC_SetPriority(TWI1_IRQn, 0);
  NVIC_EnableIRQ(TWI1_IRQn);
}
 
static void Wire1_Init(void) {
    pmc_enable_periph_clk(WIRE1_INTERFACE_ID);
  PIO_Configure(
      g_APinDescription[PIN_WIRE1_SDA].pPort,
      g_APinDescription[PIN_WIRE1_SDA].ulPinType,
      g_APinDescription[PIN_WIRE1_SDA].ulPin,
      g_APinDescription[PIN_WIRE1_SDA].ulPinConfiguration);
  PIO_Configure(
      g_APinDescription[PIN_WIRE1_SCL].pPort,
      g_APinDescription[PIN_WIRE1_SCL].ulPinType,
      g_APinDescription[PIN_WIRE1_SCL].ulPin,
      g_APinDescription[PIN_WIRE1_SCL].ulPinConfiguration);
 
  NVIC_DisableIRQ(TWI0_IRQn);
  NVIC_ClearPendingIRQ(TWI0_IRQn);
  NVIC_SetPriority(TWI0_IRQn, 0);
  NVIC_EnableIRQ(TWI0_IRQn);
}

// RGB LED Setup
int blue = 16;
int green = 15;
int red = 17;



//speaker and sd card
// include SPI, MP3 and SD libraries
#include <SPI.h>
#include <Adafruit_VS1053.h>
#include <SD.h>

// define the pins used
#define CLK 76       // SPI Clock, shared with SD card
#define MISO 74      // Input data, from VS1053/SD card
#define MOSI 75      // Output data, to VS1053/SD card
// Connect CLK, MISO and MOSI to hardware SPI pins.
// See http://arduino.cc/en/Reference/SPI "Connections"

// These are the pins used for the breakout example
#define BREAKOUT_RESET  48      // VS1053 reset pin (output)
#define BREAKOUT_CS     49     // VS1053 chip select pin (output)
#define BREAKOUT_DCS    46      // VS1053 XDCS Data/command select pin (output)

// These are common pins between breakout and shield
#define CARDCS 47     // Card chip select pin
// DREQ should be an Int pin, see http://arduino.cc/en/Reference/attachInterrupt
#define DREQ 12       // VS1053 Data request, ideally an Interrupt pin

Adafruit_VS1053_FilePlayer musicPlayer =
  // create breakout-example object!
  Adafruit_VS1053_FilePlayer(BREAKOUT_RESET, BREAKOUT_CS, BREAKOUT_DCS, DREQ, CARDCS);

//if should play sounds
boolean playq = true;
boolean playAnswer = true;
boolean playTempI = true;
boolean playYesFever = true;
boolean playNoFever = true;
boolean start = false;
boolean playResults = true;

//array of symptom questions
String questions[8] = {"headache.wav", "muscle.wav", "throat.wav", "sl.wav", "sb.wav", "cp.wav", "chills.wav", "cough.wav"};
int severity[9] = {1, 1, 2, 3, 5, 5, 2, 2};

int symptomSum = 0;


// Bluefruit setup
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"

#include "BluefruitConfig.h"

#if SOFTWARE_SERIAL_AVAILABLE
  #include <SoftwareSerial.h>
#endif

#define FACTORYRESET_ENABLE         0
#define MINIMUM_FIRMWARE_VERSION    "0.6.6"

//Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);
Adafruit_BluefruitLE_UART ble(Serial1, BLUEFRUIT_UART_MODE_PIN);

/* ...software SPI, using SCK/MOSI/MISO user-defined SPI pins and then user selected CS/IRQ/RST */
//Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_SCK, BLUEFRUIT_SPI_MISO,
//                             BLUEFRUIT_SPI_MOSI, BLUEFRUIT_SPI_CS,
//                             BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

// A small helper
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}





void setup() {
  // Set up serial
  Serial.begin(115200);
  
  Serial.println("COVID-19 Symptom Checker!");
  
  // LCD setup
  lcd.begin(16, 2);
  lcd.setCursor(4,0);
  lcd.print("COVID-19");
  lcd.setCursor(0,1);
  lcd.print("Symptom Checker!");
  

  pinMode(joyX, INPUT);
  pinMode(joySelect, INPUT_PULLUP);

  // Set up thermometer
  pinMode(SYNC_PIN, OUTPUT);
  digitalWrite(SYNC_PIN, LOW);

  Wire_Init();
  // Disable PDC channel
  pTwi->TWI_PTCR = UART_PTCR_RXTDIS | UART_PTCR_TXTDIS;
  TWI_ConfigureMaster(pTwi, TWI_CLOCK, VARIANT_MCK);

  // RGB LED Setup
  pinMode(blue, OUTPUT);
  pinMode(green, OUTPUT);
  pinMode(red, OUTPUT);

  //speaker and SD setup
  if (! musicPlayer.begin()) { // initialise the music player
     Serial.println(F("Couldn't find VS1053, do you have the right pins defined?"));
     while (1);
  }
  Serial.println(F("VS1053 found"));
 
  SD.begin(CARDCS);    // initialise the SD card
 
  // Set volume for left, right channels. lower numbers == louder volume!
  musicPlayer.setVolume(5,5);

  


  // Bluefruit setup

  while (!Serial);  // required for Flora & Micro
  delay(500);

  Serial.println(F("Adafruit Bluefruit HID Keyboard Example"));
  Serial.println(F("---------------------------------------"));

  /* Initialise the module */
  Serial.print(F("Initialising the Bluefruit LE module: "));

  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  Serial.println( F("OK!") );

  if ( FACTORYRESET_ENABLE )
  {
    /* Perform a factory reset to make sure everything is in a known state */
    Serial.println(F("Performing a factory reset: "));
    if ( ! ble.factoryReset() ){
      error(F("Couldn't factory reset"));
    }
  }

  /* Disable command echo from Bluefruit */
  ble.echo(false);

  Serial.println("Requesting Bluefruit info:");
  /* Print Bluefruit information */
  ble.info();

  /* Change the device name to make it easier to find */
  Serial.println(F("Setting device name to 'COVID Checker': "));
  if (! ble.sendCommandCheckOK(F( "AT+GAPDEVNAME=COVID Checker" )) ) {
    error(F("Could not set device name?"));
  }

  /* Enable HID Service */
  Serial.println(F("Enable HID Service (including Keyboard): "));
  if ( ble.isVersionAtLeast(MINIMUM_FIRMWARE_VERSION) )
  {
    if ( !ble.sendCommandCheckOK(F( "AT+BleHIDEn=On" ))) {
      error(F("Could not enable Keyboard"));
    }
  }else
  {
    if (! ble.sendCommandCheckOK(F( "AT+BleKeyboardEn=On"  ))) {
      error(F("Could not enable Keyboard"));
    }
  }

  /* Add or remove service requires a reset */
  Serial.println(F("Performing a SW reset (service changes require a reset): "));
  if (! ble.reset() ) {
    error(F("Couldn't reset??"));
  }


  //play intro
  musicPlayer.startPlayingFile("intro.wav");
  musicPlayer.stopPlaying();
  

}

void loop() {
  // Print position of joystick
  int xPos = analogRead(joyX);

  if(idx <= 8){
    responses[idx] = 0;
    analogWrite(blue, 0);
    analogWrite(green, 0);
    analogWrite(red, 0);
//    lcd.clear();
  }

  //start code once joystick is clicked
  //Allows user to reset at will once they have used this before
  if(digitalRead(joySelect)==0){    
      start = true;
      lcd.clear();
    }

  if(start){
  if (idx < 8){

    if (idx == 0){
      
    }

    
    lcd.setCursor(0, 0);
    lcd.print(symptoms[idx]);
    lcd.setCursor(4,1);
    lcd.print("No");
    lcd.setCursor(10,1);
    lcd.print("Yes");

    //play question audio once
    if(playq){
      //play audio
      char fileName[20];
      questions[idx].toCharArray(fileName, 20);
      musicPlayer.startPlayingFile(fileName);
      musicPlayer.stopPlaying();

      playq = false;
    }

    //set which answer the user is currently pointing at
    if (xPos < 20){
      no = true;
      yes = false;
      playAnswer = true;
    }

    else if(xPos > 1000){
      no = false;
      yes = true;
      playAnswer = true;
    }

    //Show on LCD screen and play which answer user is pointing at
    if (no){
      lcd.setCursor(2,1);
      lcd.print("->");
      lcd.setCursor(13,1);
      lcd.print("  ");

      if(playAnswer){
        musicPlayer.startPlayingFile("no.wav");
        musicPlayer.stopPlaying();
        playAnswer = false;
      }
    }

    if (yes){
      lcd.setCursor(13,1);
      lcd.print("<-");
      lcd.setCursor(2,1);
      lcd.print("  ");

      if(playAnswer){
        musicPlayer.startPlayingFile("yes.wav");
        musicPlayer.stopPlaying();
        playAnswer = false;
      }
    }

    // Joystick button is pressed
    if(digitalRead(joySelect)==0){
      clicked = true;
    }

    if(clicked){
      Serial.print("Clicked!" );
      Serial.println(yes);
      
      // Save the response in an int array
      if (yes){
        responses[idx] = 1;
        symptomSum += severity[idx];
      }
//      
      clicked = false;
      idx += 1;
      yes = false;
      no = false;
      playq = true;
      lcd.clear();
      delay(500);
    }

  }
  }

  if (idx == 8){
    start = false;
    // Print temperature instructions to LCD Screen
    lcd.setCursor(0,0);
    lcd.print("Hold thermometer");
    lcd.setCursor(0,1);
    lcd.print("Click when done");

    //play instructions for temperature
    if(playTempI){
    musicPlayer.startPlayingFile("irtherm.wav");
    musicPlayer.stopPlaying();
    playTempI = false;
    }
    
    
    // Calculate temperature
    uint16_t tempUK;
    float tempK;
    uint8_t hB, lB, pec;

    float tempF;

    digitalWrite(SYNC_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(SYNC_PIN, LOW);

    TWI_StartRead(pTwi, ADDR, TOBJ_1, 1);

    lB = readByte();
    hB = readByte();

    //last read
    TWI_SendSTOPCondition(pTwi);
    pec = readByte();

    while (!TWI_TransferComplete(pTwi));
    //TWI_WaitTransferComplete(pTwi, RECV_TIMEOUT);

    tempUK = (hB << 8) | lB;

    if(tempUK & (1 << 16))
    {
      Serial.print("Error !");
      Serial.println(tempK);
    } 
    else 
    {
      tempK = ((float)tempUK * 2) / 100 ;
      tempF = ((tempK-273.15) * 1.8) + 32;
      Serial.print(" F: ");
      Serial.println(tempF);  /// tempK - 273.15
    }

    if (digitalRead(joySelect) == 0){
      lcd.clear();
      lcd.setCursor(0,0);
      lcd.print("Temperature:");
      lcd.setCursor(0,1);
      lcd.print(tempF);
      lcd.print(" degrees F");

   //audio for temperature results
   if(tempF > 101){
    //play has fever
    musicPlayer.startPlayingFile("fever.wav");
    musicPlayer.stopPlaying();
    playYesFever = false;
    symptomSum += severity[idx];
   }
   else{
    musicPlayer.startPlayingFile("nf.wav");
    musicPlayer.stopPlaying();
    playNoFever = false;
   }
      
      delay(2000);
      lcd.clear();
      playYesFever = true;
      playNoFever = true;
      idx += 1;
    }
  } // added to close if idx(==8)

    if (idx > 8){
      playTempI = true;

      //why are we setting to blue and green?
      analogWrite(blue, 0);
      analogWrite(green, 0);

      //Decision about COVID is made based on the sum of symptoms a user has
      if(symptomSum < 3){
        //display green
        analogWrite(green, 255);
        analogWrite(blue, 0);
        analogWrite(red, 0);

        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("Stay Safe!");

        //play audio for safe
        if(playResults){
          musicPlayer.startPlayingFile("safe.wav");
          musicPlayer.stopPlaying();

          //no symptoms, but remind to be careful
          ble.print("AT+BleKeyboard=");
          ble.println("You are low risk for COVID! Please remain careful and continue to monitor your symptoms\\n");

          delay(500);

          musicPlayer.startPlayingFile("restart.wav");
          musicPlayer.stopPlaying();
          playResults = false;
        }
      }
      else{
        //display red
        analogWrite(green, 0);
        analogWrite(blue, 0);
        analogWrite(red, 255);

        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("Get Tested");

        //play audio for symptoms
        if(playResults){
          musicPlayer.startPlayingFile("unsafe.wav");
          musicPlayer.stopPlaying();

          //no symptoms, but remind to be careful
          ble.print("AT+BleKeyboard=");
          ble.println("You are showing signs of COVID. Please get quarentine immediately and get tested. Talking to a health care professional is advised\\n");

          delay(500);

          musicPlayer.startPlayingFile("restart.wav");
          musicPlayer.stopPlaying();
          playResults = false;
        }
      }

      //put inside statement to check if clicked to reset
      if(digitalRead(joySelect) == 0){
        idx = 0;
        symptomSum = 0;
        lcd.clear();
//        lcd.setCursor(4,0);
//        lcd.print("COVID-19");
//        lcd.setCursor(0,1);
//        lcd.print("Symptom Checker!");
        playResults = true;
        
      }
    ble.waitForOK();
    if(ble.waitForOK()){
      Serial.print(F("OK!"));
    }
    else{
      Serial.print(F("FAILED!"));
    }
    }
  }
//}




// Helper functions for thermometer
//for thermoeter
uint8_t readByte() {
  //TWI_WaitByteReceived(pTwi, RECV_TIMEOUT);
  while (!TWI_ByteReceived(pTwi))
    ;
  return TWI_ReadByte(pTwi);
}
 
static inline bool TWI_WaitTransferComplete(Twi *_twi, uint32_t _timeout) {
  while (!TWI_TransferComplete(_twi)) {
    if (TWI_FailedAcknowledge(_twi))
      return false;
    if (--_timeout == 0)
      return false;
  }
  return true;
}
 
static inline bool TWI_WaitByteReceived(Twi *_twi, uint32_t _timeout) {
  while (!TWI_ByteReceived(_twi)) {
    if (TWI_FailedAcknowledge(_twi))
      return false;
    if (--_timeout == 0)
      return false;
  }
  return true;
}
 
static inline bool TWI_FailedAcknowledge(Twi *pTwi) {
  return pTwi->TWI_SR & TWI_SR_NACK;
}
