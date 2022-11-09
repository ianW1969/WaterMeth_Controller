////CanBus
#include <mcp_can.h>
#include <SPI.h>

long unsigned int rxId;
unsigned char len = 0;
unsigned char rxBuf[8];
bool init_state = true;

#define CAN0_INT 9                              // Set INT to pin 9
MCP_CAN CAN0(10);                               // Set CS to pin 10

//OLED
#include <Arduino.h>
#include <U8x8lib.h>
#define OLED_RESET 4
#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif
//
U8X8_SSD1306_128X64_NONAME_HW_I2C u8x81(/* reset=*/ U8X8_PIN_NONE);
U8X8_SSD1306_128X64_NONAME_HW_I2C u8x82(/* reset=*/ U8X8_PIN_NONE);

//OLED_Display_update_inetrval
const unsigned long DisplayInterval = 333;
unsigned long previousTimeDisplay = 0;

// Boost_Pressure_and_IAT
// int offset = 125; // low pressure calibration
// int span = 895; // high pressure calibration
int offset = 111; // low pressure calibration
int span = 912; // high pressure calibration
int Boost_pressure_Pa;
int Boost_pressure_kPa;
int Boost_pressure_PSI;
int boost_pressure_pin = A3;
int Max_PSI = 0;

int IAT_voltage = 0;
// float IAT_Vin = 4.6; // 4.6 volts - USB powered***
float IAT_Vin = 4.8; // 4.8 volts - USB Power adapter powered***
float IAT_Vout = 0;
float R1 = 2000; // Temperature resistor value
float IAT_buffer = 0;
float logR2;
float R2;
float IAT;
int IAT_pin = A1;
int IAT_C = 0;
int Max_IAT = 0;

// float A = 1.297373711e-03, B = 2.587931666e-04, C = 1.639343214e-07;  // Steinhart-Hart and Hart Coefficients Max
// float A = 1.306496435e-03, B = 2.580557547e-04, C = 1.752303942e-07;  // Steinhart-Hart and Hart Coefficients Nominal
float A = 1.316012871e-03, B = 2.572431200e-04, C = 1.877359037e-07;  // Steinhart-Hart and Hart Coefficients Min***

//LEDs_Change_lights
#include <FastLED.h>
#define NUM_LEDS 20
#define DATA_PIN 2 //How boring and obvious!
#define COLOR_ORDER GRB //Green (G), Red (R), Blue (B)
#define CHIPSET WS2812B
#define BRIGHTNESS 25
#define VOLTS 5
CRGB leds[NUM_LEDS];
int LAST_LED = 0;
int NEXT_LED = 0;

//CHSV colourRed(255,0,20);  // Define a custom color.
//CHSV colourGreen(0,255,0);  // Define a custom color.
//Voltage
float Voltage = 1.00;

//RPM
int RPM = 1;
int Max_RPM = 0;
//int ARPM = 0;
//int BRPM = 0;
int RPM_NUM_LEDS = 0;


//Speed
int Speed = 0;
int Max_Speed = 0;

//W-M
int Max_WM = 0;
int WM_value_percent = 0;
int WM_pin = 6;
int WM_value_T;
int WM_value_P;
int WM_value;
int WM_Level_pin = 3;  //int WM_Level_pin = A2;
int WM_Level = 101;



//History
int HistoryValue = 0;
int Display1_Counter = 1;
int Display2_Counter = 1;
int Display1_Scroll = 4;
int Display2_Scroll = 5;
int Max_Reset = 7;



void setup() {
  Serial.begin(115200);

  //OLED
  u8x81.setI2CAddress(0x78);
  u8x81.begin();
  u8x81.setPowerSave(0);
  u8x81.clear();
  u8x82.setI2CAddress(0x7A);
  u8x82.begin();
  u8x82.setPowerSave(0);
  u8x82.clear();

  //LEDs_Change_lights
  FastLED.addLeds<CHIPSET, DATA_PIN, COLOR_ORDER>(leds, NUM_LEDS);
  FastLED.setBrightness(BRIGHTNESS);
  FastLED.clear();
  FastLED.show();

  //Water_Meth
  pinMode(WM_pin, OUTPUT);

  //Rest_&_Scroll_History
  pinMode(Max_Reset, INPUT_PULLUP);
  pinMode(Display1_Scroll, INPUT_PULLUP);
  pinMode(Display2_Scroll, INPUT_PULLUP);
  pinMode(WM_Level_pin, INPUT_PULLUP);

  //  //CANBUS
    // Initialize MCP2515 running at 8MHz with a baudrate of 500kb/s and the masks and filters disabled.
    if (CAN0.begin(MCP_STDEXT, CAN_500KBPS, MCP_8MHZ) == CAN_OK) {
      Serial.println("MCP2515 Initialized Successfully!");
    }
    else {
      Serial.println("Error Initializing MCP2515...");
    }
  
    pinMode(9, INPUT);                            // Setting pin 9 for /INT input
  
    CAN0.init_Mask(0, 0, 0x07FF0000);              // Init first mask (Std CAN. No masking applied)
    CAN0.init_Filt(0, 0, 0x02800000);              // Init first filter...
    CAN0.init_Filt(1, 0, 0x02800000);              // Init second filter...
  
    CAN0.init_Mask(1, 0, 0x07FF0000);              // Init second mask (Std CAN. No masking applied)
    CAN0.init_Filt(2, 0, 0x02880000);              // Init third filter...
    CAN0.init_Filt(3, 0, 0x02880000);              // Init fourth filter...
    CAN0.init_Filt(4, 0, 0x02880000);              // Init fifth filter...
    CAN0.init_Filt(5, 0, 0x02880000);              // Init sixth filter...
  
    CAN0.setMode(MCP_NORMAL);                     // Set operation mode to normal so the MCP2515 sends acks to received data.
  //Buzzer
  pinMode(A2, OUTPUT);
  digitalWrite(A2, LOW);
}

void loop() {

  //Sets_LAST_LED_for_Change_Buzzer
  LAST_LED = RPM / 365;

  //Rest_&_Scroll_History
  // Display1
  if (digitalRead(Display1_Scroll) == LOW) {
    delay (100);

    if (digitalRead(Display1_Scroll) == LOW) {
      u8x81.clear();
      Display1_Counter = Display1_Counter + 1;
    }
  }
  if (Display1_Counter > 10) Display1_Counter = 1;

  // Display2
  if (digitalRead(Display2_Scroll) == LOW) {
    delay (100);

    if (digitalRead(Display2_Scroll) == LOW) {
      u8x82.clear();
      Display2_Counter = Display2_Counter + 1;
    }
  }

  if (Display2_Counter > 10) Display2_Counter = 1;

  // MAX_Reset
  if (digitalRead(Max_Reset) == LOW)
  {
    Display1_Counter = 1;
    Display2_Counter = 1;
    u8x81.clear();
    u8x82.clear();
    Max_Speed = 0;
    Max_RPM = 0;
    Max_IAT = 0;
    Max_WM = 0;
    Max_PSI = 0;
  }

  //  //CANBUS
    if (!digitalRead(9))                   // If pin 9 is low, read receive buffer
    {
      //            Serial.println("before readMsgBuf");
      CAN0.readMsgBuf(&rxId, &len, rxBuf); // Read data: len = data length, buf = data byte(s)
  
      //    Serial.print("ID: ");
      //    Serial.print(rxId, HEX);
      //    Serial.print(" Data: ");
      //    for (int i = 0; i < len; i++)        // Print each byte of the data
      //
      //    {
      //      if (rxBuf[i] < 0x10)               // If data byte is less than 0x10, add a leading zero
      //      {
      //        Serial.print("0");
      //      }
      //      Serial.print(rxBuf[i], HEX);
      //      Serial.print(" ");
      //    }
      //    Serial.println();
  
  
      if (rxId == 0x280) {
  
        //          throttle = rxBuf[5] * 0.4;
        //          Serial.print("Throttle (%): ");
        //          Serial.print(throttle, HEX);
        //          Serial.print("   ");
        //          Serial.println(throttle, DEC);
        //          if (throttle > throttlemax) throttlemax = throttle;
        //          Serial.print("Throttlemax: ");
        //          Serial.println(throttlemax, DEC);
        //
        //
        //          load = rxBuf[1] * 0.39;
        //          Serial.print("Engine Load (%): ");
        //          Serial.print(load, HEX);
        //          Serial.print("   ");
        //          Serial.println(load, DEC);
        //          if (load > loadmax) loadmax = load;
        //          Serial.print("loadmax: ");
        //          Serial.println(loadmax, DEC);
  
        RPM = ((rxBuf[3] * 256) + rxBuf[2]) * 0.25;
        //      ARPM = rxBuf[3] * 256;
        //      BRPM = ARPM + rxBuf[2];
        //      RPM = BRPM * 0.25;
  
      }
  
      if (rxId == 0x288) {
  
        Speed = (rxBuf[3] * 0.759);
//        Serial.print("vehicle Speed (MPH): ");
//        Serial.print(Speed);
//        Serial.print("   ");
//        Serial.println(Speed, DEC);
        if (Speed > Max_Speed) Max_Speed = Speed;
        //          Serial.print("Max_Speed: ");
        //          Serial.println(Max_Speed, DEC);
      }
    }

  // WM level code
  //  int WM_Level = analogRead(WM_Level_pin); // get adc value
  //
  //  if (((HistoryValue >= WM_Level) && ((HistoryValue - WM_Level) > 10)) || ((HistoryValue < WM_Level) && ((WM_Level - HistoryValue) > 10)))
  //  {
  //    //sprintf(printBuffer, "ADC%d level is %d\n", WM_Level_pin, WM_Level);
  //    //Serial.print(printBuffer);
  //    HistoryValue = WM_Level;
  //  }

  // Boost Pressure
  Boost_pressure_Pa = map(analogRead(boost_pressure_pin), offset, span, 500, 4000);
  Boost_pressure_kPa = Boost_pressure_Pa / 10, 1;
  if (Boost_pressure_kPa < 20) Boost_pressure_kPa = 20;
  Boost_pressure_PSI = Boost_pressure_kPa * 0.145038;
  Boost_pressure_PSI = Boost_pressure_PSI - 14;
  if (Boost_pressure_PSI < 1) Boost_pressure_PSI = 0;
  if (Boost_pressure_PSI > Max_PSI) Max_PSI = Boost_pressure_PSI;

  // Potentiometer_in_lieu_of_CANBUS
//  int sensorValue = analogRead(A0);

  // RPM
//  RPM = map(sensorValue, 0, 1023, 100, 7500);
  if (RPM > Max_RPM) Max_RPM = RPM;

  // Speed
//  Speed = map(sensorValue, 0, 1023, 0, 155);
  if (Speed > Max_Speed) Max_Speed = Speed;

  //WM_Controller
  if (WM_value_percent > Max_WM) Max_WM = WM_value_percent;

  //IAT
  IAT_voltage = analogRead(IAT_pin);
  if (IAT_voltage) {
    IAT_buffer = IAT_voltage * IAT_Vin;
    IAT_Vout = (IAT_buffer) / 1024.0;
    IAT_buffer = (IAT_Vin / IAT_Vout) - 1;
    R2 = R1 / IAT_buffer;
  }

  logR2 = log(R2);
  IAT = (1.0 / (A + B * logR2 + C * logR2 * logR2 * logR2)); // Steinhart and Hart Equation. T  = 1 / {A + B[ln(R)] + C[ln(R)]^3}
  IAT_C =  IAT - 273.15;

  // Edit_for_WM_delivery
  if (IAT_C >= 27) {
    WM_value_T = map (IAT_C, 27, 40, 0, 255);
    WM_value_P = map(Boost_pressure_PSI, 8, 25, 0, 255);
    if (WM_value_T >= WM_value_P) WM_value = WM_value_T;
    if (WM_value_P >= WM_value_T) WM_value = WM_value_P;
    if (WM_value <= 0) WM_value = 0;
    WM_value_percent = WM_value / 2.5;
    if (Boost_pressure_PSI >= 31) WM_value_percent = 100;
    if (WM_value_percent >= 100) WM_value_percent = 100;
    WM_value = WM_value_percent * 2.55;
    analogWrite(WM_pin, WM_value);
  }
  else
  {
    WM_value_P = map(Boost_pressure_PSI, 8, 25, 0, 255);
    WM_value_T = map (IAT_C, 28, 40, 0, 255);
    if (WM_value_P >= WM_value_T) WM_value = WM_value_P;
    if (WM_value_T >= WM_value_P) WM_value = WM_value_T;
    if (WM_value <= 0) WM_value = 0;
    WM_value_percent = WM_value / 2.5;
    if (Boost_pressure_PSI >= 25) WM_value_percent = 100;
    if (WM_value_percent >= 100) WM_value_percent = 100;
    WM_value = WM_value_percent * 2.55;
    analogWrite(WM_pin, WM_value);
  }
  //    // Here as it reads 0 when with the rest of the max readings??
  if (IAT_C > Max_IAT) Max_IAT = IAT_C;

  //OLED
  unsigned long currentTimeDisplay = millis();
  if (currentTimeDisplay - previousTimeDisplay >= DisplayInterval) {

    //Dispslay1
    u8x81.setFont(u8x8_font_px437wyse700b_2x2_r);

    if (Display1_Counter == 1) {
      u8x81.drawString(1, 0, "W-M");
      u8x81.drawString(12, 5, "%");
      if (WM_value_percent < 100) u8x81.draw2x2String(8, 3, " ");
      if (WM_value_percent < 10) u8x81.draw2x2String(4, 3, " ");
      u8x81.draw2x2String(0, 3, String(WM_value_percent).c_str());

    }
    if (Display1_Counter == 2) {
      u8x81.drawString(1, 0, "PSI");
      if (Boost_pressure_PSI < 10) u8x81.draw2x2String(4, 3, " ");
      u8x81.draw2x2String(0, 3, String(Boost_pressure_PSI).c_str());

    }
    if (Display1_Counter == 3) {
      u8x81.drawString(1, 0, "IAT");
      u8x81.drawString(9, 5, "C");
      if (IAT_C < 10) u8x81.draw2x2String(4, 3, " ");
      u8x81.draw2x2String(0, 3, String(IAT_C).c_str());

    }
    if (Display1_Counter == 4) {
      u8x81.drawString(1, 0, "MPH");
      if (Speed < 100) u8x81.draw2x2String(8, 3, " ");
      if (Speed < 10) u8x81.draw2x2String(4, 3, " ");
      u8x81.draw2x2String(0, 3, String(Speed).c_str());

    }
    if (Display1_Counter == 5) {
      u8x81.drawString(1, 0, "RPM");
      if (RPM < 1000) u8x81.draw2x2String(12, 3, " ");
      u8x81.draw2x2String(0, 3, String(RPM).c_str());
    }

    if (Display1_Counter == 6) {
      u8x81.drawString(1, 0, "MAX W-M");
      u8x81.drawString(12, 5, "%");
      u8x81.draw2x2String(0, 3, String(Max_WM).c_str());

    }

    if (Display1_Counter == 7) {
      u8x81.drawString(1, 0, "MAX PSI");
      u8x81.draw2x2String(0, 3, String(Max_PSI).c_str());

    }

    if (Display1_Counter == 8) {
      u8x81.drawString(1, 0, "MAX IAT");
      u8x81.drawString(9, 5, "C");
      u8x81.draw2x2String(0, 3, String(Max_IAT).c_str());

    }

    if (Display1_Counter == 9) {
      u8x81.drawString(1, 0, "MAX MPH");
      u8x81.draw2x2String(1, 3, String(Max_Speed).c_str());

    }

    if (Display1_Counter == 10) {
      u8x81.drawString(1, 0, "MAX RPM");
      u8x81.draw2x2String(0, 3, String(Max_RPM).c_str());
    }

    //Display_2

    u8x82.setFont(u8x8_font_px437wyse700b_2x2_r);

    if (Display2_Counter == 1) {
      u8x82.drawString(1, 0, "W-M");
      u8x82.drawString(12, 5, "%");
      if (WM_value_percent < 100) u8x82.draw2x2String(8, 3, " ");
      if (WM_value_percent < 10) u8x82.draw2x2String(4, 3, " ");
      u8x82.draw2x2String(0, 3, String(WM_value_percent).c_str());

    }
    if (Display2_Counter == 2) {
      u8x82.drawString(1, 0, "PSI");
      if (Boost_pressure_PSI < 10) u8x82.draw2x2String(4, 3, " ");
      u8x82.draw2x2String(0, 3, String(Boost_pressure_PSI).c_str());

    }
    if (Display2_Counter == 3) {
      u8x82.drawString(1, 0, "IAT");
      u8x82.drawString(9, 5, "C");
      if (IAT_C < 10) u8x82.draw2x2String(4, 3, " ");
      u8x82.draw2x2String(0, 3, String(IAT_C).c_str());

    }
    if (Display2_Counter == 4) {
      u8x82.drawString(1, 0, "MPH");
      if (Speed < 100) u8x82.draw2x2String(8, 3, " ");
      if (Speed < 10) u8x82.draw2x2String(4, 3, " ");
      u8x82.draw2x2String(0, 3, String(Speed).c_str());

    }
    if (Display2_Counter == 5) {
      u8x82.drawString(1, 0, "RPM");
      if (RPM < 1000) u8x82.draw2x2String(12, 3, " ");
      u8x82.draw2x2String(0, 3, String(RPM).c_str());
    }

    if (Display2_Counter == 6) {
      u8x82.drawString(1, 0, "MAX W-M");
      u8x82.drawString(12, 5, "%");
      u8x82.draw2x2String(0, 3, String(Max_WM).c_str());

    }

    if (Display2_Counter == 7) {
      u8x82.drawString(1, 0, "MAX PSI");
      u8x82.draw2x2String(0, 3, String(Max_PSI).c_str());

    }

    if (Display2_Counter == 8) {
      u8x82.drawString(1, 0, "MAX IAT");
      u8x82.drawString(9, 5, "C");
      u8x82.draw2x2String(0, 3, String(Max_IAT).c_str());

    }

    if (Display2_Counter == 9) {
      u8x82.drawString(1, 0, "MAX MPH");
      u8x82.draw2x2String(1, 3, String(Max_Speed).c_str());

    }

    if (Display2_Counter == 10) {
      u8x82.drawString(1, 0, "MAX RPM");
      u8x82.draw2x2String(0, 3, String(Max_RPM).c_str());
    }

    previousTimeDisplay = currentTimeDisplay;
  }

  // WM level low code
  //  if (WM_Level <= 460)
  if (digitalRead(WM_Level_pin) == LOW) {
    Display1_Counter = 0;
    Display2_Counter = 0;
    u8x81.drawString(1, 0, "W-M    ");
    u8x82.drawString(1, 0, "W-M    ");
    u8x81.draw2x2String(0, 3, "    ");
    u8x82.draw2x2String(0, 3, "    ");
    u8x81.draw2x2String(0, 3, "LOW ");
    u8x82.draw2x2String(0, 3, "LOW ");
  }


  int RPM_NUM_LEDS = RPM / 365; //20LEDs divided by 7500 RPM

  if (RPM_NUM_LEDS >= 16) RPM_NUM_LEDS = 20; // LEDs jump at 16 to all 20 so all red LEDs light at once

  // Light up the LEDs
  for (int i = 0; i < 13; ++i) {

    leds[i] = CRGB(0, 255, 0);
  }
  for (int i = 13; i < 16; ++i) {

    leds[i] = CRGB(0, 0, 255);
  }
  for (int i = 16; i < 20; ++i) {

    leds[i] = CRGB(255, 0, 0);
  }

  // LEDs are switched off as RPM drops
  for (int i = RPM_NUM_LEDS; i < NUM_LEDS; ++i) {

    leds[i] = CRGB::Black;
  }

  FastLED.show();

  //Change_Buzzer

  //Sets_NEXT_LED_for_Change_Buzzer
  NEXT_LED = RPM / 365;

  //  if ((LAST_LED < NEXT_LED) && (RPM_NUM_LEDS == 13)) {
  //    Serial.println ("Change");
  //    Serial.print ("LEDS  ");
  //    Serial.println (RPM_NUM_LEDS);
  //    Serial.print ("LAST_LED  ");
  //    Serial.println (LAST_LED);
  //    Serial.print ("NEXT_LED  ");
  //    Serial.println (NEXT_LED);
  //  }
  if ((LAST_LED < NEXT_LED) && (RPM_NUM_LEDS == 13))  tone (A2, 1000, 250);

}
