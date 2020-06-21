/* SCULLCOM HOBBY ELECTRONICS
 * MILLIVOLT METER USING LTC2400 24bit ADC CHIP
 * Using the PCB designed by Greg
 * http://www.barbouri.com/2016/05/26/millivolt-meter/
 *
 * ====  Arduino Pro MIni 5V 16MHz  ====
 *
 *  Changes by dbldutch:
 *  - added code to display the sw version in the welcome screens
 *  - added code to display a real micro symbol instead of uV
 *  - added code to use an i2c LCD display
 *  - added code to go from two buttons to one with a dual function
 *  - added code to monitor the charging level of a NiCAD/NiMH cell
 *  - changed code to display decimal digits based on volts measured.
 *  - changed the way the button press is used and displayed
 *  - Version 2.00:
 *  - stripped the 4LSB in the Read_ADC, before averaging. They add no value here.
 *    Calibration uses a larger sample size. Added a loop counter to show progress.
 *  - Version 3.00 :
 *  - Changed from average based sampling to an Infinite Input Response (IIR) filter design.
 *    Details found here: https://github.com/arduino/Arduino/issues/3934
 *    and here : https://en.m.wikipedia.org/wiki/Infinite_impulse_response
 *  - Created a dynamic ADC conversion delay for the LTC2400 to wring out some idling time.
 *  - Changed to EEPROM.put & .get to also store floats. Eliminated the previous functions.
 *  - With the IIR filtering, there is no need for reduced decimals. Eliminated the code.
 *  - Added a calibration to a voltage reference to tweak the accuracy.
 *  - Changed the dual button press code to cover the zero cal and the v-ref cal functionality.
 *  - Version 3.10:
 *  - Added a dynamic filter weight algorithm to the IIR filter.
 *  - Version 3.11:
 *  - Changed the conversion delay to (re)start at the end of the LCD update cycle, such that the LTC
 *    has 165 mSec of "quiet" time on the power and data lines to do the sampling.
 *  - Fixed a compounded rounding error of the IIR filter calculation.
 *  - Added the filter weight exponent multiplier to the display.
 *  - Version 3.12:
 *  - Added some tweaks and changes to allow a precise calibration of the reference voltage. This
 *    largely determines the linearity of the meter.
 *  - Version 3.13:
 *    First step to add remote logging of the measured values. With this modification, the measured 
 *    value is send out through the serial output pins of the Arduino. This can be plotted with the Arduino
 *    IDE Serial Monitor.
 *    IMPORTANT:
 *    When you connect a serial to USB cable to the meter, you effectively connect the ground of the meter 
 *    to the ground of the Laptop or PC that you use to collect the data. Depending of how you power the PC,
 *    or what you have otherwise connected to the PC, like a network cable, a monitor or other USB devices, you
 *    could have made a connection to earth ground.
 *    This defeats the floating capability of the meter when you run it on the battery. Nasty things can happen 
 *    when you connect the common measuring lead of the meter to the DUT. You could potentially create a short to
 *    earth or lift the potential of your PC to whatever the voltage is that your common lead is connected to!
 *    Please be aware of this.
 *    
 *    To avoid this, and make the meter truly floating again, the hardware needs an additional circuit to create an optical 
 *    isolation between the PC and the meter so the instrument can still float if on battery power, and the USB monitoring 
 *    device is protected.
 *
 *   Software version:
 */
String SW_VERSION = " Version 3.14";
/*
 * LCD interface using the i2c bus:
 * http://fabo.io/212.html
 * https://github.com/FaBoPlatform/FaBoLCD-PCF8574-Library
 *
 *  LCD Connections:
 *  - SDA to A4 to analog pin 27
 *  - SCL to A5 to analog pin 28
 *
 * LTC2400 A2D convertor connections using SPI bus:
 *  - SCK to digital pin 13
 *  - SDO to digital pin 12
 *  - CS  to digital pin 10
 * 
 * Button:
 * - button to digital pin 3
 *
 * Battery indicator level:
 * - Analog pin A0 (via a 20K:10K divider) [10K is minimum value for the A2D]
 */
#include <SPI.h>                         // include SPI library (Serial Peripheral Interface)
#include <Wire.h>
#include <FaBoLCD_PCF8574.h>             // include the i2c bus interface and LCD driver code
#include <EEPROM.h>                      // include EEPROM library

//---- initialize the i2c/LCD library
FaBoLCD_PCF8574 lcd;                     // with this, there are no further code changes writing to the LCD

//---- Logging parameters
unsigned long startLogging;
unsigned long currentMillis;
boolean logging = false;
int loggingInterval = 1000;             // log every 1000 mS for 1 second

//---- Button and button debounce
const int button = 2;                    // button is connected to D2, between VCC and with a 1K to ground.
                                         // a press grounds the pin (makes it low/false), is otherwise true/high
unsigned long buttonTimer = 0;           // timer to differentiate between a long and a short button press
long longPressTime = 2000;               // 2 seconds + max 800mS overhead
unsigned long last_millis = 0;           // always use unsigned long with millis() to avoid roll-over issues
boolean buttonActive = false;
boolean longPressActive = false;

//---- LT 2400 ADC convertor
const int LTC_CS = 10;                   // set ADC chip select pin !CS (!SS) to Arduino pin D10
                                         // SPI SLCK is D13, SDO (MISO) is D12
long adcread;                            // reading from the ADC (LTC2400)
int ct = 165;                            // ADC converstion time is 160mS typical, +/-3.2 mS (data sheet)
unsigned long ct_start;                  // seed the conversion start timestamp
unsigned long ct_chk;                    // the entry timestamp, used to dynamically create the delay
float volt;                              // voltage reading from ADC
                                         // Following measurements were made after a warm up, a null calibration, followed by
                                         // a calibration with a 5V0 Ref, loading the new value and doing the measurement again.
                                         // This is a critical measurement because it determines the linearity of the meter
//float v_ref = 4.09553;                 // ADR4540B Reference Voltage measured from my A meter
float v_ref = 4.09588;                   // ADR4540B Reference Voltage measured from my B meter

//---- IIR Low Pass Filtering with a dynamic filter weight algorithm
float average;                            // holds the result of the filter
int filterWeight = 64;                   // from 4..64 Higher numbers = heavier filtering
int fw_multiplier = 1;                   // multiplier for the filterWeight calculation
int noise_level = 96;                    // 96 is +/- 234 uV input voltage differential

//---- Zero offset Calibration
int CalSetup = 0;                        // calibration check
int DecPlaces = 0;                       // limit to the number of decimal places on display
long zero_cal;                           // calibration adjustment factor
long zc_address = 0L;                    // set EEPROM memory start address location 0..3
const int cal_adj_samples = 75;          // number of samples

//---- Calibration Factor
float cal_factor = 1.0;                  // calibration factor to adjust for the linearity errors
float cal_avg;                           // the resulting value of the filter
float cal_2_5v_ref = 2.49993;            // calibrated values from my HaoQiXin Voltage Reference
float cal_5v_ref = 5.00181;              //
float cal_7_5v_ref = 7.50547;            //
float cal_10v_ref = 10.00673;            //
long cf_address = 5L;                    // EEPROM address 5..8

//---- Display the result to the LCD
String v;                                // string to hold the V, mV or uV string data for the display
String micro;                            // string to hold the real micro character
int dec_adj = 0;                         // decimal places adjustment [0..6], set manually by the button
int dec_digit = 6;                       // integer that holds number of decimal places displayed on LCD
int dTV = 5;                             // default is 10.00000 V for 10 Volt and above
int dV = 6;                              // default is 1.000000 V for 1-9 Volt range
int dmV = 4;                             // default is 100,0000 mV for MilliVolt range
int duV = 0;                             // default is 1000000 uV for MicroVolt range

//---- Arduino ADC Battery check
int batt = A0;                           // the 9V NiMH or NiCAD battery
int check_v = 0;                         // check the battery voltage
int loop_cnt = 365;                      // check it approx. every minute (loop time is 0.165 Sec)
float adc_cal = 0.0;                     // ADC calibration value
float adc_ref_volts = 5.14;              // external volt reference (VCC) -> measured
float adc_res = 1024;                    // 10 bit ADC resolution
const int adc_samples = 4;               // set number of sample readings to average the result

// Create a set of new symbols that can be displayed on the LCD
byte batt_full[8] =                      // full battery symbol, displayed at 9V
  {
  B01110,B11111,B11111,B11111,B11111,B11111,B11111,B11111
  };
byte batt_8_7[8] =                       // 8.7V level
  {
  B00000,B01110,B11111,B11111,B11111,B11111,B11111,B11111
  };
byte batt_8_3[8] =                       // 8.3V level
  {
  B00000,B00000,B01110,B11111,B11111,B11111,B11111,B11111
  };
byte batt_8_0[8] =                       // 8.0V level
  {
  B00000,B00000,B00000,B01110,B11111,B11111,B11111,B11111
  };
byte batt_7_7[8] =                       // 7.7V level
  {
  B00000,B00000,B00000,B00000,B01110,B11111,B11111,B11111
  };
byte batt_7_5[8] =                       // 7.5V level
  {
  B00000,B00000,B00000,B00000,B00000,B01110,B11111,B11111
  };
byte batt_empty[8] =                     // <7.3V Empty
  {
  B00100,B00100,B00100,B00000,B00100,B00000,B01110,B11111
  };
byte batt_charging[8] =                  // >10V Charging symbol
  {
  B00010,B00100,B01000,B11111,B00010,B10100,B11000,B11100
  };


/**************************************************************************************
 * Initialization routine, runs only at start or reboot
 */
void setup() {
  Serial.begin(9600); // ==>> activate for debug and test only

  micro = char(228);                     // real micro symbol is char(b11100100)

  pinMode(button, INPUT);                // set the pin connected to the button to input
  digitalRead(button);                   // get rid of first unreliable input

  pinMode (LTC_CS, OUTPUT);              // set LTC_CS (pin D10 on Arduino Nano) to OUTPUT mode
  digitalWrite(LTC_CS, HIGH);            // set LCT2400 chip select pin HIGH to disable  // initialize digital pin LED_BUILTIN as an output.

  SPI.begin();                           // initialise SPI bus
  SPI.setBitOrder(MSBFIRST);             // Sets the order of bits shifted out and in to SPI bus, MSBFIRST (most-significant bit first)
  SPI.setDataMode(SPI_MODE0);            // set SPI to Mode 0 (MOSI read on rising edge (CPLI=0) and SCK idle low (CPOL=0))
  SPI.setClockDivider(SPI_CLOCK_DIV16);  // divide Arduino clock by 16 to gave a 1 MHz SPI clock

  lcd.begin(16, 2);                      // set up the LCD's number of columns and rows
  lcd.setCursor(0,0);                    // set LCD cursor to column 0, row O (start of first line)
  lcd.print("Milli-Volt Meter");
  lcd.setCursor(0,1);
  lcd.print(SW_VERSION);                 // print software version to display
  Serial.print("Milli-Volt Meter ");
  Serial.println(SW_VERSION);
  delay(2000);
  lcd.clear();                           // clear dislay
  lcd.setCursor(0,0);                    // set LCD cursor to column 0, row O (start of first line)

//  ==>> only for testing!
/*
  zero_cal = 0;                          // Start with a clean slate
  Serial.println(zero_cal);
  EEPROM.put(zc_address,zero_cal);       // store calibration factor in EEPROM
  EEPROM.get(zc_address,zero_cal);       // retrieve calibration factor in EEPROM
  Serial.println(zero_cal);
  cal_factor = 1.0;
  Serial.println(cal_factor, 6);
  EEPROM.put(cf_address,cal_factor);     // store calibration factor in EEPROM
  EEPROM.get(cf_address,cal_factor);
  Serial.println(cal_factor, 6);
*/

  EEPROM.get(zc_address, zero_cal);      // get the zero cal factor from EEPROM
  EEPROM.get(cf_address, cal_factor);    // get the cal factor from EEPROM

  lcd.clear();                           // clear dislay
  lcd.setCursor(0,0);                    // set LCD cursor to column 0, row O (start of first line)
  lcd.print("Zero Cal Factor:");
  lcd.setCursor(0,1);
  lcd.print(zero_cal);                   // Briefly show calibration factor stored in EEPROM at switch on
  delay(2000);
  lcd.clear();                           // clear dislay
  lcd.setCursor(0,0);                    // set LCD cursor to column 0, row O (start of first line)
  lcd.print("V-Cal Factor:");
  lcd.setCursor(0,1);
  lcd.print(cal_factor, 6);              // Briefly show calibration factor stored in EEPROM at switch on
  delay(2000);
  lcd.clear();
  lcd.setCursor(0,0);                    // set LCD cursor to column 0, row O (start of first line)
  lcd.print("mVolt Meter      ");        // print Millivolt Meter to display and clear the rest
 
  // create a set of special symbols from the battery monitor definitions above
  lcd.createChar (0, batt_full);         // >9V
  lcd.createChar (1, batt_8_7);          // 8.7V
  lcd.createChar (2, batt_8_3);          // 8.3V
  lcd.createChar (3, batt_8_0);          // 8.0V
  lcd.createChar (4, batt_7_7);          // 7.7V
  lcd.createChar (5, batt_7_5);          // 7.5V
  lcd.createChar (6, batt_empty);        // <7.3V measurements can be inaccurate!
  lcd.createChar (7, batt_charging);     // > 10V charging and connected to wall-wart
  analogReference(DEFAULT);              // not needed here, use the external or default (=internal) A2D reference

  Monitor_batt();                        // get the battery level and show it on the display
 
  ct_start = millis();                   // seed the conversion start timestamp for the LTC2400
  for (int i=0;i<5;i++) {                // disregard the first five readings as they seem unstable
    average = Spi_Read();                // and also seed the IIR filter
  }
  startLogging = millis();               // starting timestamp for logging intervals  
}


/**************************************************************************************
 * 
 * Routine to read the data from the LTC2400 A2D convertor through the SPI interface
 * 
 */
long Spi_Read(void){                     // SPI(Serial Peripheral Interface) read sub-routine to read data form the LTC2400 ADC
                                         // and transfer 8 bits (1 byte) at a time - total of 4 bytes.
  long result = 0L;                      // result represents rolling total of the bytes transferred
  long b;                                // b is result of reading ADC output bytes

  //calculate the minimum conversion delay dynamically
  ct_chk = millis();
  unsigned int ct_delay = ct_chk - ct_start; // use the time already spent and factor that in
  if (ct_delay < ct){
    delay(ct - ct_delay);                // use the adjusted conversion delay if needed
  }
 
  digitalWrite(LTC_CS, LOW);             // LTC2400 chip select pin taken low to wake up the ADC and enable the SDO (MOSI) output
  delayMicroseconds(1);                  // timing delay but is not really required

  if (!(PINB & (1 << 4))) {              // check for a low !EOC on the MOSI pin D12, if the ADC is ready to transmit new data
                                         // if not, try again later -> this will reduce the number of readings to average
    b = SPI.transfer(0xff);              // transfer first byte most significant bits first.
    b &= 0x0f;                           // discard first 4 status bits (bits 31 to 25) mask received data with binary 00001111
    result = b;                          // result after removing first 4 bits (replacing them with 0's)
    result <<= 8;                        // shift first byte left by 8 places
    b = SPI.transfer(0xff);              // transfer second byte most significant bits first.
    result |= b;                         // add second byte to first byte by using the OR function (now 12 bits)
    result = result << 8;                // shift result left by 8 places
    b = SPI.transfer(0xff);              // transfer third byte most significant bits first.
    result |= b;                         // add third byte to result by using the OR function (now 20 bits)
    result = result << 8;                // shift result left by 8 places
    b = SPI.transfer(0xff);              // transfer fourth byte most significant bits first.
    result |= b;                         // add fourth byte to result by using the OR function (now 28 bits)
    result = result >> 4;                // get rid of the 4 LSB bits, they don't add any value in this application
 
    digitalWrite(LTC_CS, HIGH);          // LTC2400 chip enters low power (sleep mode) and disables the ADC output.
    ct_start = millis();                 // start the conversion delay timer (restarted at the end of the main loop)
    return(result);                      // return with result as the 24 bit data representing the voltage
  }
}


/**************************************************************************************
 * 
 * Routine to run a zero offset calibration.
 * We'll calculate the input offset by manually shortening the input.
 * 
 */
void Zero_Cal_Adjust() {
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("=Zero Calibrate");
  lcd.setCursor(0,1);
  lcd.print("Short input");
  delay(3000);

  int cal_counter = cal_adj_samples;     // total number of readings
  average = Spi_Read();                 // seed the IIR filter

  for (int i=0; i < cal_adj_samples; i++) {  // create a moving average with an IIR filter
    average = average + (Spi_Read() - average) / 64; // maximum filterweight
    // show the progress
    lcd.setCursor(14,1);
    if (cal_counter < 10){               // get rid of the first decimal number              
      lcd.print(" ");
    }
    lcd.print(cal_counter);
    cal_counter--;
  }
  zero_cal = round(average);            // round to the next integer value
  EEPROM.put(zc_address, zero_cal);     // store binary offset calibration factor in EEPROM
 
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("Zero Cal Adjust:");
  lcd.setCursor(0,1);
  lcd.print(zero_cal);
  delay(3000);
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("mVolt Meter      ");        // print Millivolt Meter to display and clear the rest
  DecPlaces = 0;                         // if decimal places got changed, reset it
  Monitor_batt();                        // the batt level display got erased, get it back
}


 /**************************************************************************************
  * 
 * Routine to run the voltage calibration against a voltage reference.
 * The calibration factor is creatad by connecting a 2.50 reference
 * to the input. This will enhance the linearity of the conversion.
 * 
 */
void Ref_Cal_Adjust2() {
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("=2.5V-Ref Cal");
  lcd.setCursor(0,1);
  lcd.print("Connect V-Ref");
  delay(3000);

  int cal_counter = cal_adj_samples;        // total number of readings
  cal_avg = Spi_Read();                     // seed the IIR filter

  for (int i=0; i < cal_adj_samples; i++) { //create a moving average with an IIR filter
    cal_avg = cal_avg + (Spi_Read() - cal_avg) / 64;  // maximum filter weight
    // show the progress
    lcd.setCursor(14,1);
    if (cal_counter < 10){                  //get rid of the first decimal number              
      lcd.print(" ");
    }
    lcd.print(cal_counter);
    cal_counter--;
  }
  // convert filtered result to volt and include the reference, the input divider
  // and the zero cal. The zero cal must be done before the reference cal.
  float ref_volt = (cal_avg - zero_cal) * v_ref / 16777216 * 10;
  cal_factor = (cal_2_5v_ref - ref_volt);  // absolute difference
  cal_factor = 1 + (cal_2_5v_ref - ref_volt)/cal_2_5v_ref; // multiplication cal factor per volt
  EEPROM.put(cf_address, cal_factor);    // store the calibration factor in EEPROM
 
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("VRef Cal Factor:");
  lcd.setCursor(0,1);
  lcd.print(cal_factor, 6);
  delay(3000);
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("mVolt Meter      ");        // print Millivolt Meter to display and clear the rest
  DecPlaces = 0;                         // if decimal places got changed, reset it
  Monitor_batt();                        // the batt level display got erased, put it back
}


 /**************************************************************************************
  * 
 * Routine to run the voltage calibration against a voltage reference.
 * The calibration factor is creatad by connecting a 5.0V reference
 * to the input. This will enhance the linearity of the conversion.
 * 
 */
void Ref_Cal_Adjust5() {
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("=5.0V-Ref Cal");
  lcd.setCursor(0,1);
  lcd.print("Connect V-Ref");
  delay(3000);

  int cal_counter = cal_adj_samples;        // total number of readings
  cal_avg = Spi_Read();                     // seed the IIR filter

  for (int i=0; i < cal_adj_samples; i++) { //create a moving average with an IIR filter
    cal_avg = cal_avg + (Spi_Read() - cal_avg) / 64;  // maximum filter weight
    // show the progress
    lcd.setCursor(14,1);
    if (cal_counter < 10){                  //get rid of the first decimal number              
      lcd.print(" ");
    }
    lcd.print(cal_counter);
    cal_counter--;
  }
  // convert filtered result to volt and include the reference, the input divider
  // and the zero cal. The zero cal must be done before the reference cal.
  float ref_volt = (cal_avg - zero_cal) * v_ref / 16777216 * 10;
  cal_factor = (cal_5v_ref - ref_volt);  // absolute difference
  cal_factor = 1 + (cal_5v_ref - ref_volt)/cal_5v_ref; // multiplication cal factor per volt
  EEPROM.put(cf_address, cal_factor);    // store the calibration factor in EEPROM
 
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("VRef Cal Factor:");
  lcd.setCursor(0,1);
  lcd.print(cal_factor, 6);
  delay(3000);
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("mVolt Meter      ");        // print Millivolt Meter to display and clear the rest
  DecPlaces = 0;                         // if decimal places got changed, reset it
  Monitor_batt();                        // the batt level display got erased, put it back
}

 /**************************************************************************************
  * 
 * Routine to run the voltage calibration against a voltage reference.
 * The calibration factor is creatad by connecting a 5.0V reference
 * to the input. This will enhance the linearity of the conversion.
 * 
 */
void Ref_Cal_Adjust7() {
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("=7.5V-Ref Cal");
  lcd.setCursor(0,1);
  lcd.print("Connect V-Ref");
  delay(3000);

  int cal_counter = cal_adj_samples;        // total number of readings
  cal_avg = Spi_Read();                     // seed the IIR filter

  for (int i=0; i < cal_adj_samples; i++) { //create a moving average with an IIR filter
    cal_avg = cal_avg + (Spi_Read() - cal_avg) / 64;  // maximum filter weight
    // show the progress
    lcd.setCursor(14,1);
    if (cal_counter < 10){                  //get rid of the first decimal number              
      lcd.print(" ");
    }
    lcd.print(cal_counter);
    cal_counter--;
  }
  // convert filtered result to volt and include the reference, the input divider
  // and the zero cal. The zero cal must be done before the reference cal.
  float ref_volt = (cal_avg - zero_cal) * v_ref / 16777216 * 10;
  cal_factor = (cal_7_5v_ref - ref_volt);  // absolute difference
  cal_factor = 1 + (cal_7_5v_ref - ref_volt)/cal_7_5v_ref; // multiplication cal factor per volt
  EEPROM.put(cf_address, cal_factor);    // store the calibration factor in EEPROM
 
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("VRef Cal Factor:");
  lcd.setCursor(0,1);
  lcd.print(cal_factor, 6);
  delay(3000);
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("mVolt Meter      ");        // print Millivolt Meter to display and clear the rest
  DecPlaces = 0;                         // if decimal places got changed, reset it
  Monitor_batt();                        // the batt level display got erased, put it back
}

 /**************************************************************************************
  * 
 * Routine to run the voltage calibration against a voltage reference.
 * The calibration factor is creatad by connecting a 5.0V reference
 * to the input. This will enhance the linearity of the conversion.
 * 
 */
void Ref_Cal_Adjust10() {
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("=10.0V-Ref Cal");
  lcd.setCursor(0,1);
  lcd.print("Connect V-Ref");
  delay(3000);

  int cal_counter = cal_adj_samples;        // total number of readings
  cal_avg = Spi_Read();                     // seed the IIR filter

  for (int i=0; i < cal_adj_samples; i++) { //create a moving average with an IIR filter
    cal_avg = cal_avg + (Spi_Read() - cal_avg) / 64;  // maximum filter weight
    // show the progress
    lcd.setCursor(14,1);
    if (cal_counter < 10){                  //get rid of the first decimal number              
      lcd.print(" ");
    }
    lcd.print(cal_counter);
    cal_counter--;
  }
  // convert filtered result to volt and include the reference, the input divider
  // and the zero cal. The zero cal must be done before the reference cal.
  float ref_volt = (cal_avg - zero_cal) * v_ref / 16777216 * 10;
  cal_factor = (cal_10v_ref - ref_volt);  // absolute difference
  cal_factor = 1 + (cal_10v_ref - ref_volt)/cal_10v_ref; // multiplication cal factor per volt
  EEPROM.put(cf_address, cal_factor);    // store the calibration factor in EEPROM
 
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("VRef Cal Factor:");
  lcd.setCursor(0,1);
  lcd.print(cal_factor, 6);
  delay(3000);
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("mVolt Meter      ");        // print Millivolt Meter to display and clear the rest
  DecPlaces = 0;                         // if decimal places got changed, reset it
  Monitor_batt();                        // the batt level display got erased, put it back
}


/**************************************************************************************
 * 
 * routine to check if the button was pressed, and depending on the length, decide what action to take
 * 
 */
void Button_press() {
  if (digitalRead(button) == HIGH) {     // only relevant for the polling loop
    lcd.setCursor(15, 1);                // placeholder for the button press ack
    lcd.print("0");                      // back to default
  
    if (buttonActive == false) {
      buttonActive = true;
      buttonTimer = millis();
    }
    // How much time has passed, accounting for millis() fix roll-over issues with subtraction
    if ((unsigned long) (millis() - buttonTimer > longPressTime) && (longPressActive == false)) {
      longPressActive = true;
//      Ref_Cal_Adjust2();
//      Ref_Cal_Adjust5();
//      Ref_Cal_Adjust7();
      Ref_Cal_Adjust10();
    }
  } else {
    if (buttonActive == true) {
      if (longPressActive == true) {
        longPressActive = false;
      } else {
        Zero_Cal_Adjust();
      }
      buttonActive = false;
    }
  }
}


/**************************************************************************************
 * 
 * The battery level monitor routine
 * 
 */
void Monitor_batt() {
  int i;
  long sum = 0;
  int sensorValue;
  for (i=0; i<(adc_samples); i++) {
    sensorValue = analogRead(batt) + adc_cal; // read from A0 and add the calibration factor
    delay(100);
    sum += sensorValue;
  }
  sum = sum / adc_samples;

  float batt_voltage = sum;
  // Convert the analog reading (which goes from 0 - 1023) to a voltage 0 - 12V:
  batt_voltage = (sensorValue * (adc_ref_volts / adc_res) * 3) + adc_cal;
  /*  note that during the test with a potmeter connected to 5V, the resulting maximum volt level is just below
   *  20V, if you connect to a PC with a USB cable for power.
   *  With the power coming from the 78L12, the maximum level will be 20V, unless you create a voltage divider to
   *  limit the maximum voltage coming from the potmeter to 3V, which is equal to the 20K:10K divider to the cell.
   */

  // print out the "battery" voltage level at the right-hand fields of the first line
  lcd.setCursor(15, 0);                  // start of the batt level field (line 1, last position)

  if (batt_voltage < 7.3) {              // critical batt level for the reference and the ADC is < 7.3V
    lcd.print(char(6));                  // battery is empty!
    } else if (batt_voltage < 7.7 && batt_voltage > 7.3){  // batt getting too low, connect to mains or stop
      lcd.print(char(5));                // stop measuring
    } else if (batt_voltage < 8.0 && batt_voltage > 7.7){  // batt charge is getting critical
      lcd.print(char(4));                //
    } else if (batt_voltage < 8.3 && batt_voltage > 8.0){  // batt charge is OK
      lcd.print(char(3));                //
    } else if (batt_voltage < 8.7 && batt_voltage > 8.3){  // batt charge is OK
      lcd.print(char(2));                //
    } else if (batt_voltage < 8.9 && batt_voltage > 8.7){  // batt charge is OK
      lcd.print(char(1));                //
    } else if (batt_voltage < 10 && batt_voltage > 8.9){   // batt is full
      lcd.print(char(0));
    } else if (batt_voltage > 10){      // batt is charging
      lcd.print(char(7));                // charging
    }
}


/**************************************************************************************
 * 
 * The main routine
 * 
 */
void loop() {
 
  // The minimum (also normal) looptime is 166 mSec, determined by the aquisition delay of the LTC2400
 
  Button_press();                        // check if the button was pressed

  // Check the battery level approx. every minute
  if (check_v > loop_cnt) {
      check_v = 0;
      Monitor_batt();                    // checking the battery level takes about 810 mSec
  } else {
    check_v ++;
  }

  // Take a new raw LTC2400 reading
  adcread = Spi_Read();
 
  // Check if the new reading is outside the noise level band of the filtered result
  // and dynamically adjust the filter weight accordingly.
  if ((adcread > average + noise_level)||(adcread < average - noise_level)){
                                        // reading is outside the noise band
    fw_multiplier--;                    // scale the filterWeight down with powers of 2
    if (fw_multiplier < 1){             // lower limit to 2<<1 = 4
      fw_multiplier = 1;                // bottom-out
      average = adcread;                // and reset the filter
      }
  }else{                                // the reading is inside the noise band
      fw_multiplier++;                  // scale the filterWeight up with powers of 2
      if (fw_multiplier > 6){           // upper limit is 2<<5 = 128
          fw_multiplier = 6;
      }
  }
  // update the filter weight; ranges from 4..128
  filterWeight = 2 << fw_multiplier;

  /*  Run the quisition through the IIR filter and take the new reading with a
   *  grain of filterWeight salt. ie. divide the new reading by the filterWeight factor (from 4..64)
   *  Note: average must be a float, otherwise there will be a compounded rounding error in the result.
   */
  average = average + (adcread - average) / filterWeight;

  /*
   *  Convert the filtered result to volts.
   *  16777216 = 2^24, the maximum number with 24 bits.
   *  Multiply by 10 because of the voltage divider at the inputConvert the filtered result to volts.
   *  Subtract the zero calibration factor and apply the cal_factor.
   *  The cal_factor is calculated against a known voltage reference.
   *  Cast the result to a floating point variable to get decimals.
   */
  volt = (average - zero_cal) * v_ref / 16777216 * 10 * cal_factor;
 
  // prepare for the display of the data
 
  // ==>> for debugging & Testing, to be analyzed with MS-Excel:
  /*
  Serial.print(adcread);
  Serial.print("\t");
  Serial.print(average);
  Serial.print("\t");
  Serial.println(filterWeight);
  */
  //  ==>> for testing purposes only, read and display the 9V cell instead
  //  volt = (analogRead(A0) * (adc_ref_volts / adc_res) * 2) + adc_cal;

  // logging through the serial to USB interface of the Arduino can be done and monitored through the 
  // Arduino IDE Serial monitor. The display will be in Volts with 6 decimals, although this can be 
  // extended if you want. The format is always XX.YYYYYY
  // to get a timestamp with the measurements, activate that in the Arduino Serial monitor bij
  // activating the Show Timestamp option.

  currentMillis = millis();
  if ((currentMillis - startLogging) >= loggingInterval){
    logging = true;
    startLogging = millis(); // reset timer
  }else{
    logging = false;
  }

    // for logging
    if (logging){
      Serial.println(volt,6);       // display resulution 0.1 uV
    }
    
  if (volt <0.001) {                     // check if voltage reading is below 1 milli-Volt   
    volt = volt * 1000000;               // if so multiply reading by 1.000.000 and display as micro-Volt
    v = micro + "V     " ;               // use uV on display after voltage reading
    dec_digit = duV;                     // set display to 0 decimal places (1000000 uV)

    
  } else if (volt < 1){                  // check if voltage reading is below 1 volt
    volt = volt * 1000;                  // if below 1 volt multiply by 1.000 and display as Millivolt
    v = "mV     ";                       // use mV on display after voltage reading
    dec_digit = dmV;                     // set display to 4 decimal places (100.0000 mV)
    
  } else if (volt < 10){                 // check if voltage reading is below 10 volt
    v = "V     ";                        // use V on display after voltage reading
    dec_digit = dV;                      // set display to 6 decimal places (1.000000 V)
      
  } else {                               // volt is > 10V
    v = "V     ";                        // if 10 volt or higher use letter V on display after voltage reading
    dec_digit = dTV;                     // set display to 5 decimal places (10.00000 V)
  }
  
  lcd.setCursor(0, 1);                   // set LCD cursor to Column 0 and Row 1 (second row of LCD, first column)
  lcd.print(volt, dec_digit);            // print voltage as floating number with x decimal places
  lcd.print(" ");                        // add one blank space after voltage reading
  lcd.print(v);                          // print either uV, mV or V to LCD display
  lcd.setCursor(15,1);
  lcd.print(fw_multiplier);              // show the filterweight exponent multiplier
 
  ct_start = millis();                   // After this noisy display intermezzo, reset the LTC2400 conversion time counter
                                         // such that the ADC has the full 165 mSec to sample a new acquisition
                                         // during a quiet period.
}
