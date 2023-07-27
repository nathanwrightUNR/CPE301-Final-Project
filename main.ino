#include "LiquidCrystal.h"
#include "DS1307RTC.h"
#include "TimeLib.h"
#include "Stepper.h"
#include "RTClib.h"
#include "DHT_U.h";

#define Type DHT11
#define RDA 0x80
#define TBE 0x20

// UART setup
volatile unsigned char *myUCSR0A = (unsigned char *)0xC0;
volatile unsigned char *myUCSR0B = (unsigned char *)0xC1;
volatile unsigned char *myUCSR0C = (unsigned char *)0xC2;
volatile unsigned int  *myUBRR0  = (unsigned int *) 0xC4;
volatile unsigned char *myUDR0   = (unsigned char *)0xC6;

// Timer-Counter setup
volatile unsigned char *myTCCR1A = (unsigned char *) 0x80;
volatile unsigned char *myTCCR1B = (unsigned char *) 0x81;
volatile unsigned char *myTCCR1C = (unsigned char *) 0x82;
volatile unsigned char *myTIMSK1 = (unsigned char *) 0x6F;
volatile unsigned int  *myTCNT1  = (unsigned  int *) 0x84;
volatile unsigned char *myTIFR1 =  (unsigned char *) 0x36;

// ADC setup
volatile unsigned char* my_ADMUX = (unsigned char*) 0x7C;
volatile unsigned char* my_ADCSRB = (unsigned char*) 0x7B;
volatile unsigned char* my_ADCSRA = (unsigned char*) 0x7A;
volatile unsigned int* my_ADC_DATA = (unsigned int*) 0x78;

// Define Port D OUTPUT
unsigned char* port_d = (unsigned char*) 0x2B;
unsigned char* ddr_d = (unsigned char*) 0x2A;
volatile unsigned char* pin_d = (unsigned char*) 0x29;

// Define Port H OUTPUT
unsigned char* port_h = (unsigned char*) 0x102;
unsigned char* ddr_h = (unsigned char*) 0x101;
volatile unsigned char* pin_h = (unsigned char*) 0x100;

//Define Port K OUTPUT
unsigned char* port_k = (unsigned char*) 0x108;
unsigned char* ddr_k = (unsigned char*) 0x107;
volatile unsigned char* pin_k = (unsigned char*) 0x106;

// Define Port J INPUT
unsigned char* port_j = (unsigned char*) 0x105;
unsigned char* ddr_j = (unsigned char*) 0x104;
volatile unsigned char* pin_j = (unsigned char*) 0x103;

// Define Port E INPUT
unsigned char* port_e = (unsigned char*) 0x2E;
unsigned char* ddr_e = (unsigned char*) 0x2D;
volatile unsigned char* pin_e = (unsigned char*) 0x2C;

// Enumerating states
enum State {
  STATE_DISABLED,
  STATE_IDLE,
  STATE_ERROR,
  STATE_RUNNING
};

// Initializing state
static State currentState = STATE_DISABLED;
static State previousState = STATE_DISABLED; 
static bool messagePrinted = false;

// STPMTR
// Number of steps per internal motor revolution 
const float STEPS_PER_REV = 32; 
 
//  Amount of Gear Reduction
const float GEAR_RED = 64;
 
// Number of steps per geared output rotation
const float STEPS_PER_OUT_REV = STEPS_PER_REV * GEAR_RED;
 
// Define Variables
 
// Number of Steps Required
int StepsRequired;
Stepper stepper = Stepper(STEPS_PER_REV, 22, 26, 24, 28);

// LCD
const int rs = 12, en = 11, d4 = 10, d5 = 9, d6 = 8, d7 = 7;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

// RTC
RTC_DS1307 rtc;

// DHT
DHT dht(4, Type);

void setup() {
  // initialize UART
  U0init(9600);
  
  // delay for serial monitor communication
  delay_ms(500);

  // initialize ADC, display, clock and sensor
  adc_init(); 
  lcd.begin(16, 2);
  rtc.begin();
  dht.begin();

  // GPIO OUTPUT setup
  *ddr_j  |= (0x1 << 0);
  *ddr_j  |= (0x1 << 1);
  *ddr_h  |= (0x1 << 3);
  *ddr_h  |= (0x1 << 0);
  *ddr_k  |= (0x1 << 4);
  *ddr_k  |= (0x1 << 5);
  *ddr_k  |= (0x1 << 6);

  // DC motor setup
  *port_j |=  (0x1 << 0);
  *port_j &= ~(0x1 << 1);
  
  // GPIO INPUT setup
  *ddr_d  &= ~(0x1 << 2);
  *port_d |=  (0x1 << 2);
  *ddr_e  &= ~(0x1 << 0);
  *port_e |=  (0x1 << 0);
  
  // synch date and time
  setSyncProvider(RTC.get);
  rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  if(timeStatus()!= timeSet) {
     U0putstring("Unable to sync RTC\n");
  }
  else {
     U0putstring("RTC has set the system time\n"); 
  }
}

void loop() {
  // power state variables
  static int powerState = 0;
  static int reset = 0;
  static int adc_value = 0;
  // turn off leds 
  *port_h &= ~(0x1 << 3);
  *port_k &= ~(0x1 << 6);
  *port_k &= ~(0x1 << 5);

  // turn ON button
  if (*pin_d & (0x1 << 2)) {
    delay_ms(200);
    if (*pin_d & (0x1 << 2)) {
      powerState = !powerState;
      currentState = powerState ? STATE_IDLE : STATE_DISABLED;
    }
  }
  // switch-case depending on state
  switch (currentState) {
    case STATE_DISABLED: 
    {
      // turn on yellow LED
      *port_k |= (0x1 << 5);  
      // turn off green LED   
      *port_h &= ~(0x1 << 0); 
      lcd.clear();
      // only display state if it has changed
      if (currentState != previousState) {
          U0putstring("State: DISABLED\n");
          displayStateDateAndTime();
          previousState = currentState;
          if (currentState == STATE_DISABLED) {
              messagePrinted = true;
          }
      } 
      else if (currentState == STATE_DISABLED && !messagePrinted) {
          U0putstring("State: DISABLED\n");
          displayStateDateAndTime();
          messagePrinted = true;
      } 
      // turn off red LED
      *port_k &= ~(0x1 << 4);
      // start reading water sensor
      adc_value = adc_read(0);
      reset = 0;
      // state logic for reseting from ERROR
      if ((*pin_d & (0x1 << 2)) && (currentState == STATE_DISABLED)) {
        reset = 1;
      }
      // if water level is fine and button was pressed enter IDLE
      if (reset == 1 && adc_value > 300) {
        *port_k &=   ~(0x1 << 4);
        currentState = STATE_IDLE;
      }
      // move the vent angle with the up button
      if (*pin_e & (0x1 << 4)) {
        delay_ms(100);
        if (*pin_e & (0x1 << 4)) {
          StepsRequired  =  STEPS_PER_OUT_REV / 16; 
          stepper.setSpeed(100);   
          stepper.step(StepsRequired);
          U0putstring("vent moved up by 22.5 degrees\n");
        }  
      }
      // move the vent angle with the down button
      else if (*pin_e & (0x1 << 5)) {
        delay_ms(100);
        if (*pin_e & (0x1 << 5)) {
          StepsRequired  =  STEPS_PER_OUT_REV / 16; 
          stepper.setSpeed(100);   
          stepper.step(-StepsRequired);
          U0putstring("vent moved down by 22.5 degrees\n");
        }  
      }
      break;
    }
    
    case STATE_IDLE: {
      // print state
      if (currentState != previousState) {
        U0putstring("State: IDLE\n");
        displayStateDateAndTime();
        previousState = currentState;
      }
      // turn on green LED
      *port_h |= (0x1 << 0);
      // turn OFF button
      if ((*pin_d & (0x1 << 2)) && (currentState == STATE_IDLE)) {
        currentState = STATE_DISABLED;
        *port_h &= ~(0x1 << 0);
      }
      // monitor temperature and humidity, display on LCD
      int temperature = dht.readTemperature();
      int humidity = dht.readHumidity();
      lcd.setCursor(0, 0);
      lcd.print("Temperature: ");
      lcd.print(temperature);
      lcd.setCursor(0, 1);
      lcd.print("Humidity: ");
      lcd.print(humidity);
      // move vent angle up
      if (*pin_e & (0x1 << 4)) {
        delay_ms(100);
        if (*pin_e & (0x1 << 4)) {
          StepsRequired  =  STEPS_PER_OUT_REV / 16; 
          stepper.setSpeed(100);   
          stepper.step(StepsRequired);
          U0putstring("Vent moved up by 22.5 degrees\n");
        }  
      }
      // move vent angle down
      else if (*pin_e & (0x1 << 5)) {
        delay_ms(100);
        if (*pin_e & (0x1 << 5)) {
          StepsRequired  =  STEPS_PER_OUT_REV / 16; 
          stepper.setSpeed(100);   
          stepper.step(-StepsRequired);
          U0putstring("Vent moved down by 22.5 degrees\n");
        }  
      }
      // monitor water level
      int adc_value = adc_read(0);
      if (adc_value < 200) {
        delay_ms(100);
        lcd.clear();
        currentState = STATE_ERROR;
        return true;
        break;
      }
      // if temp/humidity is above a certain threshold enter RUNNING
      if (temperature > 26 && (currentState == STATE_IDLE)) {
        currentState = STATE_RUNNING;
      }
      break;
    }
    
    case STATE_RUNNING: {
      // print state message
      if (currentState != previousState) {
        U0putstring("State: RUNNING\n");
        displayStateDateAndTime();
        previousState = currentState;
      }
      // turn off green LED, turn on blue LED
      *port_h &= ~(0x1 << 0);
      *port_k |= (0x1 << 6);
      // run fan called 
      if (dht.readTemperature() > 25) {
        // if runFan returns true, the water level is LOW
        // enter ERROR state
        if (runFan()) {
          currentState = STATE_ERROR;
        }
      }
      // return to IDLE
      else {
        currentState = STATE_IDLE;
      }
      break;

    case STATE_ERROR:
    // print state data
      if (currentState != previousState) {
        U0putstring("State: ERROR\n");
        displayStateDateAndTime();
        previousState = currentState;
      }
      // display message to lcd
      lcd.setCursor(0, 0);
      lcd.print("WATER LEVEL LOW");
      // turn on red LED
      *port_h &= ~(0x1 << 0);
      *port_k |= (0x1 << 4);
      break;
    }
  }
}

void U0init(int U0baud) {
 unsigned long FCPU = 16000000;
 unsigned int tbaud;
 tbaud = (FCPU / 16 / U0baud - 1);
 *myUCSR0A = 0x20;
 *myUCSR0B = 0x18;
 *myUCSR0C = 0x06;
 *myUBRR0  = tbaud;
}

unsigned char U0kbhit() {
  return *myUCSR0A & RDA;
}

unsigned char U0getchar() {
  return *myUDR0;
}

void U0putstring(char* U0pstring) {
  while(*U0pstring) { 
      while(!(*myUCSR0A & TBE));
      *myUDR0 = *U0pstring;
      U0pstring++;
  }
}

void U0putint(int value) {
  char str[12];
  if (value < 10) {
    sprintf(str, "0%d", value);
  } else {
    sprintf(str, "%d", value);
  }
  U0putstring(str);
}

void delay_ms(double delay_time) {
  double clk_period;
  unsigned int prescaler;
  // determine which prescaler to use
  if (delay_time <= 4) {
    prescaler = 1;
    clk_period = 0.0000000625;
  } else if (delay_time <= 33) {
    prescaler = 8;
    clk_period = 0.0000000625 * 8;
  } else if (delay_time <= 262) {
    prescaler = 64;
    clk_period = 0.0000000625 * 64;
  } else if (delay_time <= 1049) {
    prescaler = 256;
    clk_period = 0.0000000625 * 256;
  } else if (delay_time <= 4200) {
    prescaler = 1024;
    clk_period = 0.0000000625 * 1024;
  } else {
    return;
  }
  // caluclate ticks
  unsigned long ticks = delay_time / 1000 / clk_period;
  *myTCCR1B &= 0xF8;
  *myTCNT1 = (unsigned int) (65536 - ticks);
  *myTCCR1A = 0x0;
  // set prescale bit
  switch(prescaler) {
    case 1:    *myTCCR1B |= 0b00000001; break;
    case 8:    *myTCCR1B |= 0b00000010; break;
    case 64:   *myTCCR1B |= 0b00000011; break;
    case 256:  *myTCCR1B |= 0b00000100; break;
    case 1024: *myTCCR1B |= 0b00000101; break;
    default: break;
  }
  // begin counting
  while((*myTIFR1 & 0x01)==0) {}
  *myTCCR1B &= 0xF8;
  *myTIFR1 |= 0x01;
}

void displayStateDateAndTime() {
  // digital clock display of the time
  U0putint(hour());
  printDigits(minute());
  printDigits(second());
  U0putstring(" ");
  U0putint(day());
  U0putstring(" ");
  U0putint(month());
  U0putstring(" ");
  U0putint(year());
  U0putstring("\n");
}

// used to print colons when necessary
void printDigits(int digits){
  U0putstring(":");
  if(digits < 10) {
    U0putstring('0');
  }
  U0putint(digits);
}

// acd init from prev lab
void adc_init() {
  *my_ADCSRA |= 0b10000000;
  *my_ADCSRA &= 0b11011111;
  *my_ADCSRA &= 0b11110111;
  *my_ADCSRA &= 0b11111000;
  *my_ADCSRB &= 0b11110111;
  *my_ADCSRB &= 0b11111000;
  *my_ADMUX  &= 0b01111111;
  *my_ADMUX  |= 0b01000000;
  *my_ADMUX  &= 0b11011111;
  *my_ADMUX  &= 0b11100000;
}

// adc read function from prev lab
unsigned int adc_read(unsigned char adc_channel_num) {
  *my_ADMUX  &= 0b11100000;
  *my_ADCSRB &= 0b11110111;
  if(adc_channel_num > 7) {
    adc_channel_num -= 8;
    *my_ADCSRB |= 0b00001000;
  }
  *my_ADMUX  += adc_channel_num;
  *my_ADCSRA |= 0x40;
  while((*my_ADCSRA & 0x40) != 0);
  return *my_ADC_DATA;
}

bool runFan() {
  // monitor water level, humidity and temperature
  // display on LCD
  while (dht.readHumidity() > 36) {
    *port_h |= (0x1 << 3);
    int temp = dht.readTemperature();
    int hum = dht.readHumidity();
    lcd.setCursor(0, 0);
    lcd.print("Temperature: ");
    lcd.print(temp);
    lcd.setCursor(0, 1);
    lcd.print("Humidity: ");
    lcd.print(hum);
    // turn OFF button
    if (*pin_d & (0x1 << 2)) {
      delay_ms(200);
      if (*pin_d & (0x1 << 2)) {
        currentState = STATE_DISABLED;
        break;
      }
      break;
    }
    // vent angle control
    if (*pin_e & (0x1 << 4)) {
      delay_ms(100);
      if (*pin_e & (0x1 << 4)) {
        StepsRequired  =  STEPS_PER_OUT_REV / 16; 
        stepper.setSpeed(100);   
        stepper.step(StepsRequired);
        U0putstring("vent moved up by 22.5 degrees\n");
      }  
    }
    else if (*pin_e & (0x1 << 5)) {
      delay_ms(100);
      if (*pin_e & (0x1 << 5)) {
        StepsRequired  =  STEPS_PER_OUT_REV / 16; 
        stepper.setSpeed(100);   
        stepper.step(-StepsRequired);
        U0putstring("vent moved down by 22.5 degrees\n");
      }  
    }
    // if water level drops too low enter ERROR
    int adc_value = adc_read(0);
    if (adc_value < 200) {
      delay_ms(100);
      lcd.clear();
      currentState = STATE_ERROR;
      return true;
      break;
    }
    return false;
  }
}