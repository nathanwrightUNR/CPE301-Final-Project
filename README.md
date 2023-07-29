# CPE301-Final-Project

This is a program for an Arduino-based water cooler control system. The system monitors temperature, humidity, and water level, and controls a fan and a vent to maintain optimal conditions. 

## Libraries

The following libraries are used in this program:

- `LiquidCrystal.h`: This library allows you to control LCD displays. It is used for displaying temperature, humidity, and water level data.
- `DS1307RTC.h`: This library allows you to access real-time clock (RTC) chips. It is used for timekeeping.
- `TimeLib.h`: This library provides timekeeping functionality. It is used in conjunction with the RTC library.
- `Stepper.h`: This library allows you to control stepper motors. It is used for controlling the vent.
- `RTClib.h`: This is another library for accessing RTC chips. It provides additional functionality compared to DS1307RTC.h.
- `DHT_U.h`: This library allows you to read from DHT humidity and temperature sensors.

## Constants

- `Type DHT11`: This constant defines the type of DHT sensor used.
- `RDA 0x80` and `TBE 0x20`: These constants are used for UART setup.

## Variables

The program uses a number of variables to store the state of the system and the values read from the sensors. These include:

- `myUCSR0A`, `myUCSR0B`, `myUCSR0C`, `myUBRR0`, `myUDR0`: These are used for UART setup.
- `myTCCR1A`, `myTCCR1B`, `myTCCR1C`, `myTIMSK1`, `myTCNT1`, `myTIFR1`: These are used for Timer-Counter setup.
- `my_ADMUX`, `my_ADCSRB`, `my_ADCSRA`, `my_ADC_DATA`: These are used for ADC setup.
- `port_d`, `ddr_d`, `pin_d`, `port_h`, `ddr_h`, `pin_h`, `port_k`, `ddr_k`, `pin_k`, `port_j`, `ddr_j`, `pin_j`, `port_e`, `ddr_e`, `pin_e`: These are used for defining input and output ports.
- `State`: This is an enumeration of the possible states of the system: `STATE_DISABLED`, `STATE_IDLE`, `STATE_ERROR`, `STATE_RUNNING`.
- `currentState`, `previousState`: These variables store the current and previous states of the system.
- `messagePrinted`: This variable is used to control the printing of state messages.

## Functions

The program includes a number of functions for controlling the system and reading from the sensors. These include:

- `setup()`: This function initializes the system.
- `loop()`: This function is the main loop of the program. It checks the state of the system and takes appropriate actions.
- `U0init(int U0baud)`, `U0kbhit()`, `U0getchar()`, `U0putstring(char* U0pstring)`, `U0putint(int value)`: These functions are used for UART communication.
- `delay_ms(double delay_time)`: This function is used to create a delay.
- `displayStateDateAndTime()`, `printDigits(int digits)`: These functions are used to display the state, date, and time.
- `adc_init()`, `adc_read(unsigned char adc_channel_num)`: These functions are used for ADC setup and reading.
- `runFan()`: This function is used to control the fan.

## Usage

To use this program, upload it to your Arduino board using the Arduino IDE. The system will start in the `STATE_DISABLED` state. Press the ON button to switch to the `STATE_IDLE` state. The system will then monitor the temperature, humidity, and water level, and control the fan and vent as necessary. If the water level drops too low, the system will enter the `STATE_ERROR` state. Press the ON button again to return to the `STATE_DISABLED` state.