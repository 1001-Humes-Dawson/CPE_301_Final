//Dawson Humes
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>//humidity and temperature sensor library
#include <RTClib.h>//RTC library includes DS1307
#include <Stepper.h>//stepper motor library
//stepper motor
const int stepsPerRevolution = 2048;
const int rolePerMinute = 15; 
Stepper myStepper(stepsPerRevolution, 22, 25, 23, 24);
//Clock
RTC_DS1307 rtc;
char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};
//temp and humidity
#define DHT11_PIN 2
#define DHTTYPE DHT11 // DHT 11
DHT_Unified dht(DHT11_PIN, DHTTYPE);
uint32_t delayMS;
//water level
const int sensorPin = 0; //sensor pin connected to analog pin A0
int liquid_level = 0;
// UART Pointers
volatile unsigned char *myUCSR0A = (unsigned char*) 0x00C0;
volatile unsigned char *myUCSR0B = (unsigned char*) 0x00C1;
volatile unsigned char *myUCSR0C = (unsigned char*) 0x00C2;
volatile unsigned int  *myUBRR0 = (unsigned int*) 0x00C4;
volatile unsigned char *myUDR0 = (unsigned char*) 0x00C6;
// GPIO Pointers
volatile unsigned char* pin_a = (unsigned char*) 0x20;
volatile unsigned char* port_a = (unsigned char*) 0x22;
volatile unsigned char* ddr_a = (unsigned char*) 0x21;

volatile unsigned char* port_b = (unsigned char*) 0x25; 
volatile unsigned char* ddr_b  = (unsigned char*) 0x24; 
volatile unsigned char* pin_b  = (unsigned char*) 0x23; 

volatile unsigned char *pin_d = (unsigned char*) 0x29;
volatile unsigned char *port_d = (unsigned char*) 0x2B;
volatile unsigned char *ddr_d = (unsigned char*) 0x2A;

volatile unsigned char *pin_e = (unsigned char*) 0x2C;
volatile unsigned char *port_e = (unsigned char*) 0x2E;
volatile unsigned char *ddr_e = (unsigned char*) 0x2D;

volatile unsigned char *pin_h = (unsigned char*) 0x100;
volatile unsigned char *port_h = (unsigned char*) 0x102;
volatile unsigned char *ddr_h = (unsigned char*) 0x101;

volatile unsigned char *pin_g = (unsigned char*) 0x32;
volatile unsigned char *port_g = (unsigned char*) 0x34;
volatile unsigned char *ddr_g = (unsigned char*) 0x33;
// Timer Pointers
volatile unsigned char *myTCCR1A = (unsigned char *) 0x80;
volatile unsigned char *myTCCR1B = (unsigned char *) 0x81;
volatile unsigned char *myTCCR1C = (unsigned char *) 0x82;
volatile unsigned char *myTIMSK1 = (unsigned char *) 0x6F;
volatile unsigned char *myTIFR1 = (unsigned char *) 0x36;
volatile unsigned int  *myTCNT1 = (unsigned int *) 0x84;

volatile unsigned char *myTCCR3A = (unsigned char*) 0x90;
volatile unsigned char *myTCCR3B = (unsigned char*) 0x91;
volatile unsigned char *myTCCR3C = (unsigned char*) 0x92;
volatile unsigned char *myTIMSK3 = (unsigned char*) 0x71;
volatile unsigned char *myTIFR3 = (unsigned char *) 0x38;
volatile unsigned int  *myTCNT3 = (unsigned int*)  0x94;
//ADC
volatile unsigned char* my_ADMUX = (unsigned char*) 0x7C;
volatile unsigned char* my_ADCSRB = (unsigned char*) 0x7B;
volatile unsigned char* my_ADCSRA = (unsigned char*) 0x7A;
volatile unsigned int* my_ADC_DATA = (unsigned int*) 0x78;
//Interupts
volatile unsigned char *myEICRB = (unsigned char *) 0x6A;
volatile unsigned char *myEIMSK = (unsigned char *) 0x3D;
//other variables
#define RDA 0x80
#define TBE 0x20
#define liquid_min 150
#define liquid_max 400
#define temp_threshold 70
float current_temp = 0;
int state = 1;
bool off_toggle = true;
bool fan_toggle = false;
bool timer1_running;
unsigned long currentTicks;
byte in_char;

void setup() 
{
  U0Init(9600);
  adc_init();
  setup_timer_regs();
  #ifndef ESP8266
  while (!Serial); // wait for serial port to connect. Needed for native USB
  #endif
  if(! rtc.begin())
  {
    Serial.flush();
    while (1) delay(10);
  }
  if (! rtc.isrunning())
  {
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }
  dht.begin();
  myStepper.setSpeed(rolePerMinute);
  *ddr_h |= 0x01 << 6;//set outputs
  *ddr_b |= 0x01 << 6;
  *ddr_b |= 0x01 << 5;
  *ddr_b |= 0x01 << 4;
  *ddr_h |= 0x01 << 3;
  *ddr_h |= 0x01 << 4;
  *ddr_e |= 0x01 << 3;
  *ddr_g |= 0x01 << 5; 
  *ddr_a &= ~(0x01 << 0);//analog input for water level
  *ddr_d &= ~(0x01 << 1);//SDA input
  *ddr_d &= ~(0x01 << 0);//SCL input
  *ddr_d &= ~(0x01 << 2); //set inputs for interupts
  *ddr_e &= ~(0x01 << 4);
  *port_d |= 0b00000100; 
  *ddr_e &= ~(0x01 << 5); 
  *port_e |= 0b00100000;
  *myEIMSK = 0b00100100; // enable interrupt on INT 5 and 2 - PINS E5 AND D2
  *myEICRB = 0b00001000; // ICS51 set to 1, ISC50 set to 0 - Falling edge samples
}

void loop() 
{
  if(state == 1)
  {
    led_yellow();
    turn_off_fan();
  }
  else if(state == 2)
  {
    display_dht();
    led_green();
    turn_off_fan();
    fan_toggle = false;
    if(current_temp > temp_threshold)
    {
      state = 3;
      print_time();
    }
  }
  else if(state == 3)
  {
    display_dht();
    led_blue();
    turn_on_fan();
    if(current_temp < temp_threshold)
    {
      state = 2;
      print_time();
    }
  }
  else if(state == 4)
  {
    display_dht();
    display_error();
    led_red();
    turn_off_fan();
  }
  *port_h |= 0x01 << 4;//set power for sensor high to measure 
  liquid_level = adc_read(0);
  *port_h &= ~(0x01 << 4);//set low to prevent damage from continuos power
  if( (liquid_level < liquid_min) || (liquid_level > liquid_max) )
  {
    state = 4;
    print_time();
  }
}

void print_time()
{
  DateTime now = rtc.now();
  Serial.print("Fan toggled: ");
  Serial.print(now.year(), DEC);
    Serial.print('/');
    Serial.print(now.month(), DEC);
    Serial.print('/');
    Serial.print(now.day(), DEC);
    Serial.print(" (");
    Serial.print(daysOfTheWeek[now.dayOfTheWeek()]);
    Serial.print(") ");
    Serial.print(now.hour(), DEC);
    Serial.print(':');
    Serial.print(now.minute(), DEC);
    Serial.print(':');
    Serial.print(now.second(), DEC);
    Serial.println();
}

void display_error()
{
  putchar("ERROR");
}
//DHT measurement
void display_dht()
{
  // Get temperature event and print its value.
  sensors_event_t event;
  dht.temperature().getEvent(&event);
  if (isnan(event.temperature)) 
  {
    Serial.println(F("Error reading temperature!"));
  }
  else 
  {
    Serial.print(F("Temperature: "));
    Serial.print(event.temperature);
    Serial.println(F("°C"));
  }
  // Get humidity event and print its value.
  dht.humidity().getEvent(&event);
  if (isnan(event.relative_humidity)) 
  {
    Serial.println(F("Error reading humidity!"));
  }
  else 
  {
    Serial.print(F("Humidity: "));
    Serial.print(event.relative_humidity);
    Serial.println(F("%"));
  }
}
//LED, Set designated LED high and the rest low
void led_yellow()
{
  *port_b |= 0x01 << 4;
  *port_b &= ~(0x01 << 5);
  *port_b &= ~(0x01 << 6);
  *port_h &= ~(0x01 << 6);
}

void led_green()
{
  *port_b |= 0x01 << 6;
  *port_b &= ~(0x01 << 5);
  *port_b &= ~(0x01 << 4);
  *port_h &= ~(0x01 << 6);
}

void led_blue()
{
  *port_h |= 0x01 << 6;
  *port_b &= ~(0x01 << 5);
  *port_b &= ~(0x01 << 6);
  *port_b &= ~(0x01 << 4);
}

void led_red()
{
  *port_b |= 0x01 << 5;
  *port_b &= ~(0x01 << 4);
  *port_b &= ~(0x01 << 6);
  *port_h &= ~(0x01 << 6);
}

void turn_on_fan(){
  *port_g |= 0x01 << 5;//set fan inputs high
  *port_e |= 0x01 << 3;
  *ddr_h |= 0x01 << 3; 
}

void turn_off_fan(){
  *port_g &= ~(0x01 << 5);//set fan inputs low
  *port_e &= ~(0x01 << 3);
  *ddr_h &= ~(0x01 << 3); 
}
// Timer setup function
void setup_timer_regs()
{
  // setup the timer control registers
  *myTCCR1A = 0x00;
  *myTCCR1B = 0x00;
  *myTCCR1C = 0x00;
  
  // reset the TOV flag
  *myTIFR1 |= 0x01;
  
  // enable the TOV interrupt
  *myTIMSK1 |= 0x01;
}
//ISR
ISR(INT5_vect)
{
  // Set count to 0
  *myTCNT1 = 0;
  // Start the timer
  *myTCCR1B |= 0b00000001; // no prescaler
  timer1_running = true;
}
ISR(INT2_vect)
{
  // Set count to 0
  *myTCNT3 = 0;
  // Start the timer
  *myTCCR3B |= 0b00000001; // no prescaler
}
// TIMER OVERFLOW ISR
ISR(TIMER1_OVF_vect)//start or stop
{
  // Stop the Timer
  *myTCCR1B &= 0b11111000; //CSn2:0 set to 0 - no clock source
  timer1_running = false;
  //switch between disabled and idle states
  if(off_toggle == false){
    off_toggle = true;
    state = 1;
    print_time();
  }
  else if (off_toggle == true){
    off_toggle = false;
    state = 2;
    print_time();
  }
}
ISR(TIMER3_OVF_vect)//reset
{
  // Stop the Timer
  *myTCCR3B &= 0b11111000;
    // change state to idle
  if(state != 2){
    state = 2;
    print_time();
  }
}
//ADC
void adc_init()
{
  // setup the A register
  *my_ADCSRA |= 0b10000000; // set bit   7 to 1 to enable the ADC
  *my_ADCSRA &= 0b11011111; // clear bit 6 to 0 to disable the ADC trigger mode
  *my_ADCSRA &= 0b11110111; // clear bit 5 to 0 to disable the ADC interrupt
  *my_ADCSRA &= 0b11111000; // clear bit 0-2 to 0 to set prescaler selection to slow reading
  // setup the B register
  *my_ADCSRB &= 0b11110111; // clear bit 3 to 0 to reset the channel and gain bits
  *my_ADCSRB &= 0b11111000; // clear bit 2-0 to 0 to set free running mode
  // setup the MUX Register
  *my_ADMUX  &= 0b01111111; // clear bit 7 to 0 for AVCC analog reference
  *my_ADMUX  |= 0b01000000; // set bit   6 to 1 for AVCC analog reference
  *my_ADMUX  &= 0b11011111; // clear bit 5 to 0 for right adjust result
  *my_ADMUX  &= 0b11100000; // clear bit 4-0 to 0 to reset the channel and gain bits
}

unsigned int adc_read(unsigned char adc_channel_num)
{
  // clear the channel selection bits (MUX 4:0)
  *my_ADMUX  &= 0b11100000;
  // clear the channel selection bits (MUX 5)
  *my_ADCSRB &= 0b11110111;
  // set the channel number
  if(adc_channel_num > 7)
  {
    // set the channel selection bits, but remove the most significant bit (bit 3)
    adc_channel_num -= 8;
    // set MUX bit 5
    *my_ADCSRB |= 0b00001000;
  }
  // set the channel selection bits
  *my_ADMUX  += adc_channel_num;
  // set bit 6 of ADCSRA to 1 to start a conversion
  *my_ADCSRA |= 0x40;
  // wait for the conversion to complete
  while((*my_ADCSRA & 0x40) != 0);
  // return the result in the ADC data register
  return *my_ADC_DATA;
}
//UART
void U0Init(int U0baud)
{
 unsigned long FCPU = 16000000;
 unsigned int tbaud;
 tbaud = (FCPU / 16 / U0baud - 1);// Same as (FCPU / (16 * U0baud)) - 1;
 *myUCSR0A = 0x20;
 *myUCSR0B = 0x18;
 *myUCSR0C = 0x06;
 *myUBRR0  = tbaud;
}
unsigned char kbhit()
{
  return *myUCSR0A & RDA;
}
unsigned char getChar()
{
  return *myUDR0;
}
void putChar(unsigned char U0pdata)
{
  while ((*myUCSR0A & TBE) == 0);
  *myUDR0 = U0pdata;
}
