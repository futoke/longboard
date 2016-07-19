#include <printf.h>
#include <stdint.h>
#include <nRF24L01.h>
#include <RF24_config.h>
#include <FreqMeasure.h>
#include <SPI.h>
#include <Servo.h>
#include "RF24.h"

#define UART_BAUDRATE         9600
#define CHANGE_SPEED_INTERVAL 100
#define MOTOR_PIN             5
#define MOTOR_MIN             953 // 850 950, 953 - start // 850
#define MOTOR_MAX             1770 // 2300
#define ACCELERATION          map(50, 0, MOTOR_MAX-MOTOR_MIN, 0, 1023)  // 10
#define DECELERATION          map(100, 0, MOTOR_MAX-MOTOR_MIN, 0, 1023) // 10
#define DEBUG                 1

//define wheter to print debug information or not, depending on DEBUG constant value
#if defined(DEBUG) && DEBUG > 0
#define DEBUG_PRINT(arg) Serial.print(arg)
#else
#define DEBUG_PRINT(arg)
#endif

RF24 radio(9, 10); //radio(7, 8);
Servo motor;

// The new types.
typedef int16_t speed_t;
typedef enum {OFF = 0, ON = !OFF} button_state_t;
//typedef enum {INIT = 0, STAND, ACCEL, WATCH, DECEL} motion_state_t;

typedef struct {
  speed_t speed;
  button_state_t save_button;
} packet_t;

// Global 
double sum = 0;
int count = 0;
const byte address[6] = "1Node"; 
uint32_t previous_millis = 0;
uint32_t delta_millis = 0;
speed_t previous_speed = 0;

packet_t packet;

void setup()
{
#if defined(DEBUG) && DEBUG > 0
  uart_init(UART_BAUDRATE);
#endif
  radio_init();
//  gpio_init(); // remove it in case LED is not used (for MVP version)?
  motor_init();
  packet_init();
  freq_measure_init();
}

void loop()
{
  uint32_t current_millis = millis(); //Number of milliseconds since the program started
  
  //change the speed not more frequently than CHANGE_SPEED_INTERVAL
  delta_millis = current_millis - previous_millis;
  if (delta_millis >= CHANGE_SPEED_INTERVAL) { 
    previous_millis = current_millis;  
    change_speed();
  }
  receive_packet();
  freq_measure();
}

void change_speed()
{
  static speed_t current_speed = 0;

  speed_t speed;
  speed_t delta_speed;

  // Calculate the potentiometer delta speed
  delta_speed = packet.speed - previous_speed;

  if (packet.save_button == ON) {
    if (delta_speed < DECELERATION*delta_millis/(-1000.0)) {
      current_speed = previous_speed - DECELERATION*delta_millis/1000.0;
    }
    else if (delta_speed > ACCELERATION*delta_millis/1000.0)
    {
      current_speed = previous_speed + ACCELERATION*delta_millis/1000.0;
    }
    else if (delta_speed > DECELERATION*delta_millis/(-1000.0) && delta_speed < ACCELERATION*delta_millis/1000.0) {
      current_speed = packet.speed;
    }
    previous_speed = current_speed;
  }
  else
  {
    current_speed=0;
    previous_speed = previous_speed - DECELERATION*delta_millis/1000.0; ///// this is the most curious hypothesis (actually don't not how fast it will slow down while Idle
  }
  previous_speed = constrain(previous_speed, 0, 1023);
  speed = constrain(current_speed, 0, 1023);
  if (speed > 0) {
    speed = map(speed, 1, 1023, MOTOR_MIN, MOTOR_MAX); //zero down the PWM voltage while not forcing motor to run (intend to close electronic keys and prevent its heating) 
  }

  DEBUG_PRINT(F("Mapped SPEED "));
  DEBUG_PRINT(speed);
  DEBUG_PRINT(F(" : "));
  DEBUG_PRINT(packet.save_button);
  DEBUG_PRINT(F(" : "));
  DEBUG_PRINT(F("PrevSpeed "));
  DEBUG_PRINT(previous_speed);
  DEBUG_PRINT(F(" : "));
  DEBUG_PRINT(F("ACCELERATION "));
  DEBUG_PRINT(ACCELERATION*delta_millis/1000);
  DEBUG_PRINT(F(" : "));
  DEBUG_PRINT(F("DECELERATION "));
  DEBUG_PRINT(DECELERATION*delta_millis/1000);
  DEBUG_PRINT(F(" : "));
  DEBUG_PRINT(F("delta_millis "));
  DEBUG_PRINT(delta_millis);
  DEBUG_PRINT(F(" : "));
  DEBUG_PRINT(F("delta_speed "));
  DEBUG_PRINT(delta_speed);
  DEBUG_PRINT("\n");
 
  motor.writeMicroseconds(speed);
}

void receive_packet()
{
  if (radio.available()) {                                                   
    while (radio.available()) {
      radio.read(&packet, sizeof(packet));
    }
  }
}

void freq_measure()
{
  if (FreqMeasure.available()) {
    sum += FreqMeasure.read();
    count += 1;
    if (count > 30) {
      float frequency = FreqMeasure.countToFrequency(sum / count);
	  DEBUG_PRINT(F("Frequency: "));
	  DEBUG_PRINT(frequency);
      DEBUG_PRINT(F("\n"));
      sum = count = 0;
    }
  }
}

// The initialization routines.
void motor_init(void)
{
  motor.attach(MOTOR_PIN);
}

void uart_init(uint32_t baudrate)
{
  Serial.begin(baudrate);
}

void freq_measure_init(void)
{
  FreqMeasure.begin();
}

void radio_init(void)
{
  radio.begin();
  radio.openReadingPipe(1, address);
  radio.setPALevel(RF24_PA_LOW);
  radio.startListening();
  delay(10000);
}

void packet_init()
{
  packet.speed = 0;
  packet.save_button = OFF;
}