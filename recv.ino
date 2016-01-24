#include <SPI.h>
#include <Servo.h>
#include "RF24.h"

#define UART_BAUDRATE         115200
#define CHANGE_SPEED_INTERVAL 100
#define LED_OUTPUT_PIN        2
#define MOTOR_PIN             9
#define MOTOR_MIN             850
#define MOTOR_MAX             2300
#define ACCELERATION          10
#define DEBUG                 0


#if defined(DEBUG) && DEBUG > 0
    #define DEBUG_PRINT(arg) Serial.print(arg)
#else
    #define DEBUG_PRINT(arg)
#endif

RF24 radio(10,18);
Servo motor;

// The new types.
typedef int16_t speed_t;
typedef enum {OFF = 0, ON = !OFF} button_state_t;
typedef enum {INIT = 0, STAND, ACCEL, WATCH, DECEL} motion_state_t;

typedef struct {
  speed_t speed;
  button_state_t save_button;
  button_state_t led_button;
} packet_t;

// The global variables.
const byte address[6] = "1Node";
uint32_t previous_millis = 0;

packet_t packet;

void setup()
{
#if defined(DEBUG) && DEBUG > 0
    uart_init(UART_BAUDRATE);
#endif
    radio_init();
    gpio_init();
    motor_init();
    packet_init(); 
}

void loop()
{ 
    uint32_t current_millis = millis();

    if (current_millis - previous_millis >= CHANGE_SPEED_INTERVAL) {
        previous_millis = current_millis;

        change_speed();
    }
    receive_packet();
    change_led_state();    
}

void change_speed()
{
    static motion_state_t motion_state = INIT;
    static speed_t min_speed = 0;
    static speed_t current_speed = 0;
    static speed_t previous_speed = 0;
    
    speed_t speed;
    speed_t delta_speed;

    // Calculate the potentiometer delta speed.
    delta_speed = packet.speed - previous_speed;
    
    switch(motion_state) {
    case INIT:
        current_speed = 0;
        if (packet.speed > 0) {
            motion_state = STAND;
        }
        break;
    case STAND:
        current_speed = 0;
        if (packet.save_button == ON) {
            current_speed = packet.speed;
            motion_state = ACCEL;
        }
        break;
    case ACCEL:
        if (packet.save_button == ON) {
            if (packet.speed > current_speed) {
                current_speed += ACCELERATION;
            }    
        } else {
            current_speed = 0;
            min_speed = packet.speed;
            motion_state = WATCH;     
        }
        break;
    case WATCH:
        if (packet.save_button == OFF) {
            if (delta_speed < 0 && (abs(delta_speed) > (previous_speed - min_speed))) {
                current_speed = min_speed;
                min_speed = 0;
                motion_state = DECEL;      
            }
        } else {
            motion_state = STAND;   
        }
        break;
    case DECEL:
        if (packet.save_button == OFF) {
            if (current_speed > 0) {
                current_speed -= ACCELERATION;      
            } else {
                motion_state = INIT;    
            }
        } else {
            motion_state = STAND;   
        }
        break;
    default:
        break;
    }

    previous_speed = packet.speed;
    
    DEBUG_PRINT(F("SPEED "));
    DEBUG_PRINT(current_speed);  
    DEBUG_PRINT("\n");
    
    speed = constrain(current_speed, 0, 1023);
    speed = map(speed, 0, 1023, MOTOR_MIN, MOTOR_MAX);    
    motor.writeMicroseconds(speed);   
}

void change_led_state()
{
    if (packet.led_button == ON) {
        digitalWrite(LED_OUTPUT_PIN, HIGH);    
    } else {
        digitalWrite(LED_OUTPUT_PIN, LOW);
    }
}

void receive_packet()
{
    if(radio.available()) {                                                     // Variable for the received timestamp
        while (radio.available()) {                          
            radio.read(&packet, sizeof(packet));
        }
        
        DEBUG_PRINT(F("Recieve "));
        DEBUG_PRINT(packet.speed);  
        DEBUG_PRINT(F(" : "));
        DEBUG_PRINT(packet.save_button);  
        DEBUG_PRINT(F(" : "));
        DEBUG_PRINT(packet.led_button);
        DEBUG_PRINT("\n");
    }    
}

// The initialization routines.
void motor_init(void)
{
    motor.attach(MOTOR_PIN);
}

void gpio_init(void)
{
    pinMode(LED_OUTPUT_PIN, OUTPUT);
    pinMode(8, OUTPUT); // Instead the Vcc pin.
    digitalWrite(8, HIGH);
}

void uart_init(uint32_t baudrate)
{
    Serial.begin(baudrate);   
}

void radio_init(void)
{
    radio.begin();
    radio.openReadingPipe(1, address);
    radio.startListening();    
}

void packet_init()
{
    packet.speed = 0;
    packet.save_button = OFF;
    packet.led_button = OFF;   
}
