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
#define DEBUG                 1
#define JITTER                10

#define ROUND_ACCEL(x)        (((x + (ACCELERATION / 2)) / ACCELERATION) * ACCELERATION)

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

typedef struct {
  speed_t speed;
  button_state_t save_button;
  button_state_t led_button;
} packet_t;

// The global variables.
const byte address[6] = "1Node";
packet_t packet;
uint32_t previous_millis = 0;
speed_t speed = 0;
speed_t previous_speed = 0;

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
    static uint8_t break_motor = 0;
    
    speed_t delta_speed;
    speed_t current_speed;
    speed_t val_speed;
     
    if (break_motor == 0) {
        if (packet.save_button == ON) {
            if (ROUND_ACCEL(packet.speed) > speed) {
                speed += ACCELERATION;
            }
//            } else if (ROUND_ACCEL(packet.speed) < speed) {
//                speed = 0;
//            }
        } else {
            current_speed = ROUND_ACCEL(packet.speed);
            delta_speed = (current_speed - previous_speed);
            
            if (delta_speed < 0 && abs(delta_speed) > JITTER && speed >= 0) {
                break_motor = 1;
                speed = current_speed; 
            } else {
                if (speed > 0) {
                    speed = 0;
                }
            }
            
            previous_speed = current_speed;
        }
    } else {
        if (ROUND_ACCEL(speed) > ACCELERATION) {
            speed -= ACCELERATION;    
        } else {
            break_motor = 0;
            previous_speed = 0;
            speed = -1;
        }
    }
    
    DEBUG_PRINT(F("SPEED "));
    DEBUG_PRINT(speed);  
    DEBUG_PRINT("\n");

    if (speed < 0) {
        val_speed = 0;
    } else if (speed > 1023) {
        val_speed = 1023;
    } else {
        val_speed = speed;
    }
     
    val_speed = map(val_speed, 0, 1023, MOTOR_MIN, MOTOR_MAX);    
    motor.writeMicroseconds(val_speed);

    
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
    //radio.setPALevel(RF24_PA_LOW); // FIXME: delete in the future.
    radio.openReadingPipe(1, address);
    radio.startListening();    
}

void packet_init()
{
    packet.speed = 0;
    packet.save_button = OFF;
    packet.led_button = OFF;   
}