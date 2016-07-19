/*
 * avrdude -P /dev/ttyACM0 -b 19200 -c avrisp -p m328p -U lfuse:w:0x62:m -U hfuse:w:0xD9:m -U efuse:w:0xFF:m -U lock:w:0xFF:m -F
 */
#include <SPI.h>
#include "RF24.h"

#define POT_PIN               A0
#define SAFE_BUTTON_PIN       2
#define LED_BUTTON_PIN        3
#define UART_BAUDRATE         115200
#define DEBOUNCE_TIME         250 // ms
#define SEND_INTERVAL         250 // ms
#define DEBUG                 1

#if defined(DEBUG) && DEBUG > 0
    #define DEBUG_PRINT(arg) Serial.print(arg)
#else
    #define DEBUG_PRINT(arg)
#endif

RF24 radio(9,10);

// The new types.
typedef int16_t speed_t;
typedef enum {OFF = 0, ON = !OFF} button_state_t;

typedef struct {
  speed_t speed;
  button_state_t save_button;
  button_state_t led_button;
} packet_t;

typedef struct {
  float frequency;  
} telemetry_t;

// The global variables.
const byte addresses[][6] = {"trans", "recv"};
volatile packet_t packet;
telemetry_t telemetry;
uint32_t previous_millis = 0;
uint32_t button_time = 0;  
uint32_t last_button_time = 0;

void setup()
{
#if defined(DEBUG) && DEBUG > 0
    uart_init(UART_BAUDRATE);
#endif
    radio_init();
    gpio_init();
    packet_init();
    telemetry_init();
}

void loop()
{
    uint32_t current_millis = millis();

    if (current_millis - previous_millis >= SEND_INTERVAL) {
        previous_millis = current_millis;

        read_pot();
        read_save_state();
        send_packet();
    }   
}

void send_packet() 
{
    DEBUG_PRINT(F("Now sending "));
    DEBUG_PRINT(packet.speed);  
    DEBUG_PRINT(F(" : "));
    DEBUG_PRINT(packet.save_button);  
    DEBUG_PRINT(F(" : "));
    DEBUG_PRINT(packet.led_button);
    DEBUG_PRINT("\n");

    radio.stopListening();

    if (!radio.write((const void*)&packet, sizeof(packet))) {
        DEBUG_PRINT(F("failed"));
        DEBUG_PRINT("\n");
    }

    radio.startListening();
}

void read_telemetry()
{
    if(radio.available()){
        radio.read(&telemetry, sizeof(telemetry));
        DEBUG_PRINT(F("Telemetry (frequency): "));
        DEBUG_PRINT(telemetry.frequency);
        DEBUG_PRINT("\n");
      }
}

void read_led_state()
{
    button_time = millis();
    if (button_time - last_button_time > DEBOUNCE_TIME) {
        if (packet.led_button == OFF) {
            packet.led_button = ON;    
        } else {
            packet.led_button = OFF;    
        }
        last_button_time = button_time;
    }
}

void read_save_state()
{
    if (digitalRead(SAFE_BUTTON_PIN)) {
        packet.save_button = OFF;
    } else {
        packet.save_button = ON;   
    }
}

void read_pot(void)
{
    packet.speed = analogRead(POT_PIN); 
}

// The initialization routines.
void gpio_init(void)
{
    pinMode(SAFE_BUTTON_PIN, INPUT);
    attachInterrupt(
        digitalPinToInterrupt(LED_BUTTON_PIN),
        read_led_state,
        RISING
    );
}

void uart_init(uint32_t baudrate)
{
    Serial.begin(baudrate);   
}

void radio_init(void)
{
    radio.begin();
    radio.setPALevel(RF24_PA_LOW); // FIXME: delete in the future.
    radio.openWritingPipe(addresses[0]);
    radio.openReadingPipe(1,addresses[1]);
    radio.startListening();
    delay(10000);   
}

void packet_init()
{
    packet.speed = 0;
    packet.save_button = OFF;
    packet.led_button = OFF;   
}

void telemetry_init()
{
    telemetry.frequency = 0;  
}
