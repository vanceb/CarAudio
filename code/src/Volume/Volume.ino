//#include <avr/interrupt.h>
#include <Adafruit_NeoPixel.h>

#define PUSH_BUTTON 3
#define ROTARY_A 9
#define ROTARY_B 10
#define NEO_PIN 4
#define NUM_PIXELS 12
#define LED_PIN A3

#define DEBUG 1

char message[140];
int spin_count = 0;

// Constants and variables used for debounce
#define CHECK_MS 1
#define DEBOUNCE_MS 1
#define TURN_MULTIPLIER 3

volatile static uint8_t debounced_position = 0;
volatile static int16_t turned_by = 0;
static uint8_t count =  DEBOUNCE_MS / CHECK_MS;


// Define the Neopixels
Adafruit_NeoPixel ring = Adafruit_NeoPixel(NUM_PIXELS, NEO_PIN, NEO_GRB + NEO_KHZ800);
uint8_t brightness = 50;
uint32_t colour = ring.Color(255,0,0);


void log(const char* msg){
    #ifdef DEBUG
    Serial.println(msg);
    #endif
}

void setup() {
    #ifdef DEBUG
    // Setup the Serial output
    Serial.begin(9600);
    Serial.println("Debug logging started");
    #endif
    log("Performing setup");

    // Setup the LED Enable
    pinMode(A4, OUTPUT);
    digitalWrite(A4, HIGH);
  
    // Setup pins
    pinMode(PUSH_BUTTON, INPUT);
    pinMode(ROTARY_A, INPUT);
    pinMode(ROTARY_B, INPUT);

    // Setup Timer Interrupts
    cli(); // Disable interrupts
    // Reset the Timer 2 Control Registers
    TCCR2A = 0;
    TCCR2B = 0;
    // Set the timer comparison value
    // 8MHZ, 1024 prescale gives 128us per tick
    // 0.5ms interrupt = 0.5e-3/128e-6 ~= 4
    OCR2A = 5;
    //Turn on CTC mode
    TCCR2B != (1 << WGM12);
    // Set CS10 and CS12 bits for the 1024 prescaler
    TCCR2B |= (1 << CS10);
    TCCR2B |= (1 << CS12);
    //Enable the compare interrupt for Timer 2
    TIMSK2 = (1 << OCIE1A);
    // Set the prescaler
    TCCR2B != (1 << CS10);
    sei(); // Enable interrupts

    // Initialise the Neopixel ring
    ring.begin();
    updateDisplay();

    // Turn the LED on
    pinMode(LED_PIN, OUTPUT);
}

void updateDisplay() {
  for (int i = 0; i < NUM_PIXELS; i++) {
    if ( brightness > (i * 255 / NUM_PIXELS)) {
      ring.setPixelColor(i, colour);
    } else {
      ring.setPixelColor(i,0);
    }
  }
  ring.setBrightness(brightness);
  ring.show();
}

// Timer ISR
ISR(TIMER2_COMPA_vect) {
    debounceRotary();
}

void loop() {
    delay(10);
    log(turned_by);
    process_input();
    updateDisplay();
}

void process_input() {
    int tb = turned_by;
    turned_by = 0;
    //sprintf(message, "Turned by: %d", tb);
    //log(message);
    if (tb != 0) {
        //digitalWrite(LED_PIN, digitalRead(LED_PIN) ^ 1);
        int new_brightness = brightness + (tb * TURN_MULTIPLIER);
        if (new_brightness < 0) {
            brightness = 0;
        } else if (new_brightness > 255) {
            brightness = 255;
        } else {
            brightness = (uint8_t) new_brightness;
        }
        turned_by = 0;
        analogWrite(LED_PIN, brightness);
    }
    //sprintf(message, "Brightness: %d", brightness);
    //log(message);
}

void button() {
    log("Button pressed");
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
}

void spin() {
    bool a = digitalRead(ROTARY_A);
    bool b = digitalRead(ROTARY_B);
    if (a == 0) {
      if (b == 0) {
        spin_count++;
      } else {
        spin_count--;
      }
    }
    sprintf(message, "%d", spin_count);
    log(message);
}

void debounceRotary () {
    uint8_t a = digitalRead(ROTARY_A);
    uint8_t b = digitalRead(ROTARY_B);
    uint8_t position = state(a,b);

    if(position == debounced_position) {
        // In the debounced state
        // So set a timer which allows a change from the current state
        count = DEBOUNCE_MS / CHECK_MS;
    } else {
        // Something has changed - wait for new state to become stable
        if (--count == 0) {
            // Timer has expired, accept the change
            turned_by += direction(debounced_position, position);
            // Remember the debounced position
            debounced_position = position;
        }
    }
}

uint8_t state (uint8_t a, uint8_t b) {
    return (a << 1 | b) & 0x3;
}

int8_t direction (uint8_t old_state, uint8_t new_state) {
    switch (old_state) {
        case 0:
          if (new_state == 2) return -1;
          if (new_state == 1) return 1;
          break;
        case 1:
          if (new_state == 0) return -1;
          if (new_state == 3) return 1;
          break;
        case 2:
          if (new_state == 3) return -1;
          if (new_state == 0) return 1;
          break;
        case 3:
          if (new_state == 1) return -1;
          if (new_state == 2) return 1;
          break;
        default:
          break;
    }
    return 0;
}
