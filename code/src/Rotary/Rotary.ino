#include <Adafruit_NeoPixel.h>

#define PUSH_PIN 2
#define ROTARY_A 3
#define ROTARY_B 4
#define DEBOUNCE 50
#define LONG_PRESS 500

// Define the parameters for the neopixels
#define NEO_PIN 5
#define NEO_NUMPIXELS 12

#define VOL_STEP 5
#define COL_STEP 5
// Create the Neopixel object
Adafruit_NeoPixel strip = Adafruit_NeoPixel(NEO_NUMPIXELS, NEO_PIN, NEO_GRB + NEO_KHZ800);

// Set variables that will be updated in the interrupt routines
volatile bool button_state = true;
volatile unsigned long button_state_millis = 0;
volatile int turn_count = 0;

// Set variables that will store current state
bool button_not_pressed = true;
bool spun_while_pressed = false;
unsigned long button_pressed_millis = 0;
int spin_counter = 0;

int vol = 5;
int col = 0;


void setup() {
    // Setup the Serial output
    Serial.begin(9600);
    Serial.println("Performing Setup");

    // Set the pins as inputs and add internal pullups
    pinMode(PUSH_PIN, INPUT_PULLUP);
    pinMode(ROTARY_A, INPUT_PULLUP);
    pinMode(ROTARY_B, INPUT_PULLUP);

    // Attach Interrupts
    attachInterrupt(digitalPinToInterrupt(PUSH_PIN), button, CHANGE);
    attachInterrupt(digitalPinToInterrupt(ROTARY_A), spin, FALLING);

    // Startup Sequence
    strip.begin();
    walk(strip.Color(255,0,150), 160);
    strip.clear();
    strip.show();
}


void loop() {
    unsigned long now = millis();
    // Debounce the button
    if(button_state != button_not_pressed) {
        if (now > button_state_millis + DEBOUNCE) {
            // The button has settled
            if(button_state == false){
                // Button has just been pressed
                button_not_pressed = false;
                button_pressed_millis = button_state_millis;
            } else {
                // Button has just been released
                button_not_pressed = true;
                if (now > button_pressed_millis + LONG_PRESS) {
                    if (spun_while_pressed) {
                        spun_while_pressed = false;
                    } else {
                        // Deal with long press
                        doLongPress();
                    }
                } else {
                    // Deal with short press
                    doPress();
                }
            }
        }
    }
    if (spin_counter != 0) {
        if (!button_not_pressed) {
            doSpinPressed();
        } else {
            doSpin();
        }
        spin_counter = 0;
    }
    updateDisplay();
    delay(10);
}


void button() {
    button_state = digitalRead(PUSH_PIN);
    button_state_millis = millis();
}


void spin() {
    bool a = digitalRead(ROTARY_A);
    bool b = digitalRead(ROTARY_B);
    if ( a != b) {
        // Clockwise
        spin_counter++;
    } else {
        // Anticlockwise
        spin_counter--;
    }
}


void doPress() {
    // Called when the button is pressed for a short period
    Serial.println("Press");
}


void doLongPress() {
    // Called when button is pressed for a long period
    Serial.println("Long Press");
}


void doSpinPressed() {
    // Called when the control is turned whilst pressed
    if (spin_counter > 0) {
      col = (col + COL_STEP) % 255;
    } else if (spin_counter < 0) {
      col = (col - COL_STEP) % 255;
    }
    spin_counter = 0;
    spun_while_pressed = true;

}


void doSpin() {
    // Called when the control is turned
    if (spin_counter > 0) {
      vol += VOL_STEP;
    } else if (spin_counter < 0) {
      vol -= VOL_STEP;
    }
    if (vol < 0) {
      vol = 0;
    } else if (vol > 255) {
      vol = 255;
    }
    spin_counter = 0;
}

void walk(uint32_t c, uint8_t wait){
    for(int i=0; i < strip.numPixels(); i++){
        strip.setPixelColor(i, c);
        strip.show();
        delay(wait);
        strip.setPixelColor(i, 0);
    }
}

uint32_t Wheel(byte WheelPos) {
  WheelPos = 255 - WheelPos;
  if(WheelPos < 85) {
    return strip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  }
  if(WheelPos < 170) {
    WheelPos -= 85;
    return strip.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
  WheelPos -= 170;
  return strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
}

void updateDisplay() {
    strip.clear();
    strip.setBrightness(vol);
    uint32_t c = Wheel(col);
    for(int i = 0; i < NEO_NUMPIXELS; i++){
      strip.setPixelColor(i, c);
    }
    strip.show();
}

