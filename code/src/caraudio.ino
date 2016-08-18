#include <SPI.h>
#include <Adafruit_NeoPixel.h>
#include <avr/sleep.h>

/******************************************************************************
* Definitions and Declarations
******************************************************************************/

// Volume Control
#define PIN_PUSH 3
#define PIN_ROTARY_A 9
#define PIN_ROTARY_B 10

// Constants and variables used for debounce of the volume control
#define DEBOUNCE_COUNT 2 // 2ms
#define TURN_MULTIPLIER 2
#define BUTTON_LONG_PRESS 2000 // 2 seconds
#define BUTTON_HOLD 500 // 0.5 seconds


// Neopixels
#define PIN_NEO 4
#define NUM_PIXELS 12 // Number of neopixels in the ring


// Power Switching
#define PIN_PA_ON A5
#define PIN_AUDIO_PWR 7
#define PIN_PI_PWR 6
#define AUDIO_SETTLE_MS 200 // How long to let the audio come up before PA
#define PI_UP_DELAY 10000 // 10 seconds
#define PI_DOWN_DELAY 60000 // 60 seconds

// Sensing
#define PIN_IGNITION 2
#define SHUTDOWN_DELAY 30000 // Decide to shutdown after ignition loss of 30 seconds

// Notify
#define PIN_LED_ENABLE A4 // Provides the earth for other debug LEDs
#define PIN_LED_1 A3
#define PIN_LED_2 A2


// Audio control
// Setting the volume on the PGA4311
// Uses SPI
#define PIN_CS 5
#define SPI_MAX_SPEED 6000000
#define SPI_MODE SPI_MODE1
#define ENDIAN MSBFIRST

// Hard Mute
#define PIN_MUTE 8

// Source selection
#define PIN_SOURCE_1 A0
#define PIN_SOURCE_2 A1

// Colours for Neopixel ring
#define COL_STANDBY ring.color(128,0,128)
#define WALK_DELAY 80
#define SNOOZE_SAMPLE_DELAY 30
#define PI 3.141582654
#define NUM_SNOOZE_SAMPLES 128
#define MAX_BRIGHTNESS 32

/******************************************************************************
* Machine State
******************************************************************************/
enum machine_state {
    powering_up,        // Power just applied
    sleeping,           // Low power state for ignition off
    pi_up,              // Powering the Raspberry Pi up
    pi_down,            // Powering the Raspberry Pi down
    standby,            // State for ignition on
    on                  // On, ready to play
};

enum button_state {
    off,                // Not pressed
    click,              // Short push
    hold,               // Medium press
    long_press          // Long press
};

/******************************************************************************
* Global Variables
******************************************************************************/

// Machine state
enum machine_state power_state;
volatile enum machine_state next_power_state; // Could be changed during interrupt
unsigned long change_state_millis;  // Some states need a timer...

// volatile variables change in the timer interrupt routine...
volatile static uint8_t debounced_position; // Internal to the interrupt
volatile static uint8_t count; //used for debounce inside the interrupt
volatile static int16_t turned_by; // This gives the amount of turn


volatile static uint8_t button_position; // Debounced button position
volatile static uint8_t button_count; // Used to debounce the button
volatile static uint8_t previous_button_position; // state
volatile static enum button_state pressed;
volatile bool turned_while_pressed;
unsigned long button_change_millis; // so we can determine length of press


// Global variable to control volume
uint8_t request_volume;
uint8_t current_volume;
bool soft_mute;
bool hard_mute;


// Buffers for Serial Messages
char buffTx[256];
char buffRx[256];

// Define the Neopixels
Adafruit_NeoPixel ring = Adafruit_NeoPixel(NUM_PIXELS, PIN_NEO, NEO_GRB + NEO_KHZ800);
uint32_t colour;
uint32_t walk_colour;
uint8_t brightness;
uint8_t walk_light;
unsigned long move_light_millis;
uint8_t snooze_samples[NUM_SNOOZE_SAMPLES];

/******************************************************************************
* Helper Functions
******************************************************************************/

// Log to the serial port
void log(char* fmt, ...) {
    va_list args;
    va_start(args,fmt);
    vsprintf(buffTx,fmt,args);
    va_end(args);
    Serial.println(buffTx);
}

void initialise_pins() {
    // Setup the LED Enable
    pinMode(PIN_LED_ENABLE, OUTPUT);
    digitalWrite(PIN_LED_ENABLE, HIGH); // Allow LEDs to turn on (switch earth)

    // Setup the 2 non-associated LEDs
    pinMode(PIN_LED_1, OUTPUT);
    digitalWrite(PIN_LED_1, LOW); // LED off
    pinMode(PIN_LED_2, OUTPUT);
    digitalWrite(PIN_LED_2, LOW); // LED off

    // Setup the control pins
    pinMode(PIN_AUDIO_PWR, OUTPUT);
    digitalWrite(PIN_AUDIO_PWR, HIGH); // Audio processing off
    pinMode(PIN_PA_ON, OUTPUT);
    digitalWrite(PIN_PA_ON, LOW); // Power Amp off
    pinMode(PIN_PI_PWR, OUTPUT);
    digitalWrite(PIN_PI_PWR, HIGH);  // Pi off
    pinMode(PIN_SOURCE_1, OUTPUT);
    digitalWrite(PIN_SOURCE_1, LOW); // Source 1 off
    pinMode(PIN_SOURCE_2, OUTPUT);
    digitalWrite(PIN_SOURCE_2, LOW); // Source 2 off
    pinMode(PIN_MUTE, OUTPUT);
    digitalWrite(PIN_MUTE, LOW); // Hard mute the volume control

    // Setup the input Sensing
    pinMode(PIN_IGNITION, INPUT);

    // Setup volume control pins
    pinMode(PIN_PUSH, INPUT);
    pinMode(PIN_ROTARY_A, INPUT);
    pinMode(PIN_ROTARY_B, INPUT);

    // Initialise the SPI so we can talk to the volume control
    //SPI.begin();

    // Initialise the Neopixel ring
    ring.begin();

    // We don't use the ADC so power it down
    ADCSRA = 0;
}

void initialise_state() {
    // Machine state
    power_state = powering_up;
    next_power_state = pi_up;

    // volatile variables change in the interrupt routine...
    debounced_position = 0;
    turned_by = 0;
    button_position = 0;
    pressed = off;
    turned_while_pressed = false;

    // Maintain state
    previous_button_position = 0;
    button_change_millis = 0;
    count =  DEBOUNCE_COUNT;
    button_count =  DEBOUNCE_COUNT;

    // Set up Neopixel
    colour = ring.Color(128,255,128);
    walk_colour = ring.Color(255,0,255);
    // Fill the samples for sine wave snooze light
    for(int i=0; i < NUM_SNOOZE_SAMPLES; i++){
        snooze_samples[i] = (uint8_t)(MAX_BRIGHTNESS/2 + MAX_BRIGHTNESS/2*(sin(i*2*PI/NUM_SNOOZE_SAMPLES)));
    }
}


/******************************************************************************
* Input processing
******************************************************************************/

// Go to sleep, but enable external interrupts
// so that we can wake with ignition or button press
void go_to_sleep() {
    noInterrupts(); // Disable interrupts
    // Get ready to sleep
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    sleep_enable();

    // Attach external interrupts that will wake us
    attachInterrupt(digitalPinToInterrupt(PIN_IGNITION), wake_up, RISING);
    attachInterrupt(digitalPinToInterrupt(PIN_PUSH), wake_up, FALLING);
    // Clear any queued interrupts that would wake us immediately
    EIFR = bit (INTF0);  // clear flag for interrupt 0
    EIFR = bit (INTF1);  // clear flag for interrupt 1
    // We are guaranteed that the sleep_cpu call will be done as the
    // processor executes the next instrction after interrupts are
    // turned on
    interrupts();   // one cycle
    sleep_cpu();    // one cycle
}

// Wake form sleep
void wake_up() {
    noInterrupts();
    detachInterrupt(digitalPinToInterrupt(PIN_IGNITION));
    detachInterrupt(digitalPinToInterrupt(PIN_PUSH));
}

// Enable timer interrupts so we can debounce the input controls
void enable_timer_interrupt() {
    // Setup Timer Interrupts
    cli(); // Disable interrupts
    // Reset the Timer 2 Control Registers
    TCCR2A = 0;
    TCCR2B = 0;
    // Set the timer comparison value
    // 16MHZ, 1024 prescale gives 64us per tick
    // 1ms interrupt = 1e-3/64e-6 ~= 15
    OCR2A = 0xD; // -1 neede as counter registers are 0 offset
    //Turn on CTC mode
    TCCR2A |= (1 << WGM21);
    // Set bits CS20,CS21,CS22 bits for the 1024 prescaler
    TCCR2B |= (1 << CS20);
    TCCR2B |= (1 << CS21);
    TCCR2B |= (1 << CS22);
    //Enable the compare interrupt for Timer 2
    TIMSK2 = (1 << OCIE2A);
    sei(); // Enable interrupts
}

void disable_timer_interrupt() {
    // Reset the Timer 2 Control registers
    TCCR2A = 0;
    TCCR2B = 0;
    // Disable interrupts for Timer 2
    TIMSK2 = 0;
}

// Timer ISR - Set the function to call when the interrupt happens
ISR(TIMER2_COMPA_vect) {
    debounce();
}

// Function called when timer interrupt fires to debounce the input
void debounce() {
    // Setup
    uint8_t a = digitalRead(PIN_ROTARY_A);
    uint8_t b = digitalRead(PIN_ROTARY_B);
    uint8_t btn = digitalRead(PIN_PUSH);

    uint8_t position = state(a,b);

    // Debounce the rotary encoder
    if(position == debounced_position) {
        // In the debounced state
        // So set a counter which allows a change from the current state
        count = DEBOUNCE_COUNT;
    } else {
        // Something has changed - wait for new state to become stable
        if (--count == 0) {
            // Timer has expired, accept the change
            turned_by += direction(debounced_position, position);
            // Remember the debounced position
            debounced_position = position;
            if (button_position == 1) {
                turned_while_pressed = true;
            }
            log("Turned: %d", turned_by);
        }
    }

    // Debounce the pushbutton
    if (btn == button_position) {
        button_count = DEBOUNCE_COUNT;
    } else {
        if (--button_count == 0) {
            if (btn == 0) {
                // It has just been released, so check how long held for
                if (millis() > button_change_millis + BUTTON_LONG_PRESS) {
                    pressed = long_press;
                } else if (millis() > button_change_millis + BUTTON_HOLD){
                    pressed = hold;
                } else {
                    pressed = click;
                }
                log("Button: %d", pressed);
            }
            button_position = btn;
            button_change_millis = millis();
        }
    }
}

// Use the 2 rotary encoder inputs to determine a "state"
uint8_t state (uint8_t a, uint8_t b) {
    return (a << 1 | b) & 0x3;
}

// From the old and new states determine direction
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


/******************************************************************************
* State functions
******************************************************************************/

// Exit states
void state_exit_powering_up() {
    log("Exit state 'powering_up'");
}

void state_exit_sleeping() {
    log("Exit state 'sleeping'");
}

void state_exit_pi_up() {
    log("Exit state 'pi_up'");
    log("Enabling timer interrupt");
    // Enable the volume control knob
    enable_timer_interrupt();
}

void state_exit_pi_down() {
    log("Exit state 'pi_down'");
    digitalWrite(PIN_PI_PWR, HIGH); // Turn off power to the Pi
}

void state_exit_standby() {
    log("Exit state 'standby'");
}

void state_exit_on() {
    log("Exit state 'on'");
    // Apply hard mute to the volume control
    digitalWrite(PIN_MUTE, LOW);
    // Turn off the audio stages
    digitalWrite(PIN_PA_ON, LOW);
    digitalWrite(PIN_AUDIO_PWR, HIGH);
}

// Enter states
void state_enter_powering_up() {
    log("Enter state 'powering_up'");
}

void state_enter_sleeping() {
    log("Enter state 'sleeping'");
    // Give time for the message to be sent
    //delay(1000);
    go_to_sleep();
}

void state_enter_pi_up() {
    log("Enter state 'pi_up'");
    // Turn on the power to the Pi
    digitalWrite(PIN_PI_PWR, LOW);
    // Set a delay to wait for the Pi to boot
    change_state_millis = millis() + PI_UP_DELAY;
    move_light_millis = millis();
}

void state_enter_pi_down() {
    log("Enter state 'pi_down'");
    log("!POWER_DOWN"); // Send command to Pi
    // Set a delay before we remove power
    change_state_millis = millis() + PI_DOWN_DELAY;
    // Don't need to scan volume knob for movement any more
    disable_timer_interrupt();
}

void state_enter_standby() {
    log("Enter state 'standby'");
    // Reset some variables
    turned_by = 0; // Ignore any rotation whilst in standby
    turned_while_pressed = false;
    pressed = off;
    walk_light = 0;
}

void state_enter_on() {
    log("Enter state 'on'");
    // Turn on the audio stages
    digitalWrite(PIN_AUDIO_PWR, HIGH);
    digitalWrite(PIN_PA_ON, LOW);
    // Unmute the hard mute
    digitalWrite(PIN_MUTE, HIGH);
    // Reset some variables
    turned_by = 0; // Ignore any rotation whilst in standby
    turned_while_pressed = false;
    pressed = off;
}

// Run states
// All of the state change decisions should happen in these functions
void state_run_powering_up() {
    // Nothing should happen here as we move directly to sleeping
}

void state_run_sleeping() {
    next_power_state = pi_up;
}

void state_run_pi_up() {
    // Decide whether to change state
    if(millis() > change_state_millis) {
        // Delay is up so move to next state
        next_power_state = standby;
    }
}

void state_run_pi_down() {
    // Decide whether to change state
    if(millis() > change_state_millis) {
        // Delay is up so move to next state
        next_power_state = sleeping;
    }
}

void state_run_standby() {
    // Check for pushbutton press
    if (pressed != off) {
        next_power_state = on;
    }
    // Check ignition
    uint8_t ign = digitalRead(PIN_IGNITION);
    if(ign == 1) {
        change_state_millis = millis() + SHUTDOWN_DELAY;
    } else {
        if (millis() > change_state_millis) {
            next_power_state = pi_down;
        }
    }
}

void state_run_on() {
    // This is where the work should happen...
    // Check for state change
    uint8_t ign = digitalRead(PIN_IGNITION);
    if (ign == 0) {
        next_power_state = standby;
    } else if (pressed == long_press) {
        next_power_state = standby;
    }

    // Deal with the rotary input
    if (turned_by != 0) {
        if (turned_while_pressed == true) {
            // Switch source
            turned_while_pressed = false;
        } else {
            // Change the volume
            int16_t vol = request_volume + turned_by * TURN_MULTIPLIER;
            if (vol > 255) {
                vol = 255;
            } else if (vol < 0) {
                vol = 0;
            }
            request_volume = (uint8_t)vol;
            //fade(request_volume);
        }
    }
    // Reset the input states
    turned_by = 0;
}



/******************************************************************************
* Main Loops
******************************************************************************/

void setup() {
    Serial.begin(9600);
    log("Startup...");
    initialise_pins();
    initialise_state();
}

void loop() {
    if(next_power_state != power_state) {
        // We are changing state
        // Run the exit routine for the current state
        switch (power_state) {
            case powering_up:
                state_exit_powering_up();
                break;
            case sleeping:
                state_exit_sleeping();
                break;
            case standby:
                state_exit_standby();
                break;
            case pi_up:
                state_exit_pi_up();
                break;
            case pi_down:
                state_exit_pi_down();
                break;
            case on:
                state_exit_on();
                break;
            default:
                log("State error - old state");
        }
        // Run the entry routine for the new state
        switch (next_power_state) {
            case powering_up:
                state_enter_powering_up();
                break;
            case sleeping:
                state_enter_sleeping();
                break;
            case standby:
                state_enter_standby();
                break;
            case pi_up:
                state_enter_pi_up();
                break;
            case pi_down:
                state_enter_pi_down();
                break;
            case on:
                state_enter_on();
                break;
            default:
                log("State error - new state");
        }
        // Update state
        power_state = next_power_state;
    } else {
        // We are staying in the current state
        // Run the in-state routine
        switch (power_state) {
            case powering_up:
                state_run_powering_up();
                break;
            case sleeping:
                state_run_sleeping();
                break;
            case standby:
                state_run_standby();
                break;
            case pi_up:
                state_run_pi_up();
                break;
            case pi_down:
                state_run_pi_down();
                break;
            case on:
                state_run_on();
                break;
            default:
                log("State error - current state");
        }
    }
    updateDisplay();
    delay(10);
}

/**************************************************************************
* Audio control
**************************************************************************/

void audio_on() {
    pinMode(PIN_AUDIO_PWR, OUTPUT);
    digitalWrite(PIN_AUDIO_PWR, LOW);
    delay(AUDIO_SETTLE_MS);
    pinMode(PIN_PA_ON, OUTPUT);
    digitalWrite(PIN_PA_ON, HIGH);
}

void audio_off() {
    pinMode(PIN_PA_ON, OUTPUT);
    digitalWrite(PIN_PA_ON, LOW);
    delay(AUDIO_SETTLE_MS);
    pinMode(PIN_AUDIO_PWR, OUTPUT);
    digitalWrite(PIN_AUDIO_PWR, HIGH);
}

void audio_source(uint8_t source) {
    pinMode(PIN_SOURCE_1, OUTPUT);
    pinMode(PIN_SOURCE_2, OUTPUT);
    if(source == 1) {
        // fade off Source 2
        fade(0);
        digitalWrite(PIN_SOURCE_2, LOW);
        // fade on Source 1
        digitalWrite(PIN_SOURCE_1, HIGH);
        fade(request_volume);
    } else if (source == 2) {
        // fade off Source 1
        fade(0);
        digitalWrite(PIN_SOURCE_1, LOW);
        // fade on Source 2
        digitalWrite(PIN_SOURCE_2, HIGH);
        fade(request_volume);
    }
}

void fade(uint8_t to_volume) {
        uint8_t change = 1; // be careful changing this from 1...
        if (to_volume < current_volume) {
            change = -change;
        }
        while(current_volume != to_volume) {
            current_volume += change;
            audio_volume(current_volume);
            delay(4); //Control the slew rate
        }
}
void audio_volume(uint8_t volume) {
    request_volume = volume;
    if(volume == 0){
        soft_mute = true;
    } else {
        soft_mute = false;
    }
    // To do change this to deal with balance and fade
    set_volume(volume, volume, volume, volume);
}

void mute() {
    pinMode(PIN_MUTE, OUTPUT);
    digitalWrite(PIN_MUTE, LOW);
}

void unmute() {
    pinMode(PIN_MUTE, OUTPUT);
    digitalWrite(PIN_MUTE, HIGH);
}

/**************************************************************************
* SPI Code to communicate with the Audio Volume control
**************************************************************************/


void set_volume(uint8_t lf, uint8_t rf, uint8_t lr, uint8_t rr) {
    pinMode(PIN_CS, OUTPUT);
    digitalWrite(PIN_CS, LOW); //Chip Select
    SPI.beginTransaction(SPISettings(SPI_MAX_SPEED, SPI_MODE, ENDIAN));
    SPI.transfer(lf); // CHannel 4
    SPI.transfer(lr);
    SPI.transfer(rf);
    SPI.transfer(rr);
    digitalWrite(PIN_CS, HIGH);
    SPI.endTransaction();
}

/**************************************************************************
* Deal with the Neopixel Display
**************************************************************************/


void updateDisplay() {
    brightness = MAX_BRIGHTNESS;
    switch(power_state) {
        case on:
          for (int i = 0; i < NUM_PIXELS; i++) {
            if ( request_volume > (i * 255 / NUM_PIXELS)) {
              ring.setPixelColor(i, colour);
            } else {
              ring.setPixelColor(i,0);
            }
            ring.setBrightness(request_volume);
            ring.show();
          }
          break;
        case standby:
            if (millis() > move_light_millis) {
                for (int i=0; i<NUM_PIXELS; i++) {
                    ring.setPixelColor(i, 255, 255, 255);
                }
                walk_light = (walk_light + 1) % NUM_SNOOZE_SAMPLES;
                brightness = snooze_samples[walk_light];
                move_light_millis = millis() + SNOOZE_SAMPLE_DELAY;
                ring.setBrightness(brightness);
                ring.show();
            }
            break;
        case pi_up:
            if (millis() > move_light_millis) {
                walk_light = (walk_light + 1) % NUM_PIXELS;

                for (int i = 0; i < NUM_PIXELS; i++) {
                    if (i == walk_light) {
                        ring.setPixelColor(i, walk_colour);
                    } else {
                        ring.setPixelColor(i,0);
                    }
                }
                move_light_millis = millis() + WALK_DELAY;
                ring.setBrightness(brightness);
                ring.show();
            }
            break;
        default:
            brightness = 0;
            ring.setBrightness(brightness);
            ring.show();

    }
}
