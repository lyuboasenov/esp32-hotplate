#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <ezButton.h>  // the library to use for SW pin
#include "Config.h"

// OLED FeatherWing buttons map to different pins depending on board.
// The I2C (Wire) bus may also be different.
#define WIRE Wire

#define DIRECTION_CW  0   // clockwise direction
#define DIRECTION_CCW 1  // counter-clockwise direction

#define NOTE_C4  262

Adafruit_SSD1306 display = Adafruit_SSD1306(128, 32, &WIRE);

int counter = 0;
int direction = DIRECTION_CW;
int CLK_state;
int prev_CLK_state;
bool heater_on = false;
int temperature = 0;
double voltage = 0;
uint16_t value = 0;

ezButton button(ROTARY_ENCODER_SW_PIN);  // create ezButton object that attach to pin 7;

void displayState();
void readTemperature();

void setup() {
   Serial.begin(9600);

   Serial.println("ESP32 Hotplate");

   pinMode(PIEZO_BUZZER_PIN, OUTPUT);
   pinMode(HEATER_SWITCH_PIN, OUTPUT);
   pinMode(TEMP_SENSOR_PIN, INPUT);

   // configure encoder pins as inputs
   pinMode(ROTARY_ENCODER_CLK_PIN, INPUT);
   pinMode(ROTARY_ENCODER_DT_PIN, INPUT);
   button.setDebounceTime(50);  // set debounce time to 50 milliseconds

   // read the initial state of the rotary encoder's CLK pin
   prev_CLK_state = digitalRead(ROTARY_ENCODER_CLK_PIN);


   // SSD1306_SWITCHCAPVCC = generate display voltage from 3.3V internally
   display.begin(SSD1306_SWITCHCAPVCC, 0x3C); // Address 0x3C for 128x32

   Serial.println("OLED begun");

   // Show image buffer on the display hardware.
   // Since the buffer is intialized with an Adafruit splashscreen
   // internally, this will display the splashscreen.
   // display.display();
   // delay(1000);

   displayState();
}

void loop() {
   button.loop();  // MUST call the loop() function first
   readTemperature();

  // If the state of CLK is changed, then pulse occurred
  // React to only the rising edge (from LOW to HIGH) to avoid double count
  if (CLK_state != prev_CLK_state && CLK_state == HIGH) {
    // if the DT state is HIGH
    // the encoder is rotating in counter-clockwise direction => decrease the counter
    if (digitalRead(ROTARY_ENCODER_DT_PIN) == HIGH) {
      counter--;
      direction = DIRECTION_CCW;
    } else {
      // the encoder is rotating in clockwise direction => increase the counter
      counter++;
      direction = DIRECTION_CW;
    }

    Serial.print("Rotary Encoder:: direction: ");
    if (direction == DIRECTION_CW)
      Serial.print("Clockwise");
    else
      Serial.print("Counter-clockwise");

    Serial.print(" - count: ");
    Serial.println(counter);
  }

  // save last CLK state
  prev_CLK_state = CLK_state;

  if (button.isPressed()) {
      Serial.println("The button is pressed");
      heater_on = !heater_on;
      tone(PIEZO_BUZZER_PIN, NOTE_C4, 1000 / 8);
      delay(8 * 1.3);
      noTone(PIEZO_BUZZER_PIN);
  }

  if (heater_on) {
      digitalWrite(HEATER_SWITCH_PIN, HIGH);
  } else {
      digitalWrite(HEATER_SWITCH_PIN, LOW);
  }

   displayState();
  delay(10);
  yield();
}

void displayState() {
// Clear the buffer.
   display.clearDisplay();
   display.display();

   // text display tests
   display.setTextSize(1);
   display.setTextColor(SSD1306_WHITE);
   display.setCursor(0,0);
   display.println("ESP32 Hotplate");
   display.println("===============");
   display.print("Heater: ");
   if (heater_on) {
      display.println("ON");
   } else {
      display.println("OFF");
   }
   display.print("T: ");
   display.print(temperature);
   display.print("(");
   display.print(voltage);
   display.print("/");
   display.print(value);
   display.print(")");
   display.setCursor(0,0);
   display.display(); // actually display all of the above
}

void readTemperature() {
   // 0-3.3 / 0-4095
   // Vout = Vin * R2 (R1 + R2)

   const double vin = 5.11;
   // const float vin = 4.78;
   const int r1 = 192;
   const double multiplier = 365.2656308;
   const int max_adc_value = 4095;
   const double max_voltage_adjust = 3.4474655;
   //const float max_voltage_adjust = 3.3;


   const double zero_degrees = (vin * 100) / (r1 + 100);


   value = analogRead(TEMP_SENSOR_PIN);
   voltage = (value * max_voltage_adjust) / max_adc_value;
   temperature = (value * max_voltage_adjust - zero_degrees * max_adc_value) * (multiplier / max_adc_value);

   Serial.print(value);
   Serial.print(" - ");
   Serial.print(voltage);
   Serial.print(" - ");
   Serial.println(temperature);

}