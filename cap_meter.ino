#include <Wire.h>
#include <Adafruit_GFX.h>
#include "Adafruit_LEDBackpack.h"


//------------------------------------------------------------------------------
// Measurement


const int ADC_MAX = 1023;

const int PIN_SENSE = A2;
const int PIN_SOURCE = A0;

const float Ccal = 1.75e-12;  // Measured cap when no test cap is present
const float Cin = 24.48e-12;  // Input capacitance of the ADC
const float Rpull = 34.8e3;   // Digital input pull-up resistance


void measurement_init(void)
{
  pinMode(PIN_SOURCE, OUTPUT);
  pinMode(PIN_SENSE, OUTPUT);
  digitalWrite(PIN_SOURCE, LOW);
  digitalWrite(PIN_SENSE, LOW);
}


float measurement_read(void)
{
  // Make extra sure the capacitor is discharged
  pinMode(PIN_SENSE, OUTPUT);
  pinMode(PIN_SOURCE, OUTPUT);
  digitalWrite(PIN_SENSE, LOW);
  digitalWrite(PIN_SOURCE, LOW);
  delay(1);

  pinMode(PIN_SENSE, INPUT);
  digitalWrite(PIN_SOURCE, HIGH);
  int val = analogRead(PIN_SENSE);
  digitalWrite(PIN_SOURCE, LOW);

  if (val < 1000)
  {
    // The capacitor to measure was small enough that it gave a low enough
    // voltage to be resolvable when forming a voltage divider with the input
    // capacitance of the sense pin.  Use the voltage divider formula to infer
    // the capacitance.
    //
    //       Cin     (V)        (Vs)
    // GND----||-------          -------------Vcc
    //                |          |
    // ...............|..........|...............
    //       PIN_SENSE|          |PIN_SOURCE
    //                |          |
    //                -----||-----
    //                     C
    //                    (Vc)
    //
    // Vc * C = V * Cin  and  Vc = Vs - V
    //
    //     V * Cin
    // C = -------
    //     Vs - V

    return (float)val * Cin / (float)(ADC_MAX - val) - Ccal;
  }
  else
  {
    // The capacitor to measure dominated the voltage divider too much to give a
    // useful measurement, so measure its time constant against the internal
    // input pullup resistor value.
    //
    //                               Rpull
    // GND-------------          -----^^^-----Vcc
    //                |          |
    // ...............|..........|...............
    //       PIN_SENSE|          |PIN_SOURCE
    //                |          |
    //                -----||-----
    //                     C
    //                    (Vc)
    //

    unsigned long t_start;
    unsigned long t_end;
    unsigned long t_charge;
    int dig_val;

    // Ground one pin and connect the pullup to Vcc on the other
    pinMode(PIN_SENSE, OUTPUT);
    delay(1);
    pinMode(PIN_SOURCE, INPUT_PULLUP);

    // Wait for the pulled-up input to read 1, when the capacitor is charged
    t_start = micros();
    while (!digitalRead(PIN_SOURCE));
    t_end = micros();
    t_charge = t_end > t_start ? t_end - t_start : t_start - t_end;

    // Read the voltage the capacitor was charged up to
    pinMode(PIN_SOURCE, INPUT);
    val = analogRead(PIN_SOURCE);

    // Discharge the capacitor
    digitalWrite(PIN_SENSE, HIGH);
    delay(t_charge / 1000 * 5);

    // Reset the pin states
    pinMode(PIN_SOURCE, OUTPUT);
    digitalWrite(PIN_SOURCE, LOW);
    digitalWrite(PIN_SENSE, LOW);

    return -(float)t_charge * 1e-6 / Rpull / log(1.0 - (float)val / (float)ADC_MAX);
  }
}


//------------------------------------------------------------------------------
// Display


Adafruit_7segment segs = Adafruit_7segment();

// Pin assignments for unit indicator LEDs
const int LED_L = 2;
const int LED_P = 3;
const int LED_N = 4;
const int LED_U = 7;
const int LED_H = 8;

// Values for edges of each unit range
const float L_P_BOUND = 1.0e-12;
const float P_N_BOUND = 1.0e-9;
const float N_U_BOUND = 1.0e-6;
const float U_H_BOUND = 1.0e-3;

const float INF = 1.0 / 0.0;


void display_init(void)
{
  pinMode(LED_L, OUTPUT);
  pinMode(LED_P, OUTPUT);
  pinMode(LED_N, OUTPUT);
  pinMode(LED_U, OUTPUT);
  pinMode(LED_H, OUTPUT);
  digitalWrite(LED_L, HIGH);
  digitalWrite(LED_P, HIGH);
  digitalWrite(LED_N, HIGH);
  digitalWrite(LED_U, HIGH);
  digitalWrite(LED_H, HIGH);
  segs.begin(0x70);
}


void display_show(float value)
{
  if (value < L_P_BOUND)
  {
    digitalWrite(LED_L, LOW);
    digitalWrite(LED_P, HIGH);
    digitalWrite(LED_N, HIGH);
    digitalWrite(LED_U, HIGH);
    digitalWrite(LED_H, HIGH);
    value = -INF;
  }
  else if (value < P_N_BOUND)
  {
    digitalWrite(LED_L, HIGH);
    digitalWrite(LED_P, LOW);
    digitalWrite(LED_N, HIGH);
    digitalWrite(LED_U, HIGH);
    digitalWrite(LED_H, HIGH);
    value *= 1.0e12;
  }
  else if (value < N_U_BOUND)
  {
    digitalWrite(LED_L, HIGH);
    digitalWrite(LED_P, HIGH);
    digitalWrite(LED_N, LOW);
    digitalWrite(LED_U, HIGH);
    digitalWrite(LED_H, HIGH);
    value *= 1.0e9;
  }
  else if (value < U_H_BOUND)
  {
    digitalWrite(LED_L, HIGH);
    digitalWrite(LED_P, HIGH);
    digitalWrite(LED_N, HIGH);
    digitalWrite(LED_U, LOW);
    digitalWrite(LED_H, HIGH);
    value *= 1.0e6;
  }
  else
  {
    digitalWrite(LED_L, HIGH);
    digitalWrite(LED_P, HIGH);
    digitalWrite(LED_N, HIGH);
    digitalWrite(LED_U, HIGH);
    digitalWrite(LED_H, LOW);
    value = INF;
  }
  segs.print(value);
  segs.writeDisplay();
}


void display_show_serial(float value)
{
  if (value < 1.0e-12)
  {
    Serial.println("low");
  }
  else if (value < 1.0e-9 && value >= 1.0e-12)
  {
    Serial.print(value * 1.0e12);
    Serial.println(" pF");
  }
  else if (value < 1.0e-6 && value >= 1.0e-9)
  {
    Serial.print(value * 1.0e9);
    Serial.println(" nF");
  }
  else if (value < 1.0e-3 && value >= 1.0e-6)
  {
    Serial.print(value * 1.0e6);
    Serial.println(" uF");
  }
  else
  {
    Serial.println("high");
  }
}


//------------------------------------------------------------------------------
// Top level


void setup() {
  Serial.begin(9600);
  display_init();
  measurement_init();
}


void loop() {
  float capacitor_value;

  capacitor_value = measurement_read();

  display_show(capacitor_value);
  display_show_serial(capacitor_value);

  delay(100);
}
