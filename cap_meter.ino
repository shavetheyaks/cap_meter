#include <Wire.h>
#include <Adafruit_GFX.h>
#include "Adafruit_LEDBackpack.h"


const float INF = 1.0 / 0.0;


//------------------------------------------------------------------------------
// Measurement


const int ADC_MAX = 1023;

const int PIN_G = A2;         // Use this pin for the - lead of polarized caps
const int PIN_V = A0;         // Use this pin for the + lead of polarized caps

// Adjust these values as needed to calibrate
const float Ccal = 1.75e-12;  // Measured cap when no test cap is present
const float Cin = 24.48e-12;  // Input capacitance of the ADC
const float Rpull = 34.8e3;   // Digital input pull-up resistance


void measurement_discharge(void)
{
  // Make sure the capacitor is discharged
  pinMode(PIN_G, OUTPUT);
  pinMode(PIN_V, OUTPUT);
  digitalWrite(PIN_G, LOW);
  digitalWrite(PIN_V, LOW);
  delay(10);
}


// Small capacitors (< ~100pF) can be measured by forming a voltage divider
// with the input capacitance of the sense pin.  Use the voltage divider formula
// to infer the capacitance.
//
//       Cin     (V)        (Vs)
// GND----||-------          -------------Vcc
//                |          |                internal
// ...............|..........|........................
//           PIN_G|          |PIN_V           external
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
float measurement_small(void)
{
  int analog_val;

  // Make sure the capacitor is discharged so we start fresh.
  measurement_discharge();

  // Make a voltage divider with the input capacitance
  pinMode(PIN_G, INPUT);
  pinMode(PIN_V, OUTPUT);
  digitalWrite(PIN_V, HIGH);

  // Measure the voltage at the center of the voltage divider
  analog_val = analogRead(PIN_G);

  // Clean up after ourselves
  measurement_discharge();

  // The capacitor is too large to get a good measurement.
  if (analog_val >= 1000)
    return INF;

  // Use the capacitor voltage divider formula to infer the capacitance.
  // Also subtract off the extra "calibrated" capacitance.
  return (float)analog_val * Cin / (float)(ADC_MAX - analog_val) - Ccal;
}


// Larger capacitors (> ~100pF) can be measured by the time-to-charge method
// because the parasitic capacitance will be insignificant.
//
//                               Rpull
// GND-------------          -----^^^-----Vcc
//                |          |                internal
// ...............|..........|........................
//           PIN_G|          |PIN_V           external
//                |          |
//                -----||-----
//                     C
//                    (Vc)
//
float measurement_large(void)
{
  unsigned long t_start;
  unsigned long t_end;
  unsigned long t_charge;
  int analog_val;
  int dig_val;

  // Make sure the capacitor is discharged so we start fresh.
  measurement_discharge();

  // Ground one pin and connect the pullup to Vcc on the other.
  pinMode(PIN_G, OUTPUT);
  digitalWrite(PIN_G, LOW);
  pinMode(PIN_V, INPUT_PULLUP);

  // Wait for the pulled-up input to read 1 when the capacitor is charged.
  t_start = micros();
  while (!digitalRead(PIN_V));
  t_end = micros();

  // Stop charging and read the voltage the cap was charged to.
  pinMode(PIN_V, INPUT);
  analog_val = analogRead(PIN_V);

  // Clean up after ourselves.
  measurement_discharge();

  // Charge time is the difference between start and end times.
  // The conditional is to deal with the micros() timer overflowing.
  t_charge = t_end > t_start ? t_end - t_start : t_start - t_end;

  // Compute the capacitance from the V(t) for and RC circuit.
  return -(float)t_charge * 1e-6 / Rpull / log(1.0 - (float)analog_val / (float)ADC_MAX);
}


//------------------------------------------------------------------------------
// Display


// Using Adafruit's four-digit 7-segment display backpack
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
  measurement_discharge();
}


void loop() {
  float capacitor_value;

  capacitor_value = measurement_small();
  if (capacitor_value == INF)
    capacitor_value = measurement_large();

  display_show(capacitor_value);
  display_show_serial(capacitor_value);

  delay(100);
}
