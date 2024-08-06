#include <arduinoFFT.h>


#define FASTADC 1
// defines for setting and clearing register bits
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

#define SAMPLES 128              // Must be a power of 2
#define SAMPLING_FREQUENCY 10000 // Hz, must be less than 10000 due to ADC limitations

const int USE_SONAR = 0; // Set to 1 to use sonar, 0 to disregard sonar and send data every 2 seconds
float vReal[SAMPLES];
float vImag[SAMPLES];

ArduinoFFT<float> FFT = ArduinoFFT<float>(vReal, vImag, SAMPLES, SAMPLING_FREQUENCY);

const int trigPin = 9;
const int echoPin = 10;
const int numSensors = 8;
int analogPins[numSensors] = {A0, A1, A2, A3, A4, A5, A6, A7};

unsigned int sampling_period_us;
unsigned long microseconds;

const unsigned long TIMEOUT = 180000; // Timeout in milliseconds (180 seconds)
const float TOLERANCE = 0.05; // 5% tolerance
unsigned long lastChangeTime = 0;
unsigned int lastDistance = 0;
bool fftRunning = false;

unsigned long lastSendTime = 0;

void setup() {
  Serial.begin(115200); // Initialize serial communication at 115200 baud rate
  for (int i = 0; i < 8; i++) {
    pinMode(i, INPUT); // Initialize sensor pins as input
  }
}

void loop() {
  for (int i = 0; i < 8; i++) {
    Serial.print(analogRead(i));
    if (i < 7) {
      Serial.print(","); // Use comma as a delimiter
    }
  }
  Serial.println(); // Move to the next line after printing all sensor values
  delay(100); // Wait for 100 milliseconds before taking the next set of readings
}