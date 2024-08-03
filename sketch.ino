#include <arduinoFFT.h>

#define SAMPLES 256              // Must be a power of 2
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
    Serial.begin(115200);
    analogReference(INTERNAL1V1);  // Use the internal 1.1V reference voltage
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);
    
    sampling_period_us = round(1000000 * (1.0 / SAMPLING_FREQUENCY));

    if (USE_SONAR) {
        lastChangeTime = millis();
        lastDistance = measureDistance();
    }
}

void loop() {
    if (USE_SONAR) {
        unsigned int distance = measureDistance();

        if (distance > 0 && (abs(distance - lastDistance) > lastDistance * TOLERANCE)) {
        // Distance changed significantly
        lastChangeTime = millis();
        lastDistance = distance;
        fftRunning = true;
        Serial.println("START");
        }

        if (fftRunning) {
        if (millis() - lastChangeTime >= TIMEOUT) {
            // Timeout reached, halt FFT and sensor operations
            fftRunning = false;
            Serial.println("HALT");
        } else {
            performFFT();
        }
        } else {
        // Check for new significant distance change to resume operations
        if (distance > 0 && (abs(distance - lastDistance) > lastDistance * TOLERANCE)) {
            lastChangeTime = millis();
            lastDistance = distance;
            fftRunning = true;
            Serial.println("START");
        }
        }
    } else {
        if (millis() - lastSendTime >= 2000) { // Send data every 2 seconds
        performFFT();
        lastSendTime = millis();
        }
    }

    // Short delay before the next measurement
    delay(50);
}

unsigned int measureDistance() {
    long duration, distance;

    // Send a 10us pulse to trigger the HC-SR04
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    // Measure the duration of the echo pulse
    duration = pulseIn(echoPin, HIGH);

    // Calculate the distance in centimeters
    distance = (duration / 2) / 29.1;
    return distance;
}

void performFFT() {
    // Collect all frequency data
    float sensorData[numSensors][SAMPLES / 2] = {0};

    for (int sensor = 0; sensor < numSensors; sensor++) {
        // Collect samples
        for (int i = 0; i < SAMPLES; i++) {
        microseconds = micros();

        vReal[i] = analogRead(analogPins[sensor]);
        vImag[i] = 0;

        while (micros() < (microseconds + sampling_period_us)) {
            // wait
        }
        }

        // Perform FFT
        FFT.windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
        FFT.compute(FFT_FORWARD);
        FFT.complexToMagnitude();

        // Store the frequency magnitudes
        for (int i = 0; i < (SAMPLES / 2); i++) {
        sensorData[sensor][i] = vReal[i];
        }
    }

    // Send consolidated data to serial port
    for (int i = 0; i < (SAMPLES / 2); i++) {
        Serial.print((i * SAMPLING_FREQUENCY) / SAMPLES);
        Serial.print(" Hz: ");
        for (int sensor = 0; sensor < numSensors; sensor++) {
        Serial.print("Sensor ");
        Serial.print(sensor);
        Serial.print(": ");
        Serial.print(sensorData[sensor][i]);
        Serial.print(" ");
        }
        Serial.println();
    }
    Serial.println("---");  // Separator between readings
}
