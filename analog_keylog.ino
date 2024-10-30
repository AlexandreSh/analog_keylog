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

const int numSensors = 8;
int analogPins[numSensors] = {A0, A1, A2, A3, A4, A5, A6, A7};

unsigned long lastSendTime = 0;

void setup() {
    #if FASTADC
        // set prescale to 16 (faster adc)
        sbi(ADCSRA,ADPS2);
        cbi(ADCSRA,ADPS1);
        cbi(ADCSRA,ADPS0);
    #endif
    Serial.begin(115200);
    analogReference(INTERNAL1V1);  // Use the internal 1.1V reference voltage
}

void loop() {
    if (millis() - lastSendTime >= 200) { // Send data every 200 milliseconds
        for (int i = 0; i < numSensors; i++) {
            int sensorValue = analogRead(analogPins[i]);
          //  Serial.print("Sensor ");
            Serial.print(i);
            Serial.print(", ");
            Serial.print(sensorValue);
          //  Serial.print(" ");
        }
        Serial.println();
        lastSendTime = millis();
    }
}