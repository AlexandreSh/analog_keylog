#define FASTADC 1

// defines for setting and clearing register bits
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif
const float decayFactor = 0.99;
const int numSensors = 2; //8;
int analogPins[numSensors] = {A0, A1};//, A2, A3, A4, A5, A6, A7};
int maxVals[numSensors] = {0, 0};//, 0, 0, 0, 0, 0, 0};



void updateMaxVals(int i, int newValue);
void setup() {
    #if FASTADC
        // set prescale to 16 (faster adc)
        sbi(ADCSRA, ADPS2);
        cbi(ADCSRA, ADPS1);
        cbi(ADCSRA, ADPS0);
    #endif
    Serial.begin(115200); // Initialize serial communication at 115200 baud rate
    analogReference(INTERNAL1V1);  // Use the internal 1.1V reference voltage
    for (int i = 0; i < numSensors; i++) {
        pinMode(analogPins[i], INPUT); // Initialize sensor pins as input
    }
}


void loop() {
    for (int i = 0; i < numSensors; i++) {
        int newValue = analogRead(analogPins[i]);
        updateMaxVals(i, newValue);
        Serial.print(maxVals[i]);
        if (i < numSensors - 1) {
            Serial.print(","); // Use comma as a delimiter
        }
    }
    Serial.println(); // Move to the next line after printing all sensor values
    delay(20); // Short delay to make the plot smoother
}

void updateMaxVals(int i, int newValue) {
    Serial.print(newValue);   
    Serial.print(", ");   
    maxVals[i] = maxVals[i] * decayFactor;
    if (newValue > maxVals[i]) {
        maxVals[i] = newValue;
    }
}