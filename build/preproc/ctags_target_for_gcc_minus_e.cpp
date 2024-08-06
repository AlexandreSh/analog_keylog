# 1 "/home/alex/Arduino/vscode/mega/mega.ino"
# 2 "/home/alex/Arduino/vscode/mega/mega.ino" 2



// defines for setting and clearing register bits
# 17 "/home/alex/Arduino/vscode/mega/mega.ino"
const int USE_SONAR = 0; // Set to 1 to use sonar, 0 to disregard sonar and send data every 2 seconds
float vReal[128 /* Must be a power of 2*/];
float vImag[128 /* Must be a power of 2*/];

ArduinoFFT<float> FFT = ArduinoFFT<float>(vReal, vImag, 128 /* Must be a power of 2*/, 10000 /* Hz, must be less than 10000 due to ADC limitations*/);

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


        // set prescale to 16 (faster adc)
        (
# 43 "/home/alex/Arduino/vscode/mega/mega.ino" 3
       (*(volatile uint8_t *)(((uint16_t) &((*(volatile uint8_t *)(0x7A)))))) 
# 43 "/home/alex/Arduino/vscode/mega/mega.ino"
       |= 
# 43 "/home/alex/Arduino/vscode/mega/mega.ino" 3
       (1 << (2))
# 43 "/home/alex/Arduino/vscode/mega/mega.ino"
       ) ;
        (
# 44 "/home/alex/Arduino/vscode/mega/mega.ino" 3
       (*(volatile uint8_t *)(((uint16_t) &((*(volatile uint8_t *)(0x7A)))))) 
# 44 "/home/alex/Arduino/vscode/mega/mega.ino"
       &= ~
# 44 "/home/alex/Arduino/vscode/mega/mega.ino" 3
       (1 << (1))
# 44 "/home/alex/Arduino/vscode/mega/mega.ino"
       ) ;
        (
# 45 "/home/alex/Arduino/vscode/mega/mega.ino" 3
       (*(volatile uint8_t *)(((uint16_t) &((*(volatile uint8_t *)(0x7A)))))) 
# 45 "/home/alex/Arduino/vscode/mega/mega.ino"
       &= ~
# 45 "/home/alex/Arduino/vscode/mega/mega.ino" 3
       (1 << (0))
# 45 "/home/alex/Arduino/vscode/mega/mega.ino"
       ) ;

    Serial.begin(115200);
    analogReference(2); // Use the internal 1.1V reference voltage
    pinMode(trigPin, 0x1);
    pinMode(echoPin, 0x0);

    sampling_period_us = ((1000000 * (1.0 / 10000 /* Hz, must be less than 10000 due to ADC limitations*/))>=0?(long)((1000000 * (1.0 / 10000 /* Hz, must be less than 10000 due to ADC limitations*/))+0.5):(long)((1000000 * (1.0 / 10000 /* Hz, must be less than 10000 due to ADC limitations*/))-0.5));

    if (USE_SONAR) {
        lastChangeTime = millis();
        lastDistance = measureDistance();
    }
}

void loop() {
    if (USE_SONAR) {
        unsigned int distance = measureDistance();

        if (distance > 0 && (((distance - lastDistance)>0?(distance - lastDistance):-(distance - lastDistance)) > lastDistance * TOLERANCE)) {
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
        if (distance > 0 && (((distance - lastDistance)>0?(distance - lastDistance):-(distance - lastDistance)) > lastDistance * TOLERANCE)) {
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
    digitalWrite(trigPin, 0x0);
    delayMicroseconds(2);
    digitalWrite(trigPin, 0x1);
    delayMicroseconds(10);
    digitalWrite(trigPin, 0x0);

    // Measure the duration of the echo pulse
    duration = pulseIn(echoPin, 0x1);

    // Calculate the distance in centimeters
    distance = (duration / 2) / 29.1;
    return distance;
}

void performFFT() {
    // Collect all frequency data
    float sensorData[numSensors][128 /* Must be a power of 2*/ / 2] = {0};

    for (int sensor = 0; sensor < numSensors; sensor++) {
        // Collect samples
        for (int i = 0; i < 128 /* Must be a power of 2*/; i++) {
        microseconds = micros();

        vReal[i] = analogRead(analogPins[sensor]);
        vImag[i] = 0;

        while (micros() < (microseconds + sampling_period_us)) {
            // wait
        }
        }

        // Perform FFT
        FFT.windowing(FFTWindow::Hamming /* hamming */, FFTDirection::Forward);
        FFT.compute(FFTDirection::Forward);
        FFT.complexToMagnitude();

        // Store the frequency magnitudes
        for (int i = 0; i < (128 /* Must be a power of 2*/ / 2); i++) {
        sensorData[sensor][i] = vReal[i];
        }
    }

    // Send consolidated data to serial port
    for (int i = 0; i < (128 /* Must be a power of 2*/ / 2); i++) {
        Serial.print((i * 10000 /* Hz, must be less than 10000 due to ADC limitations*/) / 128 /* Must be a power of 2*/);
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
    Serial.println("---"); // Separator between readings
}
