#include <arduinoFFT.h>
#include <timerOne.h>
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
ArduinoFFT<float> FFT = ArduinoFFT<float>(vReal, vImag, SAMPLES, SAMPLING_FREQUENCY);

float vReal[SAMPLES];
float vImag[SAMPLES];
unsigned int sampling_period_us;

const unsigned int tamanhoFila = 10;
class Queue;
struct SensorPz {
    //int pin;
    int maxVal;
    int smoothVal;
    unsigned long lastPeakTmp;
    unsigned long lastValTmp;
    const char type; //diametro do sensor
    Queue filaLeituras; //fila para os dados para o serial (armazenar picos numa fila?)
    Queue filaSmooth; //fila para os dados para o serial (armazenar picos numa fila?)
    SensorPz(char t, unsigned capacity1 = tamanhoFila, unsigned capacity2 = tamanhoFila): type(t), filaLeituras(capacity1), filaSmooth(capacity2) {} // virou POO?
};
SensorPz sensorPz[numSensors] = {
    {0, 0, 0, 0, 0, 'A'},
    {0, 0, 0, 0, 0, 'B'},
    {0, 0, 0, 0, 0, 'A'},
    {0, 0, 0, 0, 0, 'B'},
    {0, 0, 0, 0, 0, 'A'},
    {0, 0, 0, 0, 0, 'B'},
    {0, 0, 0, 0, 0, 'A'},
    {0, 0, 0, 0, 0, 'B'}
};

const int numSensors = 8;
int analogPins[numSensors] = {A0, A1, A2, A3, A4, A5, A6, A7};

const float decayFactor = 0.99;
const float adjustFactor = 0.5;
const int intervaloMedicao = 20; //de quanto em quanto tempo medir os sensores no tempo, msec
volatile bool dominioTempo = false;

unsigned long microseconds;
const unsigned long TIMEOUT = 180000; // Timeout in milliseconds (180 seconds)
bool fftRunning = false;

unsigned long lastSendTime = 0;

void setup() {
    #if FASTADC
        // set prescale to 16 (faster adc)
        sbi(ADCSRA,ADPS2) ;
        cbi(ADCSRA,ADPS1) ;
        cbi(ADCSRA,ADPS0) ;
    #endif
    Serial.begin(115200);
    analogReference(INTERNAL1V1);  // Use the internal 1.1V reference voltage
    sampling_period_us = round(1000000 * (1.0 / SAMPLING_FREQUENCY));
    Timer1.Initialize(intervaloMedicao * 1000); //interrupção para rodar leituras no dominio do tempo
    Timer1.attachInterrupt(timerIsr);
}

void loop() {
    if (millis() - lastSendTime >= 2000) { // dominio da frequencia a cada 2 segundos
    performFFT();
    lastSendTime = millis();
    }
    if(dominioTempo){
        funcTempoSlo();
        dominioTempo = false;
    }
    // Short delay before the next measurement
    delay(50);
}

void timerIsr() {
    for (int i = 0; i < numSensors; i++) {
        funcTempoFast(i, analogRead(analogPins[i]));
    }
    Serial.println();
    dominioTempo = true;
}

void funcTempoSlo(){ 
    for (int i = 0; i < numSensors; i++) {
        Serial.print(analogPins[i]);
        Serial.print(",");
        Serial.print(sensorPz[i].smoothVals);
        if (i < numSensors - 1) {
            Serial.print(","); 
        }
    }
    Serial.println();
}

void funcTempoFast(int i, int newValue) {                       //atualiza o máximo valor desde o último nulo (menor que 2)
    sensorPz[i].smoothVals = sensorPz[i].maxVals * decayFactor; //caso haja uma leitura não nula, smoothVals é atualizado 
    if (newValue > sensorPz[i].maxVals) {                       //da diferença entre o máximo e o novo valor multiplicado por adjustFactor
        sensorPz[i].smoothVals = newValue;
        if (newValue > sensorPz[i].maxVals){sensorPz[i].maxVals = newValue;lastPeakTmp = micros();return;}//finaliza se for maior que o ultimo pico
    }else{
        if (sensorPz[i].smoothVals < 2) {sensorPz[i].maxVals = 0;return;}//finaliza se for zero
        sensorPz[i].smoothVals = sensorPz[i].smoothVals + (sensorPz[i].maxVals - newValue) * adjustFactor;
        lastValTmp = micros();
    }
    return;
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

class Queue;{
    public:
        int front, rear, size;
        unsigned capacity;
        int* array;
    public:
        Queue(unsigned capacity){
            this->capacity = capacity;
            front = size = 0;
            rear = capacity - 1;
            array = new int[(this->capacity];
        }
        ~Queue(){
            delete[] array;
        }
        bool isFull(Queue* queue){
            return (queue->size == queue->capacity);
        }
        bool isEmpty(Queue* queue){
            return(queue->size == 0);
        }
        void enqueue(int item){
            if(isFull()){
                //faz algo se a fila estiver cheia
                dequeue();
                return;
            }
            rear = (rear + 1)%capacity;
            array[rear] = item;
            size = size + 1;
        }
        int dequeue(){
            if (size == 1) {
                // If the queue has only one item left, don't actually remove it
                return array[front];
            }
            if(isEmpty()){
                //faz algo se a fila estiver vazia
                return INT_MIN;
            }
            int item = array[front];
            front = (front + 1)%capacity;
            size = size - 1;
            return item;
        }   
        int front(){
            if(isEmpty()){
                return INT_MIN;
            }
            return array[front];
        }
        int rear(){
            if(isEmpty()){
                return INT_MIN;
            }
            return array[rear];
        }
};