#define FASTADC 1

// defines for setting and clearing register bits
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

const float decayFactor = 0.99;
const float adjustFactor = 0.5;
const int numSensors = 2; //8;
int analogPins[numSensors] = {A0, A1};//, A2, A3, A4, A5, A6, A7};
struct SensorPz {
    //int pin;
    int maxVal;
    int smoothVal;
    unsigned long lastPeakTmp;
    unsigned long lastValTmp;
    const char type; //diametro do sensor
};
SensorPz sensorPz[numSensors] = {
    {0, 0, 0, 0, 0, 'A'},
    {0, 0, 0, 0, 0, 'B'}/*,
    {0, 0, 0, 0, 0, 'C'},
    {0, 0, 0, 0, 0, 'D'},
    {0, 0, 0, 0, 0, 'E'},
    {0, 0, 0, 0, 0, 'F'},
    {0, 0, 0, 0, 0, 'G'},
    {0, 0, 0, 0, 0, 'H'}*/
};
//int maxVals[numSensors] = {0, 0};//, 0, 0, 0, 0, 0, 0};
//int smoothVals[numSensors] = {0, 0};//, 0, 0, 0, 0, 0, 0};
class Queue;
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
        Serial.print(sensorPz[i].smoothVals);
        if (i < numSensors - 1) {
            Serial.print(","); // Use comma as a delimiter
        }
    }
    Serial.println(); // Move to the next line after printing all sensor values
    delay(20); // Short delay to make the plot smoother
}

void updateMaxVals(int i, int newValue) {   //atualiza o máximo valor desde o último nulo (menor que 2)
    Serial.print(newValue);                 //caso haja uma leitura não nula, smoothVals é atualizado 
    Serial.print(", ");                     //da diferença entre o máximo e o novo valor multiplicado por adjustFactor
    sensorPz[i].smoothVals = sensorPz[i].maxVals * decayFactor;
    if (newValue > sensorPz[i].maxVals) {
        sensorPz[i].smoothVals = newValue;
        if (newValue > sensorPz[i].maxVals){sensorPz[i].maxVals = newValue;lastPeakTmp = micros();return;}//finaliza se for maior que o ultimo pico
    }else{
        if (sensorPz[i].smoothVals < 2) {sensorPz[i].maxVals = 0;return;}//finaliza se for zero
        sensorPz[i].smoothVals = sensorPz[i].smoothVals + (sensorPz[i].maxVals - newValue) * adjustFactor;
        lastValTmp = micros();
    }
    return;
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
                return;
            }
            rear = (rear + 1)%capacity;
            array[rear] = item;
            size = size + 1;
        }
        int dequeue(){
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