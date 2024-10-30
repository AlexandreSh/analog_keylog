#include <arduinoFFT.h>
#include <TimerOne.h>
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

void setup() {
    #if FASTADC
        // set prescale to 16 (faster adc)
        sbi(ADCSRA,ADPS2) ;
        cbi(ADCSRA,ADPS1) ;
        cbi(ADCSRA,ADPS0) ;
    #endif
    Serial.begin(115200);
    analogReference(INTERNAL2V56);  // Use the internal 2,56V reference voltage
    sampling_period_us = round(1000000 * (1.0 / SAMPLING_FREQUENCY));
    Timer1.initialize(intervaloMedicao * 1000); //interrupção para rodar leituras no dominio do tempo
    Timer1.attachInterrupt(timerIsr);
}

void loop() {
  // Read the input on analog pin 1
  int sensorValue = analogRead(A1);
  
  // Print the sensor value to the serial monitor
  Serial.println(sensorValue);
  
  // Wait for 100 milliseconds before the next loop
  delay(15);
}