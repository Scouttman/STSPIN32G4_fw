#include <Arduino.h>
#include "SimpleFOC.h"
#include "SimpleFOCDrivers.h"
#include "drivers/stspin32g4/STSPIN32G4.h"

#define LED_WHITE PB_10 
#define LED_RED PC_13

STSPIN32G4 driver = STSPIN32G4();
BLDCMotor motor = BLDCMotor(7);

Encoder encoder = Encoder(PB5, PB4, 2048, PB6);

// interrupt routine intialisation
void doA(){encoder.handleA();}
void doB(){encoder.handleB();}
void doI(){encoder.handleIndex();}

unsigned long long startTime;
unsigned int counter;

void setup() {
  Serial.begin(115200);
  Serial.println("Start");

  // setup timer 5 to count micro seconds
  __HAL_RCC_TIM6_CLK_ENABLE();
  TIM6->PSC = HAL_RCC_GetPCLK1Freq()/1000000 - 1;
  TIM6->CR1 = TIM_CR1_CEN;

  /* Initialize all configured peripherals */
  pinMode(LED_WHITE, OUTPUT);
  pinMode(LED_RED, OUTPUT);

  // enable/disable quadrature mode
  encoder.quadrature = Quadrature::ON;
  
  // initialise encoder hardware
  encoder.init();
  encoder.enableInterrupts(doA, doB, doI);
  motor.linkSensor(&encoder);

  driver.voltage_power_supply = 24.0f;
  driver.voltage_limit = 1.0f;
  driver.init();
  motor.voltage_limit = driver.voltage_limit / 2.0f;
  // motor.controller = MotionControlType::velocity;
  // set the torque control type
  motor.phase_resistance = 12.5; // 12.5 Ohms
  motor.torque_controller = TorqueControlType::voltage;
  // set motion control loop to be used
  motor.controller = MotionControlType::torque;
  motor.linkDriver(&driver);
  motor.init();

  motor.PID_velocity.P = 0.1f;
  motor.PID_velocity.I = 0;
  motor.PID_velocity.D = 0;

  // initialize motor
  motor.init();
  // align encoder and start FOC
  motor.initFOC();
  // driver.voltage_limit = 6;

  Serial.println(F("Motor ready."));
  Serial.println(F("Set the target angle using serial terminal:"));
  _delay(1000);
  startTime = millis();
}

void loop() {
  // main FOC algorithm function
  motor.loopFOC();
  motor.move(1.0f); // 5 rad/s open loop
  delayMicroseconds(100); // STM32G4 is very fast, add a delay in open loop if we do nothing else
  if(driver.isFault()){
    digitalWrite(LED_WHITE, HIGH);
    digitalWrite(LED_RED, HIGH);
  }else{
    digitalWrite(LED_WHITE, LOW);
    digitalWrite(LED_RED, LOW);
  }
  if(counter%100){
    Serial.println(motor.voltage.d);
  }
}