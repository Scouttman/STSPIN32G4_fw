#include <Arduino.h>
#include "main.h"
#include "pin.h"
#include "stspin32g4.h"
#define _STM32_DEF_ // this shoudn't be needed
#include <SimpleFOC.h>

#ifdef SIMPLEFOC_H
// BLDCMotor motor = BLDCMotor(11); // for xing
// BLDCMotor motor = BLDCMotor(7); // for gartt
BLDCMotor motor = BLDCMotor(7); // for drive module

// BLDC driver instance
BLDCDriver6PWM driver = BLDCDriver6PWM(INH1,INL1,INH2,INL2,INH3,INL3);
// BLDCDriver6PWM driver = BLDCDriver6PWM(LED_YELLOW,INL1,INH2,INL2,INH3,INL3);
// int phA_h,int phA_l,int phB_h,int phB_l,int phC_h,int phC_l, int en = NOT_SET
float target_velocity = 10;
#endif

static void MX_I2C3_Init(void);

I2C_HandleTypeDef hi2c3;
STSPIN32G4_HandleTypeDef HdlSTSPING4;
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;

Encoder encoder = Encoder(PB5, PB4, 2048, PB3); //AMT
// Encoder encoder = Encoder(PB5, PB4, 5000, PB3);
// interrupt routine intialisation
void doA(){encoder.handleA();}
void doB(){encoder.handleB();}
void doI(){encoder.handleIndex();}
unsigned long long startTime;
unsigned int counter = 0;

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
  // digitalWrite(LED_WHITE, HIGH);
  // analogWriteFrequency(12000);
  analogWrite(LED_WHITE, LOW);
  digitalWrite(LED_RED, LOW);

  motor.PID_velocity.P = 0.1f;
  motor.PID_velocity.I = 30;
  motor.PID_velocity.D = 0;

  // enable/disable quadrature mode
  encoder.quadrature = Quadrature::ON;
  
  // initialise encoder hardware
  encoder.init();
  encoder.enableInterrupts(doA, doB, doI);
  motor.linkSensor(&encoder);

  //   /* USER CODE BEGIN 2 */
  // pwm frequency to be used [Hz]
  // for atmega328 fixed to 32kHz
  // esp32/stm32/teensy configurable
  driver.pwm_frequency = 32000;
  // power supply voltage [V]
  driver.voltage_power_supply = 24;
  // Max DC voltage allowed - default voltage_power_supply
  driver.voltage_limit = 8.0; //>3 breaks things?1.5;//5.0;
  // dead_zone [0,1] - default 0.02f - 2%
  driver.dead_zone = 0.05f;

  // driver init
  if(!driver.init()){
    // Error_Handler();
  }

  setupDriver();
  motor.linkDriver(&driver);
  motor.voltage_limit = 4;
  motor.voltage_sensor_align = 1;

  // 
  motor.controller = MotionControlType::velocity;
  
  // driver.voltage_limit = 0.5; // lower voltage for homing
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
  digitalWrite(LED_WHITE, !digitalRead(READY));
  digitalWrite(LED_RED, !digitalRead(NFAULT));
  if(!digitalRead(NFAULT)){
    Serial.print("Error:");
    STSPIN32G4_statusRegTypeDef statusReg;
	  STSPIN32G4_getStatus(&HdlSTSPING4, &statusReg);
    // Serial.println((int)statusReg, BIN);
    if(statusReg.vccUvlo){
      Serial.println("VCC UVLO (voltage to low)");
    }
    if(statusReg.thsd){
      Serial.println("voltage regulator thermal shutdown");
    }
    if(statusReg.vdsp){
      Serial.println("Overvoltage protection");
    }
    if(statusReg.reset){
      Serial.println("the device performed a reset or power up\n this could mean a brown out occurred");
    }
  }

  motor.loopFOC();
  // driver.setPwm(1,1,1);
  // delay(100);
  motor.move(100*_2PI);

  if(millis() - startTime > 10000){
    motor.disable();
  }
  
  if(counter%10000==1){
    // display the angle and the angular velocity to the terminal
    // Serial.println(encoder.getAngle());
    Serial.println(encoder.getVelocity()/_2PI);
    // Serial.println() #TODO print voltage level
  }
  counter++;
}

void setupDriver(){
  pinMode(WAKE, OUTPUT);
  pinMode(READY, INPUT_PULLUP);
  pinMode(NFAULT, INPUT_PULLUP);
  digitalWrite(WAKE, HIGH); // STSPIN init should set this high but why not

  /*************************************************/
  /*   STSPIN32G4 driver component initialization  */
  /*************************************************/
  MX_I2C3_Init();
  STSPIN32G4_init( &HdlSTSPING4 );
  // Purge registers to default values
  if(STSPIN32G4_reset(&HdlSTSPING4) != STSPIN32G4_OK){
    Error_Handler();
  }

  // Disable the Buck regulator and demap VCC UVLO on NFAULT pin
  STSPIN32G4_confVCC blah;
  STSPIN32G4_confVCC vcc = {.voltage = blah._EXT, .useNFAULT=true, .useREADY=false};
  if(STSPIN32G4_setVCC(&HdlSTSPING4, vcc) != STSPIN32G4_OK){Error_Handler();}

  // Configure VDSP protection with 4us deglitch time and map triggering on NFAULT pin
  STSPIN32G4_confVDSP hmm;
  STSPIN32G4_confVDSP vdsp = {.deglitchTime=hmm._6us, .useNFAULT=true};
  if(STSPIN32G4_setVDSP(&HdlSTSPING4, vdsp) != STSPIN32G4_OK){Error_Handler();}

  // After reset a clearing of fault is needed to enable GHSx/GLSx outputs
  if(STSPIN32G4_clearFaults(&HdlSTSPING4) != STSPIN32G4_OK){Error_Handler();}
}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.Timing = 0x30A0A7FB;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

