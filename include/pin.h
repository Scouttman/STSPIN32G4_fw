#define LED_WHITE PB_10 
#define LED_RED PC_13

// MCU to gate driver internal connection
#define WAKE    PE7     //Push-pull output
#define INL1    PE8     //TIM1_CH1N(1)
#define INH1    PE9     //TIM1_CH1(1)
#define INL2    PE10    //TIM1_CH2N(1)
#define INH2    PE11    //TIM1_CH2(1)
#define INL3    PE12    //TIM1_CH3N(1)
#define INH3    PE13    //TIM1_CH3(1)
// The READY open-drain output indicates if the device status does not allow the driving of the external MOSFETs.
// The READY output can be programmed to report one or more of the following conditions:
// • The VCC supply is below the undervoltage threshold.
// • Thermal shutdown.
// • Entering standby mode.
// By default, the pin reports the standby and the VCC undervoltage condition (refer to Table 19).
// pulled low of fault
#define READY   PE14    //Input with pull-up, TIM1_BKIN2(2)(optional)
//nFAULT output
// The nFAULT open-drain output indicates a critical failure condition. The output can be configured to report one or
// more of the following failures:
// • The device performed a RESET (this signaling cannot be masked)
// • VDS monitoring protection is triggered
// • Thermal shutdown
// • The VCC supply is below the undervoltage threshold
// By default, the nFAULT pin reports all above failures (refer to Table 20).
// pulled low of fault
#define NFAULT  PE15    //Input with pull-up, TIM1_BKIN (2)(optional)
#define SCL     PC8     //I2C3_SCL
#define SDA     PC9     //I2C3_SDA