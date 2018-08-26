/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
!!GlobalInfo
product: Pins v3.0
processor: MK82FN256xxx15
package_id: MK82FN256VLL15
mcu_data: ksdk2_0
processor_version: 2.0.0
board: FRDM-K82F
pin_labels:
- {pin_num: '95', pin_signal: PTD4/LLWU_P14/SPI0_PCS1/LPUART0_RTS_b/FTM0_CH4/FB_AD2/SDRAM_A10/EWM_IN/SPI1_PCS0, label: 'J2[6]/D10/SPI0_PCS1', identifier: DIR}
- {pin_num: '93', pin_signal: PTD2/LLWU_P13/SPI0_SOUT/LPUART2_RX/FTM3_CH2/FB_AD4/SDRAM_A12/I2C0_SCL, label: PTD2, identifier: RX}
- {pin_num: '94', pin_signal: PTD3/SPI0_SIN/LPUART2_TX/FTM3_CH3/FB_AD3/SDRAM_A11/I2C0_SDA, label: PTD3, identifier: TX}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */

#include "fsl_common.h"
#include "fsl_port.h"
#include "pin_mux.h"

/*FUNCTION**********************************************************************
 *
 * Function Name : BOARD_InitBootPins
 * Description   : Calls initialization functions.
 *
 *END**************************************************************************/
void BOARD_InitBootPins(void) {
    init_led_pins();
    init_dynamixel_uart();
    init_esp_uart();
}

#define PIN4_IDX                         4u   /*!< Pin number for pin 4 in a port */
#define PIN6_IDX                         6u   /*!< Pin number for pin 6 in a port */

/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
BOARD_InitButtons:
- options: {prefix: BOARD_, coreID: core0, enableClock: 'true'}
- pin_list:
  - {pin_num: '78', peripheral: GPIOC, signal: 'GPIO, 6', pin_signal: CMP0_IN0/PTC6/LLWU_P10/SPI0_SOUT/PDB0_EXTRG/I2S0_RX_BCLK/FB_AD9/SDRAM_A17/I2S0_MCLK/FXIO0_D14,
    direction: INPUT, slew_rate: slow, open_drain: disable, pull_select: down, pull_enable: disable}
  - {pin_num: '40', peripheral: GPIOA, signal: 'GPIO, 4', pin_signal: TSI0_CH5/PTA4/LLWU_P3/FTM0_CH1/FXIO0_D14/EMVSIM0_VCCEN/NMI_b, direction: INPUT, slew_rate: slow,
    open_drain: disable, pull_select: down, pull_enable: disable, passive_filter: disable}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */

/*FUNCTION**********************************************************************
 *
 * Function Name : BOARD_InitButtons
 * Description   : Configures pin routing and optionally pin electrical features.
 *
 *END**************************************************************************/
void BOARD_InitButtons(void) {
  CLOCK_EnableClock(kCLOCK_PortA);                           /* Port A Clock Gate Control: Clock enabled */
  CLOCK_EnableClock(kCLOCK_PortC);                           /* Port C Clock Gate Control: Clock enabled */

  const port_pin_config_t porta4_pin40_config = {
    kPORT_PullDisable,                                       /* Internal pull-up/down resistor is disabled */
    kPORT_SlowSlewRate,                                      /* Slow slew rate is configured */
    kPORT_PassiveFilterDisable,                              /* Passive filter is disabled */
    kPORT_OpenDrainDisable,                                  /* Open drain is disabled */
    kPORT_LowDriveStrength,                                  /* Low drive strength is configured */
    kPORT_MuxAsGpio,                                         /* Pin is configured as PTA4 */
    kPORT_UnlockRegister                                     /* Pin Control Register fields [15:0] are not locked */
  };
  PORT_SetPinConfig(PORTA, PIN4_IDX, &porta4_pin40_config);  /* PORTA4 (pin 40) is configured as PTA4 */
  const port_pin_config_t portc6_pin78_config = {
    kPORT_PullDisable,                                       /* Internal pull-up/down resistor is disabled */
    kPORT_SlowSlewRate,                                      /* Slow slew rate is configured */
    kPORT_PassiveFilterDisable,                              /* Passive filter is disabled */
    kPORT_OpenDrainDisable,                                  /* Open drain is disabled */
    kPORT_LowDriveStrength,                                  /* Low drive strength is configured */
    kPORT_MuxAsGpio,                                         /* Pin is configured as PTC6 */
    kPORT_UnlockRegister                                     /* Pin Control Register fields [15:0] are not locked */
  };
  PORT_SetPinConfig(PORTC, PIN6_IDX, &portc6_pin78_config);  /* PORTC6 (pin 78) is configured as PTC6 */
}


#define PIN18_IDX                       18u   /*!< Pin number for pin 18 in a port */
#define PIN19_IDX                       19u   /*!< Pin number for pin 19 in a port */

/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
BOARD_InitOSC:
- options: {prefix: BOARD_, coreID: core0, enableClock: 'true'}
- pin_list:
  - {pin_num: '50', peripheral: OSC, signal: EXTAL0, pin_signal: EXTAL0/PTA18/FTM0_FLT2/FTM_CLKIN0/TPM_CLKIN0, slew_rate: no_init, open_drain: no_init, pull_select: no_init,
    pull_enable: no_init}
  - {pin_num: '51', peripheral: OSC, signal: XTAL0, pin_signal: XTAL0/PTA19/FTM1_FLT0/FTM_CLKIN1/LPTMR0_ALT1/LPTMR1_ALT1/TPM_CLKIN1, slew_rate: no_init, open_drain: no_init,
    pull_select: no_init, pull_enable: no_init}
  - {pin_num: '31', peripheral: RTC, signal: XTAL32, pin_signal: XTAL32}
  - {pin_num: '32', peripheral: RTC, signal: EXTAL32, pin_signal: EXTAL32}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */

/*FUNCTION**********************************************************************
 *
 * Function Name : BOARD_InitOSC
 * Description   : Configures pin routing and optionally pin electrical features.
 *
 *END**************************************************************************/
void BOARD_InitOSC(void) {
  CLOCK_EnableClock(kCLOCK_PortA);                           /* Port A Clock Gate Control: Clock enabled */

  PORT_SetPinMux(PORTA, PIN18_IDX, kPORT_PinDisabledOrAnalog); /* PORTA18 (pin 50) is configured as EXTAL0 */
  PORT_SetPinMux(PORTA, PIN19_IDX, kPORT_PinDisabledOrAnalog); /* PORTA19 (pin 51) is configured as XTAL0 */
}


#define PIN14_IDX                       14u   /*!< Pin number for pin 14 in a port */
#define PIN15_IDX                       15u   /*!< Pin number for pin 15 in a port */

/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
BOARD_InitDEBUG_UART:
- options: {prefix: BOARD_, coreID: core0, enableClock: 'true'}
- pin_list:
  - {pin_num: '87', peripheral: LPUART4, signal: TX, pin_signal: PTC15/LPUART4_TX/FB_AD24/SDRAM_D24/FXIO0_D21, direction: OUTPUT, slew_rate: fast, open_drain: disable,
    pull_select: down, pull_enable: disable}
  - {pin_num: '86', peripheral: LPUART4, signal: RX, pin_signal: PTC14/LPUART4_RX/FB_AD25/SDRAM_D25/FXIO0_D20, slew_rate: fast, open_drain: disable, pull_select: down,
    pull_enable: disable}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */

/*FUNCTION**********************************************************************
 *
 * Function Name : BOARD_InitDEBUG_UART
 * Description   : Configures pin routing and optionally pin electrical features.
 *
 *END**************************************************************************/
void BOARD_InitDEBUG_UART(void) {
  CLOCK_EnableClock(kCLOCK_PortC);                           /* Port C Clock Gate Control: Clock enabled */

  const port_pin_config_t portc14_pin86_config = {
    kPORT_PullDisable,                                       /* Internal pull-up/down resistor is disabled */
    kPORT_FastSlewRate,                                      /* Fast slew rate is configured */
    kPORT_PassiveFilterDisable,                              /* Passive filter is disabled */
    kPORT_OpenDrainDisable,                                  /* Open drain is disabled */
    kPORT_LowDriveStrength,                                  /* Low drive strength is configured */
    kPORT_MuxAlt3,                                           /* Pin is configured as LPUART4_RX */
    kPORT_UnlockRegister                                     /* Pin Control Register fields [15:0] are not locked */
  };
  PORT_SetPinConfig(PORTC, PIN14_IDX, &portc14_pin86_config); /* PORTC14 (pin 86) is configured as LPUART4_RX */
  const port_pin_config_t portc15_pin87_config = {
    kPORT_PullDisable,                                       /* Internal pull-up/down resistor is disabled */
    kPORT_FastSlewRate,                                      /* Fast slew rate is configured */
    kPORT_PassiveFilterDisable,                              /* Passive filter is disabled */
    kPORT_OpenDrainDisable,                                  /* Open drain is disabled */
    kPORT_LowDriveStrength,                                  /* Low drive strength is configured */
    kPORT_MuxAlt3,                                           /* Pin is configured as LPUART4_TX */
    kPORT_UnlockRegister                                     /* Pin Control Register fields [15:0] are not locked */
  };
  PORT_SetPinConfig(PORTC, PIN15_IDX, &portc15_pin87_config); /* PORTC15 (pin 87) is configured as LPUART4_TX */
}


#define PIN8_IDX                         8u   /*!< Pin number for pin 8 in a port */
#define PIN9_IDX                         9u   /*!< Pin number for pin 9 in a port */
#define PIN10_IDX                       10u   /*!< Pin number for pin 10 in a port */

/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
init_led_pins:
- options: {callFromInitBoot: 'true', prefix: ONBOARD_, coreID: core0, enableClock: 'true'}
- pin_list:
  - {pin_num: '80', peripheral: GPIOC, signal: 'GPIO, 8', pin_signal: CMP0_IN2/PTC8/FTM3_CH4/I2S0_MCLK/FB_AD7/SDRAM_A15/FXIO0_D16, identifier: LED_RED, direction: OUTPUT}
  - {pin_num: '81', peripheral: GPIOC, signal: 'GPIO, 9', pin_signal: CMP0_IN3/PTC9/FTM3_CH5/I2S0_RX_BCLK/FB_AD6/SDRAM_A14/FTM2_FLT0/FXIO0_D17, direction: OUTPUT}
  - {pin_num: '82', peripheral: GPIOC, signal: 'GPIO, 10', pin_signal: PTC10/I2C1_SCL/FTM3_CH6/I2S0_RX_FS/FB_AD5/SDRAM_A13/FXIO0_D18, direction: OUTPUT}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */

/*FUNCTION**********************************************************************
 *
 * Function Name : init_led_pins
 * Description   : Configures pin routing and optionally pin electrical features.
 *
 *END**************************************************************************/
void init_led_pins(void) {
  CLOCK_EnableClock(kCLOCK_PortC);                           /* Port C Clock Gate Control: Clock enabled */

  PORT_SetPinMux(PORTC, PIN10_IDX, kPORT_MuxAsGpio);         /* PORTC10 (pin 82) is configured as PTC10 */
  PORT_SetPinMux(PORTC, PIN8_IDX, kPORT_MuxAsGpio);          /* PORTC8 (pin 80) is configured as PTC8 */
  PORT_SetPinMux(PORTC, PIN9_IDX, kPORT_MuxAsGpio);          /* PORTC9 (pin 81) is configured as PTC9 */
}


#define PIN2_IDX                         2u   /*!< Pin number for pin 2 in a port */
#define PIN3_IDX                         3u   /*!< Pin number for pin 3 in a port */

#define PIN4_IDX                         4u   /*!< Pin number for pin 4 in a port */

/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
init_dynamixel_uart:
- options: {callFromInitBoot: 'true', prefix: DYNAMIXEL_, coreID: core0, enableClock: 'true'}
- pin_list:
  - {pin_num: '94', peripheral: LPUART2, signal: TX, pin_signal: PTD3/SPI0_SIN/LPUART2_TX/FTM3_CH3/FB_AD3/SDRAM_A11/I2C0_SDA, direction: OUTPUT}
  - {pin_num: '93', peripheral: LPUART2, signal: RX, pin_signal: PTD2/LLWU_P13/SPI0_SOUT/LPUART2_RX/FTM3_CH2/FB_AD4/SDRAM_A12/I2C0_SCL}
  - {pin_num: '95', peripheral: GPIOD, signal: 'GPIO, 4', pin_signal: PTD4/LLWU_P14/SPI0_PCS1/LPUART0_RTS_b/FTM0_CH4/FB_AD2/SDRAM_A10/EWM_IN/SPI1_PCS0, direction: OUTPUT}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */

/*FUNCTION**********************************************************************
 *
 * Function Name : init_dynamixel_uart
 * Description   : Configures pin routing and optionally pin electrical features.
 *
 *END**************************************************************************/
void init_dynamixel_uart(void) {
  CLOCK_EnableClock(kCLOCK_PortD);                           /* Port D Clock Gate Control: Clock enabled */

  PORT_SetPinMux(PORTD, PIN2_IDX, kPORT_MuxAlt3);            /* PORTD2 (pin 93) is configured as LPUART2_RX */
  PORT_SetPinMux(PORTD, PIN3_IDX, kPORT_MuxAlt3);            /* PORTD3 (pin 94) is configured as LPUART2_TX */
  PORT_SetPinMux(PORTD, PIN4_IDX, kPORT_MuxAsGpio);          /* PORTD4 (pin 95) is configured as PTD4 */
}


#define PIN1_IDX                         1u   /*!< Pin number for pin 1 in a port */

#define PIN2_IDX                         2u   /*!< Pin number for pin 2 in a port */
#define SOPT5_LPUART0RXSRC_LPUART_RX  0x00u   /*!< LPUART0 receive data source select: LPUART0_RX pin */
#define SOPT5_LPUART0TXSRC_LPUART_TX  0x00u   /*!< LPUART0 transmit data source select: LPUART0_TX pin */

/*
 * TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
init_esp_uart:
- options: {callFromInitBoot: 'true', prefix: ESP_, coreID: core0, enableClock: 'true'}
- pin_list:
  - {pin_num: '37', peripheral: LPUART0, signal: RX, pin_signal: TSI0_CH2/PTA1/LPUART0_RX/FTM0_CH6/I2C3_SDA/FXIO0_D11/EMVSIM0_IO/JTAG_TDI}
  - {pin_num: '38', peripheral: LPUART0, signal: TX, pin_signal: TSI0_CH3/PTA2/LPUART0_TX/FTM0_CH7/I2C3_SCL/FXIO0_D12/EMVSIM0_PD/JTAG_TDO/TRACE_SWO, direction: OUTPUT}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS ***********
 */

/*FUNCTION**********************************************************************
 *
 * Function Name : init_esp_uart
 * Description   : Configures pin routing and optionally pin electrical features.
 *
 *END**************************************************************************/
void init_esp_uart(void) {
  CLOCK_EnableClock(kCLOCK_PortA);                           /* Port A Clock Gate Control: Clock enabled */

  PORT_SetPinMux(PORTA, PIN1_IDX, kPORT_MuxAlt2);            /* PORTA1 (pin 37) is configured as LPUART0_RX */
  PORT_SetPinMux(PORTA, PIN2_IDX, kPORT_MuxAlt2);            /* PORTA2 (pin 38) is configured as LPUART0_TX */
  SIM->SOPT5 = ((SIM->SOPT5 &
    (~(SIM_SOPT5_LPUART0TXSRC_MASK | SIM_SOPT5_LPUART0RXSRC_MASK))) /* Mask bits to zero which are setting */
      | SIM_SOPT5_LPUART0TXSRC(SOPT5_LPUART0TXSRC_LPUART_TX) /* LPUART0 transmit data source select: LPUART0_TX pin */
      | SIM_SOPT5_LPUART0RXSRC(SOPT5_LPUART0RXSRC_LPUART_RX) /* LPUART0 receive data source select: LPUART0_RX pin */
    );
}

/*******************************************************************************
 * EOF
 ******************************************************************************/
