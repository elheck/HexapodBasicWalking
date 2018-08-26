#ifndef _PIN_MUX_H_
#define _PIN_MUX_H_


/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*! @brief Direction type  */
typedef enum _pin_mux_direction
{
  kPIN_MUX_DirectionInput = 0U,         /* Input direction */
  kPIN_MUX_DirectionOutput = 1U,        /* Output direction */
  kPIN_MUX_DirectionInputOrOutput = 2U  /* Input or output direction */
} pin_mux_direction_t;

/*!
 * @addtogroup pin_mux
 * @{
 */

/*******************************************************************************
 * API
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif

/*!
 * @brief Calls initialization functions.
 *
 */
void BOARD_InitBootPins(void);

/* PORTC6 (number 78), SW3 */
#define BOARD_SW3_GPIO                                                     GPIOC   /*!< GPIO device name: GPIOC */
#define BOARD_SW3_PORT                                                     PORTC   /*!< PORT device name: PORTC */
#define BOARD_SW3_GPIO_PIN                                                    6U   /*!< PORTC pin index: 6 */
#define BOARD_SW3_PIN_NAME                                                  PTC6   /*!< Pin name */
#define BOARD_SW3_LABEL                                                    "SW3"   /*!< Label */
#define BOARD_SW3_NAME                                                     "SW3"   /*!< Identifier name */
#define BOARD_SW3_DIRECTION                              kPIN_MUX_DirectionInput   /*!< Direction */

/* PORTA4 (number 40), SW2 */
#define BOARD_SW2_GPIO                                                     GPIOA   /*!< GPIO device name: GPIOA */
#define BOARD_SW2_PORT                                                     PORTA   /*!< PORT device name: PORTA */
#define BOARD_SW2_GPIO_PIN                                                    4U   /*!< PORTA pin index: 4 */
#define BOARD_SW2_PIN_NAME                                                  PTA4   /*!< Pin name */
#define BOARD_SW2_LABEL                                                    "SW2"   /*!< Label */
#define BOARD_SW2_NAME                                                     "SW2"   /*!< Identifier name */
#define BOARD_SW2_DIRECTION                              kPIN_MUX_DirectionInput   /*!< Direction */


/*!
 * @brief Configures pin routing and optionally pin electrical features.
 *
 */
void BOARD_InitButtons(void);

/* PORTA18 (number 50), Y2[3]/EXTAL */
#define BOARD_EXTAL0_PERIPHERAL                                              OSC   /*!< Device name: OSC */
#define BOARD_EXTAL0_SIGNAL                                               EXTAL0   /*!< OSC signal: EXTAL0 */
#define BOARD_EXTAL0_PIN_NAME                                             EXTAL0   /*!< Pin name */
#define BOARD_EXTAL0_LABEL                                         "Y2[3]/EXTAL"   /*!< Label */
#define BOARD_EXTAL0_NAME                                               "EXTAL0"   /*!< Identifier name */

/* PORTA19 (number 51), Y2[1]/XTAL */
#define BOARD_XTAL0_PERIPHERAL                                               OSC   /*!< Device name: OSC */
#define BOARD_XTAL0_SIGNAL                                                 XTAL0   /*!< OSC signal: XTAL0 */
#define BOARD_XTAL0_PIN_NAME                                               XTAL0   /*!< Pin name */
#define BOARD_XTAL0_LABEL                                           "Y2[1]/XTAL"   /*!< Label */
#define BOARD_XTAL0_NAME                                                 "XTAL0"   /*!< Identifier name */

/* XTAL32 (number 31), Y1[2]/XTAL32_RTC */
#define BOARD_XTAL32K_PERIPHERAL                                             RTC   /*!< Device name: RTC */
#define BOARD_XTAL32K_SIGNAL                                              XTAL32   /*!< RTC signal: XTAL32 */
#define BOARD_XTAL32K_PIN_NAME                                            XTAL32   /*!< Pin name */
#define BOARD_XTAL32K_LABEL                                   "Y1[2]/XTAL32_RTC"   /*!< Label */
#define BOARD_XTAL32K_NAME                                             "XTAL32K"   /*!< Identifier name */

/* EXTAL32 (number 32), Y1[1]/EXTAL32_RTC */
#define BOARD_EXTAL32K_PERIPHERAL                                            RTC   /*!< Device name: RTC */
#define BOARD_EXTAL32K_SIGNAL                                            EXTAL32   /*!< RTC signal: EXTAL32 */
#define BOARD_EXTAL32K_PIN_NAME                                          EXTAL32   /*!< Pin name */
#define BOARD_EXTAL32K_LABEL                                 "Y1[1]/EXTAL32_RTC"   /*!< Label */
#define BOARD_EXTAL32K_NAME                                           "EXTAL32K"   /*!< Identifier name */


/*!
 * @brief Configures pin routing and optionally pin electrical features.
 *
 */
void BOARD_InitOSC(void);

/* PORTC15 (number 87), OpenSDA_UART */
#define BOARD_DEBUG_UART_TX_PERIPHERAL                                   LPUART4   /*!< Device name: LPUART4 */
#define BOARD_DEBUG_UART_TX_SIGNAL                                            TX   /*!< LPUART4 signal: TX */
#define BOARD_DEBUG_UART_TX_PIN_NAME                                  LPUART4_TX   /*!< Pin name */
#define BOARD_DEBUG_UART_TX_LABEL                                 "OpenSDA_UART"   /*!< Label */
#define BOARD_DEBUG_UART_TX_NAME                                 "DEBUG_UART_TX"   /*!< Identifier name */
#define BOARD_DEBUG_UART_TX_DIRECTION                   kPIN_MUX_DirectionOutput   /*!< Direction */

/* PORTC14 (number 86), OpenSDA_UART */
#define BOARD_DEBUG_UART_RX_PERIPHERAL                                   LPUART4   /*!< Device name: LPUART4 */
#define BOARD_DEBUG_UART_RX_SIGNAL                                            RX   /*!< LPUART4 signal: RX */
#define BOARD_DEBUG_UART_RX_PIN_NAME                                  LPUART4_RX   /*!< Pin name */
#define BOARD_DEBUG_UART_RX_LABEL                                 "OpenSDA_UART"   /*!< Label */
#define BOARD_DEBUG_UART_RX_NAME                                 "DEBUG_UART_RX"   /*!< Identifier name */


/*!
 * @brief Configures pin routing and optionally pin electrical features.
 *
 */
void BOARD_InitDEBUG_UART(void);

/* PORTC8 (number 80), J1[14]/J12[17]/D6/LEDRGB_RED/Camera Reset */
#define ONBOARD_LED_RED_GPIO                                               GPIOC   /*!< GPIO device name: GPIOC */
#define ONBOARD_LED_RED_PORT                                               PORTC   /*!< PORT device name: PORTC */
#define ONBOARD_LED_RED_GPIO_PIN                                              8U   /*!< PORTC pin index: 8 */
#define ONBOARD_LED_RED_PIN_NAME                                            PTC8   /*!< Pin name */
#define ONBOARD_LED_RED_LABEL        "J1[14]/J12[17]/D6/LEDRGB_RED/Camera Reset"   /*!< Label */
#define ONBOARD_LED_RED_NAME                                           "LED_RED"   /*!< Identifier name */
#define ONBOARD_LED_RED_DIRECTION                       kPIN_MUX_DirectionOutput   /*!< Direction */

/* PORTC9 (number 81), J1[16]/D7/LEDRGB_GREEN */
#define ONBOARD_LED_GREEN_GPIO                                             GPIOC   /*!< GPIO device name: GPIOC */
#define ONBOARD_LED_GREEN_PORT                                             PORTC   /*!< PORT device name: PORTC */
#define ONBOARD_LED_GREEN_GPIO_PIN                                            9U   /*!< PORTC pin index: 9 */
#define ONBOARD_LED_GREEN_PIN_NAME                                          PTC9   /*!< Pin name */
#define ONBOARD_LED_GREEN_LABEL                         "J1[16]/D7/LEDRGB_GREEN"   /*!< Label */
#define ONBOARD_LED_GREEN_NAME                                       "LED_GREEN"   /*!< Identifier name */
#define ONBOARD_LED_GREEN_DIRECTION                     kPIN_MUX_DirectionOutput   /*!< Direction */

/* PORTC10 (number 82), J1[12]/D5/LEDRGB_BLUE */
#define ONBOARD_LED_BLUE_GPIO                                              GPIOC   /*!< GPIO device name: GPIOC */
#define ONBOARD_LED_BLUE_PORT                                              PORTC   /*!< PORT device name: PORTC */
#define ONBOARD_LED_BLUE_GPIO_PIN                                            10U   /*!< PORTC pin index: 10 */
#define ONBOARD_LED_BLUE_PIN_NAME                                          PTC10   /*!< Pin name */
#define ONBOARD_LED_BLUE_LABEL                           "J1[12]/D5/LEDRGB_BLUE"   /*!< Label */
#define ONBOARD_LED_BLUE_NAME                                         "LED_BLUE"   /*!< Identifier name */
#define ONBOARD_LED_BLUE_DIRECTION                      kPIN_MUX_DirectionOutput   /*!< Direction */


/*!
 * @brief Configures pin routing and optionally pin electrical features.
 *
 */
void init_led_pins(void);

/* PORTD3 (number 94), PTD3 */
#define DYNAMIXEL_TX_PERIPHERAL                                          LPUART2   /*!< Device name: LPUART2 */
#define DYNAMIXEL_TX_SIGNAL                                                   TX   /*!< LPUART2 signal: TX */
#define DYNAMIXEL_TX_PIN_NAME                                         LPUART2_TX   /*!< Pin name */
#define DYNAMIXEL_TX_LABEL                                                "PTD3"   /*!< Label */
#define DYNAMIXEL_TX_NAME                                                   "TX"   /*!< Identifier name */
#define DYNAMIXEL_TX_DIRECTION                          kPIN_MUX_DirectionOutput   /*!< Direction */

/* PORTD2 (number 93), PTD2 */
#define DYNAMIXEL_RX_PERIPHERAL                                          LPUART2   /*!< Device name: LPUART2 */
#define DYNAMIXEL_RX_SIGNAL                                                   RX   /*!< LPUART2 signal: RX */
#define DYNAMIXEL_RX_PIN_NAME                                         LPUART2_RX   /*!< Pin name */
#define DYNAMIXEL_RX_LABEL                                                "PTD2"   /*!< Label */
#define DYNAMIXEL_RX_NAME                                                   "RX"   /*!< Identifier name */

/* PORTD4 (number 95), J2[6]/D10/SPI0_PCS1 */
#define DYNAMIXEL_DIR_GPIO                                                 GPIOD   /*!< GPIO device name: GPIOD */
#define DYNAMIXEL_DIR_PORT                                                 PORTD   /*!< PORT device name: PORTD */
#define DYNAMIXEL_DIR_GPIO_PIN                                                4U   /*!< PORTD pin index: 4 */
#define DYNAMIXEL_DIR_PIN_NAME                                              PTD4   /*!< Pin name */
#define DYNAMIXEL_DIR_LABEL                                "J2[6]/D10/SPI0_PCS1"   /*!< Label */
#define DYNAMIXEL_DIR_NAME                                                 "DIR"   /*!< Identifier name */
#define DYNAMIXEL_DIR_DIRECTION                         kPIN_MUX_DirectionOutput   /*!< Direction */


/*!
 * @brief Configures pin routing and optionally pin electrical features.
 *
 */
void init_dynamixel_uart(void);

/* PORTA1 (number 37), J2[18]/J12[4]/U6[6]/D14/I2C SDA */
#define ESP_I2C_SDA_PERIPHERAL                                           LPUART0   /*!< Device name: LPUART0 */
#define ESP_I2C_SDA_SIGNAL                                                    RX   /*!< LPUART0 signal: RX */
#define ESP_I2C_SDA_PIN_NAME                                          LPUART0_RX   /*!< Pin name */
#define ESP_I2C_SDA_LABEL                      "J2[18]/J12[4]/U6[6]/D14/I2C SDA"   /*!< Label */
#define ESP_I2C_SDA_NAME                                               "I2C_SDA"   /*!< Identifier name */

/* PORTA2 (number 38), J2[20]J12[3]/J19[6]/U6[4]/I2C_SCL */
#define ESP_I2C_SCL_PERIPHERAL                                           LPUART0   /*!< Device name: LPUART0 */
#define ESP_I2C_SCL_SIGNAL                                                    TX   /*!< LPUART0 signal: TX */
#define ESP_I2C_SCL_PIN_NAME                                          LPUART0_TX   /*!< Pin name */
#define ESP_I2C_SCL_LABEL                    "J2[20]J12[3]/J19[6]/U6[4]/I2C_SCL"   /*!< Label */
#define ESP_I2C_SCL_NAME                                               "I2C_SCL"   /*!< Identifier name */
#define ESP_I2C_SCL_DIRECTION                           kPIN_MUX_DirectionOutput   /*!< Direction */


/*!
 * @brief Configures pin routing and optionally pin electrical features.
 *
 */
void init_esp_uart(void);

#if defined(__cplusplus)
}
#endif

/*!
 * @}
 */
#endif /* _PIN_MUX_H_ */

/*******************************************************************************
 * EOF
 ******************************************************************************/
