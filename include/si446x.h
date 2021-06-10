/**
 * @file si446x.h
 * @author Sunip K. Mukherjee (sunipkmukherjee@gmail.com)
 * @brief 
 * @version 2.0
 * @date 2021-06-04
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#ifndef _SI443X_H_
#define _SI443X_H_
#include <stdio.h>
#include <stdint.h>

#ifndef eprintf
#define eprintf(str, ...)                                                        \
    {                                                                            \
        fprintf(stderr, "%s, %d: " str "\n", __func__, __LINE__, ##__VA_ARGS__); \
        fflush(stderr);                                                          \
    }
#endif

#define SI446X_MAX_PACKET_LEN 128 ///< Maximum packet length

#define SI446X_MAX_TX_POWER 127 ///< Maximum TX power (+20dBm/100mW)

// Raspberry Pi pin assignments
#define SI446X_SDN 13 ///! Shutdown pin
#define SI446X_IRQ 11 ///! Receive interrupt

/**
* @brief Data structure for storing chip info from ::si446x_getInfo()
*/
typedef struct
{
    uint8_t chipRev;   ///< Chip revision
    uint16_t part;     ///< Part ID
    uint8_t partBuild; ///< Part build
    uint16_t id;       ///< ID
    uint8_t customer;  ///< Customer
    uint8_t romId;     ///< ROM ID (3 = revB1B, 6 = revC2A)

    uint8_t revExternal; ///< Revision external
    uint8_t revBranch;   ///< Revision branch
    uint8_t revInternal; ///< Revision internal
    uint16_t patch;      ///< Patch
    uint8_t func;        ///< Function
} si446x_info_t;

/**
* @brief GPIOs for passing to ::si446x_writeGPIO(), or for masking when reading from ::si446x_readGPIO()
*/
typedef enum
{
    SI446X_GPIO0 = 0, ///< GPIO 1
    SI446X_GPIO1 = 1, ///< GPIO 2
    SI446X_GPIO2 = 2, ///< GPIO 3
    SI446X_GPIO3 = 3, ///< GPIO 4
    SI446X_NIRQ = 4,  ///< NIRQ
    SI446X_SDO = 5    ///< SDO
} si446x_gpio_t;

/**
* @brief Radio states, returned from ::si446x_getState()
*/
typedef enum
{
    SI446X_STATE_NOCHANGE = 0x00,
    SI446X_STATE_SLEEP = 0x01, ///< This will never be returned since SPI activity will wake the radio into ::SI446X_STATE_SPI_ACTIVE
    SI446X_STATE_SPI_ACTIVE = 0x02,
    SI446X_STATE_READY = 0x03,
    SI446X_STATE_READY2 = 0x04,  ///< Will return as ::SI446X_STATE_READY
    SI446X_STATE_TX_TUNE = 0x05, ///< Will return as ::SI446X_STATE_TX
    SI446X_STATE_RX_TUNE = 0x06, ///< Will return as ::SI446X_STATE_RX
    SI446X_STATE_TX = 0x07,
    SI446X_STATE_RX = 0x08
} si446x_state_t;

#ifndef DOXYGEN
static char *si446x_state_str[] = {
    "Si446x State: No change",
    "Si446x State: Sleep",
    "Si446x State: SPI active",
    "Si446x State: Ready",
    "Si446x State: Ready",
    "Si446x State: TX tune",
    "Si446x State: RX tune",
    "Si446x State: TX",
    "Si446x State: RX"};
#endif

enum SI446X_RETVAL
{
    SI446X_ERROR = -1,
    SI446X_TOUT = 0,
    SI446X_SUCCESS = 1,
    SI446X_CRC_INVALID = 2,
};

#ifndef DOXYGEN
static char *si446x_errstr[] = {
    "Si446x: Timed out",
    "Si446x: Success",
    "Si446x: CRC invalid on receiver"};
#endif

#ifdef __cplusplus
// extern "C" {
#endif
/**
* @brief Initialise, must be called before anything else!
*
* @return (none)
*/
void si446x_init(void);

/**
* @brief Get chip info, see ::si446x_info_t
*
* @see ::si446x_info_t
* @param [info] Pointer to allocated ::si446x_info_t struct to place data into
* @return (none)
*/
void si446x_getInfo(si446x_info_t *info);

/**
* @brief Get the current RSSI, the chip needs to be in receive mode for this to work
*
* @return The current RSSI in dBm (usually between -130 and 0)
*/
int16_t si446x_getRSSI(void);

/**
* @brief Set the transmit power. The output power does not follow the \p pwr value, see the Si446x datasheet for a pretty graph
*
* 0 = -32dBm (<1uW)\n
* 7 = 0dBm (1mW)\n
* 12 = 5dBm (3.2mW)\n
* 22 = 10dBm (10mW)\n
* 40 = 15dBm (32mW)\n
* 100 = 20dBm (100mW)
*
* @param [pwr] A value from 0 to 127
* @return (none)
*/
void si446x_setTxPower(uint8_t pwr);

/**
* @brief Enable interrupts and wait for RX IRQ/timeout
*
* @param buff Pointer to buffer to place data
* @param len Size of buffer in bytes.
* @param rssi RSSI of the received signal (dBm)
* @return Positive (number of bytes received) on success, negative on error, 0 on timeout
*/
int si446x_read(void *buff, ssize_t len, int16_t *rssi);
/**
 * @brief Transmit data
 * 
 * @param buff Pointer to buffer where data is read from
 * @param len Length of output buffer (<= 128 bytes)
 * @return int Positive on success, negative on error, 0 on ongoing transmission
 */
int si446x_write(void *buff, uint8_t len);

/**
 * @brief Enable PIPE mode on the spacecraft
 * 
 */
static inline void si446x_en_pipe(void)
{
    static char cmd[] = "ES+W22000320\r";
    eprintf("Sending PIPE command");
    si446x_write(cmd, sizeof(cmd) - 1);
    eprintf("Sent PIPE command");
    return;
}

/*-*
* @brief Changes will be applied next time the radio enters RX mode (NOT SUPPORTED)
*
* @param [mode] TODO
* @param [address] TODO
* @return (none)
*/
//void si446x_setAddress(si446x_addrMode_t mode, uint8_t address);

/**
* @brief Set the low battery voltage alarm
*
* The ::SI446X_CB_LOWBATT() callback will be ran when the supply voltage drops below this value. The WUT must be configured with ::si446x_setupWUT() to enable periodically checking the battery level.
*
* @param [voltage] The low battery threshold in millivolts (1050 - 3050).
* @return (none)
*/
void si446x_setLowBatt(uint16_t voltage);

/**
* @brief Configure the wake up timer
* 
* This function will also reset the timer.\n
*\n
* The Wake Up Timer (WUT) can be used to periodically run a number of features:\n
* ::SI446X_WUT_RUN Simply wake up the microcontroller when the WUT expires and run the ::SI446X_CB_WUT() callback.\n
* ::SI446X_WUT_BATT Check battery voltage - If the battery voltage is below the threshold set by ::si446x_setLowBatt() then wake up the microcontroller and run the ::SI446X_CB_LOWBATT() callback.\n
* ::SI446X_WUT_RX Enter receive mode for a length of time determinded by the ldc and r parameters (NOT SUPPORTED YET! dont use this option)\n
*\n
* For more info see the GLOBAL_WUT_M, GLOBAL_WUT_R and GLOBAL_WUT_LDC properties in the Si446x API docs.\n
*
* @note When first turning on the WUT this function will take around 300us to complete
* @param [r] Exponent value for WUT and LDC (Maximum valus is 20)
* @param [m] Mantissia value for WUT
* @param [ldc] Mantissia value for LDC (NOT SUPPORTED YET, just pass 0 for now)
* @param [config] Which WUT features to enable ::SI446X_WUT_RUN ::SI446X_WUT_BATT ::SI446X_WUT_RX These can be bitwise OR'ed together to enable multiple features.
* @return (none)
*/
void si446x_setupWUT(uint8_t r, uint16_t m, uint8_t ldc, uint8_t config);

/**
* @brief Disable the wake up timer
*
* @return (none)
*/
void si446x_disableWUT(void);

/**
* @brief Enter sleep mode
*
* If WUT is enabled then the radio will keep the internal 32KHz RC enabled with a current consumption of 740nA, otherwise the current consumption will be 40nA without WUT.
* Sleep will fail if the radio is currently transmitting.
*
* @note Any SPI communications with the radio will wake the radio into ::SI446X_STATE_SPI_ACTIVE mode. ::si446x_sleep() will need to called again to put it back into sleep mode.
*
* @return 0 on failure (busy transmitting something), 1 on success
*/
uint8_t si446x_sleep(void);

/**
* @brief Get the radio status
*
* @see ::si446x_state_t
* @return The current radio status
*/
si446x_state_t si446x_getState(void);

/**
* @brief Read pin ADC value
*
* @param [pin] The GPIO pin number (0 - 3)
* @return ADC value (0 - 2048, where 2048 is 3.6V)
*/
uint16_t si446x_adc_gpio(uint8_t pin);

/**
* @brief Read supply voltage
*
* @return Supply voltage in millivolts
*/
uint16_t si446x_adc_battery(void);

/**
* @brief Read temperature
*
* @return Temperature in C
*/
float si446x_adc_temperature(void);

/**
* @brief Configure GPIO/NIRQ/SDO pin
*
* @note NIRQ and SDO pins should not be changed, unless you really know what you're doing. 2 of the GPIO pins (usually 0 and 1) are also usually used for the RX/TX RF switch and should also be left alone.
*
* @param [pin] The pin, this can only take a single pin (don't use bitwise OR), see ::si446x_gpio_t
* @param [value] The new pin mode, this can be bitwise OR'd with the ::SI446X_PIN_PULL_EN option, see ::si446x_gpio_mode_t ::si446x_nirq_mode_t ::si446x_sdo_mode_t
* @return (none)
*/
void si446x_writeGPIO(si446x_gpio_t pin, uint8_t value);

/**
* @brief Read GPIO pin states
*
* @return The pin states. Use ::si446x_gpio_t to mask the value to get the state for the desired pin.
*/
uint8_t si446x_readGPIO(void);

/**
* @brief Get all values of a property group
*
* @param [buff] Pointer to memory to place group values, if this is NULL then nothing will be dumped, just the group size is returned
* @param [group] The group to dump
* @return Size of the property group
*/
uint8_t si446x_dump(void *buff, uint8_t group);

/**
 * @brief Get read interrupt timeout (ms)
 * 
 * @return int Timeout in ms
 */
int si446x_get_read_irq_tout(void);
/**
 * @brief Set read interrupt timeout (ms)
 * 
 * @param tout Timeout in ms (-1 to disable poll timeout)
 * @return int Set timeout in ms
 */
int si446x_set_read_irq_tout(int tout);

#ifdef __cplusplus
// }
#endif

#endif // _SI443X_H_