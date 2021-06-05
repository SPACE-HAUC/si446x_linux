/**
 * @file si446x.c
 * @author Sunip K. Mukherjee (sunipkmukherjee@gmail.com)
 * @brief 
 * @version 0.1
 * @date 2021-06-01
 * 
 * @copyright Copyright (c) 2021
 * 
 */

#define _GNU_SOURCE

#include <spibus/spibus.h>
#include <gpiodev/gpiodev.h>

static spibus si446x_spi[1];

#include <string.h>
#include <stdint.h>
#include <pthread.h>
#include <unistd.h>
#include <stdbool.h>
#include <stdlib.h>
#include <time.h>
#include <sys/time.h>

#ifndef eprintf
#define eprintf(str, ...)                                                        \
	{                                                                            \
		fprintf(stderr, "%s, %d: " str "\n", __func__, __LINE__, ##__VA_ARGS__); \
		fflush(stderr);                                                          \
	}
#endif

int get_diff(struct timespec *tm, int tout_ms)
{
	if (tout_ms < 0)
		return -1;
	if (clock_gettime(CLOCK_REALTIME, tm) < 0)
		return -1;
	time_t sec = (tout_ms / 1000) + tm->tv_sec;
	long nsec = (tout_ms % 1000) * 1000000L + tm->tv_nsec;
	tm->tv_sec = sec + (nsec / 1000000000L);
	tm->tv_nsec = nsec % 1000000000L;
	return 1;
}

static pthread_mutex_t si446x_spi_access[1] = {PTHREAD_MUTEX_INITIALIZER};

#include "si446x.h"
#include "si446x_config.h"
#include "si446x_defs.h"

#include "radio_config.h"

#include "ringbuf.h"

#define IDLE_STATE SI446X_IDLE_MODE

// When FIFOs are combined it becomes a 129 byte FiFO
// The first byte is used for length, then the remaining 128 bytes for the packet data
#define MAX_PACKET_LEN SI446X_MAX_PACKET_LEN

#define IRQ_PACKET 0
#define IRQ_MODEM 1
#define IRQ_CHIP 2

#define rssi_dBm(val) ((val / 2) - 134)

#define delay_ms(ms) usleep(ms * 1000)
#define delay_us(us) usleep(us)

#define spiSelect() (gpioWrite(SI446X_CSN, GPIO_LOW))
#define spiDeselect() (gpioWrite(SI446X_CSN, GPIO_HIGH))

static inline void spi_transfer_nr(uint8_t data)
{
	spibus_xfer(si446x_spi, &data, sizeof(uint8_t));
}

static inline uint8_t spi_transfer(uint8_t data)
{
	uint8_t out;
	spibus_xfer_full(si446x_spi, &out, sizeof(uint8_t), &data, sizeof(uint8_t));
	return out;
}

static const uint8_t config[] = RADIO_CONFIGURATION_DATA_ARRAY;

static int SI446X_READ_TOUT = 10000; // 10 second timeout by default

static volatile uint8_t enabledInterrupts[3];

// http://stackoverflow.com/questions/10802324/aliasing-a-function-on-a-c-interface-within-a-c-application-on-linux
#if defined(__cplusplus)
extern "C"
{
#endif
	static void __empty_callback0(void)
	{
	}
	static void __empty_callback1(int16_t param1) { (void)(param1); }
#if defined(__cplusplus)
}
#endif

void __attribute__((weak, alias("__empty_callback0"))) SI446X_CB_CMDTIMEOUT(void);
void __attribute__((weak, alias("__empty_callback1"))) SI446X_CB_RXBEGIN(int16_t rssi);
void __attribute__((weak)) SI446X_CB_RXCOMPLETE(uint8_t length, int16_t rssi)
{
	(void)(length);
	(void)(rssi);
}
void __attribute__((weak, alias("__empty_callback1"))) SI446X_CB_RXINVALID(int16_t rssi);
void __attribute__((weak, alias("__empty_callback0"))) SI446X_CB_SENT(void);
void __attribute__((weak, alias("__empty_callback0"))) SI446X_CB_WUT(void);
void __attribute__((weak, alias("__empty_callback0"))) SI446X_CB_LOWBATT(void);

static inline uint8_t cselect(void)
{
	spiSelect();
	return 1;
}

static inline uint8_t cdeselect(void)
{
	spiDeselect();
	return 0;
}

#define CHIPSELECT() for (uint8_t _cs = cselect(); _cs; _cs = cdeselect())

#define SI446X_NO_INTERRUPT() for (uint8_t __unused_var = 1; __unused_var; __unused_var = 0)

static int interrupt_on()
{
	pthread_mutex_lock(si446x_spi_access);
	return 1;
}

static void interrupt_off(int *in)
{
	(void)in;
	pthread_mutex_unlock(si446x_spi_access);
}

#define SI446X_ATOMIC() for (int _cs2 = interrupt_on(), _var_clean __attribute__((__cleanup__(interrupt_off))) = 1; _cs2; _cs2 = 0)

// Read CTS and if its ok then read the command buffer
static uint8_t getResponse(void *buff, uint8_t len)
{
	uint8_t cts = 0;

	SI446X_ATOMIC()
	{
		CHIPSELECT()
		{
			// Send command
			spi_transfer_nr(SI446X_CMD_READ_CMD_BUFF);

			// Get CTS value
			cts = (spi_transfer(0xFF) == 0xFF);

			if (cts)
			{
				// Get response data
				for (uint8_t i = 0; i < len; i++)
					((uint8_t *)buff)[i] = spi_transfer(0xFF);
			}
		}
	}
	return cts;
}

// Keep trying to read the command buffer, with timeout of around 500ms
static uint8_t waitForResponse(void *out, uint8_t outLen, uint8_t useTimeout)
{
	// With F_CPU at 8MHz and SPI at 4MHz each check takes about 7us + 10us delay
	uint16_t timeout = 40000;
	while (!getResponse(out, outLen))
	{
		delay_us(10);
		if (useTimeout && !--timeout)
		{
			SI446X_CB_CMDTIMEOUT();
			return 0;
		}
	}
	return 1;
}

static void doAPI(void *data, uint8_t len, void *out, uint8_t outLen)
{
	SI446X_NO_INTERRUPT()
	{
		if (waitForResponse(NULL, 0, 1)) // Make sure it's ok to send a command
		{
			SI446X_ATOMIC()
			{
				CHIPSELECT()
				{
					for (uint8_t i = 0; i < len; i++)
						spi_transfer_nr(((uint8_t *)data)[i]); // (pgm_read_byte(&((uint8_t*)data)[i]));
				}
			}

			if (((uint8_t *)data)[0] == SI446X_CMD_IRCAL) // If we're doing an IRCAL then wait for its completion without a timeout since it can sometimes take a few seconds
				waitForResponse(NULL, 0, 0);
			else if (out != NULL) // If we have an output buffer then read command response into it
				waitForResponse(out, outLen, 1);
		}
	}
}

#if !DOXYGEN
/**
* @brief Enable or disable callbacks. This is mainly to configure what events should wake the microcontroller up.
*
* @param [callbacks] The callbacks to configure (multiple callbacks should be bitewise OR'd together)
* @param [state] Enable or disable the callbacks passed in \p callbacks parameter (1 = Enable, 0 = Disable)
* @return (none)
*/
void si446x_setupCallback(uint16_t callbacks, uint8_t state);
#endif

// Configure a bunch of properties (up to 12 properties in one go)
static void setProperties(uint16_t prop, void *values, uint8_t len)
{
	// len must not be greater than 12

	uint8_t data[16] = {
		SI446X_CMD_SET_PROPERTY,
		(uint8_t)(prop >> 8),
		len,
		(uint8_t)prop};

	// Copy values into data, starting at index 4
	memcpy(data + 4, values, len);

	doAPI(data, len + 4, NULL, 0);
}

// Set a single property
static inline void setProperty(uint16_t prop, uint8_t value)
{
	setProperties(prop, &value, 1);
}

// Read a bunch of properties
static void getProperties(uint16_t prop, void *values, uint8_t len)
{
	uint8_t data[] = {
		SI446X_CMD_GET_PROPERTY,
		(uint8_t)(prop >> 8),
		len,
		(uint8_t)prop};

	doAPI(data, sizeof(data), values, len);
}

// Read a single property
static inline uint8_t getProperty(uint16_t prop)
{
	uint8_t val;
	getProperties(prop, &val, 1);
	return val;
}

// Do an ADC conversion
static uint16_t getADC(uint8_t adc_en, uint8_t adc_cfg, uint8_t part)
{
	uint8_t data[6] = {
		SI446X_CMD_GET_ADC_READING,
		adc_en,
		adc_cfg};
	doAPI(data, 3, data, 6);
	return (data[part] << 8 | data[part + 1]);
}

// Read a fast response register
static uint8_t getFRR(uint8_t reg)
{
	uint8_t frr = 0;
	SI446X_ATOMIC()
	{
		CHIPSELECT()
		{
			spi_transfer_nr(reg);
			frr = spi_transfer(0xFF);
		}
	}
	return frr;
}

// Ge the patched RSSI from the beginning of the packet
static int16_t getLatchedRSSI(void)
{
	uint8_t frr = getFRR(SI446X_CMD_READ_FRR_A);
	int16_t rssi = rssi_dBm(frr);
	return rssi;
}

// Get current radio state
static si446x_state_t getState(void)
{
	uint8_t state = getFRR(SI446X_CMD_READ_FRR_B);
	if (state == SI446X_STATE_TX_TUNE)
		state = SI446X_STATE_TX;
	else if (state == SI446X_STATE_RX_TUNE)
		state = SI446X_STATE_RX;
	else if (state == SI446X_STATE_READY2)
		state = SI446X_STATE_READY;
	return (si446x_state_t)state;
}

// Set new state
static void setState(si446x_state_t newState)
{
	uint8_t data[] = {
		SI446X_CMD_CHANGE_STATE,
		newState};
	doAPI(data, sizeof(data), NULL, 0);
}

// Clear RX and TX FIFOs
static void clearFIFO(void)
{
	// 'static const' saves 20 bytes of flash here, but uses 2 bytes of RAM
	static const uint8_t clearFifo[] = {
		SI446X_CMD_FIFO_INFO,
		SI446X_FIFO_CLEAR_RX | SI446X_FIFO_CLEAR_TX};
	doAPI((uint8_t *)clearFifo, sizeof(clearFifo), NULL, 0);
}

static void interrupt(void *buff)
{
	uint8_t data = SI446X_CMD_GET_INT_STATUS;
	doAPI(&data, sizeof(data), buff, 8);
}

// Similar to interrupt() but with the option of not clearing certain interrupt flags
static void interrupt2(void *buff, uint8_t clearPH, uint8_t clearMODEM, uint8_t clearCHIP)
{
	uint8_t data[] = {
		SI446X_CMD_GET_INT_STATUS,
		clearPH,
		clearMODEM,
		clearCHIP};
	doAPI(data, sizeof(data), buff, 8);
}

// Reset the RF chip
static void resetDevice(void)
{
	gpioWrite(SI446X_SDN, GPIO_HIGH);
	delay_ms(50);
	gpioWrite(SI446X_SDN, GPIO_LOW);
	delay_ms(50);
}

// Apply the radio configuration
static void applyStartupConfig(void)
{
	uint8_t buff[17];
	for (uint16_t i = 0; i < sizeof(config); i++)
	{
		memcpy(buff, &config[i], sizeof(buff));
		doAPI(&buff[1], buff[0], NULL, 0);
		i += buff[0];
	}
}

static void si446x_destroy(void)
{
	gpioUnregisterIRQ(SI446X_IRQ);
	si446x_sleep();
	spibus_destroy(si446x_spi);
}

typedef struct
{
	ringbuf_t rbuf;
	pthread_mutex_t lock[1];
	pthread_mutex_t avail_m[1];
	pthread_cond_t avail[1];
	int16_t rssi;
} c_ringbuf;

static void si446x_receive(void *_data)
{
	c_ringbuf *data = (c_ringbuf *)_data;
	bool read_rx_fifo = false;
	bool read_rssi = false;
	int16_t _rssi = 0;
	uint8_t len = 0;
	while (gpioRead(SI446X_IRQ) == GPIO_LOW)
	{
		read_rssi = false;
		// else, we have an interrupt to process
		uint8_t interrupts[8];
		interrupt(interrupts); // read in IRQ vectors
		// We could read the enabled interrupts properties instead of keep their states in RAM, but that would be much slower
		interrupts[2] &= enabledInterrupts[IRQ_PACKET];
		interrupts[4] &= enabledInterrupts[IRQ_MODEM];
		interrupts[6] &= enabledInterrupts[IRQ_CHIP];

		// Valid PREAMBLE and SYNC, packet data now begins
		if (interrupts[4] & (1 << SI446X_SYNC_DETECT_PEND))
		{
			//fix_invalidSync_irq(1);
			//		si446x_setupCallback(SI446X_CBS_INVALIDSYNC, 1); // Enable INVALID_SYNC when a new packet starts, sometimes a corrupted packet will mess the radio up
			if (!read_rssi)
				_rssi = getLatchedRSSI();
			(data->rssi) = _rssi;
			// eprintf("Sync detect: RSSI %d", rssi);
			SI446X_CB_RXBEGIN(_rssi);
		}

		// Valid packet
		if (interrupts[2] & (1 << SI446X_PACKET_RX_PEND))
		{
			len = 0;
			SI446X_ATOMIC()
			{
				CHIPSELECT()
				{
					spi_transfer_nr(SI446X_CMD_READ_RX_FIFO);
					len = spi_transfer(0xFF); // read 1 byte
				}
			}
			setState(SI446X_STATE_RX);
			if (!read_rssi)
				_rssi = getLatchedRSSI();
			// eprintf("RX packet pending: RSSI %d, length: %u", rssi, len);
			SI446X_CB_RXCOMPLETE(len, _rssi);
			(data->rssi) = _rssi;
			if (len != 0xff)
				read_rx_fifo = true;
		}
		// Corrupted packet
		// NOTE: This will still be called even if the address did not match, but the packet failed the CRC
		// This will not be called if the address missed, but the packet passed CRC
		if (interrupts[2] & (1 << SI446X_CRC_ERROR_PEND))
		{
#if IDLE_STATE == SI446X_STATE_READY
			if (getState() == SI446X_STATE_SPI_ACTIVE)
				setState(IDLE_STATE); // We're in sleep mode (acually, we're now in SPI active mode) after an invalid packet to fix the INVALID_SYNC issue
#endif
			SI446X_CB_RXINVALID(getLatchedRSSI()); // TODO remove RSSI stuff for invalid packets, entering SLEEP mode looses the latched value?
			eprintf("CRC invalid");
		}

		// Packet sent
		if (interrupts[2] & (1 << SI446X_PACKET_SENT_PEND))
			SI446X_CB_SENT();

		if (interrupts[6] & (1 << SI446X_LOW_BATT_PEND))
			SI446X_CB_LOWBATT();

		if (interrupts[6] & (1 << SI446X_WUT_PEND))
			SI446X_CB_WUT();

		if (read_rx_fifo)
		{
			uint8_t buff[MAX_PACKET_LEN];
			memset(buff, 0x0, MAX_PACKET_LEN);
			SI446X_ATOMIC()
			{
				CHIPSELECT()
				{
					spi_transfer_nr(SI446X_CMD_READ_RX_FIFO);
					for (uint8_t i = 0; i < len; i++)
						((uint8_t *)buff)[i] = spi_transfer(0xFF);
				}
			}
			setState(SI446X_STATE_RX);
			// copy data to buffer
			pthread_mutex_lock(data->lock);
			if (ringbuf_memcpy_into(data->rbuf, buff, len) != NULL)
				eprintf("Buffer head is NULL");
			if (ringbuf_bytes_used(data->rbuf) > 0) // data available
				pthread_cond_signal(data->avail);	// let the read function know
			pthread_mutex_unlock(data->lock);
		}
	}
}

c_ringbuf dbuf[1];

#define BUFFER_MAX_SIZE (MAX_PACKET_LEN * 64)

void si446x_init()
{
	gpioSetMode(SI446X_CSN, GPIO_OUT);
	gpioWrite(SI446X_CSN, GPIO_HIGH); // set CS to high on init
	gpioSetMode(SI446X_SDN, GPIO_OUT);
	gpioSetMode(SI446X_IRQ, GPIO_IRQ_FALL);
	gpioSetPullUpDown(SI446X_IRQ, GPIO_PUD_UP); // added pull up on pin

	si446x_spi->bus = 0;
	si446x_spi->cs = 0;
	si446x_spi->mode = SPI_MODE_0;
	si446x_spi->lsb = 0;
	si446x_spi->bits = 8;
	si446x_spi->speed = 4000000;
	si446x_spi->cs_internal = 1;
	si446x_spi->sleeplen = 0;
	si446x_spi->internal_rotation = false;

	if (spibus_init(si446x_spi) < 0)
	{
		eprintf("Error initializing SPI");
		exit(0);
	}

	resetDevice();
	applyStartupConfig();
	interrupt(NULL);
	si446x_sleep();

	enabledInterrupts[IRQ_PACKET] = (1 << SI446X_PACKET_RX_PEND) | (1 << SI446X_CRC_ERROR_PEND);
	//enabledInterrupts[IRQ_MODEM] = (1<<SI446X_SYNC_DETECT_PEND);
	// set up receive interrupt callback
	memset(dbuf, 0x0, sizeof(c_ringbuf));
	dbuf->rbuf = ringbuf_new(BUFFER_MAX_SIZE);
	if (dbuf->rbuf == NULL)
	{
		eprintf("Could not allocate memory for buffer");
		exit(-1);
	}
	pthread_mutex_init(dbuf->lock, NULL);
	pthread_mutex_init(dbuf->avail_m, NULL);
	if (gpioRegisterIRQ(SI446X_IRQ, GPIO_IRQ_FALL, &si446x_receive, dbuf, SI446X_TOUT) <= 0)
	{
		eprintf("Could not set up receiver interrupt");
		exit(-2);
	}
	si446x_setupCallback(SI446X_CBS_RXBEGIN, 1); // enable receive interrupt
	atexit(si446x_destroy);
}

void si446x_getInfo(si446x_info_t *info)
{
	uint8_t data[8] = {
		SI446X_CMD_PART_INFO};
	doAPI(data, 1, data, 8);

	info->chipRev = data[0];
	info->part = (data[1] << 8) | data[2];
	info->partBuild = data[3];
	info->id = (data[4] << 8) | data[5];
	info->customer = data[6];
	info->romId = data[7];

	data[0] = SI446X_CMD_FUNC_INFO;
	doAPI(data, 1, data, 6);

	info->revExternal = data[0];
	info->revBranch = data[1];
	info->revInternal = data[2];
	info->patch = (data[3] << 8) | data[4];
	info->func = data[5];
}

int16_t si446x_getRSSI()
{
	uint8_t data[3] = {
		SI446X_CMD_GET_MODEM_STATUS,
		0xFF};
	doAPI(data, 2, data, 3);
	int16_t rssi = rssi_dBm(data[2]);
	return rssi;
}

si446x_state_t si446x_getState()
{
	// TODO what about the state change delay with transmitting?
	return getState();
}

void si446x_setTxPower(uint8_t pwr)
{
	setProperty(SI446X_PA_PWR_LVL, pwr);
}

void si446x_setLowBatt(uint16_t voltage)
{
	// voltage should be between 1500 and 3050
	uint8_t batt = (voltage / 50) - 30; //((voltage * 2) - 3000) / 100;
	setProperty(SI446X_GLOBAL_LOW_BATT_THRESH, batt);
}

void si446x_setupWUT(uint8_t r, uint16_t m, uint8_t ldc, uint8_t config)
{
	// Maximum value of r is 20

	// The API docs say that if r or m are 0, then they will have the same effect as if they were 1, but this doesn't seem to be the case?

	// Check valid config
	// TODO needed?
	if (!(config & (SI446X_WUT_RUN | SI446X_WUT_BATT | SI446X_WUT_RX)))
		return;

	SI446X_NO_INTERRUPT()
	{
		// Disable WUT
		setProperty(SI446X_GLOBAL_WUT_CONFIG, 0);

		uint8_t doRun = !!(config & SI446X_WUT_RUN);
		uint8_t doBatt = !!(config & SI446X_WUT_BATT);
		uint8_t doRx = (config & SI446X_WUT_RX);

		// Setup WUT interrupts
		uint8_t intChip = 0; //getProperty(SI446X_INT_CTL_CHIP_ENABLE); // No other CHIP interrupts are enabled so dont bother reading the current state
		//intChip &= ~((1<<SI446X_INT_CTL_CHIP_LOW_BATT_EN)|(1<<SI446X_INT_CTL_CHIP_WUT_EN));
		intChip |= doBatt << SI446X_INT_CTL_CHIP_LOW_BATT_EN;
		intChip |= doRun << SI446X_INT_CTL_CHIP_WUT_EN;
		enabledInterrupts[IRQ_CHIP] = intChip;
		setProperty(SI446X_INT_CTL_CHIP_ENABLE, intChip);

		// Set WUT clock source to internal 32KHz RC
		if (getProperty(SI446X_GLOBAL_CLK_CFG) != SI446X_DIVIDED_CLK_32K_SEL_RC)
		{
			setProperty(SI446X_GLOBAL_CLK_CFG, SI446X_DIVIDED_CLK_32K_SEL_RC);
			delay_us(300); // Need to wait 300us for clock source to stabilize, see GLOBAL_WUT_CONFIG:WUT_EN info
		}

		// Setup WUT
		uint8_t properties[5];
		properties[0] = doRx ? SI446X_GLOBAL_WUT_CONFIG_WUT_LDC_EN_RX : 0;
		properties[0] |= doBatt << SI446X_GLOBAL_WUT_CONFIG_WUT_LBD_EN;
		properties[0] |= (1 << SI446X_GLOBAL_WUT_CONFIG_WUT_EN);
		properties[1] = m >> 8;
		properties[2] = m;
		properties[3] = r | SI446X_LDC_MAX_PERIODS_TWO | (1 << SI446X_WUT_SLEEP);
		properties[4] = ldc;
		setProperties(SI446X_GLOBAL_WUT_CONFIG, properties, sizeof(properties));
	}
}

void si446x_disableWUT()
{
	SI446X_NO_INTERRUPT()
	{
		setProperty(SI446X_GLOBAL_WUT_CONFIG, 0);
		setProperty(SI446X_GLOBAL_CLK_CFG, 0);
	}
}

// TODO
// ADDRESS MATCH (only useful with address mode on)
// ADDRESS MISS (only useful with address mode on)
// PACKET SENT
// PACKET RX (must never be turned off, otherwise RX mode would be pointless)
// PACKET RX INVALID (must never be turned off, otherwise RX mode would be pointless)
// PACKET BEGIN (SYNC, modem)
// WUT and LOWBATT (cant turn off/on from here, use wutSetup instead)
// INVALID SYNC (the fix thing)
void si446x_setupCallback(uint16_t callbacks, uint8_t state)
{
	SI446X_NO_INTERRUPT()
	{
		uint8_t data[2];
		getProperties(SI446X_INT_CTL_PH_ENABLE, data, sizeof(data));

		if (state)
		{
			data[0] |= callbacks >> 8;
			data[1] |= callbacks;
		}
		else
		{
			data[0] &= ~(callbacks >> 8);
			data[1] &= ~callbacks;
		}

		// TODO
		// make sure RXCOMPELTE, RXINVALID and RXBEGIN? are always enabled

		enabledInterrupts[IRQ_PACKET] = data[0];
		enabledInterrupts[IRQ_MODEM] = data[1];
		setProperties(SI446X_INT_CTL_PH_ENABLE, data, sizeof(data));
	}
	/*
	// TODO remove
	uint8_t data[4];
	data[0] = 0xFF;
	data[1] = 0b11111100;
	data[2] = 0b11111011;
	data[3] = 0xff;
	setProperties(SI446X_INT_CTL_ENABLE, data, sizeof(data));
*/
}

uint8_t si446x_sleep()
{
	if (getState() == SI446X_STATE_TX)
		return 0;
	setState(SI446X_STATE_SLEEP);
	return 1;
}

int si446x_read(void *buff, ssize_t maxlen, int16_t *rssi)
{
	bool empty = false;
	pthread_mutex_lock(dbuf->lock);
	empty = ringbuf_is_empty(dbuf->rbuf);
	pthread_mutex_unlock(dbuf->lock);
	if (!empty) // buffer not empty, can read now
	{
		goto read;
	}
	else
	{
		struct timespec tm;
		if (get_diff(&tm, SI446X_READ_TOUT) < 0)
			return -1; // error
		if (!pthread_cond_timedwait(dbuf->avail, dbuf->avail_m, &tm))
			return 0; // time out
		goto read;
	}
read:
	pthread_mutex_lock(dbuf->lock);
	*rssi = dbuf->rssi;
	ssize_t avail_sz = ringbuf_bytes_used(dbuf->rbuf);
	if (avail_sz < maxlen)
		maxlen = avail_sz; // we read only as much as we can
	if (ringbuf_memcpy_from(buff, dbuf->rbuf, maxlen) == NULL)
	{
		eprintf("Read 0 bytes");
		maxlen = 0;
	}
	pthread_mutex_unlock(dbuf->lock);
	return maxlen;
}

static int Si446x_TX(void *packet, uint8_t len, uint8_t channel, si446x_state_t onTxFinish)
{
	// TODO what happens if len is 0?

#if SI446X_FIXED_LENGTH
	// Stop the unused parameter warning
	((void)(len));
#endif

	SI446X_NO_INTERRUPT()
	{
		if (getState() == SI446X_STATE_TX) // Already transmitting
			return 0;					   // error, already transmitting

		// TODO collision avoid or maybe just do collision detect (RSSI jump)

		setState(IDLE_STATE);
		clearFIFO();
		interrupt2(NULL, 0, 0, 0xFF);

		SI446X_ATOMIC()
		{
			// Load data to FIFO
			CHIPSELECT()
			{
				spi_transfer_nr(SI446X_CMD_WRITE_TX_FIFO);
#if !SI446X_FIXED_LENGTH
				spi_transfer_nr(len);
				for (uint8_t i = 0; i < len; i++)
					spi_transfer_nr(((uint8_t *)packet)[i]);
#else
				for (uint8_t i = 0; i < SI446X_FIXED_LENGTH; i++)
					spi_transfer_nr(((uint8_t *)packet)[i]);
#endif
			}
		}

#if !SI446X_FIXED_LENGTH
		// Set packet length
		setProperty(SI446X_PKT_FIELD_2_LENGTH_LOW, len);
#endif

		// Begin transmit
		uint8_t data[] = {
			SI446X_CMD_START_TX,
			channel,
			(uint8_t)(onTxFinish << 4),
			0,
			SI446X_FIXED_LENGTH,
			0,
			0};
		doAPI(data, sizeof(data), NULL, 0);

#if !SI446X_FIXED_LENGTH
		// Reset packet length back to max for receive mode
		setProperty(SI446X_PKT_FIELD_2_LENGTH_LOW, MAX_PACKET_LEN);
#endif
	}
	return 1;
}

int si446x_write(void *buff, uint8_t len)
{
	if (len > MAX_PACKET_LEN)
	{
		eprintf("Packet size %u > %u not allowed", len, (uint8_t)MAX_PACKET_LEN);
		return -1;
	}
	return Si446x_TX(buff, len, 0, SI446X_STATE_RX);
}

static void Si446x_RX(uint8_t channel)
{
	SI446X_NO_INTERRUPT()
	{
		setState(IDLE_STATE);
		clearFIFO();
		//fix_invalidSync_irq(0);
		//si446x_setupCallback(SI446X_CBS_INVALIDSYNC, 0);
		//setProperty(SI446X_PKT_FIELD_2_LENGTH_LOW, MAX_PACKET_LEN); // TODO ?
		interrupt2(NULL, 0, 0, 0xFF); // TODO needed?

		// TODO RX timeout to sleep if WUT LDC enabled

		uint8_t data[] = {
			SI446X_CMD_START_RX,
			channel,
			0,
			0,
			SI446X_FIXED_LENGTH,
			SI446X_STATE_NOCHANGE, // RX Timeout
			IDLE_STATE,			   // RX Valid
			SI446X_STATE_SLEEP	   // IDLE_STATE // RX Invalid (using SI446X_STATE_SLEEP for the INVALID_SYNC fix)
		};
		doAPI(data, sizeof(data), NULL, 0);
	}
}

uint16_t si446x_adc_gpio(uint8_t pin)
{
	uint16_t result = getADC(SI446X_ADC_CONV_GPIO | pin, (SI446X_ADC_SPEED << 4) | SI446X_ADC_RANGE_3P6, 0);
	return result;
}

uint16_t si446x_adc_battery()
{
	uint16_t result = getADC(SI446X_ADC_CONV_BATT, (SI446X_ADC_SPEED << 4), 2);
	result = ((uint32_t)result * 75) / 32; // result * 2.34375;
	return result;
}

float si446x_adc_temperature()
{
	float result = getADC(SI446X_ADC_CONV_TEMP, (SI446X_ADC_SPEED << 4), 4);
	result = (899 / 4096.0) * result - 293;
	return result;
}

void si446x_writeGPIO(si446x_gpio_t pin, uint8_t value)
{
	uint8_t data[] = {
		SI446X_CMD_GPIO_PIN_CFG,
		SI446X_GPIO_MODE_DONOTHING,
		SI446X_GPIO_MODE_DONOTHING,
		SI446X_GPIO_MODE_DONOTHING,
		SI446X_GPIO_MODE_DONOTHING,
		SI446X_NIRQ_MODE_DONOTHING,
		SI446X_SDO_MODE_DONOTHING,
		SI446X_GPIO_DRV_HIGH};
	data[pin + 1] = value;
	doAPI(data, sizeof(data), NULL, 0);
}

uint8_t si446x_readGPIO()
{
	uint8_t data[4] = {
		SI446X_CMD_GPIO_PIN_CFG};
	doAPI(data, 1, data, sizeof(data));
	uint8_t states = data[0] >> 7 | (data[1] & 0x80) >> 6 | (data[2] & 0x80) >> 5 | (data[3] & 0x80) >> 4;
	return states;
}

uint8_t si446x_dump(void *buff, uint8_t group)
{
	static const uint8_t groupSizes[] = {
		SI446X_PROP_GROUP_GLOBAL, 0x0A,
		SI446X_PROP_GROUP_INT, 0x04,
		SI446X_PROP_GROUP_FRR, 0x04,
		SI446X_PROP_GROUP_PREAMBLE, 0x0E,
		SI446X_PROP_GROUP_SYNC, 0x06,
		SI446X_PROP_GROUP_PKT, 0x40,
		SI446X_PROP_GROUP_MODEM, 0x60,
		SI446X_PROP_GROUP_MODEM_CHFLT, 0x24,
		SI446X_PROP_GROUP_PA, 0x07,
		SI446X_PROP_GROUP_SYNTH, 0x08,
		SI446X_PROP_GROUP_MATCH, 0x0C,
		SI446X_PROP_GROUP_FREQ_CONTROL, 0x08,
		SI446X_PROP_GROUP_RX_HOP, 0x42,
		SI446X_PROP_GROUP_PTI, 0x04};

	uint8_t length = 0;
	for (uint8_t i = 0; i < sizeof(groupSizes); i += 2)
	{
		uint8_t buff[2];
		memcpy(buff, &groupSizes[i], sizeof(buff));

		if (buff[0] == group)
		{
			length = buff[1];
			break;
		}
	}

	if (buff == NULL)
		return length;

	for (uint8_t i = 0; i < length; i += 16)
	{
		uint8_t count = length - i;
		if (count > 16)
			count = 16;
		getProperties((group << 8) | i, ((uint8_t *)buff) + i, count);
	}

	return length;
}

int si446x_get_read_irq_tout(void)
{
	return SI446X_READ_TOUT;
}

int si446x_set_read_irq_tout(int tout)
{
	SI446X_READ_TOUT = tout;
	return SI446X_READ_TOUT;
}