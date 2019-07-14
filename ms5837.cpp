/****************************************************************************
 *
 *   Copyright (c) 2012-2015 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file ms5837.cpp
 * Driver for the MS5837-30 barometric pressure sensor connected via I2C.
 * @author Vera Cheung
 */

#include <px4_config.h>
#include <px4_defines.h>

#include <drivers/device/i2c.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>
#include <stdlib.h>
#include <semaphore.h>
#include <string.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>

#include <nuttx/arch.h>
#include <nuttx/wqueue.h>
#include <nuttx/clock.h>

#include <arch/board/board.h>
#include <board_config.h>

#include <drivers/device/device.h>
#include <drivers/drv_baro.h>
#include <drivers/drv_hrt.h>
#include <drivers/device/ringbuffer.h>

#include <perf/perf_counter.h>
#include <systemlib/err.h>
#include <platforms/px4_getopt.h>

#include "ms5837.h"

#include "board_config.h"

/* Configuration Constants */
#define MS5837_BUS_DEFAULT PX4_I2C_BUS_EXPANSION
#define MS5837_BASEADDR 0x76
#define MS5837_DEVICE_PATH "/dev/ms5837"

/*
 * MS5837 internal constants and data structures.
 */
#define ADDR_CMD_CONVERT_D1_OSR256		0x40	/* write to this address to start pressure conversion */
#define ADDR_CMD_CONVERT_D1_OSR512		0x42	/* write to this address to start pressure conversion */
#define ADDR_CMD_CONVERT_D1_OSR1024		0x44	/* write to this address to start pressure conversion */
#define ADDR_CMD_CONVERT_D1_OSR2048		0x46	/* write to this address to start pressure conversion */
#define ADDR_CMD_CONVERT_D1_OSR4096		0x48	/* write to this address to start pressure conversion */
#define ADDR_CMD_CONVERT_D2_OSR256		0x50	/* write to this address to start temperature conversion */
#define ADDR_CMD_CONVERT_D2_OSR512		0x52	/* write to this address to start temperature conversion */
#define ADDR_CMD_CONVERT_D2_OSR1024		0x54	/* write to this address to start temperature conversion */
#define ADDR_CMD_CONVERT_D2_OSR2048		0x56	/* write to this address to start temperature conversion */
#define ADDR_CMD_CONVERT_D2_OSR4096		0x58	/* write to this address to start temperature conversion */

/* helper macro for handling report buffer indices */
#define INCREMENT(_x, _lim)	do { __typeof__(_x) _tmp = _x+1; if (_tmp >= _lim) _tmp = 0; _x = _tmp; } while(0)

/* helper macro for arithmetic - returns the square of the argument */
#define POW2(_x)		((_x) * (_x))

/*
  use an OSR of 1024 to reduce the self-heating effect of the
  sensor. Information from MS tells us that some individual sensors
  are quite sensitive to this effect and that reducing the OSR can
  make a big difference
  
  change this to effect resolution
 */
#define ADDR_CMD_CONVERT_D1			ADDR_CMD_CONVERT_D1_OSR1024
#define ADDR_CMD_CONVERT_D2			ADDR_CMD_CONVERT_D2_OSR1024

/*
 * Maximum internal conversion time for OSR 1024 is 2.28 ms. We set an update
 * rate of 100Hz which is be very safe not to read the ADC before the
 * conversion finished
 * 
 * data is from MS5611 datasheet, but it should be the same as MS5837.
 */
#define MS5837_CONVERSION_INTERVAL	4000	/* microseconds */
#define MS5837_MEASUREMENT_RATIO	3	/* pressure measurements per temperature measurement */

#ifndef CONFIG_SCHED_WORKQUEUE
# error This requires CONFIG_SCHED_WORKQUEUE.
#endif

class MS5837 : public device::I2C
{
public:
	MS5837(int bus = MS5837_BUS_DEFAULT, int address = MS5837_BASEADDR);
	virtual ~MS5837();
	
	virtual int 		init();

	virtual ssize_t		read(device::file_t *filp, char *buffer, size_t buflen);
	virtual int			ioctl(device::file_t *filp, int cmd, unsigned long arg);
	
	// print some info about the driver
	void				print_info();

protected:
	virtual int			probe();  // this function is necessary for the device.hpp file, do not rename.
	
private:
	ms5837::prom_s 				_prom; // stores values from EEPROM
	struct work_s				_work{};
	ringbuffer::RingBuffer		*_reports;
	unsigned					_measure_ticks;
	bool						_collect_phase;
	unsigned					_measure_phase;
	bool 						_sensor_ok;
	
	/* intermediate temperature values per MS5837 datasheet */
	int32_t			_TEMP;
	int64_t			_OFF;
	int64_t			_SENS;
	int32_t			P_sum;
	int				cnter;
	
	/* set up UORB publication*/
	orb_advert_t		_baro_topic;
	int					_orb_class_instance;
	int					_class_instance;
	
	perf_counter_t		_sample_perf;
	perf_counter_t		_measure_perf;
	perf_counter_t		_comms_errors;
	
	// the following are specific to this driver
	/**
	* Initialise the automatic measurement state machine and start it.
	*
	* @note This function is called at open and error time.  It might make sense
	*       to make it more aggressive about resetting the bus in case of errors.
	*/
	void				start(unsigned delay_ticks = 1);
	
	/**
	* Stop the automatic measurement state machine.
	*/
	void				stop();

	/**
	 * Send a measure command to the MS5837.
	 *
	 */
	int					measure();
	
	/**
	 * Collect the result of the most recent measurement.
	 */
	virtual int			collect();
	
	/**
	 * Perform a poll cycle; collect from the previous measurement
	 * and start a new one.
	 *
	 * This is the heart of the measurement state machine.  This function
	 * alternately starts a measurement, or collects the data from the
	 * previous measurement.
	 *
	 * When the interval between measurements is greater than the minimum
	 * measurement interval, a gap is inserted between collection
	 * and measurement to provide the most recent measurement possible
	 * at the next interval.
	 */
	void				cycle();

	/**
	 * Static trampoline from the workq context; because we don't have a
	 * generic workq wrapper yet.
	 *
	 * @param arg		Instance pointer for the driver that is polling.
	 */
	static void			cycle_trampoline(void *arg);
	
	// the below are I2C interface codes
	/**
	* Test whether the device supported by the driver is present at a
	* specific address.
	*
	* @param address	The I2C bus address to probe.
	* @return			True if the device is present.
	*/
	int					probe_address(uint8_t address);
	
	/**
	 * Send a reset command to the MS5837.
	 *
	 * This is required after any bus reset.
	 */
	int					reset();
	
	/**
	 * Read the MS5837 PROM
	 *
	 * @return		PX4_OK if the PROM reads successfully.
	 */
	int					read_prom();
	
	/**
	 * MS5837 crc4 (cyclic redundancy check) method 
	 * from datasheet for 16 bytes (8 short values)
	 * checks data validiy for MS5837, which has 112-bit prom memory. 
	 * 
	 * @param address - raw prom data. 
	 * @return bool - true or false. 
	 */
	bool crc4(uint16_t *eep_data);

};

/*
 * Driver 'main' command.1
 */
extern "C" __EXPORT int ms5837_main(int argc, char *argv[]);

MS5837::MS5837(int bus, int address) :
	I2C("MS5837", MS5837_DEVICE_PATH, bus, address, 400000),
	_reports(nullptr),
	_measure_ticks(0),
	_collect_phase(false),
	_measure_phase(0),
	_sensor_ok(false),
	_TEMP(0),
	_OFF(0),
	_SENS(0),
	P_sum(0),
	cnter(0),
	_baro_topic(nullptr),
	_orb_class_instance(-1),
	_class_instance(-1),
	_sample_perf(perf_alloc(PC_ELAPSED, "ms5837_read")),
	_measure_perf(perf_alloc(PC_ELAPSED, "ms5837_measure")),
	_comms_errors(perf_alloc(PC_COUNT, "ms5837_com_err"))	
{
}

MS5837::~MS5837()
{
	/* make sure we are truly inactive */
	stop();

	/* free any existing reports */
	if (_reports != nullptr) {
		delete _reports;
	}
	
	if (_class_instance != -1) {
		unregister_class_devname(BARO_BASE_DEVICE_PATH, _class_instance);
	}

	// free perf counters
	perf_free(_sample_perf);
	perf_free(_measure_perf);
	perf_free(_comms_errors);
}

int
MS5837::init()
/* initiate the driver */
{
	int ret;
	
	/* do I2C init and probe first */
	if (I2C::init() != OK) {
		ret = PX4_ERROR;
		return ret;
	}
	
	/* allocate basic report buffers */
	_reports = new ringbuffer::RingBuffer(2, sizeof(sensor_baro_s));
	
	/* set_device_address(MS5837_BASEADDR); 
	 * looks like it's set in function probe_address
	 */
	
	if (_reports == nullptr) {
		PX4_DEBUG("can't get memory for reports");
		ret = -ENOMEM;
		return ret;
	}
	
	_class_instance = register_class_devname(BARO_BASE_DEVICE_PATH);

	/* get a publish handle on the uORB barometer topic */
	struct baro_report brp = {};
	
	/* CBH - that orb prio high might set priority of the sensor */
	_baro_topic = orb_advertise_multi(ORB_ID(sensor_baro), &brp,
					&_orb_class_instance, ORB_PRIO_HIGH);
	
	if(_baro_topic == nullptr) {
		PX4_ERR("failed to create sensor_baro object");
	}
	
	ret = OK;
	
	_sensor_ok = true;
	
	return ret;
}

int
MS5837::probe()
{
	_retries = 10;
	if (PX4_OK == probe_address(MS5837_BASEADDR)) {
		/* disable retries, device gets confused */
		_retries = 0;
		return PX4_OK;
	}
	return -EIO;
}

int
MS5837::probe_address(uint8_t address)
{	
	/* set address to try */
	set_device_address(address);
	
	/* send reset command */
	if (PX4_OK != reset()){
		return -EIO;
	}
	
	/* read PROM */
	if (PX4_OK != read_prom()){
		return -EIO;
	}
	
	return PX4_OK;
}

int 
MS5837::reset()
{
	unsigned old_retrycount = _retries;
	uint8_t cmd = ADDR_RESET_CMD;
	int result;
	
	/* bump retry count */
	_retries = 10;
	result = transfer(&cmd, 1, nullptr, 0);
	_retries = old_retrycount;
	
	return result;
}

int
MS5837::read_prom()
{
	uint8_t prom_word[2];
	uint16_t c[8]; //buffer for storing prom values. 
	
	/* wait for PROM contents to be in device and for it to settle down */
	usleep(4000);
	
	//uint8_t last_val = 0;
	bool all_zero = true;
	
	/* read and convert PROM words, populate the prom struct */
	for (int i = 0; i < 7; i++) {
		uint8_t cmd = ADDR_PROM_SETUP + (i << 1);
		
		if (PX4_OK != transfer(&cmd, 1, &prom_word[0], 2)) {
			break;
		}
		
		/* assembly 16 bit value */
		c[i] = (prom_word[0] << 8) | prom_word[1];
		
		if (c[i] != 0){
			all_zero = false;
		}
	}
	
	/* calculate CRC. If successful, plug those numbers into the _prom struct */
	bool status_crc4 = crc4(&c[0]);

	if (status_crc4 && !all_zero){
		_prom.factory_setup = c[0];
		_prom.c1_pressure_sens = c[1];
		_prom.c2_pressure_offset = c[2];
		_prom.c3_temp_coeff_pres_sens = c[3];
		_prom.c4_temp_coeff_pres_offset = c[4];
		_prom.c5_reference_temp = c[5];
		_prom.c6_temp_coeff_temp = c[6];
		_prom.serial_and_crc = c[7];
		return PX4_OK;
		
	} else {
		return -EIO;
		
	}		
}

bool 
MS5837::crc4(uint16_t *eep_data)
{
	uint8_t cnt;
	uint16_t n_rem;
	uint8_t n_bit;
	
	n_rem = 0x00;
	
	eep_data[7] = 0;
	
	/* save the read crc */
	const uint16_t crc_read = (eep_data[0] & 0xf000) >> 12;
	
	/* remove the CRC byte */
	eep_data[0] &= ~0xf000;
	
	for  (cnt = 0; cnt < 16; cnt++) {
		/* uneven bytes */
		if (cnt & 1) {
			n_rem ^= (uint8_t)((eep_data[cnt >> 1]) & 0x00FF);
		} else {
			n_rem ^= (uint8_t)(eep_data[cnt >> 1] >> 8);
		}
		
		for (n_bit = 8; n_bit > 0; n_bit--) {
			if (n_rem & 0x8000) {
				n_rem = (n_rem << 1) ^ 0x3000;
			} else {
				n_rem = (n_rem << 1);
			}
		}
	}

	uint16_t comp_nrem = (n_rem >> 12) & 0xF;
	
	return crc_read == comp_nrem; 
}

int
MS5837::ioctl(device::file_t *filp, int cmd, unsigned long arg)
{
	switch (cmd) {

	case SENSORIOCSPOLLRATE: {
			switch (arg) {

			/* switching to manual polling */
			case SENSOR_POLLRATE_MANUAL:
				stop();
				_measure_ticks = 0;
				return OK;

			/* external signalling (DRDY) not supported */
			case SENSOR_POLLRATE_EXTERNAL:

			/* zero would be bad */
			case 0:
				return -EINVAL;

			/* set default/max polling rate */
			case SENSOR_POLLRATE_MAX:
			case SENSOR_POLLRATE_DEFAULT: {
					/* do we need to start internal polling? */
					bool want_start = (_measure_ticks == 0);

					/* set interval for next measurement to minimum legal value */
					_measure_ticks = USEC2TICK(MS5837_CONVERSION_INTERVAL);

					/* if we need to start the poll state machine, do it */
					if (want_start) {
						start();
					}

					return OK;
				}

			/* adjust to a legal polling interval in Hz */
			default: {
					/* do we need to start internal polling? */
					bool want_start = (_measure_ticks == 0);

					/* convert hz to tick interval via microseconds */
					unsigned ticks = USEC2TICK(1000000 / arg);

					/* check against maximum rate */
					if (ticks < USEC2TICK(MS5837_CONVERSION_INTERVAL)) {
						return -EINVAL;
					}

					/* update interval for next measurement */
					_measure_ticks = ticks;

					/* if we need to start the poll state machine, do it */
					if (want_start) {
						start();
					}

					return OK;
				}
			}
		}

	case SENSORIOCGPOLLRATE:
		if (_measure_ticks == 0) {
			return SENSOR_POLLRATE_MANUAL;
		}

		return (1000 / _measure_ticks);

	case SENSORIOCSQUEUEDEPTH: {
			/* lower bound is mandatory, upper bound is a sanity check */
			if ((arg < 1) || (arg > 100)) {
				return -EINVAL;
			}

			irqstate_t flags = px4_enter_critical_section();

			if (!_reports->resize(arg)) {
				px4_leave_critical_section(flags);
				return -ENOMEM;
			}

			px4_leave_critical_section(flags);

			return OK;
		}

	case SENSORIOCRESET:
		/* CBH see if it should be OK or -EINVAL */
		/* CBH see above. Should that be atomic leave or px4_leave_critical_section */
		return OK;

	default:
		/* give it to the superclass */
		return I2C::ioctl(filp, cmd, arg);
	}
}

ssize_t
MS5837::read(device::file_t *filp, char *buffer, size_t buflen)
{
	unsigned count = buflen / sizeof(struct baro_report);
	struct baro_report *brp = reinterpret_cast<struct baro_report *>(buffer);
	int ret = 0;
	
	/* buffer must be large enough */
	if (count < 1) {
		return -ENOSPC;
	}
	
	/*if automatic measurement enabled */
	if(_measure_ticks > 0) {
		
		/*
		 * While there is space in the caller's buffer, and reports, copy them.
		 * Note that we may be pre-empted by the workq thread while we are doing this;
		 * we are careful to avoid racing with them.
		 */
		while (count--) {
			if (_reports->get(brp)) {
				ret += sizeof(*brp);
				brp++;
			}
		}
		 
		/* if there was no data, warn the caller */
		return ret ? ret : -EAGAIN;
	}
	
	/* manual measurement - run one conversion */
	do {
		_measure_phase = 0;
		_reports->flush();
		
		/* do temperature first */
		if (OK != measure()) {
			ret = -EIO;
			break;
		}
		
		/* wait for it to complete */
		usleep(MS5837_CONVERSION_INTERVAL);

		if (OK != collect()) {
			ret = -EIO;
			break;
		}

		/* now do a pressure measurement */
		if (OK != measure()) {
			ret = -EIO;
			break;
		}

		/* wait for it to complete */
		usleep(MS5837_CONVERSION_INTERVAL);

		if (OK != collect()) {
			ret = -EIO;
			break;
		}		

		/* state machine will have generated a report, copy it out */
		if (_reports->get(brp)) {
			ret = sizeof(*brp);
		}
		
	} while (0);
	
	return ret;
}

int 
MS5837::measure()
{
	int ret;
	
	perf_begin(_measure_perf);
	
	/*
	 * In phase zero, request temperature; in other phases, request pressure.
	 */
	unsigned addr = (_measure_phase == 0) ? ADDR_CMD_CONVERT_D2 : ADDR_CMD_CONVERT_D1;
	
	/*
	 * Send the command to begin a measurement.
	 */

	_retries = 0;
	uint8_t cmd = addr;
	ret = transfer(&cmd, 1, nullptr, 0);
	
	if(OK != ret) {
		perf_count(_comms_errors);
		PX4_DEBUG("i2c::transfer returned %d", ret);
	}
	
	perf_end(_measure_perf);
	
	return ret;
}

int
MS5837::collect()
{
	int ret = -EIO;
	uint8_t buf[3] = {0, 0, 0};
	/* this is the 8 bit command */
	uint8_t cmd = 0;
	
	perf_begin(_sample_perf);
	
	struct baro_report report;
	/* this process should occur at end of conversion, so the best approximation of time */
	report.timestamp = hrt_absolute_time();
	report.error_count = perf_event_count(_comms_errors);
	
	/* the transfer function syntax is transfer (address send, int #ofbits, address rcv, int #ofbits) */
	ret = transfer (&cmd, 1, &buf[0], 3);
	
	if (ret < 0) {
		PX4_DEBUG("error reading from sensor: %d", ret);
		perf_count(_comms_errors);
		perf_end(_sample_perf);
		return ret;
	}
	
	uint32_t comp_val = (buf[0] << 16) | (buf[1] << 8) | buf[2];
	
	/* handle measurement and convert into usable data */
	if (_measure_phase == 0) {
		
		/* temperature offset (in ADC units) */
		/* CBH - the original had an array being subtracted by a scalar. is that okay? */
		int32_t dT = (int32_t)comp_val - ((int32_t)_prom.c5_reference_temp << 8);
		
		/* absolute temperature in centidegrees - note intermediate value is outside 32-bit range */
		_TEMP = 2000 + (int32_t)(((int64_t)dT * _prom.c6_temp_coeff_temp) >> 23);
		
		/* base sensor scale/offset values */
		/* values taken from datasheet for MS5837 */
		_OFF  = ((int64_t)_prom.c2_pressure_offset << 16) + (((int64_t)_prom.c4_temp_coeff_pres_offset * dT) >> 7);
		_SENS = ((int64_t)_prom.c1_pressure_sens << 15) + (((int64_t)_prom.c3_temp_coeff_pres_sens * dT) >> 8);
		
		/* MS5837 temperature compensation */
		
		if (_TEMP < 2000) {
			
			//int32_t T2 = ((int64_t)3 * ((int64_t)dT * (int64_t)dT)/(int64_t)8589934592);
			int64_t T2 = 3*(POW2((int64_t)dT) >> 33);
			
			int64_t f = POW2((int64_t)_TEMP - 2000);
			int64_t OFF2 = 3 * f >> 1;
			int64_t SENS2 = 5 * f >> 3;
			
			_TEMP -= T2;
			_OFF  -= OFF2;
			_SENS -= SENS2;	
		} else {
			
			int64_t T2 = 2*(POW2((int64_t)dT) >> 37);
			int64_t f = POW2((int64_t)_TEMP - 2000);
			int64_t OFF2 = 1 * f >> 4;

			_TEMP -= T2;
			_OFF  -= OFF2;
		}
		
	} else {

		/* pressure calculation, result in Pa */
		int32_t P = (((comp_val * _SENS) >> 21) - _OFF) >> 13;
		P_sum += P;
		cnter += 1;
		
		if (cnter == 25) {
			
			P_sum /= 25;
		
			/* generate a new report */
			report.temperature = _TEMP / 100.0f;
			report.pressure = (P_sum / 10.0f);		/* convert to millibar */
			
			/* return device ID */
			report.device_id = get_device_id();
			
			/* publish it to uORB */
			if (_baro_topic != nullptr) {
				/* publish it */
				orb_publish(ORB_ID(sensor_baro), _baro_topic, &report);
			}
			
			_reports->force(&report);
			
			/* notify anyone waiting for data */
			poll_notify(POLLIN);
			
			P_sum = 0;
			cnter = 0;
		}
	}
	/* Update measurement state machine, next time, it will do pressure measurement */
	INCREMENT(_measure_phase, MS5837_MEASUREMENT_RATIO + 1);
	
	ret = OK;
	
	perf_end(_sample_perf);
	
	return ret; 
}

void
MS5837::start(unsigned delay_ticks)
{
	/* reset the report ring and state machine */
	_collect_phase = false;
	_measure_phase = 0;
	_reports->flush();
	
	/* schedule a cycle to start things */
	/* the cycle will bring all the disparate functions together */
	work_queue(HPWORK, &_work, (worker_t)&MS5837::cycle_trampoline, this, delay_ticks);
}

void
MS5837::stop()
{
	work_cancel(HPWORK, &_work);
}

void 
MS5837::cycle_trampoline(void *arg)
{
	MS5837 *dev = reinterpret_cast<MS5837 *>(arg);
	
	dev->cycle();
}

void
MS5837::cycle()
{
	int ret;
	
	/* check if collection phase */
	if (_collect_phase) {
		
		/* perform collection */
		ret = collect();
		
		if (ret != OK) {
			PX4_DEBUG("collection error!");
			/* restart measurement state machine */
			ret = reset();
			
			/* set a 2.8 ms wait after the reset */
			start(USEC2TICK(2800));
			return;
		}
		
		/* next phase is measurement */
		_collect_phase = false;
		
		/*
		 * Is there a collect->measure gap?
		 * Don't inject one after temperature measurements, so we can keep
		 * doing pressure measurements at something close to the desired rate.
		 */
		
		if ((_measure_phase != 0) &&
			(_measure_ticks > USEC2TICK(MS5837_CONVERSION_INTERVAL))) {

			/* schedule a fresh cycle call when we are ready to measure again */
			work_queue(HPWORK, 
						&_work, 
						(worker_t)&MS5837::cycle_trampoline,					
						this, 
						_measure_ticks - USEC2TICK(MS5837_CONVERSION_INTERVAL));
						
			return;
		}
	}
	
	/* measurement phase */
	ret = measure();
	
	if (ret != OK) {
		PX4_DEBUG("measurement error!");
		/* restart measurement state machine */
		ret = reset();
		
		/* reset collection state machine and try again */
		/* do we need to wait 2.8 ms again? */
		start();
		return;
	}
	
	/* next phase is collection */
	_collect_phase = true;
	
	/* schedule a fresh cycle call when the measurement is done */
	work_queue(HPWORK, 
				&_work, 
				(worker_t)&MS5837::cycle_trampoline,					
				this, 
				USEC2TICK(MS5837_CONVERSION_INTERVAL));
}
	
void 
MS5837::print_info()
{
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
	printf("poll interval:  %u ticks\n", _measure_ticks);
	_reports->print_info("report queue");
	printf("ms5837");
	sensor_baro_s brp = {};
	_reports->get(&brp);
	print_message(brp);
}

/**
 * Local functions in support of the shell command. 
 */
 
namespace ms5837
{
	
MS5837 *g_dev;

int		con_start();
int		con_start_bus(int i2c_bus);
int 	con_stop();
int		con_test();
int		con_reset();
int		con_info();

/**
 *
 * Attempt to start driver on all available I2C busses.
 *
 * This function will return only if the driver is successfully started
 * or failed to start. 
 *
 */
 
int
con_start()
{
	if (g_dev != nullptr) {
		PX4_ERR("already started");
		return PX4_ERROR;
	}
	
	for (unsigned i = 0; i < NUM_I2C_BUS_OPTIONS; i++) {
		if (con_start_bus(i2c_bus_options[i]) == PX4_OK) {
			PX4_ERR("device on bus %d", i);
			return PX4_OK;
		}
	}
	
	// driver is not okay
	PX4_ERR("none of the devices started");
	
	return PX4_ERROR;
}

/**
 * Start the driver on a specific bus.
 *
 * This function only returns if the sensor is up and running
 * or could not be detected successfully.
 */

int
con_start_bus(int i2c_bus)
{
	int fd = -1;
	
	if (g_dev != nullptr) {
		PX4_ERR("driver already started");
		return PX4_ERROR;
	}
	
	/* if not created, create the new driver
	 * note that address is not set here. the address is default MS5837_BASEADDR
	 * and declared in the constructor
	 */
	g_dev = new MS5837(i2c_bus);
	
	if (g_dev == nullptr) {
		PX4_ERR("failed to construct");
		goto fail;
	}
	
	if (OK != g_dev->init()){
		PX4_ERR("failed to initialize");
		/*
		PX4_ERR("error code %d", g_dev->err_c.err1);
		PX4_ERR("error code %d", g_dev->err_c.err2);
		PX4_ERR("error code %d", g_dev->err_c.err3);
		PX4_ERR("error code %d", g_dev->err_c.err4);
		PX4_ERR("error code %d", g_dev->err_c.err5);
		PX4_ERR("error code %d", g_dev->err_c.err6);
		PX4_ERR("error code %d", g_dev->err_c.err7);
		PX4_ERR("error code %d", g_dev->err_c.err8);
		*/
		goto fail;
	}
	
	/* set poll rate to default, starts automatic data collection */
	fd = px4_open(MS5837_DEVICE_PATH, O_RDONLY);
	
	if (fd < 0) {
		//errx(1, "can't open baro device");
		PX4_ERR("fd was lower than one, baro device not opened");
		goto fail;
	}
	
	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
		//errx(1, "failed setting default poll rate");
		PX4_ERR("failed to reset default poll rate");
		goto fail;
	}
	
	px4_close(fd);
	return PX4_OK;
	
fail:

	if (fd >= 0) {
		px4_close(fd);
	}
		
	if (g_dev != nullptr) {
		delete g_dev;
		g_dev = nullptr;
	}
	
	return PX4_ERROR;
}

/**
 * Stop the driver
 */
 
int 
con_stop()
 {
	if (g_dev != nullptr) {
		delete g_dev;
		g_dev = nullptr;
		
	} else {
		PX4_ERR("driver not running");
		return PX4_ERROR;
	}
	
	return PX4_OK;
 }

/**
 * Perform some basic functional tests on the driver;
 * make sure we can collect data from the sensor in polled
 * and automatic modes.
 */ 

int
con_test()
{
	struct baro_report report;
	ssize_t sz;
	int ret;
	
	int fd;
	
	fd = px4_open(MS5837_DEVICE_PATH, O_RDONLY);
	
	if (fd < 0){
		PX4_ERR("%d open failed (try 'ms5837 start' if the driver is not running)", fd);
		return PX4_ERROR;
	}
	
	/* do a simple demand read */
	sz = read(fd, &report, sizeof(report));
	
	if (sz != sizeof(report)) {
		PX4_ERR("immediate read failed");
		return PX4_ERROR;
	}
	
	print_message(report);
	
	/* start sensor, set poll at 2Hz and queue depth at 10 */
	
	if (OK != ioctl(fd, SENSORIOCSQUEUEDEPTH, 10)) {
		PX4_ERR("failed to set queue depth");
		return PX4_ERROR;
	}
	
	if (OK != ioctl(fd, SENSORIOCSPOLLRATE, 2)) {
		PX4_ERR("failed to set 2Hz poll rate");
		return PX4_ERROR;
	}
	
	/* read sensor 5x and report each value */
	for (unsigned i = 0; i < 5; i++) {
		struct pollfd fds;
		
		/* wait for data to be ready */
		fds.fd = fd;
		fds.events = POLLIN;
		ret = poll(&fds, 1, 6000);

		if (ret != 1) {
			PX4_ERR("timed out waiting for sensor data");
			PX4_ERR("ret is %d", ret);
			return PX4_ERROR;
		}
	
		/* now go get it */
		sz = read(fd, &report, sizeof(report));
		
		if (sz != sizeof(report)) {
			PX4_ERR("periodic read failed");
			return PX4_ERROR;
		}	

		print_message(report);		
	}
	
	/* reset the sensor polling to default rate */
	if (OK != ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT)) {
		PX4_ERR("failed to set default poll rate");
		return PX4_ERROR;
	}

	PX4_INFO("PASS");
	px4_close(fd);
	return PX4_OK;
}	

/**
 * Reset the driver.
 */
int
con_reset()
{
	int fd = px4_open(MS5837_DEVICE_PATH, O_RDONLY);

	if (fd < 0) {
		PX4_ERR("failed");
		return PX4_ERROR;
	}

	if (ioctl(fd, SENSORIOCRESET, 0) < 0) {
		PX4_ERR("driver reset failed");
		return PX4_ERROR;
	}

	if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
		PX4_ERR("driver poll restart failed");
		return PX4_ERROR;
	}

	return PX4_OK;
}

/**
 * Print a little info about the driver.
 */
int
con_info()
{
	if (g_dev == nullptr) {
		PX4_ERR("driver poll restart failed");
		return PX4_ERROR;
	}

	printf("state @ %p\n", g_dev);
	g_dev->print_info();

	return PX4_OK;
}

} /* namespace */

static void
ms5837_usage()
{
	PX4_INFO("usage: ms5837 command [options]");
	PX4_INFO("options:");
	PX4_INFO("\t-b --bus i2cbus (%d)", MS5837_BUS_DEFAULT);
	PX4_INFO("\t-a --all");
	PX4_INFO("command:");
	PX4_INFO("\tstart|stop|test|reset|info");
}

int
ms5837_main(int argc, char *argv[])
{
	int ch;
	int myoptind = 1;
	const char *myoptarg = nullptr;
	bool start_all = false;
	
	int i2c_bus = MS5837_BUS_DEFAULT;
	
	/* jump over start/off/etc and look at options first */
	/* CBH - do i need to add colon to end of "ab" */
	while ((ch = px4_getopt(argc, argv, "ab:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'b':
			i2c_bus = atoi(myoptarg);
			break;
		
		case 'a':
			start_all = true;
			break;
		
		default:
			PX4_WARN("Unknown option!");
			goto out_error;
		}
	}
	
	/*
	 * Start/load driver
	 */ 
	if (!strcmp(argv[myoptind], "start")) {
		if (start_all) {
			return ms5837::con_start();

		} else {
			return ms5837::con_start_bus(i2c_bus);
		}
	}
	
	/*
	 * Start/load driver
	 */ 
	if (!strcmp(argv[myoptind], "stop")) {
		return ms5837::con_stop();
	}
	
	/*
	 * Test the driver/device.
	 */
	if (!strcmp(argv[myoptind], "test")) {
		return ms5837::con_test();
	}

	/*
	 * Reset the driver.
	 */
	if (!strcmp(argv[myoptind], "reset")) {
		return ms5837::con_reset();
	}

	/*
	 * Print driver information.
	 */
	if (!strcmp(argv[myoptind], "info") || !strcmp(argv[myoptind], "status")) {
		return ms5837::con_info();
	}

out_error:
	ms5837_usage();
	return PX4_ERROR;
}


		
	

	
	
	
