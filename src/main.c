/*
 * Copyright (c) 2016 Linaro Limited.
 *               2016 Intel Corporation.
 * 				 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>
#include <zephyr/types.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/drivers/gpio.h>
#include <nrfx_gpiote.h>
#include <sensor/hx711/hx711.h>
#include <stddef.h>

#ifdef CONFIG_TRUSTED_EXECUTION_NONSECURE
#define TEST_PARTITION	slot1_ns_partition
#else
#define TEST_PARTITION	slot1_partition
#endif

#define TEST_PARTITION_OFFSET	FIXED_PARTITION_OFFSET(TEST_PARTITION)
#define TEST_PARTITION_DEVICE	FIXED_PARTITION_DEVICE(TEST_PARTITION)
#define FLASH_PAGE_SIZE         4096
#define CALIBRATION_VALUE_ADDR  0x40000 // Address where the calibration value will be stored in flash memory
#define LED3 4
#define LED4 5
#define SWITCH1 8
#define SWITCH2 9

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

const struct device *hx711_dev;
struct sensor_value slope;
const struct device *flash_dev = TEST_PARTITION_DEVICE;
struct sensor_value slope2 = {.val1 = 0, .val2 = 0};

void measure(void) {
	static struct sensor_value weight;
	int ret;

	ret = sensor_sample_fetch(hx711_dev);
	if(ret != 0) {
		LOG_ERR("Cannot take measurement: %d", ret);
	} else {
		sensor_channel_get(hx711_dev, HX711_SENSOR_CHAN_WEIGHT, &weight);
		LOG_INF("Weight: %d.%06d grams", weight.val1, weight.val2);
	}
}

void measure_with_slope(void) {
    static struct sensor_value weight;
    int ret;

    ret = sensor_sample_fetch(hx711_dev);
    if (ret != 0) {
        LOG_ERR("Cannot take measurement: %d", ret);
        return;
    }

    sensor_channel_get(hx711_dev, HX711_SENSOR_CHAN_WEIGHT, &weight);
    int32_t calibrated_val = ((int32_t)weight.val1 * slope.val1 + (int32_t)weight.val2 * slope.val2 / 1000000);
    weight.val1 = calibrated_val / 1000000;
    weight.val2 = calibrated_val % 1000000;
    LOG_INF("Weight with slope: %d.%06d grams", weight.val1, weight.val2);
}

void measureWithSlope(void) {
    static struct sensor_value weight;
    int ret;

    ret = sensor_sample_fetch(hx711_dev);
    if(ret != 0) {
        LOG_ERR("Cannot take measurement: %d", ret);
    } else {
        // Use the stored slope to calculate weight
        sensor_channel_get(hx711_dev, HX711_SENSOR_CHAN_WEIGHT, &weight);
        weight.val1 *= slope2.val1;
        weight.val2 *= slope2.val1;
        weight.val2 += weight.val1 * slope2.val2;
        weight.val1 *= slope2.val1;
        weight.val2 /= 1000000;
        weight.val1 += weight.val2;
        weight.val2 = weight.val2 % 1000000;
        LOG_INF("Weight: %d.%06d grams", weight.val1, weight.val2);
    }
}

void readWriteCalibration(void) {
    LOG_INF("Do you want to calibrate load sensor?");
    LOG_INF("To Make a Choice Push Switch to ON Before Count Down Ends");
    LOG_INF("SWITCH 1: Yes");
    LOG_INF("SWITCH 2: No");
    LOG_INF("Waiting 5 seconds for user response...");

    nrf_gpio_cfg_output(LED3);
    nrf_gpio_cfg_output(LED4);
    nrf_gpio_cfg_input(SWITCH1, NRF_GPIO_PIN_PULLUP);
    nrf_gpio_cfg_input(SWITCH2, NRF_GPIO_PIN_PULLUP);

    for(int i = 5; i >= 0; i--) {
        LOG_INF(" %d..", i);
        k_msleep(1000);
    }
}

void deviceDisable(void) {
	// Disable switch buttons
	nrfx_gpiote_pin_uninit(SWITCH1);
	nrfx_gpiote_pin_uninit(SWITCH2);

	// Disable LED pins
	nrf_gpio_pin_clear(LED3);
	nrf_gpio_pin_clear(LED4);
	nrfx_gpiote_pin_uninit(LED3);
	nrfx_gpiote_pin_uninit(LED4);

	// Disable load sensor
	nrfx_gpiote_pin_uninit(2);
	nrfx_gpiote_pin_uninit(3);
}

void deviceEnable(void) {
	// Enable switch buttons
	nrf_gpio_cfg_input(SWITCH1, NRF_GPIO_PIN_PULLUP);
	nrf_gpio_cfg_input(SWITCH2, NRF_GPIO_PIN_PULLUP);

	// Enable LED pins
	nrf_gpio_cfg_output(LED3);
	nrf_gpio_cfg_output(LED4);
	nrf_gpio_pin_set(LED3);
	nrf_gpio_pin_set(LED4);

	// Enable load sensor
	nrf_gpio_cfg_input(2, (GPIO_ACTIVE_HIGH | GPIO_PULL_UP));
	nrf_gpio_cfg_input(3, GPIO_ACTIVE_HIGH);
}

void main(void) {
	int calibration_weight = 1000; // weight in grams
    hx711_dev = DEVICE_DT_GET_ANY(avia_hx711);
    __ASSERT(hx711_dev == NULL, "Failed to get device binding");
    LOG_INF("Device is %p, name is %s", hx711_dev, hx711_dev->name);

    readWriteCalibration();
    
    if(nrf_gpio_pin_read(SWITCH1)==0) {
        nrf_gpio_pin_set(LED3);
        LOG_INF("Switch 1 Pressed: Option 1 Choosen");
        LOG_INF("Calculating offset...");
        avia_hx711_tare(hx711_dev, 5);

        LOG_INF("Waiting for known weight of %d grams...", calibration_weight);
        for(int i = 5; i >= 0; i--) {
            LOG_INF(" %d..", i);
            k_msleep(1000);
        }

        LOG_INF("Calculating slope...");
        avia_hx711_calibrate(hx711_dev, calibration_weight, 5);
        slope = avia_hx711_calibrate(hx711_dev, calibration_weight, 5);

        uint8_t write_buf[sizeof(slope)];
        memcpy(write_buf, &slope, sizeof(slope));
        LOG_INF("Wrote slope to flash: %d.%06d", slope.val1, slope.val2);

        int rc = flash_erase(flash_dev, CALIBRATION_VALUE_ADDR, FLASH_PAGE_SIZE);
        if (rc) {
            LOG_ERR("Failed to erase flash memory: %d", rc);
            return;
        }

        rc = flash_write(flash_dev, CALIBRATION_VALUE_ADDR, write_buf, sizeof(write_buf));
        if (rc) {
            LOG_ERR("Failed to write to flash memory: %d", rc);
            return;
        }

        nrf_gpio_pin_clear(LED3);

        while(true) {
            deviceEnable();
            measure();
            deviceDisable();
		    k_sleep(K_SECONDS(5));
        }
    }

    else if(nrf_gpio_pin_read(SWITCH2)==0) {
        nrf_gpio_pin_set(LED4);
        LOG_INF("Switch 2 Pressed: Option 2 Choosen");

        // Read slope from memory
        uint8_t read_buf[sizeof(slope2)];
        int rc = flash_read(flash_dev, CALIBRATION_VALUE_ADDR, read_buf, sizeof(read_buf));
        if (rc) {
            LOG_ERR("Failed to read from flash memory: %d", rc);
            return;
        }

        memcpy(&slope2, read_buf, sizeof(slope2));
        LOG_INF("Read slope from flash: %d.%06d", slope2.val1, slope2.val2);

        // Read control slope from memory
        struct sensor_value stored_slope; 
        rc = flash_read(flash_dev, CALIBRATION_VALUE_ADDR, &stored_slope, sizeof(stored_slope));
        if (rc) {
            LOG_ERR("Failed to read from flash memory: %d", rc);
            return;
        }

        // Compare stored and calculated slopes
        if (stored_slope.val1 == slope2.val1 && stored_slope.val2 == slope2.val2) {
            LOG_INF("Slope matches the value stored in memory");
        } else {
            LOG_INF("Slope does not match the value stored in memory");
        }

        nrf_gpio_pin_clear(LED4);

        while(true) {
            deviceEnable();
            measureWithSlope();
            deviceDisable();
		    k_sleep(K_SECONDS(5));
        }
    }

    else if((nrf_gpio_pin_read(SWITCH1)==0) && (nrf_gpio_pin_read(SWITCH2)==0)) {
        nrf_gpio_pin_set(LED3);
        nrf_gpio_pin_set(LED4);
        LOG_INF("Error: Push Just One Switch, Auto-Choosing Option 2");

        // Read slope from memory
        uint8_t read_buf[sizeof(slope2)];
        int rc = flash_read(flash_dev, CALIBRATION_VALUE_ADDR, read_buf, sizeof(read_buf));
        if (rc) {
            LOG_ERR("Failed to read from flash memory: %d", rc);
            return;
        }

        memcpy(&slope2, read_buf, sizeof(slope2));
        LOG_INF("Read slope from flash: %d.%06d", slope2.val1, slope2.val2);

        // Read control slope from memory
        struct sensor_value stored_slope; 
        rc = flash_read(flash_dev, CALIBRATION_VALUE_ADDR, &stored_slope, sizeof(stored_slope));
        if (rc) {
            LOG_ERR("Failed to read from flash memory: %d", rc);
            return;
        }

        // Compare stored and calculated slopes
        if (stored_slope.val1 == slope2.val1 && stored_slope.val2 == slope2.val2) {
            LOG_INF("Slope matches the value stored in memory");
        } else {
            LOG_INF("Slope does not match the value stored in memory");
        }

        nrf_gpio_pin_clear(LED4);

        while(true) {
            deviceEnable();
            measureWithSlope();
            deviceDisable();
		    k_sleep(K_SECONDS(5));
        }
    }

    else if((nrf_gpio_pin_read(SWITCH1)!=0) && (nrf_gpio_pin_read(SWITCH2)!=0)) {
        LOG_ERR("Error: Timeout No Button Pressed, Auto-Choosing Option 2");

        // Read slope from memory
        uint8_t read_buf[sizeof(slope2)];
        int rc = flash_read(flash_dev, CALIBRATION_VALUE_ADDR, read_buf, sizeof(read_buf));
        if (rc) {
            LOG_ERR("Failed to read from flash memory: %d", rc);
            return;
        }

        memcpy(&slope2, read_buf, sizeof(slope2));
        LOG_INF("Read slope from flash: %d.%06d", slope2.val1, slope2.val2);

        // Read control slope from memory
        struct sensor_value stored_slope; 
        rc = flash_read(flash_dev, CALIBRATION_VALUE_ADDR, &stored_slope, sizeof(stored_slope));
        if (rc) {
            LOG_ERR("Failed to read from flash memory: %d", rc);
            return;
        }

        // Compare stored and calculated slopes
        if (stored_slope.val1 == slope2.val1 && stored_slope.val2 == slope2.val2) {
            LOG_INF("Slope matches the value stored in memory");
        } else {
            LOG_INF("Slope does not match the value stored in memory");
        }

        nrf_gpio_pin_clear(LED4);

        while(true) {
            deviceEnable();
            measureWithSlope();
            deviceDisable();
		    k_sleep(K_SECONDS(5));
        }
    }
    
    else {
        LOG_ERR("Error: Timeout No Button Pressed, Auto-Choosing Option 2");

        // Read slope from memory
        uint8_t read_buf[sizeof(slope2)];
        int rc = flash_read(flash_dev, CALIBRATION_VALUE_ADDR, read_buf, sizeof(read_buf));
        if (rc) {
            LOG_ERR("Failed to read from flash memory: %d", rc);
            return;
        }

        memcpy(&slope2, read_buf, sizeof(slope2));
        LOG_INF("Read slope from flash: %d.%06d", slope2.val1, slope2.val2);

        // Read control slope from memory
        struct sensor_value stored_slope; 
        rc = flash_read(flash_dev, CALIBRATION_VALUE_ADDR, &stored_slope, sizeof(stored_slope));
        if (rc) {
            LOG_ERR("Failed to read from flash memory: %d", rc);
            return;
        }

        // Compare stored and calculated slopes
        if (stored_slope.val1 == slope2.val1 && stored_slope.val2 == slope2.val2) {
            LOG_INF("Slope matches the value stored in memory");
        } else {
            LOG_INF("Slope does not match the value stored in memory");
        }

        nrf_gpio_pin_clear(LED4);

        while(true) {
            deviceEnable();
            measureWithSlope();
            deviceDisable();
		    k_sleep(K_SECONDS(5));
        }
    }
}