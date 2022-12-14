#ifndef __BLE_SPP_SERVER_DEMO_H__
#define __BLE_SPP_SERVER_DEMO_H__

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "driver/adc.h"
#include "driver/i2c.h"
#include "esp_adc_cal.h"

#include "esp_sleep.h"
#include "esp_timer.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include "driver/uart.h"
#include "string.h"

#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "i2cdev.h"

#include "bno055.h"
#include "tca9548.h"

#define DEFAULT_VREF    3300        //Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES   64          //Multisampling

#define LED_R 4
#define LED_G 17
#define LED_B 13
#define BUTTON_IO_NUM  35
#define BUTTON_ACTIVE_LEVEL   1
#define USB_DETECT 27
#define CHRG 21

#define SENSOR_COUNT 6

#define bno055_use
/*
 * DEFINES
 ****************************************************************************************
 */
//#define SUPPORT_HEARTBEAT
//#define SPP_DEBUG_MODE

#define spp_sprintf(s,...)         sprintf((char*)(s), ##__VA_ARGS__)
#define SPP_DATA_MAX_LEN           (512)
#define SPP_CMD_MAX_LEN            (20)
#define SPP_STATUS_MAX_LEN         (20)
#define SPP_DATA_BUFF_MAX_LEN      (2*1024)


#define GATTS_TABLE_TAG  "GATTS_SPP_DEMO"

#define SPP_PROFILE_NUM             1
#define SPP_PROFILE_APP_IDX         0
#define ESP_SPP_APP_ID              0x56
#define SAMPLE_DEVICE_NAME          "SMART_GLOVE"    //The Device Name Characteristics in GAP
#define SPP_SVC_INST_ID	            0

/// Characteristic UUID
#define ESP_GATT_UUID_SPP_DATA_RECEIVE      0xABF1
#define ESP_GATT_UUID_SPP_DATA_NOTIFY       0xABF2
#define ESP_GATT_UUID_SPP_COMMAND_RECEIVE   0xABF3
#define ESP_GATT_UUID_SPP_COMMAND_NOTIFY    0xABF4
///Attributes State Machine
#define CHECK_ARG(VAL) do { if (!(VAL)) return ESP_ERR_INVALID_ARG; } while (0)

#endif
