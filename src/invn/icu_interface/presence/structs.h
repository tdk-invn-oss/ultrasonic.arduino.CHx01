/*
 * ________________________________________________________________________________________________________
 * Copyright (c) 2021-2022 InvenSense Inc. All rights reserved.
 *
 * This software, related documentation and any modifications thereto (collectively �Software�) is subject
 * to InvenSense and its licensors' intellectual property rights under U.S. and international copyright
 * and other intellectual property rights laws.
 *
 * InvenSense and its licensors retain all intellectual property and proprietary rights in and to the Software
 * and any use, reproduction, disclosure or distribution of the Software without an express license agreement
 * from InvenSense is strictly prohibited.
 *
 * EXCEPT AS OTHERWISE PROVIDED IN A LICENSE AGREEMENT BETWEEN THE PARTIES, THE SOFTWARE IS
 * PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED
 * TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT.
 * EXCEPT AS OTHERWISE PROVIDED IN A LICENSE AGREEMENT BETWEEN THE PARTIES, IN NO EVENT SHALL
 * INVENSENSE BE LIABLE FOR ANY DIRECT, SPECIAL, INDIRECT, INCIDENTAL, OR CONSEQUENTIAL DAMAGES, OR ANY
 * DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT,
 * NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE
 * OF THE SOFTWARE.
 * ________________________________________________________________________________________________________
 */

#ifndef _ICU_INTERFACE_PRESENCE_STRUCTS_H_
#define _ICU_INTERFACE_PRESENCE_STRUCTS_H_

#include <stdint.h>

///////////////
// Constants //
///////////////
#define ICU_PRESENCE_VERSION_X 1 /*!< version major */
#define ICU_PRESENCE_VERSION_Y 7 /*!< version minor */
#define ICU_PRESENCE_VERSION_Z 3 /*!< version patch */
#define ICU_PRESENCE_ALGO_ID   2 /*!< algo id */

#define ICU_PRESENCE_MAX_IQ_SAMPLES                     (265)   /*!< default measurement RX samples */
#define ICU_PRESENCE_CONFIG_SIZE                        (28)    /*!< Size in bytes of the params structure to allocate */
#define ICU_PRESENCE_OUTPUT_MASK_PRESENCE_DETECTED      (0x02)  /*!< Mask ID of a presence detection event */
#define ICU_PRESENCE_OUTPUT_MASK_DETECTION_LOSS         (0x01)  /*!< Mask ID of a detection loss event */
#define ICU_PRESENCE_OUTPUT_MASK_PRESENCE_INIT_ERROR    (0xFF)  /*!< Mask ID of an error event: the number of IQ data is less than what the algo expects */
#define ICU_PRESENCE_EVENT_MODE_WHILE_DETECTION_TRUE    (0x01)  /*!< Event output will trigger interrupt whenever presence detection output is true */
#define ICU_PRESENCE_EVENT_MODE_ON_NEW_DETECTION        (0x02)  /*!< Event output will trigger interrupt when presence detection output changes from false to true */
#define ICU_PRESENCE_EVENT_MODE_ON_DETECTION_LOSS       (0x04)  /*!< Event output will trigger interrupt when presence detection output changes from true to false */
#define ICU_PRESENCE_EVENT_MODE_ON_DETECTION_CHANGE     (0x06)  /*!< Event output will trigger interrupt whenever presence detection output changes */
#define ICU_PRESENCE_SENSOR_TYPE_ICU10201               (10201) /*!< Connected sensor is of type ICU10201 */
#define ICU_PRESENCE_SENSOR_TYPE_ICU20201               (20201) /*!< Connected sensor is of type ICU20201 */
#define ICU_PRESENCE_SENSOR_TYPE_ICU30201               (30201) /*!< Connected sensor is of type ICU30201 */

///////////////////////////
// Default configuration //
///////////////////////////


////////////////////////////////
// Default User Configuration //
////////////////////////////////

#define ICU_PRESENCE_DEFAULT_HOLD_PRESENCE_S            1       /*!< default user setting hold_presence_s in seconds */
#define ICU_PRESENCE_DEFAULT_SENSITIVITY                90      /*!< default user setting sensitivity [0-100] */
#define ICU_PRESENCE_DEFAULT_EVENT_MODE                 ICU_PRESENCE_EVENT_MODE_WHILE_DETECTION_TRUE  /*!< default user setting event_mode */

//////////////////////////////////
// Typical Sensor Configuration //
//////////////////////////////////

#define ICU_PRESENCE_RECOMMENDED_ODR_US                 100000  /*!< recommended measurement output data rate in microseconds */
#define ICU_PRESENCE_RECOMMENDED_PULSE_LENGTH           512     /*!< recommended measurement pulse length in SMCLK cycles */
#define ICU_PRESENCE_RECOMMENDED_CIC_ODR                4       /*!< recommended measurement CIC_ODR used to down sample IQ data */
#define ICU_PRESENCE_TYPICAL_ICU10201_FOP               180000  /*!< typical ICU10201 operating frequency in Hertz */
#define ICU_PRESENCE_TYPICAL_ICU20201_FOP               80000   /*!< typical ICU20201 operating frequency in Hertz */
#define ICU_PRESENCE_TYPICAL_ICU30201_FOP               56000   /*!< typical ICU30201 operating frequency in Hertz */
#define ICU_PRESENCE_TYPICAL_LISTENING_SAMPLES          0       /*!< typical ICU10201 measurement listening samples before transmit */
#define ICU_PRESENCE_DEFAULT_SENSOR_TYPE                ICU_PRESENCE_SENSOR_TYPE_ICU20201  /*!< sensor type must be provided, otherwise algo assumes ICU20201 */

///////////////////////////
// Structure definitions //
///////////////////////////

/*! \struct IcuPresenceSensorConfig
* Structure to configure the sensor measurement settings \ingroup IcuPresence
*/
typedef struct 
{
	uint32_t fop;                     /*!< operating frequency of the sensor, usually around 80000 Hz for ICU20201 */
	uint32_t odr;                     /*!< output data rate in us */
	uint16_t pulse_len;               /*!< width of the transmitted pulse in SMCLK cycles */
	uint16_t sensor_type;             /*!< 10201-> ICU10201, 20201-> ICU20201 (default), 30201-> ICU30201 */
	uint8_t cic_odr;                  /*!< sensor cic_odr (range-axis data rate) */
	uint8_t listening_samples;        /*!< number of samples to listen before transmit */
} IcuPresenceSensorConfig;

/*! \struct IcuPresenceUserConfig
* Structure to configure the user settings \ingroup IcuPresence
*/
typedef struct
{
	uint8_t sensitivity;              /*!< sensitivity of the detection between 0 and 100. */
	uint8_t hold_presence_s;          /*!< seconds to hold presence detection */
	uint8_t event_mode;               /*!< determines how the event output will behave to trigger an interrupt */
} IcuPresenceUserConfig;



/*! \struct IcuPresenceConfig
 * Structure of algo configuration. 
 * The recommended values can be computed using sensor configuration (IcuPresenceSensorConfig) and 
 * user configuration (IcuPresenceUserConfig)
 * To fill up with recommended values, use the function icu_interface_presence_generate_config()  \ingroup IcuPresence
 */
typedef struct IcuPresenceConfig
{
	int32_t max_envelope_sum_zone_thr_hi;  /*!< maximum allowed threshold high for detection */
	int32_t max_envelope_sum_zone_thr_lo;  /*!< maximum allowed threshold low for detection */
	int16_t envelope_lo_max_time;          /*!< maximum time to stay with threshold low */
	uint16_t range_factor;                 /*!< value to multiply by the sample id to convert into cm */
	uint16_t detection_hold;               /*!< samples to hold presence detection */
	uint8_t event_mode;                    /*!< defines the behavior of the event output */
	uint8_t sensor_type;                   /*!< 0-> ICU10201, 1-> ICU20201 (default), 2-> ICU30201 */
	uint8_t listening_samples;             /*!< number of rx samples pre-tx */
	uint8_t thresh_zone_q5[4];             /*!< threshold for envelope mean detection (q5) */
	uint8_t thresh_var_zone[4];            /*!< threshold for envelope var detection (q-15) */
	uint8_t reserved[3];                   /*!< bytes for alignment */
}IcuPresenceConfig;


/*! \struct IcuPresenceOutput
 * Output structure (presence detection events and range)  \ingroup IcuPresence
 */
typedef struct 
{
	uint16_t motion_range_cm;        /*!< The estimated range of presence in cm */
	uint16_t confidence;             /*!< The confidence score of the output in q15 where 0 is no confidence, and 32767 is 100% confidence */
	uint8_t presence_detection;		 /*!< User presence: 0-> no presence, 1-> presence. */
	volatile uint8_t event;          /*!< Event detection. Binary value: 0-> no interrupt, 1-> interrupt.*/
	volatile uint8_t mask;           /*!< Event type. For possible values, check ICU_PRESENCE_OUTPUT_MASK definitions above */
	uint8_t reserved;                /*!< Reserved byte for alignment */
} IcuPresenceOutput;

#endif