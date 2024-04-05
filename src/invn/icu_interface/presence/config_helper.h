#ifndef _ICU_INTERFACE_PRESENCE_CONFIG_HELPER_H_
#define _ICU_INTERFACE_PRESENCE_CONFIG_HELPER_H_

#include <invn/icu_interface/presence/structs.h>

#define ICU_PRESENCE_SPPED_OF_SOUND_MM_PER_SEC       34300  // speed of sound [cm]

#define ICU_INTERFACE_PRESENCE_DEFAULT_USER_CONFIG {\
	.sensitivity = ICU_PRESENCE_DEFAULT_SENSITIVITY,\
	.hold_presence_s = ICU_PRESENCE_DEFAULT_HOLD_PRESENCE_S,\
	.event_mode = ICU_PRESENCE_DEFAULT_EVENT_MODE,\
}

static __inline void icu_interface_presence_generate_default_user_config(IcuPresenceUserConfig * user_config)
{
	const IcuPresenceUserConfig defaultConfig = ICU_INTERFACE_PRESENCE_DEFAULT_USER_CONFIG;
	*user_config = defaultConfig;
}

static __inline uint8_t icu_interface_presence_generate_config(const IcuPresenceSensorConfig * sensor_config
                                   , const IcuPresenceUserConfig   * user_config
                                   , IcuPresenceConfig         *algo_config)
{
	if (sensor_config->odr == 0)
		return 1;

	if ((sensor_config->cic_odr > 7) || (sensor_config->cic_odr < 1))
		return 2;

	if (sensor_config->pulse_len == 0)
		return 4;
	
	if (sensor_config->sensor_type == ICU_PRESENCE_SENSOR_TYPE_ICU10201)
		algo_config->sensor_type = 0;
	else if (sensor_config->sensor_type == ICU_PRESENCE_SENSOR_TYPE_ICU30201)
		algo_config->sensor_type = 2;
	else
		algo_config->sensor_type = 1;

	/*
	 * Here we convert sensitivity to detection threshold base on signal envelop metric
	 * Sensitivity from 0 - 100
	 * Most sensitive: Sensitivity = 100 => thr =    1 sigma
	 * Less sensitive: Sensitivity =   0 => thr = 16.5 sigma

	 * Most sensitive: Sensitivity = 100 => thr_var = 1 gamma
	 * Less sensitive: Sensitivity =   0 => thr_var = 33 gamma
	 */
	//uint8_t sensitivity = MIN(user_config->sensitivity, 100);
	uint8_t sensitivity = (user_config->sensitivity > 100) ? 100 : user_config->sensitivity;
	algo_config->thresh_zone_q5[0] = (200L - 2L * sensitivity) + 32;
	algo_config->thresh_zone_q5[1] = (200L - 2L * sensitivity) + 32;
	algo_config->thresh_zone_q5[2] = (200L - 2L * sensitivity) + 32;
	algo_config->thresh_zone_q5[3] = (200L - 2L * sensitivity) + 32;

	
	/*
	 * Here we convert sensitivity to detection threshold based on signal variance metric
	 * Sensitivity from 0 - 100
	 */
	if (algo_config->sensor_type == 1)
	{
		algo_config->max_envelope_sum_zone_thr_hi = 23376 + ((uint32_t)(algo_config->thresh_zone_q5[0]) << 9);
		algo_config->max_envelope_sum_zone_thr_lo = algo_config->max_envelope_sum_zone_thr_hi - 10000;
		algo_config->thresh_var_zone[0] = (104L - sensitivity) >> 2;
		algo_config->thresh_var_zone[1] = (104L - sensitivity) >> 2;
		algo_config->thresh_var_zone[2] = (104L - sensitivity) >> 2;
		algo_config->thresh_var_zone[3] = (104L - sensitivity) >> 2;
	}
	else {
		algo_config->thresh_var_zone[0] = 30;
		algo_config->thresh_var_zone[1] = 30;
		algo_config->thresh_var_zone[2] = 30;
		algo_config->thresh_var_zone[3] = 30;
		algo_config->max_envelope_sum_zone_thr_hi = 100000;
		algo_config->max_envelope_sum_zone_thr_lo = 70000;
	}
	/* 
	 * Relation between cic_odr and range resolution
	 * scale_flt = 343000 / 2 / (fop * 2 ^ (odr - 7))
	 * scale_q10 = 343000 * 2^10 / 2 / (fop * 2 ^ (odr - 7))
	 */
	algo_config->range_factor = ((uint32_t) ICU_PRESENCE_SPPED_OF_SOUND_MM_PER_SEC << 10) / 2 / (sensor_config->fop >> (7 - sensor_config->cic_odr)); //q10
	
	
	/*
	 * Time base detection is related to measurement rate
	 * Relation between cic_odr and range resolution
	 * scale_flt = 343000 / 2 / (fop * 2 ^ (odr - 7))
	 * scale_q10 = 343000 * 2^10 / 2 / (fop * 2 ^ (odr - 7))
	 */
	uint32_t rate = (1L << 30) / sensor_config->odr; // ~1sec in q10
	algo_config->detection_hold = (uint16_t) ((rate * user_config->hold_presence_s) >> 10) + 1; // in number of samples
	algo_config->envelope_lo_max_time = 600; // in number of samples
	algo_config->listening_samples = sensor_config->listening_samples; // in number of samples
	
	/*
	 * Set event mode 
	 */
	if (((user_config->event_mode % 2) == 1) || (user_config->event_mode == 0) || (user_config->event_mode > 7)){
		algo_config->event_mode = 1;
	}
	else {

		algo_config->event_mode = user_config->event_mode;
	}
	return 0;
}

#endif // #ifndef _ICU_INTERFACE_PRESENCE_INTERFACE_H_