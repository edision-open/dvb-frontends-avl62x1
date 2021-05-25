// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * AV201x Airoha Technology silicon tuner driver
 *
 * Copyright (C) 2014 Luis Alves <ljalvs@gmail.com>
 */

#ifndef AV201X_H
#define AV201X_H

#include <linux/kconfig.h>
#include <media/dvb_frontend.h>

typedef enum av201x_id {
	ID_AV2011,
	ID_AV2012,
	ID_AV2018,
} av201x_id_t;

struct av201x_config {
	/* tuner i2c address */
	u8 i2c_address;
	/* tuner type */
	av201x_id_t id;

	/* crystal freq in kHz */
	u32 xtal_freq;
};

#if IS_ENABLED(CONFIG_MEDIA_TUNER_AV201X)
extern struct dvb_frontend *av201x_attach(struct dvb_frontend *fe,
		struct av201x_config *cfg, struct i2c_adapter *i2c);
#else
static inline struct dvb_frontend *av201x_attach(struct dvb_frontend *fe,
		struct av201x_config *cfg, struct i2c_adapter *i2c)
{
	printk(KERN_WARNING "%s: driver disabled by Kconfig\n", __func__);
	return NULL;
}
#endif

struct avl_tuner;

extern u32 av201x_get_max_lpf(struct avl_tuner *, u32 *);
extern u32 av201x_get_min_lpf(struct avl_tuner *, u32 *);
extern u32 av201x_get_lpf_step_size(struct avl_tuner *, u32 *);
extern u32 av201x_get_agc_slope(struct avl_tuner *, s32 *);
extern u32 av201x_get_min_gain_voltage(struct avl_tuner *, u32 *);
extern u32 av201x_get_max_gain_voltage(struct avl_tuner *, u32 *);
extern u32 av201x_get_rf_freq_step_size(struct avl_tuner *, u32 *);

#endif /* AV201X_H */
