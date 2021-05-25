// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Availink AVL62X1 DVB-S/S2/S2X demodulator driver
 * Supports AVL6221 and AVL6261. NOT AVL6211
 *
 * Copyright (C) 2020 Availink, Inc. (gpl@availink.com)
 *
 */

#ifndef _AVL62X1_H_
#define _AVL62X1_H_

#include <linux/firmware.h>
#include <linux/dvb/frontend.h>
#include <linux/dvb/version.h>
#include <media/dvb_frontend.h>

#include "avl62x1_lib.h"

#define str(a) #a
#define xstr(a) str(a)

#define DVB_VER_INT(maj,min) (((maj) << 16) + (min))
#define DVB_VER_ATLEAST(maj, min) \
 (DVB_VER_INT(DVB_API_VERSION,  DVB_API_VERSION_MINOR) >= DVB_VER_INT(maj, min))

#define AVL62X1_FIRMWARE	"availink/dvb-fe-avl62x1.fw"

#define AVL62X1_VERSION xstr(AVL62X1_VER_MAJOR) "." xstr(AVL62X1_VER_MINOR) "." xstr(AVL62X1_VER_BUILD)

#define AVL62X1_T2MI_CTRL_PROP			isdbt_sb_segment_idx
//isdbt_sb_segment_idx fields
#define AVL62X1_T2MI_CTRL_VALID_STREAM_MASK	(0x80000000)

//isdbt_sb_segment_idx fields
#define AVL62X1_T2MI_PID_SHIFT		16
#define AVL62X1_T2MI_PLP_ID_SHIFT	0

struct avl62x1_config {
	//structure of user-configurable parameters
	struct avl62x1_chip_pub *chip_pub;
};

extern struct dvb_frontend *avl62x1_attach(struct avl62x1_config *config,
					   struct i2c_adapter *i2c);

extern int avl62x1_blindscan_attach(struct dvb_frontend *fe,
				    struct proc_dir_entry *root_dir);

#endif /* _AVL62X1_H_ */
