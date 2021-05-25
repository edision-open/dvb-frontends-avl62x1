// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Availink AVL62X1 DVB-S/S2/S2X demodulator driver
 * Supports AVL6221 and AVL6261. NOT AVL6211
 *
 * Copyright (C) 2020 Availink, Inc. (gpl@availink.com)
 *
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/proc_fs.h>
#include <linux/sched.h>
#include <linux/sched/signal.h>
#include <linux/seq_file.h>
#include <linux/vmalloc.h>
#include <linux/dvb/frontend.h>
#include <media/dvb_frontend.h>

#include "avl62x1.h"
#include "avl62x1_api.h"
#include "avl_tuner.h"
#include "av201x.h"

enum avl62x1_tune_state {
	avl62x1_tune_state_idle,
	avl62x1_tune_state_tuner_trylock,
	avl62x1_tune_state_tuner_status,
	avl62x1_tune_state_demod_trylock,
	avl62x1_tune_state_demod_status,
	avl62x1_tune_state_read_status,
};

struct avl62x1_bs_state {
	refcount_t running;
	struct task_struct *thread;
	struct dtv_frontend_properties min_prop;
	struct dtv_frontend_properties max_prop;

	int progress;
	int index;
	struct dtv_frontend_properties *props;
	int num_props;

	struct avl62x1_blind_scan_params params;
	struct avl62x1_blind_scan_info info;
};

struct avl62x1_priv {
	struct i2c_adapter *i2c;
	struct dvb_frontend frontend;
	struct avl62x1_chip *chip;
	const struct firmware *fw;

	enum avl62x1_tune_state tune_state;
	int tune_timeout;

	struct avl62x1_bs_state bs_state;
};

static int avl62x1_blindscan_props(struct dtv_frontend_properties *props,
				   struct avl62x1_carrier_info *carrier_info,
				   struct avl62x1_stream_info *stream_info)
{
	props->frequency = carrier_info->rf_freq_khz;

	props->symbol_rate = carrier_info->symbol_rate_hz;

	props->cnr.len = 1;
	props->cnr.stat[0].scale = FE_SCALE_DECIBEL;
	props->cnr.stat[0].svalue = carrier_info->snr_db_x100 * 10;

	props->scrambling_sequence_index = carrier_info->pl_scrambling & AVL62X1_PL_SCRAM_XSTATE_MASK;
	if (carrier_info->pl_scrambling & AVL62X1_PL_SCRAM_XSTATE)
		__avl62x1_conv_xlfsr_state_to_n(props->scrambling_sequence_index, &props->scrambling_sequence_index);

	if (carrier_info->signal_type == avl62x1_dvbs) {
		props->delivery_system = SYS_DVBS;
		props->modulation = QPSK;
		props->pilot = PILOT_OFF;

		switch (carrier_info->code_rate.dvbs_code_rate) {
		case avl62x1_dvbs_cr_1_2:
			props->fec_inner = FEC_1_2;
			break;
		case avl62x1_dvbs_cr_2_3:
			props->fec_inner = FEC_2_3;
			break;
		case avl62x1_dvbs_cr_3_4:
			props->fec_inner = FEC_3_4;
			break;
		case avl62x1_dvbs_cr_5_6:
			props->fec_inner = FEC_5_6;
			break;
		case avl62x1_dvbs_cr_7_8:
			props->fec_inner = FEC_7_8;
			break;
		default:
			break;
		}

		props->rolloff = ROLLOFF_35;
		props->stream_id = NO_STREAM_ID_FILTER;
		props->AVL62X1_T2MI_CTRL_PROP = NO_STREAM_ID_FILTER;
	} else if (carrier_info->signal_type == avl62x1_dvbs2) {
		props->delivery_system = SYS_DVBS2;

		switch (carrier_info->modulation) {
		case avl62x1_qpsk:
			props->modulation = QPSK;
			break;
		case avl62x1_8psk:
			props->modulation = PSK_8;
			break;
		case avl62x1_16apsk:
			props->modulation = APSK_16;
			break;
		default:
			props->modulation = APSK_32;
			break;
		}

		if (carrier_info->pilot == avl62x1_pilot_on)
			props->pilot = PILOT_ON;
		else
			props->pilot = PILOT_OFF;

		switch (carrier_info->code_rate.dvbs2_code_rate) {
		case avl62x1_dvbs2_cr_2_5:
			props->fec_inner = FEC_2_5;
			break;
		case avl62x1_dvbs2_cr_1_2:
			props->fec_inner = FEC_1_2;
			break;
		case avl62x1_dvbs2_cr_3_5:
			props->fec_inner = FEC_3_5;
			break;
		case avl62x1_dvbs2_cr_2_3:
			props->fec_inner = FEC_2_3;
			break;
		case avl62x1_dvbs2_cr_3_4:
			props->fec_inner = FEC_3_4;
			break;
		case avl62x1_dvbs2_cr_4_5:
			props->fec_inner = FEC_4_5;
			break;
		case avl62x1_dvbs2_cr_5_6:
			props->fec_inner = FEC_5_6;
			break;
		case avl62x1_dvbs2_cr_8_9:
			props->fec_inner = FEC_8_9;
			break;
		case avl62x1_dvbs2_cr_9_10:
			props->fec_inner = FEC_9_10;
			break;
		default:
			props->fec_inner = FEC_AUTO;
			break;
		}

		props->rolloff = ROLLOFF_AUTO;

		if (carrier_info->sis_mis == avl62x1_mis)
			props->stream_id = stream_info->isi;
		else
			props->stream_id = NO_STREAM_ID_FILTER;

		if (stream_info->stream_type == avl62x1_t2mi) {
			props->AVL62X1_T2MI_CTRL_PROP  = AVL62X1_T2MI_CTRL_VALID_STREAM_MASK;
			props->AVL62X1_T2MI_CTRL_PROP |= stream_info->t2mi.plp_id;
			props->AVL62X1_T2MI_CTRL_PROP |= stream_info->t2mi.pid << AVL62X1_T2MI_PID_SHIFT;
		} else {
			props->AVL62X1_T2MI_CTRL_PROP = NO_STREAM_ID_FILTER;
		}
	}

	return 0;
}

static int avl62x1_blindscan_get_streams(struct dvb_frontend *fe,
					 struct avl62x1_carrier_info *carrier)
{
	struct avl62x1_priv *priv = fe->demodulator_priv;
	struct avl62x1_bs_state *state = &priv->bs_state;
	struct avl62x1_stream_info *streams, *stream;
	uint8_t num_streams;
	uint8_t num_splp_mplps;
	uint16_t cntr;
	avl62x1_lock_status lock;
	const uint16_t timeout = 10;
	const uint32_t delay = 20;
	int i, j, ret;

	ret = avl62x1_get_num_streams(&num_streams, priv->chip);
	if (ret != AVL_EC_OK)
		return 0;

	streams = kvcalloc(num_streams, sizeof(*stream), GFP_KERNEL);
	if (!streams)
		return -ENOMEM;

	ret = avl62x1_blindscan_get_stream_list(carrier, streams, num_streams, priv->chip);
	if (ret != AVL_EC_OK)
		goto err_free_streams;

	for (i = 0; i < num_streams; i++) {
		stream = streams + i;

		if (stream->stream_type == avl62x1_t2mi) {
			ret = avl62x1_switch_stream(stream, priv->chip);

			lock = avl62x1_status_unlocked;
			cntr = 0;
			do {
				avl_bsp_delay(delay);
				ret = avl62x1_get_lock_status(&lock, priv->chip);
			} while ((lock == avl62x1_status_unlocked) && (cntr < timeout) && (ret == AVL_EC_OK));

			if (ret != AVL_EC_OK || cntr >= timeout)
				continue;

			avl62x1_get_t2mi_plp_list(&stream->t2mi.plp_list, priv->chip);

			num_splp_mplps = stream->t2mi.plp_list.list_size;
		} else {
			num_splp_mplps = 1;
		}

		for (j = 0; j < num_splp_mplps; j++) {
			struct dtv_frontend_properties *props = state->props + state->num_props;

			if (stream->stream_type == avl62x1_t2mi)
				stream->t2mi.plp_id = stream->t2mi.plp_list.list[j];

			avl62x1_blindscan_props(props, carrier, stream);

			state->num_props++;
		}
	}

err_free_streams:
	kvfree(streams);

	return 0;
}

static int avl62x1_blindscan_get_carriers(struct dvb_frontend *fe)
{
	struct avl62x1_priv *priv = fe->demodulator_priv;
	struct avl62x1_bs_state *state = &priv->bs_state;
	struct avl62x1_carrier_info *carriers, *carrier;
	uint16_t cntr;
	enum avl62x1_discovery_status status;
	const uint16_t timeout = 20;
	const uint32_t delay = 100;
	int i, ret;

	carriers = kvcalloc(state->info.num_carriers, sizeof(*carrier), GFP_KERNEL);
	if (!carriers)
		return -ENOMEM;

	ret = avl62x1_blindscan_get_carrier_list(&state->params, &state->info, carriers, priv->chip);
	if (ret != AVL_EC_OK)
		goto err_free_carriers;

	for (i = 0; i < state->info.num_carriers; i++) {
		carrier = carriers + i;

		ret = avl62x1_blindscan_confirm_carrier(&state->params, carrier, priv->chip);

		status = avl62x1_discovery_running;
		cntr = 0;
		do {
			ret = avl62x1_get_discovery_status(&status, priv->chip);
			avl_bsp_delay(delay);
			cntr++;
		} while ((status == avl62x1_discovery_running) && (cntr < timeout) && (ret == AVL_EC_OK));

		if (ret != AVL_EC_OK || cntr >= timeout)
			continue;

		ret = avl62x1_blindscan_get_streams(fe, carrier);
		if (ret)
			goto err_free_carriers;
	}

err_free_carriers:
	kvfree(carriers);

	return 0;
}

static int avl62x1_blindscan_thread(void *data)
{
	struct dvb_frontend *fe = data;
	struct dtv_frontend_properties *c = &fe->dtv_property_cache;
	struct avl62x1_priv *priv = fe->demodulator_priv;
	struct avl62x1_bs_state *state = &priv->bs_state;
	uint16_t bw_ratio, bs_min_sr;
	u32 bs_min_symbol_rate = 1000000;
	u32 frequency = state->min_prop.frequency;
	uint16_t cntr;
	const uint16_t timeout = 200;
	const uint32_t delay = 100;
	int i, j, k, ret;

	/* TODO: workaround */
	avl_bms_read16(priv->chip->chip_pub->i2c_addr,
		       c_AVL62X1_S2X_BWfilt_to_Rsamp_ratio_saddr, &bw_ratio);
	avl_bms_read16(priv->chip->chip_pub->i2c_addr,
		       c_AVL62X1_S2X_blind_scan_min_sym_rate_kHz_saddr, &bs_min_sr);

	if (state->min_prop.symbol_rate < bs_min_symbol_rate)
		state->min_prop.symbol_rate = bs_min_symbol_rate;

	while (frequency < state->max_prop.frequency) {
		if (kthread_should_stop())
			break;

		priv->chip->chip_pub->tuner->rf_freq_hz = frequency * 1000;
		priv->chip->chip_pub->tuner->lpf_hz = 0xffffffff;
		priv->chip->chip_pub->tuner->blindscan_mode = 1;
		avl62x1_optimize_carrier(priv->chip->chip_pub->tuner, NULL, priv->chip);
		c->frequency = frequency;
		c->symbol_rate = 0;
		if (fe->ops.i2c_gate_ctrl)
			fe->ops.i2c_gate_ctrl(fe, 1);
		if (fe->ops.tuner_ops.set_params)
			fe->ops.tuner_ops.set_params(fe);
		if (fe->ops.i2c_gate_ctrl)
			fe->ops.i2c_gate_ctrl(fe, 0);
		avl_bsp_delay(250);

		state->params.tuner_center_freq_100khz = frequency / 100;
		state->params.tuner_lpf_100khz = priv->chip->chip_pub->tuner->lpf_hz / 100000;
		state->params.min_symrate_khz = state->min_prop.symbol_rate / 1000;

		ret = avl62x1_blindscan_start(&state->params, priv->chip);
		if (ret != AVL_EC_OK)
			break;

		state->info.finished = 0;
		cntr = 0;
		do {
			if (kthread_should_stop()) {
				avl62x1_blindscan_cancel(priv->chip);
				break;
			}

			avl_bsp_delay(delay);
			ret = avl62x1_blindscan_get_status(&state->info, priv->chip);
			cntr++;
		} while (!state->info.finished && (cntr < timeout) && (ret == AVL_EC_OK));

		if (ret != AVL_EC_OK || cntr >= timeout) {
			frequency += (priv->chip->chip_pub->tuner->lpf_hz / 1000);
			continue;
		}

		if (state->info.num_carriers > 0) {
			ret = avl62x1_blindscan_get_carriers(fe);
			if (ret)
				break;
		}

		frequency += state->info.next_freq_step_hz / 1000;
		if (frequency > state->max_prop.frequency)
			frequency = state->max_prop.frequency;
	}

	for (i = 0; i < state->num_props; i++) {
		for (j = i + 1; j < state->num_props; j++) {
			struct dtv_frontend_properties *p = state->props + i;
			struct dtv_frontend_properties *n = state->props + j;

			if ((AVL_abs(p->frequency - n->frequency) <= 5000) &&
			    (AVL_abs(p->symbol_rate - n->symbol_rate) <= 500000) &&
			    (p->scrambling_sequence_index == n->scrambling_sequence_index) &&
			    (p->stream_id == n->stream_id) &&
			    (p->AVL62X1_T2MI_CTRL_PROP == n->AVL62X1_T2MI_CTRL_PROP)) {
				for (k = j; k < state->num_props; k++)
					*(state->props + k) = *(state->props + (k + 1));

				state->num_props--;

				j--;
			}
		}
	}

	/* TODO: workaround */
	avl_bms_write16(priv->chip->chip_pub->i2c_addr,
			c_AVL62X1_S2X_BWfilt_to_Rsamp_ratio_saddr, bw_ratio);
	avl_bms_write16(priv->chip->chip_pub->i2c_addr,
			c_AVL62X1_S2X_blind_scan_min_sym_rate_kHz_saddr, bs_min_sr);
	avl_bms_write32(priv->chip->chip_pub->i2c_addr,
			c_AVL62X1_S2X_bs_cent_freq_tuner_Hz_iaddr, 0);

	refcount_set(&state->running, 0);

	return 0;
}

static int avl62x1_blindscan_ctrl_show(struct seq_file *m, void *v)
{
	struct dvb_frontend *fe = m->private;
	struct avl62x1_priv *priv = fe->demodulator_priv;
	struct avl62x1_bs_state *state = &priv->bs_state;

	seq_printf(m, "%d %d %d \n",
		   refcount_read(&state->running),
		   state->num_props,
		   state->progress);

	return 0;
}

static int avl62x1_blindscan_ctrl_open(struct inode *inode, struct file *file)
{
	return single_open(file, avl62x1_blindscan_ctrl_show, PDE_DATA(inode));
}

static ssize_t avl62x1_blindscan_ctrl_write(struct file *file,
					    const char __user *buf,
					    size_t count, loff_t *ppos)
{
	struct dvb_frontend *fe = PDE_DATA(file_inode(file));
	struct avl62x1_priv *priv = fe->demodulator_priv;
	struct avl62x1_bs_state *state = &priv->bs_state;
	u32 val[5];
	char *p;

	p = memdup_user_nul(buf, count);
	if (IS_ERR(p))
		return PTR_ERR(p);

	if (sscanf(p, "%u%u%u%u%u", &val[0], &val[1], &val[2], &val[3], &val[4]) != 5) {
		kfree(p);
		return -EINVAL;
	}

	kfree(p);

	if (val[0]) {
		if (val[1] < 950 || val[2] > 2150 || val[3] < 1 || val[4] > 60)
			return -EINVAL;

		if (signal_pending(current))
			return -EINTR;

		mb();

		if (refcount_read(&state->running))
			return -EBUSY;

		refcount_set(&state->running, 1);

		state->min_prop.frequency = val[1] * 1000;
		state->max_prop.frequency = val[2] * 1000;
		state->min_prop.symbol_rate = val[3] * 1000000;
		state->max_prop.symbol_rate = val[4] * 1000000;
		state->progress = 0;
		state->index = 0;

		memset(state->props, 0, array_size(1000, sizeof(struct dtv_frontend_properties)));
		state->num_props = 0;

		state->thread = kthread_run(avl62x1_blindscan_thread, fe, "blindscand");
		if (IS_ERR(state->thread)) {
			refcount_set(&state->running, 0);
			return PTR_ERR(state->thread);
		}
	} else {
		if (refcount_read(&state->running))
			kthread_stop(state->thread);
	}

	return count;
}

static struct proc_ops avl62x1_blindscan_ctrl_fops = {
	.proc_open = avl62x1_blindscan_ctrl_open,
	.proc_read = seq_read,
	.proc_lseek = seq_lseek,
	.proc_release = single_release,
	.proc_write = avl62x1_blindscan_ctrl_write,
};

static int avl62x1_blindscan_info_show(struct seq_file *m, void *v)
{
	struct dvb_frontend *fe = m->private;
	struct avl62x1_priv *priv = fe->demodulator_priv;
	struct avl62x1_bs_state *state = &priv->bs_state;
	struct dtv_frontend_properties *props = state->props + state->index;
	u32 t2mi_plp_id;
	u32 t2mi_pid;

	if (props->AVL62X1_T2MI_CTRL_PROP != NO_STREAM_ID_FILTER &&
	    props->AVL62X1_T2MI_CTRL_PROP & AVL62X1_T2MI_CTRL_VALID_STREAM_MASK) {
		t2mi_plp_id = props->AVL62X1_T2MI_CTRL_PROP & 0xff;
		t2mi_pid = (props->AVL62X1_T2MI_CTRL_PROP >> AVL62X1_T2MI_PID_SHIFT) & 0x1fff;
	} else {
		t2mi_plp_id = NO_STREAM_ID_FILTER;
		t2mi_pid = 0x1000;
	}

	seq_printf(m, "%d %u %u %d %d %d %d %d %d %d %d %d %d %d \n",
		   state->index,
		   props->frequency,
		   props->symbol_rate,
		   props->delivery_system,
		   INVERSION_AUTO,
		   props->pilot,
		   props->fec_inner,
		   props->modulation,
		   props->rolloff,
		   1,
		   props->stream_id,
		   props->scrambling_sequence_index,
		   t2mi_plp_id,
		   t2mi_pid);

	return 0;
}

static int avl62x1_blindscan_info_open(struct inode *inode, struct file *file)
{
	return single_open(file, avl62x1_blindscan_info_show, PDE_DATA(inode));
}

static ssize_t avl62x1_blindscan_info_write(struct file *file,
					    const char __user *buf,
					    size_t count, loff_t *ppos)
{
	struct dvb_frontend *fe = PDE_DATA(file_inode(file));
	struct avl62x1_priv *priv = fe->demodulator_priv;
	struct avl62x1_bs_state *state = &priv->bs_state;
	int val;
	int ret;

	ret = kstrtoint_from_user(buf, count, 10, &val);
	if (ret)
		return ret;

	if (val < 0 || val >= state->num_props)
		return -EINVAL;

	state->index = val;

	return count;
}

static struct proc_ops avl62x1_blindscan_info_fops = {
	.proc_open = avl62x1_blindscan_info_open,
	.proc_read = seq_read,
	.proc_lseek = seq_lseek,
	.proc_release = single_release,
	.proc_write = avl62x1_blindscan_info_write,
};

static int avl62x1_t2mi_show(struct seq_file *m, void *v)
{
	return 0;
}

int avl62x1_blindscan_attach(struct dvb_frontend *fe,
			     struct proc_dir_entry *root_dir)
{
	if (!proc_create_data("bs_ctrl", S_IRUGO | S_IWUSR, root_dir,
			      &avl62x1_blindscan_ctrl_fops, fe))
		return -ENOMEM;

	if (!proc_create_data("bs_info", S_IRUGO | S_IWUSR, root_dir,
			      &avl62x1_blindscan_info_fops, fe))
		return -ENOMEM;

	if (!proc_create_single_data("t2mi", S_IRUGO | S_IWUSR, root_dir,
				     avl62x1_t2mi_show, fe))
		return -ENOMEM;

	return 0;
}
EXPORT_SYMBOL(avl62x1_blindscan_attach);

static void avl62x1_release(struct dvb_frontend *fe)
{
	struct avl62x1_priv *priv = fe->demodulator_priv;
	struct avl62x1_bs_state *state = &priv->bs_state;

	vfree(state->props);
	kfree(priv->chip->chip_pub);
	kfree(priv->chip->chip_priv);
	kfree(priv->chip);
	kfree(priv);
}

static int avl62x1_init(struct dvb_frontend *fe)
{
	struct avl62x1_priv *priv = fe->demodulator_priv;

	priv->tune_state = avl62x1_tune_state_idle;

	return 0;
}

static int avl62x1_sleep(struct dvb_frontend *fe)
{
	return 0;
}

static int avl62x1_tune(struct dvb_frontend *fe, bool re_tune,
			unsigned int mode_flags, unsigned int *delay,
			enum fe_status *status)
{
	struct dtv_frontend_properties *c = &fe->dtv_property_cache;
	struct avl62x1_priv *priv = fe->demodulator_priv;
	struct avl62x1_bs_state *state = &priv->bs_state;
	struct avl62x1_carrier_info carrier_info;
	struct avl62x1_stream_info stream_info;
	avl62x1_lock_status lock;
	uint16_t sig_strength;
	int16_t snr_db_x100, sqi;
	u32 tuner_status;
	int ret;

	if (refcount_read(&state->running)) {
		priv->tune_state = avl62x1_tune_state_idle;
		*delay = 3 * HZ;
		*status = 0;
		return 0;
	}

	if (re_tune) {
		priv->tune_state = avl62x1_tune_state_tuner_trylock;

		if (c->symbol_rate < 1000)
			priv->tune_state = avl62x1_tune_state_idle;
	}

	switch (priv->tune_state) {
	case avl62x1_tune_state_idle:
		*delay  = 3 * HZ;
		*status = 0;
		break;
	case avl62x1_tune_state_tuner_trylock:
		if (fe->ops.i2c_gate_ctrl)
			fe->ops.i2c_gate_ctrl(fe, 1);
		if (fe->ops.tuner_ops.set_params)
			fe->ops.tuner_ops.set_params(fe);
		if (fe->ops.i2c_gate_ctrl)
			fe->ops.i2c_gate_ctrl(fe, 0);

		priv->tune_timeout = 250;
		*delay  = HZ / 100;
		*status = 0;
		priv->tune_state = avl62x1_tune_state_tuner_status;
		break;
	case avl62x1_tune_state_tuner_status:
		if (fe->ops.i2c_gate_ctrl)
			fe->ops.i2c_gate_ctrl(fe, 1);
		if (fe->ops.tuner_ops.get_status)
			fe->ops.tuner_ops.get_status(fe, &tuner_status);
		if (fe->ops.i2c_gate_ctrl)
			fe->ops.i2c_gate_ctrl(fe, 0);
		if (tuner_status == TUNER_STATUS_LOCKED) {
			*delay  = HZ / 100;
			*status = 0;
			priv->tune_state = avl62x1_tune_state_demod_trylock;
			break;
		}

		if (--priv->tune_timeout > 0) {
			*delay  = HZ / 100;
			*status = 0;
		} else {
			*delay  = HZ;
			*status = FE_TIMEDOUT;
		}
		break;
	case avl62x1_tune_state_demod_trylock:
		if (c->stream_id != NO_STREAM_ID_FILTER &&
		    c->stream_id & 0xfffff00) {
			c->scrambling_sequence_index = (c->stream_id >> 8) & 0x3ffff;
			if (((c->stream_id >> 26) & 0x3) == 0)
				__avl62x1_conv_xlfsr_state_to_n(c->scrambling_sequence_index, &c->scrambling_sequence_index);
		}

		carrier_info.rf_freq_khz = c->frequency;
		carrier_info.carrier_freq_offset_hz = 0;
		carrier_info.symbol_rate_hz = c->symbol_rate;
		if (c->scrambling_sequence_index)
			carrier_info.pl_scrambling = c->scrambling_sequence_index;
		else
			carrier_info.pl_scrambling = AVL62X1_PL_SCRAM_AUTO;

		stream_info.carrier_idx = 0;
		if (c->stream_id != NO_STREAM_ID_FILTER)
			stream_info.isi = c->stream_id & 0xff;
		else
			stream_info.isi = 0;
		if (c->AVL62X1_T2MI_CTRL_PROP != NO_STREAM_ID_FILTER &&
		    c->AVL62X1_T2MI_CTRL_PROP & AVL62X1_T2MI_CTRL_VALID_STREAM_MASK) {
			stream_info.stream_type = avl62x1_t2mi;
			stream_info.t2mi.plp_id = c->AVL62X1_T2MI_CTRL_PROP & 0xff;
			stream_info.t2mi.raw_mode = 0;
			stream_info.t2mi.pid_autodiscover = 0;
			stream_info.t2mi.pid = (c->AVL62X1_T2MI_CTRL_PROP >> AVL62X1_T2MI_PID_SHIFT) & 0x1fff;
		} else {
			stream_info.stream_type = avl62x1_transport;
			stream_info.t2mi.plp_id = 0;
			stream_info.t2mi.raw_mode = 0;
			stream_info.t2mi.pid_autodiscover = 0;
			stream_info.t2mi.pid = 0x1000;
		}

		ret = avl62x1_lock_tp(&carrier_info, &stream_info, AVL_FALSE,
				      priv->chip);

		priv->tune_timeout = 250;
		*delay  = HZ / 100;
		*status = 0;
		priv->tune_state = avl62x1_tune_state_demod_status;
		break;
	case avl62x1_tune_state_demod_status:
		ret = avl62x1_get_lock_status(&lock, priv->chip);
		if (ret == AVL_EC_OK && lock == avl62x1_status_locked) {
			*delay = HZ / 5;
			*status = FE_HAS_SIGNAL | FE_HAS_CARRIER |
				  FE_HAS_VITERBI | FE_HAS_SYNC | FE_HAS_LOCK;
			priv->tune_state = avl62x1_tune_state_read_status;
			break;
		}

		if (--priv->tune_timeout > 0) {
			*delay  = HZ / 100;
			*status = 0;
		} else {
			*delay  = HZ;
			*status = FE_TIMEDOUT;
		}
		break;
	case avl62x1_tune_state_read_status:
		fe->ops.read_status(fe, status);

		if (*status & FE_HAS_SIGNAL) {
			ret = avl62x1_get_signal_strength(&sig_strength, priv->chip);

			c->strength.len = 2;
			c->strength.stat[0].scale = FE_SCALE_DECIBEL;
			c->strength.stat[0].svalue = -80 + sig_strength / 2;
			c->strength.stat[1].scale = FE_SCALE_RELATIVE;
			c->strength.stat[1].uvalue = (sig_strength * 0xffff) / 100;
			if (c->strength.stat[1].uvalue > 0xffff)
				c->strength.stat[1].uvalue = 0xffff;
		} else {
			c->strength.len = 1;
			c->strength.stat[0].scale = FE_SCALE_NOT_AVAILABLE;
		}

		if (*status & FE_HAS_VITERBI) {
			ret = avl62x1_get_snr(&snr_db_x100, priv->chip);

			c->cnr.len = 2;
			c->cnr.stat[0].scale = FE_SCALE_DECIBEL;
			c->cnr.stat[0].svalue = snr_db_x100 * 10;
			c->cnr.stat[1].scale = FE_SCALE_RELATIVE;
			/* max SNR is about 15dB */
			if (snr_db_x100 >= 1500)
				sqi = 100;
			else if (snr_db_x100 <= 0)
				sqi = 10;
			else
				sqi = snr_db_x100 * 90 / 1500 + 10;
			c->cnr.stat[1].uvalue = (sqi * 0xffff) / 100;
			if (c->cnr.stat[1].uvalue > 0xffff)
				c->cnr.stat[1].uvalue = 0xffff;
		} else {
			c->cnr.len = 1;
			c->cnr.stat[0].scale = FE_SCALE_NOT_AVAILABLE;
		}

		*delay = HZ;
		break;
	}

	return 0;
}

static enum dvbfe_algo avl62x1_get_frontend_algo(struct dvb_frontend *fe)
{
	return DVBFE_ALGO_HW;
}

static int avl62x1_get_frontend(struct dvb_frontend *fe,
				struct dtv_frontend_properties *c)
{
	struct avl62x1_priv *priv = fe->demodulator_priv;
	struct avl62x1_bs_state *state = &priv->bs_state;
	avl62x1_lock_status lock;
	struct avl62x1_carrier_info carrier_info;
	struct avl62x1_stream_info stream_info;
	int ret;

	if (refcount_read(&state->running))
		return 0;

	ret = avl62x1_get_lock_status(&lock, priv->chip);

	if (ret == AVL_EC_OK && lock == avl62x1_status_locked) {
		ret = avl62x1_get_signal_info(&carrier_info, priv->chip);
		ret = avl62x1_get_stream_info(&stream_info, priv->chip);

		c->symbol_rate = carrier_info.symbol_rate_hz;

		if (carrier_info.spectrum_invert == avl62x1_specpol_inverted)
			c->inversion = INVERSION_ON;
		else
			c->inversion = INVERSION_OFF;

		if (carrier_info.signal_type == avl62x1_dvbs) {
			c->modulation = QPSK;

			c->pilot = PILOT_OFF;

			switch (carrier_info.code_rate.dvbs_code_rate) {
			case avl62x1_dvbs_cr_1_2:
				c->fec_inner = FEC_1_2;
				break;
			case avl62x1_dvbs_cr_2_3:
				c->fec_inner = FEC_2_3;
				break;
			case avl62x1_dvbs_cr_3_4:
				c->fec_inner = FEC_3_4;
				break;
			case avl62x1_dvbs_cr_5_6:
				c->fec_inner = FEC_5_6;
				break;
			case avl62x1_dvbs_cr_7_8:
				c->fec_inner = FEC_7_8;
				break;
			default:
				c->fec_inner = FEC_AUTO;
				break;
			}

			c->rolloff = ROLLOFF_35;
		} else if (carrier_info.signal_type == avl62x1_dvbs2) {
			if (carrier_info.pilot == avl62x1_pilot_on)
				c->pilot = PILOT_ON;
			else
				c->pilot = PILOT_OFF;

			switch (carrier_info.modulation) {
			case avl62x1_qpsk:
				c->modulation = QPSK;
				break;
			case avl62x1_8psk:
				c->modulation = PSK_8;
				break;
			case avl62x1_16apsk:
				c->modulation = APSK_16;
				break;
			default:
				c->modulation = APSK_32;
				break;
			}

			switch (carrier_info.code_rate.dvbs2_code_rate) {
			case avl62x1_dvbs2_cr_2_5:
				c->fec_inner = FEC_2_5;
				break;
			case avl62x1_dvbs2_cr_1_2:
				c->fec_inner = FEC_1_2;
				break;
			case avl62x1_dvbs2_cr_3_5:
				c->fec_inner = FEC_3_5;
				break;
			case avl62x1_dvbs2_cr_2_3:
				c->fec_inner = FEC_2_3;
				break;
			case avl62x1_dvbs2_cr_3_4:
				c->fec_inner = FEC_3_4;
				break;
			case avl62x1_dvbs2_cr_4_5:
				c->fec_inner = FEC_4_5;
				break;
			case avl62x1_dvbs2_cr_5_6:
				c->fec_inner = FEC_5_6;
				break;
			case avl62x1_dvbs2_cr_8_9:
				c->fec_inner = FEC_8_9;
				break;
			case avl62x1_dvbs2_cr_9_10:
				c->fec_inner = FEC_9_10;
				break;
			default:
				c->fec_inner = FEC_AUTO;
				break;
			}

			switch (carrier_info.roll_off) {
			case avl62x1_rolloff_35:
				c->rolloff = ROLLOFF_35;
				break;
			case avl62x1_rolloff_25:
				c->rolloff = ROLLOFF_25;
				break;
			case avl62x1_rolloff_20:
				c->rolloff = ROLLOFF_20;
				break;
			default:
				c->rolloff = ROLLOFF_AUTO;
				break;
			}

			if (carrier_info.sis_mis != avl62x1_mis)
				c->stream_id = NO_STREAM_ID_FILTER;

			if (stream_info.stream_type != avl62x1_t2mi)
				c->AVL62X1_T2MI_CTRL_PROP = NO_STREAM_ID_FILTER;
		}
	}

	return 0;
}

static int avl62x1_read_status(struct dvb_frontend *fe, enum fe_status *status)
{
	struct avl62x1_priv *priv = fe->demodulator_priv;
	struct avl62x1_bs_state *state = &priv->bs_state;
	avl62x1_lock_status lock;
	int ret;

	*status = 0;

	if (refcount_read(&state->running))
		return 0;

	if (priv->tune_state == avl62x1_tune_state_read_status) {
		ret = avl62x1_get_lock_status(&lock, priv->chip);

		if (ret == AVL_EC_OK && lock == avl62x1_status_locked) {
			*status = FE_HAS_SIGNAL | FE_HAS_CARRIER |
				  FE_HAS_VITERBI | FE_HAS_SYNC | FE_HAS_LOCK;
		}
	}

	return 0;
}

static int avl62x1_read_ber(struct dvb_frontend *fe, u32 *ber)
{
	*ber = 0;

	return 0;
}

static int avl62x1_read_signal_strength(struct dvb_frontend *fe,
					u16 *signal_strength)
{
	struct dtv_frontend_properties *c = &fe->dtv_property_cache;
	int i;

	*signal_strength = 0;

	for (i = 0; i < c->strength.len; i++)
		if (c->strength.stat[i].scale == FE_SCALE_RELATIVE)
			*signal_strength = (uint16_t)c->strength.stat[i].uvalue;

	return 0;
}

static int avl62x1_read_snr(struct dvb_frontend *fe, u16 *snr)
{
	struct dtv_frontend_properties *c = &fe->dtv_property_cache;
	int i;

	*snr = 0;

	for (i = 0; i < c->cnr.len; i++)
		if (c->cnr.stat[i].scale == FE_SCALE_RELATIVE)
			*snr = (uint16_t)c->cnr.stat[i].uvalue;

	return 0;
}

static int avl62x1_diseqc_send_master_cmd(struct dvb_frontend *fe,
					  struct dvb_diseqc_master_cmd *d)
{
	struct avl62x1_priv *priv = fe->demodulator_priv;
	struct avl62x1_diseqc_tx_status tx_status = { .tx_complete = 0 };
	int ret;

	ret = avl62x1_send_diseqc_data(d->msg, d->msg_len, priv->chip);
	if (ret != AVL_EC_OK)
		return -EIO;

	do {
		usleep_range(4000, 5000);
		ret = avl62x1_get_diseqc_tx_status(&tx_status, priv->chip);
	} while (tx_status.tx_complete != 1);
	if (ret != AVL_EC_OK)
		return -EIO;

	return 0;
}

static int avl62x1_diseqc_recv_slave_reply(struct dvb_frontend *fe,
					   struct dvb_diseqc_slave_reply *d)
{
	return 0;
}

static int avl62x1_diseqc_send_burst(struct dvb_frontend *fe,
				     enum fe_sec_mini_cmd burst)
{
	struct avl62x1_priv *priv = fe->demodulator_priv;
	int ret;
	uint8_t tone = burst == SEC_MINI_A ? 0 : 1;
	uint8_t count = 8;

	ret = avl62x1_send_diseqc_tone(tone, count, priv->chip);
	if (ret != AVL_EC_OK)
		return -EIO;

	return 0;
}

static int avl62x1_set_tone(struct dvb_frontend *fe, enum fe_sec_tone_mode tone)
{
	struct avl62x1_priv *priv = fe->demodulator_priv;
	int ret = 0;

	if (tone == SEC_TONE_ON) {
		if (priv->chip->chip_priv->diseqc_op_status !=
		    avl62x1_dos_continuous)
			ret = avl62x1_diseqc_tone_on(priv->chip);
	} else {
		if (priv->chip->chip_priv->diseqc_op_status ==
		    avl62x1_dos_continuous)
			ret = avl62x1_diseqc_tone_off(priv->chip);
	}
	if (ret != AVL_EC_OK)
		return -EIO;

	return 0;
}

static int avl62x1_set_voltage(struct dvb_frontend *fe,
			       enum fe_sec_voltage voltage)
{
	struct avl62x1_priv *priv = fe->demodulator_priv;
	avl62x1_gpio_pin_value enable;
	avl62x1_gpio_pin_value sel;
	int ret;

	switch (voltage) {
	case SEC_VOLTAGE_13:
		enable = avl62x1_gpio_value_logic_1;
		sel = avl62x1_gpio_value_logic_0;
		break;
	case SEC_VOLTAGE_18:
		enable = avl62x1_gpio_value_logic_1;
		sel = avl62x1_gpio_value_logic_1;
		break;
	case SEC_VOLTAGE_OFF:
		enable = avl62x1_gpio_value_logic_0;
		sel = avl62x1_gpio_value_logic_0;
		break;
	}

	ret = avl62x1_set_gpio_value(avl62x1_gpio_pin_lnb_pwr_en, enable,
				     priv->chip);
	if (ret != AVL_EC_OK)
		return -EIO;

	ret = avl62x1_set_gpio_value(avl62x1_gpio_pin_lnb_pwr_sel, sel,
				     priv->chip);
	if (ret != AVL_EC_OK)
		return -EIO;

	return 0;
}

static int avl62x1_i2c_gate_ctrl(struct dvb_frontend *fe, int enable)
{
	struct avl62x1_priv *priv = fe->demodulator_priv;
	int ret;

	if (enable)
		ret = avl62x1_enable_tuner_i2c(priv->chip);
	else
		ret = avl62x1_disable_tuner_i2c(priv->chip);
	if (ret != AVL_EC_OK)
		return -EIO;

	return 0;
}

static struct dvb_frontend_ops avl62x1_ops = {
	.info = {
		.name = "Availink AVL6261",
		.frequency_min_hz =  950 * MHz,
		.frequency_max_hz = 2150 * MHz,
		.frequency_stepsize_hz =  1011 * kHz,
		.frequency_tolerance_hz = 5 * MHz,
		.symbol_rate_min = 0,
		.symbol_rate_max = 60000000,
		.caps = FE_CAN_INVERSION_AUTO |
			FE_CAN_FEC_1_2 |
			FE_CAN_FEC_2_3 |
			FE_CAN_FEC_3_4 |
			FE_CAN_FEC_4_5 |
			FE_CAN_FEC_5_6 |
			FE_CAN_FEC_6_7 |
			FE_CAN_FEC_7_8 |
			FE_CAN_FEC_8_9 |
			FE_CAN_FEC_AUTO |
			FE_CAN_QPSK |
			FE_CAN_TRANSMISSION_MODE_AUTO |
			FE_CAN_GUARD_INTERVAL_AUTO |
			FE_CAN_HIERARCHY_AUTO |
			FE_CAN_MULTISTREAM |
			FE_CAN_2G_MODULATION |
			FE_CAN_RECOVER
	},

	.delsys = { SYS_DVBS2, SYS_DVBS, },

	.release = avl62x1_release,
	.init = avl62x1_init,
	.sleep = avl62x1_sleep,
	.tune = avl62x1_tune,
	.get_frontend_algo = avl62x1_get_frontend_algo,
	.get_frontend = avl62x1_get_frontend,

	.read_status = avl62x1_read_status,
	.read_ber = avl62x1_read_ber,
	.read_signal_strength = avl62x1_read_signal_strength,
	.read_snr = avl62x1_read_snr,

	.diseqc_send_master_cmd = avl62x1_diseqc_send_master_cmd,
	.diseqc_recv_slave_reply = avl62x1_diseqc_recv_slave_reply,
	.diseqc_send_burst = avl62x1_diseqc_send_burst,
	.set_tone = avl62x1_set_tone,
	.set_voltage = avl62x1_set_voltage,

	.i2c_gate_ctrl = avl62x1_i2c_gate_ctrl,
};

static struct avl_tuner av201x_avl_tuner = {
	.blindscan_mode = 0,
	.more_params = NULL,
	.initialize = NULL,
	.lock = NULL,
	.get_lock_status = NULL,
	.get_rf_strength = NULL,
	.get_max_lpf = &av201x_get_max_lpf,
	.get_min_lpf = &av201x_get_min_lpf,
	.get_lpf_step_size = &av201x_get_lpf_step_size,
	.get_agc_slope = &av201x_get_agc_slope,
	.get_min_gain_voltage = &av201x_get_min_gain_voltage,
	.get_max_gain_voltage = &av201x_get_max_gain_voltage,
	.get_rf_freq_step_size = &av201x_get_rf_freq_step_size,
};

static struct av201x_config av201x_avl_config = {
	.i2c_address = 0x62,
	.id = ID_AV2018,
	.xtal_freq = 27000,
};

static int avl62x1_get_firmware(struct dvb_frontend *fe)
{
	unsigned int fw_maj, fw_min, fw_build;
	int fw_status;
	struct avl62x1_priv *priv = fe->demodulator_priv;

	fw_status = request_firmware(&priv->fw, AVL62X1_FIRMWARE, priv->i2c->dev.parent);
	if (fw_status != 0) {
		pr_info("firmware file %s not found", AVL62X1_FIRMWARE);
		return fw_status;
	}

	priv->chip->chip_priv->patch_data = (unsigned char *)(priv->fw->data);

	fw_maj = priv->chip->chip_priv->patch_data[24]; //major rev
	fw_min = priv->chip->chip_priv->patch_data[25]; //SDK-FW API rev
	fw_build = (priv->chip->chip_priv->patch_data[26] << 8) |
			priv->chip->chip_priv->patch_data[27]; //internal rev
	if (fw_min != 8) {
		pr_info("Firmware version %d.%d.%d incompatible with this driver version", fw_maj, fw_min, fw_build);
		pr_info("Firmware minor version must be %d", 8);
		release_firmware(priv->fw);
		return 1;
	}

	return 0;
}

struct dvb_frontend *avl62x1_attach(struct avl62x1_config *config,
				    struct i2c_adapter *i2c)
{
	struct avl62x1_priv *priv;
	struct avl62x1_ver_info ver_info;
	struct avl62x1_diseqc_params params;
	struct avl62x1_error_stats_config err_stats_config;
	struct avl62x1_ber_config ber_config;

	priv = kzalloc(sizeof(*priv), GFP_KERNEL);
	if (!priv)
		goto err;

	priv->chip = kzalloc(sizeof(*priv->chip), GFP_KERNEL);
	if (!priv->chip)
		goto err_free_priv;

	priv->chip->chip_priv = kzalloc(sizeof(*priv->chip->chip_priv), GFP_KERNEL);
	if (!priv->chip->chip_priv)
		goto err_free_chip;

	priv->chip->chip_pub = kzalloc(sizeof(*priv->chip->chip_pub), GFP_KERNEL);
	if (!priv->chip->chip_pub)
		goto err_free_chip_priv;

	priv->frontend.demodulator_priv = priv;
	priv->frontend.ops = avl62x1_ops;
	priv->i2c = i2c;

	memcpy(priv->chip->chip_pub, config->chip_pub,
	       sizeof(struct avl62x1_chip_pub));

	priv->chip->chip_pub->tuner = &av201x_avl_tuner;

	avl_bsp_assoc_i2c_adapter(priv->chip->chip_pub->i2c_addr, i2c);

	avl62x1_init_chip_object(priv->chip);

	if (avl62x1_get_firmware(&priv->frontend))
		goto err_free_chip_pub;

	avl_bsp_reset();

	avl62x1_initialize(priv->chip);

	avl62x1_get_version(&ver_info, priv->chip);
	pr_info("Firmware Version %d.%d.%d (%d.%d.%d)",
		ver_info.firmware.major, ver_info.firmware.minor, ver_info.firmware.build,
		ver_info.driver.major, ver_info.driver.minor, ver_info.driver.build);

	params.tone_freq_khz = 22;
	params.tx_gap = avl62x1_dtxg_15ms;
	params.tx_waveform = avl62x1_dwm_normal;
	params.rx_timeout = avl62x1_drt_150ms;
	params.rx_waveform = avl62x1_dwm_normal;
	avl62x1_init_diseqc(&params, priv->chip);

	err_stats_config.error_stats_mode = avl62x1_error_stats_auto;
	err_stats_config.auto_error_stats_type = avl62x1_error_stats_time;
	err_stats_config.time_threshold_ms = 3000;
	err_stats_config.bytes_threshold = 0;
	avl62x1_config_error_stats(&err_stats_config, priv->chip);

	ber_config.test_pattern = avl62x1_test_lfsr_23;
	ber_config.fb_inversion = avl62x1_lfsr_fb_inverted;
	ber_config.lfsr_sync = 0;
	ber_config.lfsr_start_pos = 4;
	avl62x1_reset_ber(&ber_config, priv->chip);

	avl62x1_set_voltage(&priv->frontend, SEC_VOLTAGE_OFF);

	priv->bs_state.props = vmalloc(array_size(1000, sizeof(struct dtv_frontend_properties)));
	if (!priv->bs_state.props)
		goto err_release_firmware;

	if (!av201x_attach(&priv->frontend, &av201x_avl_config, i2c))
		goto err_free_props;

	return &priv->frontend;

err_free_props:
	vfree(priv->bs_state.props);
err_release_firmware:
	release_firmware(priv->fw);
err_free_chip_pub:
	kfree(priv->chip->chip_pub);
err_free_chip_priv:
	kfree(priv->chip->chip_priv);
err_free_chip:
	kfree(priv->chip);
err_free_priv:
	kfree(priv);
err:
	return NULL;
}
EXPORT_SYMBOL(avl62x1_attach);

MODULE_DESCRIPTION("Availink AVL62X1 DVB-S/S2/S2X demodulator driver");
MODULE_AUTHOR("Availink, Inc. (gpl@availink.com)");
MODULE_LICENSE("GPL");
MODULE_VERSION(AVL62X1_VERSION);
MODULE_FIRMWARE(AVL62X1_FIRMWARE);
