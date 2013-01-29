/*
 *  linux/drivers/mmc/core/core.h
 *
 *  Copyright (C) 2003 Russell King, All Rights Reserved.
 *  Copyright 2007 Pierre Ossman
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#ifndef _MMC_CORE_CORE_H
#define _MMC_CORE_CORE_H

#include <linux/delay.h>

#define MMC_CMD_RETRIES        3

struct mmc_bus_ops {
	int (*awake)(struct mmc_host *);
	int (*sleep)(struct mmc_host *);
	void (*remove)(struct mmc_host *);
	void (*detect)(struct mmc_host *);
	int (*suspend)(struct mmc_host *);
	int (*resume)(struct mmc_host *);
	int (*power_save)(struct mmc_host *);
	int (*power_restore)(struct mmc_host *);
	int (*alive)(struct mmc_host *);
};

void mmc_attach_bus(struct mmc_host *host, const struct mmc_bus_ops *ops);
void mmc_detach_bus(struct mmc_host *host);

void mmc_init_erase(struct mmc_card *card);

void mmc_set_chip_select(struct mmc_host *host, int mode);
void mmc_set_clock(struct mmc_host *host, unsigned int hz);
void mmc_gate_clock(struct mmc_host *host);
void mmc_ungate_clock(struct mmc_host *host);
void mmc_set_ungated(struct mmc_host *host);
void mmc_set_bus_mode(struct mmc_host *host, unsigned int mode);
void mmc_set_bus_width(struct mmc_host *host, unsigned int width);
u32 mmc_select_voltage(struct mmc_host *host, u32 ocr);
int mmc_set_signal_voltage(struct mmc_host *host, int signal_voltage,
			   bool cmd11);
void mmc_set_timing(struct mmc_host *host, unsigned int timing);
void mmc_set_driver_type(struct mmc_host *host, unsigned int drv_type);
void mmc_power_off(struct mmc_host *host);

static inline void mmc_delay(unsigned int ms)
{
	if (ms < 1000 / HZ) {
		cond_resched();
		mdelay(ms);
	} else {
		msleep(ms);
	}
}

void mmc_rescan(struct work_struct *work);
void mmc_start_host(struct mmc_host *host);
void mmc_stop_host(struct mmc_host *host);

int _mmc_detect_card_removed(struct mmc_host *host);

int mmc_attach_mmc(struct mmc_host *host);
int mmc_attach_sd(struct mmc_host *host);
int mmc_attach_sdio(struct mmc_host *host);

/* Module parameters */
extern bool use_spi_crc;

/* Debugfs information for hosts and cards */
void mmc_add_host_debugfs(struct mmc_host *host);
void mmc_remove_host_debugfs(struct mmc_host *host);

void mmc_add_card_debugfs(struct mmc_card *card);
void mmc_remove_card_debugfs(struct mmc_card *card);

void mmc_init_context_info(struct mmc_host *host);

#if defined(CONFIG_BLK_DEV_IO_TRACE)

extern void mmc_add_trace(unsigned int type, struct mmc_queue_req *mqrq);
#define mmc_add_trace_msg(q, fmt, ...)				\
	do {							\
		struct blk_trace *bt;				\
		if (likely(q)) {				\
			bt = (q)->blk_trace;			\
			if (unlikely(bt))			\
				__trace_note_message(bt,	\
					fmt, ##__VA_ARGS__);	\
		}						\
	} while (0)

enum mmc_trace_act {
	__MMC_TA_EAO_REV_DONE,		/* completion done of previous op */
	__MMC_TA_EAO_FETCH_NEW_REQ,	/* fetch by arrived new request */
	__MMC_TA_EAO_FETCH_REQ,		/* fetch by normal queue thread */
};

struct mmc_trace {
	u32		ta_type;	/* action type */
	char		ta_info[3];	/* action info to output */
	u32		cnt_async;	/* info for counting req of req list */
	u32		cnt_sync;
};

#else
#define mmc_add_trace(type, mqrq)		do {} while (0)
#define mmc_add_trace_msg(q, fmt, ...)		do {} while (0)

#endif /* CONFIG_BLK_DEV_IO_TRACE */

#endif
