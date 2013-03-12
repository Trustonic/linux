/*
 *  linux/drivers/mmc/core/mmc_trace.c
 *
 *  Copyright (C) 2013 Samsung Electronics Co., Ltd. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/tracepoint.h>
#include <linux/types.h>
#include <linux/blkdev.h>
#include <linux/blktrace_api.h>
#include <linux/module.h>
#include <linux/mmc/mmc_trace.h>
#include <linux/mmc/core.h>
#include <linux/mmc/host.h>
#include "../card/queue.h"

#ifdef CONFIG_BLK_DEV_IO_TRACE
static const struct {
	const char *act[2];
	/* This function is to action something for preparation */
	int	   (*prefunc)(void);
} ta_info[] = {
	[__MMC_TA_ASYNC_REQ_FETCH] = {{ "eN", "fetch_async_reqest" }, NULL },
	[__MMC_TA_REQ_FETCH] = {{ "eF", "fetch_request" }, NULL },
	[__MMC_TA_REQ_DONE] = {{ "eC", "request_done" }, NULL },
	[__MMC_TA_PRE_DONE] = {{ "eP", "prepare_done" }, NULL },
	[__MMC_TA_MMC_ISSUE] = {{ "eI", "cmd_issue" }, NULL },
	[__MMC_TA_MMC_DONE] = {{ "eD", "cmd_done" }, NULL },
	[__MMC_TA_MMC_DMA_DONE] = {{ "eA", "dma_done" }, NULL },
};

static inline struct request *__get_request(struct mmc_queue_req *mqrq)
{
	return mqrq ? mqrq->req : NULL;
}

static inline struct request_queue *__get_request_queue(struct request *req)
{
	return req ? req->q : NULL;
}

void mmc_add_trace(unsigned int type, void *m)
{
	struct request *req;
	struct request_queue *q;
	struct blk_trace *bt;
	struct mmc_trace mt;
	struct request_list *rl;
	struct mmc_queue_req *mqrq = (struct mmc_queue_req *)m;

	req = __get_request(mqrq);
	if (unlikely(!req))
		return;

	q = __get_request_queue(req);
	if (unlikely(!q))
		return;

	bt = q->blk_trace;

	if (likely(!bt) || unlikely(bt->trace_state != Blktrace_running))
		return;

	/* trace type */
	mt.ta_type = type;

	memcpy(mt.ta_info, ta_info[type].act[0], 3);

	/* request information */
	rl = &q->rq;

	/* async, sync info update for trace */
	mt.cnt_sync = rl->count[BLK_RW_SYNC];
	mt.cnt_async = rl->count[BLK_RW_ASYNC];

	/* add trace point */
	blk_add_driver_data(q, req, &mt, sizeof(struct mmc_trace));
}
EXPORT_SYMBOL(mmc_add_trace);
#endif /* CONFIG_BLK_DEV_IO_TRACE */

