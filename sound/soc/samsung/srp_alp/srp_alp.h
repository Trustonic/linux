#ifndef __SRP_ALP_H
#define __SRP_ALP_H

#define SRP_DEV_MINOR	(250)

/* Base address */
#define SRP_IRAM_BASE	(0x02020000)
#define SRP_DMEM_BASE	(0x03000000)

/* IRAM Size for exnos4x12 */
#define IRAM_SIZE	((soc_is_exynos4412() || soc_is_exynos4212()) ? \
			(0x40000) : (0x20000))

/* SRAM information */
#if defined(CONFIG_ARCH_EXYNOS4)
#define DMEM_SIZE	(0x20000)
#define ICACHE_SIZE	(0x10000)
#elif defined(CONFIG_ARCH_EXYNOS5)
#define DMEM_SIZE	(0x28000)
#define ICACHE_SIZE	(0x18000)
#endif
#define CMEM_SIZE	(0x9000)
#define INT_MEM_SIZE	(DMEM_SIZE + ICACHE_SIZE + CMEM_SIZE)

/* IBUF/OBUF Size */
#define IBUF_SIZE	(0x4000)
#define WBUF_SIZE	(IBUF_SIZE * 4)
#if defined(CONFIG_ARCH_EXYNOS4)
#define OBUF_SIZE	(0x8000)
#elif defined(CONFIG_ARCH_EXYNOS5)
#define OBUF_SIZE	(0x4000)
#endif

/* Start threshold */
#define START_THRESHOLD	(IBUF_SIZE * 3)

/* IDMA Buffer */
#if defined(CONFIG_ARCH_EXYNOS4)
#define IDMA_OFFSET	((soc_is_exynos4412() || soc_is_exynos4212()) ? \
			(0x38000) : (0x18000))
#define SRP_IDMA_BASE	(SRP_IRAM_BASE + IDMA_OFFSET)
#elif defined(CONFIG_ARCH_EXYNOS5)
#define IDMA_OFFSET	(0x4)
#define SRP_IDMA_BASE	(SRP_DMEM_BASE + IDMA_OFFSET)
#endif

/* Commbox & Etc information */
#define COMMBOX_SIZE	(0x308)

/* Reserved memory on DRAM */
#define BASE_MEM_SIZE	(CONFIG_AUDIO_SAMSUNG_MEMSIZE_SRP << 10)
#define BITSTREAM_SIZE_MAX	(0x7FFFFFFF)

/* F/W Endian Configuration */
#ifdef USE_FW_ENDIAN_CONVERT
#define ENDIAN_CHK_CONV(VAL)		\
	(((VAL >> 24) & 0x000000FF) |	\
	((VAL >> 8) & 0x0000FF00) |	\
	((VAL << 8) & 0x00FF0000) |	\
	((VAL << 24) & 0xFF000000))
#else
#define ENDIAN_CHK_CONV(VAL)	(VAL)
#endif

/* For Debugging */
#ifdef CONFIG_SND_SAMSUNG_RP_DEBUG
#define srp_info(x...)	pr_info("SRP: " x)
#define srp_debug(x...)	pr_debug("SRP: " x)
#define srp_err(x...)	pr_err("SRP_ERR: " x)
#else
#define srp_info(x...)
#define srp_debug(x...)
#define srp_err(x...)
#endif

/* For SRP firmware */
struct srp_fw_info {
	const struct firmware *vliw;		/* VLIW */
	const struct firmware *cga;		/* CGA */
	const struct firmware *data;		/* DATA */

	unsigned char *base_va;		/* Virtual address of base */
	unsigned int base_pa;		/* Physical address of base */
	unsigned int vliw_pa;		/* Physical address of VLIW */
	unsigned int cga_pa;		/* Physical address of CGA */
	unsigned int data_pa;		/* Physical address of DATA */
	unsigned char *vliw_va;
	unsigned char *cga_va;
	unsigned char *data_va;

	unsigned long vliw_size;	/* Size of VLIW */
	unsigned long cga_size;		/* Size of CGA */
	unsigned long data_size;	/* Size of DATA */
};

/* OBUF/IBUF information */
struct srp_buf_info {
	void		*mmapped_addr;
	void		*addr;
	unsigned int	mmapped_size;
	unsigned int	size;
	int		num;
};

/* Decoding information */
struct srp_dec_info {
	unsigned int sample_rate;
	unsigned int channels;
};

struct srp_for_suspend {
	unsigned char	*ibuf;
	unsigned char	*obuf;
	unsigned char	*commbox;
};

struct srp_info {
	struct srp_buf_info	ibuf_info;
	struct srp_buf_info	obuf_info;
	struct srp_buf_info	pcm_info;

	struct srp_fw_info	fw_info;
	struct srp_dec_info	dec_info;
	struct srp_for_suspend  sp_data;
	struct clk		*clk;

	void __iomem	*iram;
	void __iomem	*dmem;
	void __iomem	*icache;
	void __iomem	*cmem;
	void __iomem	*commbox;

	/* IBUF informaion */
	unsigned char	*ibuf0;
	unsigned char	*ibuf1;
	unsigned int	ibuf0_pa;
	unsigned int	ibuf1_pa;
	unsigned int	ibuf_num;
	unsigned long	ibuf_size;
	unsigned long	ibuf_offset;
	unsigned int	ibuf_next;
	unsigned int	ibuf_empty[2];

	/* OBUF informaion */
	unsigned char	*obuf0;
	unsigned char	*obuf1;
	unsigned int	obuf0_pa;
	unsigned int	obuf1_pa;
	unsigned int	obuf_num;
	unsigned long	obuf_size;
	unsigned long	obuf_offset;
	unsigned int	obuf_fill_done[2];
	unsigned int	obuf_copy_done[2];
	unsigned int	obuf_ready;
	unsigned int	obuf_next;

	/* Temporary BUF informaion */
	unsigned char	*wbuf;
	unsigned long	wbuf_size;
	unsigned long	wbuf_pos;
	unsigned long	wbuf_fill_size;

	/* Decoding informaion */
	unsigned long	set_bitstream_size;
	unsigned long	pcm_size;

	/* SRP status information */
	unsigned int	decoding_started;
	unsigned int	is_opened;
	unsigned int	is_running;
	unsigned int	is_pending;
	unsigned int	block_mode;
	unsigned int	stop_after_eos;
	unsigned int	wait_for_eos;
	unsigned int	prepare_for_eos;
	unsigned int	play_done;
	unsigned int	idma_addr;
	unsigned int	data_offset;

	bool	pm_suspended;
	bool	pm_resumed;
	bool	hw_reset_stat;

	/* Parameter to control Runtime PM */
	void	*pm_info;
};

/* SRP Pending On/Off status */
enum {
	RUN = 0,
	STALL,
};

/* Request Suspend/Resume */
enum {
	SUSPEND = 0,
	RESUME,
	SW_RESET,
};

extern void srp_prepare_pm(void *info);
#endif /* __SRP_ALP_H */
