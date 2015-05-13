#include <linux/module.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/err.h>
#include <linux/interrupt.h>
//#include <linux/completion.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_i2c.h>
#include <linux/of_device.h>
#include <linux/slab.h>
#include <linux/i2c-omap.h>
#include<linux/irq.h>
//#include <linux/pm_runtime.h>
#include <linux/pinctrl/consumer.h>

#include <rtdm/rtdm_driver.h>
//#include <rtdev.h>
#include<rtdm/rtdm.h>

/* I2C controller revisions */
#define OMAP_I2C_OMAP1_REV_2		0x20

/* I2C controller revisions present on specific hardware */
#define OMAP_I2C_REV_ON_2430		0x00000036
#define OMAP_I2C_REV_ON_3430_3530	0x0000003C
#define OMAP_I2C_REV_ON_3630		0x00000040
#define OMAP_I2C_REV_ON_4430_PLUS	0x50400002

/* timeout waiting for the controller to respond */
//#define OMAP_I2C_TIMEOUT (msecs_to_jiffies(1000))
#define OMAP_I2C_TIMEOUT		 1000000000

/* timeout for pm runtime autosuspend */
#define OMAP_I2C_PM_TIMEOUT		1000	/* ms */

/* For OMAP3 I2C_IV has changed to I2C_WE (wakeup enable) */
enum {
	OMAP_I2C_REV_REG = 0,
	OMAP_I2C_IE_REG,
	OMAP_I2C_STAT_REG,
	OMAP_I2C_IV_REG,
	OMAP_I2C_WE_REG,
	OMAP_I2C_SYSS_REG,
	OMAP_I2C_BUF_REG,
	OMAP_I2C_CNT_REG,
	OMAP_I2C_DATA_REG,
	OMAP_I2C_SYSC_REG,
	OMAP_I2C_CON_REG,
	OMAP_I2C_OA_REG,
	OMAP_I2C_SA_REG,
	OMAP_I2C_PSC_REG,
	OMAP_I2C_SCLL_REG,
	OMAP_I2C_SCLH_REG,
	OMAP_I2C_SYSTEST_REG,
	OMAP_I2C_BUFSTAT_REG,
	/* only on OMAP4430 */
	OMAP_I2C_IP_V2_REVNB_LO,
	OMAP_I2C_IP_V2_REVNB_HI,
	OMAP_I2C_IP_V2_IRQSTATUS_RAW,
	OMAP_I2C_IP_V2_IRQENABLE_SET,
	OMAP_I2C_IP_V2_IRQENABLE_CLR,
};

/* I2C Interrupt Enable Register (OMAP_I2C_IE): */
#define OMAP_I2C_IE_XDR		(1 << 14)	/* TX Buffer drain int enable */
#define OMAP_I2C_IE_RDR		(1 << 13)	/* RX Buffer drain int enable */
#define OMAP_I2C_IE_XRDY	(1 << 4)	/* TX data ready int enable */
#define OMAP_I2C_IE_RRDY	(1 << 3)	/* RX data ready int enable */
#define OMAP_I2C_IE_ARDY	(1 << 2)	/* Access ready int enable */
#define OMAP_I2C_IE_NACK	(1 << 1)	/* No ack interrupt enable */
#define OMAP_I2C_IE_AL		(1 << 0)	/* Arbitration lost int ena */

/* I2C Status Register (OMAP_I2C_STAT): */
#define OMAP_I2C_STAT_XDR	(1 << 14)	/* TX Buffer draining */
#define OMAP_I2C_STAT_RDR	(1 << 13)	/* RX Buffer draining */
#define OMAP_I2C_STAT_BB	(1 << 12)	/* Bus busy */
#define OMAP_I2C_STAT_ROVR	(1 << 11)	/* Receive overrun */
#define OMAP_I2C_STAT_XUDF	(1 << 10)	/* Transmit underflow */
#define OMAP_I2C_STAT_AAS	(1 << 9)	/* Address as slave */
#define OMAP_I2C_STAT_AD0	(1 << 8)	/* Address zero */
#define OMAP_I2C_STAT_XRDY	(1 << 4)	/* Transmit data ready */
#define OMAP_I2C_STAT_RRDY	(1 << 3)	/* Receive data ready */
#define OMAP_I2C_STAT_ARDY	(1 << 2)	/* Register access ready */
#define OMAP_I2C_STAT_NACK	(1 << 1)	/* No ack interrupt enable */
#define OMAP_I2C_STAT_AL	(1 << 0)	/* Arbitration lost int ena */

/* I2C WE wakeup enable register */
#define OMAP_I2C_WE_XDR_WE	(1 << 14)	/* TX drain wakup */
#define OMAP_I2C_WE_RDR_WE	(1 << 13)	/* RX drain wakeup */
#define OMAP_I2C_WE_AAS_WE	(1 << 9)	/* Address as slave wakeup*/
#define OMAP_I2C_WE_BF_WE	(1 << 8)	/* Bus free wakeup */
#define OMAP_I2C_WE_STC_WE	(1 << 6)	/* Start condition wakeup */
#define OMAP_I2C_WE_GC_WE	(1 << 5)	/* General call wakeup */
#define OMAP_I2C_WE_DRDY_WE	(1 << 3)	/* TX/RX data ready wakeup */
#define OMAP_I2C_WE_ARDY_WE	(1 << 2)	/* Reg access ready wakeup */
#define OMAP_I2C_WE_NACK_WE	(1 << 1)	/* No acknowledgment wakeup */
#define OMAP_I2C_WE_AL_WE	(1 << 0)	/* Arbitration lost wakeup */

#define OMAP_I2C_WE_ALL		(OMAP_I2C_WE_XDR_WE | OMAP_I2C_WE_RDR_WE | \
				OMAP_I2C_WE_AAS_WE | OMAP_I2C_WE_BF_WE | \
				OMAP_I2C_WE_STC_WE | OMAP_I2C_WE_GC_WE | \
				OMAP_I2C_WE_DRDY_WE | OMAP_I2C_WE_ARDY_WE | \
				OMAP_I2C_WE_NACK_WE | OMAP_I2C_WE_AL_WE)

/* I2C Buffer Configuration Register (OMAP_I2C_BUF): */
#define OMAP_I2C_BUF_RDMA_EN	(1 << 15)	/* RX DMA channel enable */
#define OMAP_I2C_BUF_RXFIF_CLR	(1 << 14)	/* RX FIFO Clear */
#define OMAP_I2C_BUF_XDMA_EN	(1 << 7)	/* TX DMA channel enable */
#define OMAP_I2C_BUF_TXFIF_CLR	(1 << 6)	/* TX FIFO Clear */

/* I2C Configuration Register (OMAP_I2C_CON): */
#define OMAP_I2C_CON_EN		(1 << 15)	/* I2C module enable */
#define OMAP_I2C_CON_BE		(1 << 14)	/* Big endian mode */
#define OMAP_I2C_CON_OPMODE_HS	(1 << 12)	/* High Speed support */
#define OMAP_I2C_CON_STB	(1 << 11)	/* Start byte mode (master) */
#define OMAP_I2C_CON_MST	(1 << 10)	/* Master/slave mode */
#define OMAP_I2C_CON_TRX	(1 << 9)	/* TX/RX mode (master only) */
#define OMAP_I2C_CON_XA		(1 << 8)	/* Expand address */
#define OMAP_I2C_CON_RM		(1 << 2)	/* Repeat mode (master only) */
#define OMAP_I2C_CON_STP	(1 << 1)	/* Stop cond (master only) */
#define OMAP_I2C_CON_STT	(1 << 0)	/* Start condition (master) */

/* I2C SCL time value when Master */
#define OMAP_I2C_SCLL_HSSCLL	8
#define OMAP_I2C_SCLH_HSSCLH	8

/* I2C System Test Register (OMAP_I2C_SYSTEST): */
#ifdef DEBUG
#define OMAP_I2C_SYSTEST_ST_EN		(1 << 15)	/* System test enable */
#define OMAP_I2C_SYSTEST_FREE		(1 << 14)	/* Free running mode */
#define OMAP_I2C_SYSTEST_TMODE_MASK	(3 << 12)	/* Test mode select */
#define OMAP_I2C_SYSTEST_TMODE_SHIFT	(12)		/* Test mode select */
#define OMAP_I2C_SYSTEST_SCL_I		(1 << 3)	/* SCL line sense in */
#define OMAP_I2C_SYSTEST_SCL_O		(1 << 2)	/* SCL line drive out */
#define OMAP_I2C_SYSTEST_SDA_I		(1 << 1)	/* SDA line sense in */
#define OMAP_I2C_SYSTEST_SDA_O		(1 << 0)	/* SDA line drive out */
#endif

/* OCP_SYSSTATUS bit definitions */
#define SYSS_RESETDONE_MASK		(1 << 0)

/* OCP_SYSCONFIG bit definitions */
#define SYSC_CLOCKACTIVITY_MASK		(0x3 << 8)
#define SYSC_SIDLEMODE_MASK		(0x3 << 3)
#define SYSC_ENAWAKEUP_MASK		(1 << 2)
#define SYSC_SOFTRESET_MASK		(1 << 1)
#define SYSC_AUTOIDLE_MASK		(1 << 0)

#define SYSC_IDLEMODE_SMART		0x2
#define SYSC_CLOCKACTIVITY_FCLK		0x2

/* Errata definitions */
#define I2C_OMAP_ERRATA_I207		(1 << 0)
#define I2C_OMAP_ERRATA_I462		(1 << 1)


#define SOC_PRCM_REGS                        (0x44E00000)
#define SOC_PRCM_SIZE 				(0x400 )
#define CM_PER_I2C1_CLKCTRL		(1 << 1)

#define DEVICE_NAME	"omap-i2c"
#define DRIVER_NAME	"omap-i2c"

typedef struct omap_i2c_dev {
//	spinlock_t		lock1;		/* IRQ synchronization */
	struct device		*dev;
	void __iomem		*base;		/* virtual */
	int			irq;
	int			reg_shift;      /* bit shift for I2C register addresses */
//	struct completion	cmd_complete;
	struct resource		*ioarea;
	u32			latency;	/* maximum mpu wkup latency */
	void			(*set_mpu_wkup_lat)(struct device *dev,long latency);
	u32			speed;		/* Speed of bus in kHz */
	u32			flags;
	u16			cmd_err;
	u8			*buf;
	u8			*regs;
	size_t			buf_len;
//	struct i2c_adapter	adapter;
	u8			threshold;
	u8			fifo_size;	/* use as flag and value
						 * fifo_size==0 implies no fifo
						 * if set, should be trsh+1
						 */
	u32			rev;
	unsigned		b_hw:1;		/* bad h/w fixes */
	unsigned		receiver:1;	/* true when we're in receiver mode */
	u16			iestate;	/* Saved interrupt register */
	u16			pscstate;
	u16			scllstate;
	u16			sclhstate;
	u16			syscstate;
	u16			westate;
	u16			errata;
	rtdm_irq_t		irq_handle;
	rtdm_event_t		w_event;
	rtdm_lock_t		lock;
	int 			mode;
	u16			addr;
	u16			add_ten;
	__u16			read;
	u16			i2c_stat_nack;
	int 			stop;
//	int			pm;
	struct rtdm_device 	rtdm_dev; 
	struct pinctrl		*pins;
}MY_DEV;

static const u8 reg_map_ip_v1[] = {
	[OMAP_I2C_REV_REG] = 0x00,
	[OMAP_I2C_IE_REG] = 0x01,
	[OMAP_I2C_STAT_REG] = 0x02,
	[OMAP_I2C_IV_REG] = 0x03,
	[OMAP_I2C_WE_REG] = 0x03,
	[OMAP_I2C_SYSS_REG] = 0x04,
	[OMAP_I2C_BUF_REG] = 0x05,
	[OMAP_I2C_CNT_REG] = 0x06,
	[OMAP_I2C_DATA_REG] = 0x07,
	[OMAP_I2C_SYSC_REG] = 0x08,
	[OMAP_I2C_CON_REG] = 0x09,
	[OMAP_I2C_OA_REG] = 0x0a,
	[OMAP_I2C_SA_REG] = 0x0b,
	[OMAP_I2C_PSC_REG] = 0x0c,
	[OMAP_I2C_SCLL_REG] = 0x0d,
	[OMAP_I2C_SCLH_REG] = 0x0e,
	[OMAP_I2C_SYSTEST_REG] = 0x0f,
	[OMAP_I2C_BUFSTAT_REG] = 0x10,
};

static const u8 reg_map_ip_v2[] = {
	[OMAP_I2C_REV_REG] = 0x04,
	[OMAP_I2C_IE_REG] = 0x2c,
	[OMAP_I2C_STAT_REG] = 0x28,
	[OMAP_I2C_IV_REG] = 0x34,
	[OMAP_I2C_WE_REG] = 0x34,
	[OMAP_I2C_SYSS_REG] = 0x90,
	[OMAP_I2C_BUF_REG] = 0x94,
	[OMAP_I2C_CNT_REG] = 0x98,
	[OMAP_I2C_DATA_REG] = 0x9c,
	[OMAP_I2C_SYSC_REG] = 0x10,
	[OMAP_I2C_CON_REG] = 0xa4,
	[OMAP_I2C_OA_REG] = 0xa8,
	[OMAP_I2C_SA_REG] = 0xac,
	[OMAP_I2C_PSC_REG] = 0xb0,
	[OMAP_I2C_SCLL_REG] = 0xb4,
	[OMAP_I2C_SCLH_REG] = 0xb8,
	[OMAP_I2C_SYSTEST_REG] = 0xbC,
	[OMAP_I2C_BUFSTAT_REG] = 0xc0,
	[OMAP_I2C_IP_V2_REVNB_LO] = 0x00,
	[OMAP_I2C_IP_V2_REVNB_HI] = 0x04,
	[OMAP_I2C_IP_V2_IRQSTATUS_RAW] = 0x24,
	[OMAP_I2C_IP_V2_IRQENABLE_SET] = 0x2c,
	[OMAP_I2C_IP_V2_IRQENABLE_CLR] = 0x30,
};

static inline void omap_i2c_write_reg(MY_DEV *i2c_dev, int reg, u16 val)
{
	iowrite16(val, i2c_dev->base+(i2c_dev->regs[reg] << i2c_dev->reg_shift));
}

static inline u16 omap_i2c_read_reg(MY_DEV *i2c_dev, int reg)
{

        return ioread16(i2c_dev->base + (i2c_dev->regs[reg] << i2c_dev->reg_shift));
}

static void __omap_i2c_init(MY_DEV *dev)
{
	u16	mask,stat;
	
//        mask = omap_i2c_read_reg(dev, OMAP_I2C_IE_REG);
//        stat = omap_i2c_read_reg(dev, OMAP_I2C_STAT_REG);
//       rtdm_printk("_omap_i2c_init start:mask=%ud\n",mask);
//        rtdm_printk("_omap_i2c_init start:stat=%ud\n",stat);

	rtdm_printk("__omap_i2c_init started\n");
	rtdm_printk("address of local struct MY_DEV=%d\n",dev);

	omap_i2c_write_reg(dev, OMAP_I2C_CON_REG, 0);
	rtdm_printk("write in omap_i2c_con_reg\n");

	/* Setup clock prescaler to obtain approx 12MHz I2C module clock: */
	omap_i2c_write_reg(dev, OMAP_I2C_PSC_REG, dev->pscstate);

	rtdm_printk("write omap_i2c_psc_reg\n");

	/* SCL low and high time values */
	omap_i2c_write_reg(dev, OMAP_I2C_SCLL_REG, dev->scllstate);
	rtdm_printk("write in omap_i2c_scll_reg\n");

	omap_i2c_write_reg(dev, OMAP_I2C_SCLH_REG, dev->sclhstate);
	rtdm_printk("write in omap_i2c_sclh_reg\n");

	if (dev->rev >= OMAP_I2C_REV_ON_3430_3530)
		omap_i2c_write_reg(dev, OMAP_I2C_WE_REG, dev->westate);

	/* Take the I2C module out of reset: */
	omap_i2c_write_reg(dev, OMAP_I2C_CON_REG, OMAP_I2C_CON_EN);

	/*
	 * Don't write to this register if the IE state is 0 as it can
	 * cause deadlock.
	 */
	if (dev->iestate)
		omap_i2c_write_reg(dev, OMAP_I2C_IE_REG, dev->iestate);


        mask = omap_i2c_read_reg(dev, OMAP_I2C_IE_REG);
        stat = omap_i2c_read_reg(dev, OMAP_I2C_STAT_REG);

        rtdm_printk("_omap_i2c_init ended:mask=%ud\n",mask);
        rtdm_printk("_omap_i2c_init ended:stat=%ud\n",stat);


	rtdm_printk("__omap_i2c_init ended\n");
}

static int omap_i2c_reset(MY_DEV *dev)
{

	rtdm_printk("omap_i2c_reset start\n");
	unsigned long timeout;
	u16 sysc;
	rtdm_printk("address of local struct MY_DEV=%d\n",dev);
	if (dev->rev >= OMAP_I2C_OMAP1_REV_2) {
		sysc = omap_i2c_read_reg(dev, OMAP_I2C_SYSC_REG);

		/* Disable I2C controller before soft reset */
		omap_i2c_write_reg(dev, OMAP_I2C_CON_REG,omap_i2c_read_reg(dev, OMAP_I2C_CON_REG) & ~(OMAP_I2C_CON_EN));

		omap_i2c_write_reg(dev, OMAP_I2C_SYSC_REG, SYSC_SOFTRESET_MASK);
		/* For some reason we need to set the EN bit before the
		 * reset done bit gets set. */
		timeout = jiffies + OMAP_I2C_TIMEOUT;
		omap_i2c_write_reg(dev, OMAP_I2C_CON_REG, OMAP_I2C_CON_EN);
		while (!(omap_i2c_read_reg(dev, OMAP_I2C_SYSS_REG) &
			 SYSS_RESETDONE_MASK)) {
			if (time_after(jiffies, timeout)) {
				dev_warn(dev->dev, "timeout waiting "
						"for controller reset\n");
				return -ETIMEDOUT;
			}
			msleep(1);
		}

		/* SYSC register is cleared by the reset; rewrite it */
		omap_i2c_write_reg(dev, OMAP_I2C_SYSC_REG, sysc);

	}

	 rtdm_printk("omap_i2c_reset end\n");

	return 0;
}

static int omap_i2c_init(MY_DEV *dev)
{

	rtdm_printk("omap_i2c_init start\n");

	rtdm_printk("address of local struct MY_DEV=%d\n",dev);

	u16 psc = 0;
	u16  scll = 0;
	u16 sclh = 0;
	u16 fsscll = 0, fssclh = 0, hsscll = 0, hssclh = 0;
	unsigned long fclk_rate = 12000000;
	unsigned long internal_clk = 0;
	struct clk *fclk;

	rtdm_printk("omap_i2c_init started\n");

	if (dev->rev >= OMAP_I2C_REV_ON_3430_3530) {
		/*
		 * Enabling all wakup sources to stop I2C freezing on
		 * WFI instruction.
		 * REVISIT: Some wkup sources might not be needed.
		 */

		rtdm_printk("set det->westate\n");
		dev->westate = OMAP_I2C_WE_ALL;
	}

	if (dev->flags & OMAP_I2C_FLAG_ALWAYS_ARMXOR_CLK) {
		/*
		 * The I2C functional clock is the armxor_ck, so there's
		 * no need to get "armxor_ck" separately.  Now, if OMAP2420
		 * always returns 12MHz for the functional clock, we can
		 * do this bit unconditionally.
		 */
		 rtdm_printk("fclk ,fclk_rate\n");

		fclk = clk_get(dev->dev, "fck");
		fclk_rate = clk_get_rate(fclk);
		clk_put(fclk);

		/* TRM for 5912 says the I2C clock must be prescaled to be
		 * between 7 - 12 MHz. The XOR input clock is typically
		 * 12, 13 or 19.2 MHz. So we should have code that produces:
		 *
		 * XOR MHz	Divider		Prescaler
		 * 12		1		0
		 * 13		2		1
		 * 19.2		2		1
		 */
		if (fclk_rate > 12000000)
			psc = fclk_rate / 12000000;
	}

	if (!(dev->flags & OMAP_I2C_FLAG_SIMPLE_CLOCK)) {

		/*
		 * HSI2C controller internal clk rate should be 19.2 Mhz for
		 * HS and for all modes on 2430. On 34xx we can use lower rate
		 * to get longer filter period for better noise suppression.
		 * The filter is iclk (fclk for HS) period.
		 */
		if (dev->speed > 400 ||
			       dev->flags & OMAP_I2C_FLAG_FORCE_19200_INT_CLK)
			internal_clk = 19200;
		else if (dev->speed > 100)
			internal_clk = 9600;
		else
			internal_clk = 4000;
		fclk = clk_get(dev->dev, "fck");
		fclk_rate = clk_get_rate(fclk) / 1000;
		clk_put(fclk);

		/* Compute prescaler divisor */
		psc = fclk_rate / internal_clk;
		psc = psc - 1;

		/* If configured for High Speed */
		if (dev->speed > 400) {
			unsigned long scl;

			/* For first phase of HS mode */
			scl = internal_clk / 400;
			fsscll = scl - (scl / 3) - 7;
			fssclh = (scl / 3) - 5;

			/* For second phase of HS mode */
			scl = fclk_rate / dev->speed;
			hsscll = scl - (scl / 3) - 7;
			hssclh = (scl / 3) - 5;
		} else if (dev->speed > 100) {
			unsigned long scl;

			/* Fast mode */
			scl = internal_clk / dev->speed;
			fsscll = scl - (scl / 3) - 7;
			fssclh = (scl / 3) - 5;
		} else {
			/* Standard mode */
			fsscll = internal_clk / (dev->speed * 2) - 7;
			fssclh = internal_clk / (dev->speed * 2) - 5;
		}
		scll = (hsscll << OMAP_I2C_SCLL_HSSCLL) | fsscll;
		sclh = (hssclh << OMAP_I2C_SCLH_HSSCLH) | fssclh;
	} else {
		/* Program desired operating rate */
		fclk_rate /= (psc + 1) * 1000;
		if (psc > 2)
			psc = 2;
		scll = fclk_rate / (dev->speed * 2) - 7 + psc;
		sclh = fclk_rate / (dev->speed * 2) - 7 + psc;
	}

	dev->iestate = (OMAP_I2C_IE_XRDY | OMAP_I2C_IE_RRDY |
			OMAP_I2C_IE_ARDY | OMAP_I2C_IE_NACK |
			OMAP_I2C_IE_AL)  | ((dev->fifo_size) ?
				(OMAP_I2C_IE_RDR | OMAP_I2C_IE_XDR) : 0);

	dev->pscstate = psc;
	dev->scllstate = scll;
	dev->sclhstate = sclh;

//	 u16 mask,stat;
//        mask = omap_i2c_read_reg(dev, OMAP_I2C_IE_REG);
//        stat = omap_i2c_read_reg(dev, OMAP_I2C_STAT_REG);

//        rtdm_printk("omap_i2c_init:mask=%ud\n",mask);
//        rtdm_printk("omap_i2c_init:stat=%ud\n",stat);
	__omap_i2c_init(dev);
	rtdm_printk("omap_i2c_init ended\n");
	return 0;
}

static int resume(struct device *dev)
{
       rtdm_printk("resume start\n");
      struct platform_device *pdev = to_platform_device(dev);
   struct omap_i2c_dev *_dev = platform_get_drvdata(pdev);

	 rtdm_printk("address of local struct MY_DEV=%d\n",_dev);


      if (!_dev->regs)
              return 0;


      __omap_i2c_init(_dev);

      rtdm_printk("omap_i2c_runtime_resume exit\n");

      rtdm_printk("resume end\n");

      return 0;
}

static int suspend(struct device *dev)
{
	rtdm_printk("suspend start\n");
	struct platform_device *pdev = to_platform_device(dev);
	struct omap_i2c_dev *_dev = platform_get_drvdata(pdev);
	
	rtdm_printk("address of local struct MY_DEV=%d\n",_dev);
	
	
	_dev->iestate = omap_i2c_read_reg(_dev, OMAP_I2C_IE_REG);
	rtdm_printk("omap_i2c_read_reg for _dev->iestate\n");
	
	omap_i2c_write_reg(_dev, OMAP_I2C_IE_REG, 0);
	rtdm_printk("omap_i2c_write_reg for OMAP_I2C_IE_REG\n");
	
	if (_dev->rev < OMAP_I2C_OMAP1_REV_2) {
		omap_i2c_read_reg(_dev, OMAP_I2C_IV_REG); /* Read clears */
		rtdm_printk("omap_i2c_read_reg for OMAP_I2C_IV_REG\n");
	} 
	else 
	{
		omap_i2c_write_reg(_dev, OMAP_I2C_STAT_REG, _dev->iestate);
		rtdm_printk("omap_i2c_write_reg for OMAP_I2C_STAT_REG\n");
	
		/* Flush posted write */
		omap_i2c_read_reg(_dev, OMAP_I2C_STAT_REG);
		rtdm_printk("omap_i2c_read_reg for OMAP_I2C_STAT_REG\n");
	}
	
	rtdm_printk("suspend end\n");

	return 0;
}

static int omap_i2c_wait_for_bb(struct omap_i2c_dev *dev)
{
	rtdm_printk("omap_i2c_wait_for_bb start\n");
	unsigned long timeout,systime,systime1;

	rtdm_printk("address of local struct MY_DEV=%d\n",dev);

	systime = rtdm_clock_read();
	
	timeout = systime + OMAP_I2C_TIMEOUT;
	
	while(omap_i2c_read_reg(dev,OMAP_I2C_STAT_REG) & OMAP_I2C_STAT_BB)
	{
	systime1 = rtdm_clock_read();
	
		if(systime1 > timeout)
		{
			rtdm_printk("timeout waiting for bus ready\n");
			return -ETIMEDOUT;
		}
		rtdm_task_sleep(1000000);
	}
	rtdm_printk("omap_i2c_wait_for_bb end\n");
	return 0;
}

static void omap_i2c_resize_fifo(struct omap_i2c_dev *dev,size_t size,bool is_rx)
{
	u16	buf;
	rtdm_printk("omap_i2c_resize_fifo start\n");
	rtdm_printk("address of local struct MY_DEV=%d\n",dev);
	if(dev->flags & OMAP_I2C_FLAG_NO_FIFO)
		return;

	dev->threshold = clamp((u8)size,(u8) 1, dev->fifo_size);
	buf = omap_i2c_read_reg(dev, OMAP_I2C_BUF_REG);
	if(is_rx)
	{
	buf &= ~(0x3f << 8);
	buf |= ((dev->threshold - 1) << 8) | OMAP_I2C_BUF_RXFIF_CLR;
	}
	else
	{
	buf &= ~0x3f;
	buf |= (dev->threshold - 1) | OMAP_I2C_BUF_TXFIF_CLR ; 
	}
	omap_i2c_write_reg(dev, OMAP_I2C_BUF_REG, buf);
	
	if(dev->rev < OMAP_I2C_REV_ON_3630)
		dev->b_hw = 1; /*enable hardware fixes */

	if(dev->set_mpu_wkup_lat != NULL)
		dev->latency = (1000000 * dev->threshold) / (1000 * dev->speed/8);

	rtdm_printk("omap_i2c_resize_fifo end\n");
}
 
static int i2c_open_nrt(struct rtdm_dev_context *context,rtdm_user_info_t *user_info_t,int oflags_t)
{
	int err;
	 rtdm_printk("i2c_open_nrt start\n");

//	 MY_DEV * dev = (MY_DEV *) context->dev_private;
	MY_DEV *dev=(MY_DEV *)context->device->device_data;

	 rtdm_printk("address of local struct MY_DEV=%d\n",dev);

	rtdm_printk("dev->base=%x\n",dev->base);
	rtdm_printk("dev->irq=%d\n",dev->irq);

	err=rtdm_irq_enable(&dev->irq_handle);//enable irq
        if(err<0)
        {
                rtdm_printk("error in rtdm_irq_disable\n");
                return err;
        }
	rtdm_event_init(&dev->w_event,0);
	rtdm_lock_init(&dev->lock);
        rtdm_printk("rtdm_event_init done\n");
	rtdm_printk("i2c_open_nrt end\n");
	return 0;
}

int omap_i2c_rdwr_msg(MY_DEV *dev,int stop)
{
        u16 w,err;
//	 unsigned long timeout,systime,systime1;
	rtdm_printk("omap_i2c_rdwr_msg start\n");
	rtdm_printk("address of local struct MY_DEV=%d\n",dev);
	rtdm_printk("dev->addr=%x\n",dev->addr);
//	rtdm_printk("omap_i2c_rdwr_msg dev->buf=%ud",*dev->buf);
	u16 mask,stat,omap_add;
//        mask = omap_i2c_read_reg(dev, OMAP_I2C_IE_REG);
//        stat = omap_i2c_read_reg(dev, OMAP_I2C_STAT_REG);

//        rtdm_printk("omap_i2c_rdwrmask=%ud\n",mask);
//        rtdm_printk("stat=%ud\n",stat);

	dev->receiver = !!(dev->read & I2C_M_RD);

        //note:size_t is an unsigned integer type of at least 16 bit 

        omap_i2c_resize_fifo(dev, dev->buf_len , dev->receiver);


        omap_i2c_write_reg(dev,OMAP_I2C_SA_REG ,dev->addr);
   	omap_add = omap_i2c_read_reg(dev, OMAP_I2C_SA_REG);

	printk("OMAP_I2C_SA_REG=%x\n",omap_add);
	rtdm_printk("omap_i2c_write_reg\n");

	//Note: Passing addr of void pointer which is type cast to u8 to the pointer buf
//        dev->buf = (u8 *)buf;

	rtdm_printk("dev->buf done type cast\n");

//        barrier();

        omap_i2c_write_reg(dev,OMAP_I2C_CNT_REG,dev->buf_len);
	rtdm_printk("write OMAP_I2C_CNT_REG\n");


        w = omap_i2c_read_reg(dev, OMAP_I2C_BUF_REG);
	rtdm_printk("read OMAP_I2C_BUF_REG\n");


        w |= OMAP_I2C_BUF_RXFIF_CLR | OMAP_I2C_BUF_TXFIF_CLR;

        omap_i2c_write_reg(dev,OMAP_I2C_BUF_REG,w);
	rtdm_printk("write to OMAP_I2C_BUF_REG\n");

//	mask = omap_i2c_read_reg(dev, OMAP_I2C_IE_REG);
//        stat = omap_i2c_read_reg(dev, OMAP_I2C_STAT_REG);

//        rtdm_printk("value after writing to omap_i2c_buf_reg:mask=%ud\n",mask);
//       rtdm_printk("value after writing to omap_i2c_buf_reg:stat=%ud\n",stat);

        //init event
//	rtdm_event_init(&dev->w_event,0);

        dev->cmd_err = 0;

	dev->stop = stop;
	rtdm_printk("...............stop=%d\n",stop);
	rtdm_printk("...............dev->b_hw=%d\n",dev->b_hw);

        w = OMAP_I2C_CON_EN | OMAP_I2C_CON_MST | OMAP_I2C_CON_STT ;

        if(dev->speed > 400)
                w |= OMAP_I2C_CON_OPMODE_HS;

//        if(dev->stop & I2C_M_STOP)
//                stop =1;

//        if(dev->add_ten & I2C_M_TEN)
//                w |= OMAP_I2C_CON_XA;

        if(!(dev->read & I2C_M_RD))
	{rtdm_printk("trasmit flag enabled in control register\n");
         w |= OMAP_I2C_CON_TRX;
	}

        if(!dev->b_hw && stop)
	{	rtdm_printk("stop condition\n");
                w |= OMAP_I2C_CON_STP;
	}

	rtdm_printk("entering ISR.................................\n");
        omap_i2c_write_reg(dev,OMAP_I2C_CON_REG,w);
	rtdm_printk("write to OMAP_I2C_CON_REG\n");

	//check for stop condition
//	if(dev->b_hw && dev->stop)
//        {
//                systime = rtdm_clock_read();

//                timeout = systime + OMAP_I2C_TIMEOUT;

//                u16 con = omap_i2c_read_reg(dev, OMAP_I2C_CON_REG);
//		rtdm_printk("read from OMAP_I2C_CON_REG\n");

//                while(con & OMAP_I2C_CON_STT)
//                {

//                con = omap_i2c_read_reg(dev,OMAP_I2C_CON_REG);
//                systime1 = rtdm_clock_read();

//                        if(systime1 > timeout)
//                        {
//                        dev_err(dev->dev,"controller time out waiting for start condition to finish\n");
//                        return -ETIMEDOUT;
//                        }
//                        cpu_relax();
	
//                }
//        w |= OMAP_I2C_CON_STP;
//        w &= ~OMAP_I2C_CON_STT;
//        omap_i2c_write_reg(dev, OMAP_I2C_CON_REG , w);
//        }
	
	rtdm_printk("rtdm_event_timedwait start\n");

//	err=rtdm_event_wait(&dev->w_event);

//	err=rtdm_task_init(&dev->wait_task,"wait_event_irq",wait_event_irq, dev,1,0);
//	if(err<0 )
//        {
//                rtdm_printk("error in task2 init\n");
//                return 0;
//        }
//	printk("rtdm_event_wait%d\n",err);

		err=rtdm_event_wait(&dev->w_event);
//	err=rtdm_event_timedwait(&dev->w_event,OMAP_I2C_TIMEOUT,NULL);
        	if(err<0)
        	{ 
        	dev_err(dev->dev,"controller timed out\n");
        	omap_i2c_reset(dev);
        	__omap_i2c_init(dev);
        	rtdm_printk("rtdm_event_timedwait: timeout\n");
        	return -ETIMEDOUT;
        	}
	rtdm_printk("rtdm_event_timedwait end\n");	
	
        if(likely(!dev->cmd_err))
        {
		rtdm_printk(".......no dev->cmd_err return 0.......\n");
                return 0;               
        }
	
        if(dev->cmd_err & (OMAP_I2C_STAT_AL | OMAP_I2C_STAT_ROVR | OMAP_I2C_STAT_XUDF))
        {
        omap_i2c_reset(dev);
        __omap_i2c_init(dev);
        return -EIO;
        }
	
        if(dev->cmd_err & OMAP_I2C_STAT_NACK)
        {
                if(dev->i2c_stat_nack & I2C_M_IGNORE_NAK)
                return 0;

                if (stop)
                {
                        w = omap_i2c_read_reg(dev, OMAP_I2C_CON_REG);
                        w |= OMAP_I2C_CON_STP;
                        omap_i2c_write_reg(dev, OMAP_I2C_CON_REG,w);
                }
                return -EREMOTEIO;
        }
        return -EIO;
}

static ssize_t i2c_rd_rt(struct rtdm_dev_context *context,rtdm_user_info_t * user_info, void *buf,size_t nbyte)
{
	MY_DEV *dev=(MY_DEV *)context->device->device_data;
//	MY_DEV * dev = (MY_DEV *) context->dev_private;
	int r,i;
	int ret;
	int num;
//	u16 w;
//	int stop;
	 dev->read = 0x0001; //I2C_M_RD  
	 dev->buf_len = nbyte;

	rtdm_printk("i2c_rd_rt start\n");
	rtdm_printk("slave add to read=%x\n",dev->addr);

	 u8 *tmp;
        tmp=(u8 *)rtdm_malloc(nbyte);
//	dev->buf=0;
        dev->buf=(u8 *)tmp;
//	u16 mask,stat;
	rtdm_printk("..................dev->buf_len=%d\n",dev->buf_len);
	rtdm_printk("BUFFER ADDRESS IN I2C_RD_RT=%x\n",dev->buf);
	rtdm_printk("BUFFER ADDRESS IN I2C_RD_RT=%x\n",tmp);

//	resume(dev->dev);	
//	mask = omap_i2c_read_reg(dev, OMAP_I2C_IE_REG);
//        stat = omap_i2c_read_reg(dev, OMAP_I2C_STAT_REG);
//        rtdm_printk("mask=%ud\n",mask);
//        rtdm_printk("stat=%ud\n",stat);

//	r=pm_runtime_get_sync(dev->dev);
//	if(IS_ERR_VALUE(r))
//		goto out;

	r = omap_i2c_wait_for_bb(dev);
        if (r < 0)
	{	rtdm_printk("erro in omap_i2c_wait_for_bb\n");
                goto out;
	}
	if(dev->set_mpu_wkup_lat != NULL)
        {
        	dev->set_mpu_wkup_lat(dev->dev, dev->latency);
        }

	rtdm_printk("slave add to read=%x\n",dev->addr);

	num=1;
	for(i=0;i < num ;i++)
	{
	r = omap_i2c_rdwr_msg(dev,(i == (num - 1)));
	if(r !=0)
		break;
	}
	
//	r=omap_i2c_wait_for_bb(dev);
//        if(r < 0)
//                goto out;

	if(r == 0)
        	ret=nbyte;

//	rtdm_printk("BUFFER ADDRESS IN I2C_RD_RT=%x\n",dev->buf);
//        rtdm_printk("TMP ADDRESS IN I2C_RD_RT=%x\n",tmp);
//        rtdm_printk("TMP VALUE OF I2C_RD_RT=%x\n",*tmp);
//        tmp=tmp+1;
//        rtdm_printk("TMP ADDRESS IN I2C_RD_RT=%x\n",tmp);
//        rtdm_printk("TMP VALUE OF I2C_RD_RT=%x\n",*tmp);
//	if(rtdm_safe_copy_to_user(user_info,buf,(void *)tmp, dev->buf_len))
//                rtdm_printk("ERROR : can't copy data from driver\n");

	r=omap_i2c_wait_for_bb(dev);
        if(r < 0)
	{	
		rtdm_printk("error in omap_i2c_wait_for_bb\n");
               goto out;
	}
       if (dev->set_mpu_wkup_lat != NULL)
                dev->set_mpu_wkup_lat(dev->dev, -1);

	rtdm_printk("BUFFER ADDRESS IN I2C_RD_RT=%x\n",dev->buf);
	rtdm_printk("BUFFER ADDRESS IN I2C_RD_RT=%x\n",tmp);
	rtdm_printk("BUFFER VALUE OF I2C_RD_RT=%x\n",*tmp);
	tmp=tmp+1;
	rtdm_printk("BUFFER ADDRESS IN I2C_RD_RT=%x\n",tmp);
	rtdm_printk("BUFFER VALUE OF I2C_RD_RT=%x\n",*tmp);
	tmp=tmp-1;

	if(rtdm_safe_copy_to_user(user_info,buf,(void *)tmp, nbyte))
                rtdm_printk("ERROR : can't copy data from driver\n");

        rtdm_free(tmp);
out:
//        pm_runtime_mark_last_busy(dev->dev);
//        pm_runtime_put_autosuspend(dev->dev);

	rtdm_printk("i2c_rd_rt end\n");
        return ret;
}

static ssize_t i2c_wr_rt(struct rtdm_dev_context *context,rtdm_user_info_t * user_info,const void *buf, size_t nbyte)
{

	MY_DEV *dev=(MY_DEV *)context->device->device_data;
	int r,i;
        int ret;
        int num;
	char *tmp;
//      u16 w;
//	u16 mask,stat;
	dev->read = 0x0000;//I2C_M_RD  
	dev->buf_len = nbyte;
	tmp=rtdm_malloc(nbyte);

	  if ((rtdm_safe_copy_from_user(user_info,tmp, buf, dev->buf_len)))
                rtdm_printk("ERROR : can't copy data to driver\n");

	   dev->buf=(char *)tmp;

	rtdm_printk("i2c_wr_rt start\n");

	rtdm_printk("dev->buf=%x\n",*dev->buf);
        rtdm_printk("dev->buf=%x\n",*tmp);	 

//	r = pm_runtime_get_sync(dev->dev);
//        if (IS_ERR_VALUE(r))
//                goto out;

	rtdm_printk("slave add to write=%x\n",dev->addr);
	rtdm_printk("read or write=%d\n",dev->read);
	rtdm_printk("..................dev->buf_len=%d\n",dev->buf_len);
	 rtdm_printk("BUFFER ADDRESS IN I2C_WR_RT=%x",dev->buf);

	r = omap_i2c_wait_for_bb(dev);
        if (r < 0)
                goto out;

        if(dev->set_mpu_wkup_lat != NULL)
        {
                dev->set_mpu_wkup_lat(dev->dev, dev->latency);
        }

	num=1;
        for(i=0;i < num ;i++)
        {
	        r = omap_i2c_rdwr_msg(dev,(i == (num - 1)));
	        if(r !=0)
		{	rtdm_printk("omap_i2c_rdwr_msg\n");
	                break;
		}
        }

        if(r == 0)
	{	
                ret=nbyte;
	}

	r = omap_i2c_wait_for_bb(dev);

//        if (r < 0)
//                goto out;

        if(dev->set_mpu_wkup_lat != NULL)
                dev->set_mpu_wkup_lat(dev->dev,-1);
        out:
//        pm_runtime_mark_last_busy(dev->dev);
//        pm_runtime_put_autosuspend(dev->dev);
	rtdm_free(tmp);
	rtdm_printk("i2c_wr_rt end \n");
        return ret;
}

#define I2C_SLAVE		1

#define I2C_STOP		2

#define I2C_TEN			3

#define OMAP_I2C_NACK		4

static int i2c_ioctl_rt(struct rtdm_dev_context *context,rtdm_user_info_t * user_info,unsigned int request, void *arg)
{
	rtdm_printk("i2c_ioctl_rt start\n");
	int ret = 0;
//	MY_DEV * dev = (MY_DEV *) context->dev_private;
	MY_DEV *dev=(MY_DEV *)context->device->device_data;
	rtdm_printk("address of local struct MY_DEV=%d\n",dev);
	rtdm_printk("request for ioctl=%d",request);
	rtdm_printk("argument=%hx",(u16)arg);
//include i2c-dev.h
	switch (request) 
	{
	case I2C_SLAVE:
		rtdm_printk("I2C_SLAVE\n");
		dev->addr=(int )(u16 *)arg;		
		break;
		
	case I2C_STOP:
		rtdm_printk("I2C_STOP\n");
		dev->stop=(int)(unsigned long)arg;
		break;
			
	case I2C_TEN:
		rtdm_printk("I2C_TEN\n");
		dev->add_ten=(int) (u16 *)arg;	
		break;

	case OMAP_I2C_NACK:
		rtdm_printk("OMAP_I2C_NACK\n");
		dev->i2c_stat_nack = (int)(u16 *)arg;
		break;
	
//	case I2C_TIMEOUT:
//		break;
	
	default:
		rtdm_printk("default\n");
			
		rtdm_printk("ERROR : unknown request\n");
		ret = -1;
		break;
	};
	rtdm_printk("i2c_ioctl_rt close\n");
	return ret;
}	

static int i2c_close_nrt(struct rtdm_dev_context *context,rtdm_user_info_t * user_info)
{
	int err;
	 rtdm_printk("i2c_close_nrt start\n");
	 MY_DEV *dev=(MY_DEV *)context->device->device_data;
	 rtdm_printk("address of private struct=%d\n",dev);
	rtdm_free(dev->buf);
	
//	int err;
	err=rtdm_irq_disable(&dev->irq_handle);//enable irq
        if(err<0)
        {
                rtdm_printk("error in rtdm_irq_disable\n");
                return err;
        }
	rtdm_printk("rtdm_irq_disable\n");
//	rtdm_irq_free(&dev->irq_handle);
//	rtdm_printk("rtdm_irq_free\n");

//	rtdm_task_destroy(&dev->wd_task);
//	rtdm_printk("wd_task\n");
	rtdm_event_destroy(&dev->w_event);
	rtdm_printk("w_event\n");
//	rtdm_task_destroy(&dev->wd_task2);
//	rtdm_printk("wd_task2\n");
//	rtdm_task_destroy(&dev->wd_task3);
//	rtdm_printk("wd_task3\n");
//	rtdm_task_destroy(&dev->wd_task4);
//	rtdm_printk("wd_task4\n");
//	rtdm_task_destroy(&dev->wd_task5);
//	rtdm_printk("wd_task5\n");
//	rtdm_task_destroy(&dev->wait_task);
//	rtdm_printk("wait_task\n");
//	rtdm_printk("rtdm_task_destroy\n");
	rtdm_printk("i2c_close_nrt end\n");
	return 0;
}


static inline void omap_i2c_complete_cmd(MY_DEV *dev,u16 err)
{
	rtdm_printk("rtdm_event_signal in omap_i2c_complete_cmd\n");
	dev->cmd_err |=err;
	printk("rtdm_event_signal\n");
	rtdm_event_signal(&dev->w_event);
}

static inline void omap_i2c_ack_stat(MY_DEV *dev, u16 stat)
{
	rtdm_printk("writing stat\n");
	rtdm_printk("stat=%x",stat);
	omap_i2c_write_reg(dev, OMAP_I2C_STAT_REG, stat);
}

static inline void i2c_omap_errata_i207(MY_DEV *dev, u16 stat)
{
	rtdm_printk("i2c_omap_errata_i207 started\n");
	/*
	 * I2C Errata(Errata Nos. OMAP2: 1.67, OMAP3: 1.8)
	 * Not applicable for OMAP4.
	 * Under certain rare conditions, RDR could be set again
	 * when the bus is busy, then ignore the interrupt and
	 * clear the interrupt.
	 */
	if (stat & OMAP_I2C_STAT_RDR) {
		/* Step 1: If RDR is set, clear it */
		omap_i2c_ack_stat(dev, OMAP_I2C_STAT_RDR);

		/* Step 2: */
		if (!(omap_i2c_read_reg(dev, OMAP_I2C_STAT_REG) & OMAP_I2C_STAT_BB)) 
		{

			/* Step 3: */
			if (omap_i2c_read_reg(dev, OMAP_I2C_STAT_REG)	& OMAP_I2C_STAT_RDR) 
			{
				omap_i2c_ack_stat(dev, OMAP_I2C_STAT_RDR);
				dev_dbg(dev->dev, "RDR when bus is busy.\n");
			}

		}
	}
	rtdm_printk("i2c_omap_errata_i207 ended\n");
}

static irqreturn_t my_isr_1(int irq,void *dev_id)
{
	 irqreturn_t ret;
        rtdm_printk("..............my_isr_1..............\n");

        ret = IRQ_HANDLED;

	return ret;
}


//static int rtdm_my_isr_1(rtdm_irq_t *irq_context)
//{
  //      MY_DEV *dev=rtdm_irq_get_arg(irq_context,struct omap_i2c_dev );
//
//	rtdm_printk("............rtdm_my_isr_1...........\n");
//
//	suspend(dev->dev);
//
//	return IRQ_HANDLED;
//}
//#ifdef CONFIG_ARCH_OMAP15XX


static int rtdm_my_isr_1(rtdm_irq_t *irq_context)
{
	rtdm_printk("omap_i2c_omap1_isr started\n");
	int val,size=1;	
	struct omap_i2c_dev *dev=rtdm_irq_get_arg(irq_context,struct omap_i2c_dev );
	u16 iv,w;

//	 MY_DEV *dev = dev_id;

//	if(pm_runtime_suspended(dev->dev))
//	return IRQ_NONE;
	
	iv=omap_i2c_read_reg(dev, OMAP_I2C_IV_REG);
	rtdm_printk("IV=%x",iv);
	switch (iv)
	{
	case 0x00:
		break;
	case 0x01:
		dev_err(dev->dev,"Arbitration lost\n");
		omap_i2c_complete_cmd(dev, OMAP_I2C_STAT_AL);
		break;
	case 0x02:
		omap_i2c_complete_cmd(dev,OMAP_I2C_STAT_NACK);
		omap_i2c_write_reg(dev,OMAP_I2C_CON_REG,OMAP_I2C_CON_STP);
		break;
	case 0x03:
		omap_i2c_complete_cmd(dev,0);
		break;
	case 0x04:
		if(dev->buf_len)
		{
			w=omap_i2c_read_reg(dev,OMAP_I2C_DATA_REG);
			*dev->buf++ = w;
			dev->buf_len--;
			if(dev->buf_len)
			{
				*dev->buf++ = w >> 8;
				dev->buf_len--;
			}
		}
		else
		dev_err(dev->dev , "RRDY IRQ while no data requested\n");
		break;
	case 0x05:
		if(dev->buf_len)
		{
			w = *dev->buf++;
			dev->buf_len--;
			if(dev->buf_len){
				w |= *dev->buf++ <<8;
				dev->buf_len--;
			}
			omap_i2c_write_reg(dev,OMAP_I2C_DATA_REG, w);
		}
		else
			dev_err(dev->dev, "XRDY IRQ while no data to send\n");
		break;
	default:
		return IRQ_NONE ;
	}
	rtdm_printk("omap_i2c_omap1_isr ended\n");
	return RTDM_IRQ_HANDLED;
		
}
//#else
//#define omap_i2c_omap1_isr	NULL
//#endif


static int errata_omap3_i462(MY_DEV *dev)
{
	rtdm_printk("errata_omap3_i462 started\n");
	unsigned long timeout = 10000;
	u16 stat;

	rtdm_printk("address of local struct MY_DEV=%d\n",dev);


	do {
		stat = omap_i2c_read_reg(dev, OMAP_I2C_STAT_REG);
		if (stat & OMAP_I2C_STAT_XUDF)
			break;

		if (stat & (OMAP_I2C_STAT_NACK | OMAP_I2C_STAT_AL)) {
			omap_i2c_ack_stat(dev, (OMAP_I2C_STAT_XRDY |
							OMAP_I2C_STAT_XDR));
			if (stat & OMAP_I2C_STAT_NACK) {
				dev->cmd_err |= OMAP_I2C_STAT_NACK;
				omap_i2c_ack_stat(dev, OMAP_I2C_STAT_NACK);
			}

			if (stat & OMAP_I2C_STAT_AL) {
				dev_err(dev->dev, "Arbitration lost\n");
				dev->cmd_err |= OMAP_I2C_STAT_AL;
				omap_i2c_ack_stat(dev, OMAP_I2C_STAT_AL);
			}

			return -EIO;
		}

		cpu_relax();
	} while (--timeout);

	if (!timeout) {
		dev_err(dev->dev, "timeout waiting on XUDF bit\n");
		return 0;
	}
	rtdm_printk("errata_omap3_i462 ended\n");

	return 0;
}



static void omap_i2c_receive_data(struct omap_i2c_dev *dev, u8 num_bytes,bool is_rdr)
{
	rtdm_printk("omap_i2c_receive_data started\n");
	u16		w;
//	u16	mask,stat;
	rtdm_printk("address of local struct MY_DEV=%d\n",dev);
        rtdm_printk("BUFFER ADDRESS IN RECEIVE MODE=%x\n",dev->buf);
	while (num_bytes--) 
	{
	
//		mask = omap_i2c_read_reg(dev, OMAP_I2C_IE_REG);
//                 stat = omap_i2c_read_reg(dev, OMAP_I2C_STAT_REG);
//                 printk("value of IE in isr=%x\n",mask);
//                 printk("value of status in isr=%x\n",stat);

		w = omap_i2c_read_reg(dev, OMAP_I2C_DATA_REG);
		*dev->buf++ = w;
		dev->buf_len--;

		rtdm_printk("BUFFER ADDRESS IN RECEIVE MODE=%x\n",dev->buf);
                rtdm_printk("buffer value in omap_i2c_receive_data=%x\n",w);

		/*
		 * Data reg in 2430, omap3 and
		 * omap4 is 8 bit wide
		 */
		if (dev->flags & OMAP_I2C_FLAG_16BIT_DATA_REG) 
		{
			*dev->buf++ = w >> 8;
			dev->buf_len--;
		}
	}
	rtdm_printk("omap_i2c_receive_data ended\n");
}

static int omap_i2c_transmit_data(struct omap_i2c_dev *dev, u8 num_bytes,bool is_xdr)
{
	rtdm_printk("omap_i2c_trasmit_data started\n");
	u16		w;
//	u16		mask,stat;

	rtdm_printk("address of local struct MY_DEV=%d\n",dev);
	rtdm_printk("BUFFER ADDRESS IN TRASMIT MODE=%x\n",dev->buf);


	while (num_bytes--) 
	{	
		w = *dev->buf++;
		dev->buf_len--;

		rtdm_printk("BUFFER ADDRESS IN TRASMIT MODE=%x\n",dev->buf);
	        rtdm_printk("buffer value in omap_i2c_trasmit_data=%x\n",w);
		/*
		 * Data reg in 2430, omap3 and
		 * omap4 is 8 bit wide
		 */
		if (dev->flags & OMAP_I2C_FLAG_16BIT_DATA_REG) 
		{
			w |= *dev->buf++ << 8;
			dev->buf_len--;
		}
		if (dev->errata & I2C_OMAP_ERRATA_I462) 
		{
			int ret;
		
			ret = errata_omap3_i462(dev);
			if (ret < 0)
				return ret;
		}
//	         mask = omap_i2c_read_reg(dev, OMAP_I2C_IE_REG);
//                 stat = omap_i2c_read_reg(dev, OMAP_I2C_STAT_REG);
//                 printk("value of IE in isr=%x\n",mask);
//                 printk("value of status in isr=%x\n",stat);
	
		omap_i2c_write_reg(dev, OMAP_I2C_DATA_REG, w);
	}	
//	rtdm_printk("buffer value=%x\n",dev->buf);
//	rtdm_printk("buffer value=%x\n",w);	
	rtdm_printk("omap_i2c_trasmit_data ended\n");
	return 0;
}

static irqreturn_t my_isr_2(int irq,void *dev_id)
{
	 irqreturn_t ret;

         MY_DEV *dev = dev_id;

	rtdm_printk("..............my_isr_2..............\n");

//	ret = IRQ_HANDLED;

//	disable_irq(87);

//	suspend(dev->dev);

//	return IRQ_NONE;
	return IRQ_HANDLED;
}


///static int rtdm_my_isr_2(rtdm_irq_t *irq_context)
//{
//        MY_DEV *dev=rtdm_irq_get_arg(irq_context,struct omap_i2c_dev );

//        rtdm_printk("............rtdm_my_isr_2...........\n");

//        suspend(dev->dev);

//        return IRQ_HANDLED;
//}

//static irqreturn_t omap_i2c_isr(int irq,void *dev_id)

static int rtdm_my_isr_2(rtdm_irq_t *irq_context)
//static irqreturn_t omap_i2c_isr_thread(MY_DEV *dev)
{
		unsigned long flags;		
		u16 bits;
		u16 stat,w;
		u16 mask;
		int err = 0, count = 0;
		MY_DEV *dev = rtdm_irq_get_arg(irq_context,struct omap_i2c_dev);
		rtdm_lockctx_t context1;
		rtdm_printk("......rtdm_my_isr_2.........\n");
		unsigned long timeout,systime,systime1;

		 mask = omap_i2c_read_reg(dev, OMAP_I2C_IE_REG);
		 stat = omap_i2c_read_reg(dev, OMAP_I2C_STAT_REG);
       		 printk("value of IE in isr=%x",mask);
        	 printk("value of status in isr=%x",stat);


		err = rtdm_irq_disable(&dev->irq_handle);
		rtdm_lock_get_irqsave(&dev->lock,context1);
//		err = rtdm_irq_disable(&dev->irq_handle);
                if(err<0)
                    rtdm_printk("error in rtdm_irq_enable\n");

		do
		{
		        mask = omap_i2c_read_reg(dev, OMAP_I2C_IE_REG);
		        stat = omap_i2c_read_reg(dev, OMAP_I2C_STAT_REG);

		        rtdm_printk("mask=%ud\n",mask);
		        rtdm_printk("stat=%ud\n",stat);

			bits = omap_i2c_read_reg(dev, OMAP_I2C_IE_REG);
			stat = omap_i2c_read_reg(dev, OMAP_I2C_STAT_REG);
			stat &=bits;
			if(dev->receiver)
				stat &= ~(OMAP_I2C_STAT_XDR | OMAP_I2C_STAT_XRDY);
			else
				stat &= ~(OMAP_I2C_STAT_RDR | OMAP_I2C_STAT_RRDY);
			if(!stat)
			goto out;
		
//			dev_dbg(dev->dev, "IRQ (ISR = 0x%04x)\n", stat);
	
			if(count++ ==100)
			{
				dev_warn(dev->dev,"Too much work in one IRQ\n");
				break;
			}
		
			if(stat & OMAP_I2C_STAT_NACK)
			{
				rtdm_printk("OMAP_I2C_STAT_NACK\n");
				err|=OMAP_I2C_STAT_NACK;
				omap_i2c_ack_stat(dev,OMAP_I2C_STAT_NACK);
				break;
			}
			if (stat & OMAP_I2C_STAT_AL) 
			{
				rtdm_printk("OMAP_I2C_STAT_AL\n");
				dev_err(dev->dev, "Arbitration lost\n");
				err |= OMAP_I2C_STAT_AL;
				omap_i2c_ack_stat(dev, OMAP_I2C_STAT_AL);
				break;
			}
		/*
		 * ProDB0017052: Clear ARDY bit twice
		 */
			if (stat & (OMAP_I2C_STAT_ARDY | OMAP_I2C_STAT_NACK |
						OMAP_I2C_STAT_AL)) 
			{
				omap_i2c_ack_stat(dev, (OMAP_I2C_STAT_RRDY |
						OMAP_I2C_STAT_RDR |
						OMAP_I2C_STAT_XRDY |
						OMAP_I2C_STAT_XDR |
						OMAP_I2C_STAT_ARDY));
				break;
			}

			if (stat & OMAP_I2C_STAT_RDR) 
			{
			rtdm_printk("OMAP_I2C_STAT_RDR\n");
				u8 num_bytes = 1;

				if (dev->fifo_size)
					num_bytes = dev->buf_len;

				omap_i2c_receive_data(dev, num_bytes, true);

				if (dev->errata & I2C_OMAP_ERRATA_I207)
					i2c_omap_errata_i207(dev, stat);

				omap_i2c_ack_stat(dev, OMAP_I2C_STAT_RDR);
				continue;
			}

			if (stat & OMAP_I2C_STAT_RRDY) 
			{

				rtdm_printk("OMAP_I2C_STAT_RRDY\n");
				u8 num_bytes = 1;

				if (dev->threshold)
					num_bytes = dev->threshold;

				omap_i2c_receive_data(dev, num_bytes, false);
				omap_i2c_ack_stat(dev, OMAP_I2C_STAT_RRDY);
				continue;
			}

			if (stat & OMAP_I2C_STAT_XDR) 
			{
				u8 num_bytes = 1;
				int ret;

				rtdm_printk("OMAP_I2C_STAT_XDR\n");
				if (dev->fifo_size)
					num_bytes = dev->buf_len;

				ret = omap_i2c_transmit_data(dev, num_bytes, true);
				if (ret < 0)
					break;

				omap_i2c_ack_stat(dev, OMAP_I2C_STAT_XDR);
				continue;
			}

			if (stat & OMAP_I2C_STAT_XRDY) 
			{
				u8 num_bytes = 1;
				int ret;
				
				rtdm_printk("OMAP_I2C_STAT_XRDY\n");
				
				if (dev->threshold)
					num_bytes = dev->threshold;

				rtdm_printk("dev->threshold=%d\n",dev->threshold);
				
				ret = omap_i2c_transmit_data(dev, num_bytes, false);
				if (ret < 0)
					break;
				
				omap_i2c_ack_stat(dev, OMAP_I2C_STAT_XRDY);
				continue;
			}
				
			if (stat & OMAP_I2C_STAT_ROVR) 
			{
			rtdm_printk("OMAP_I2C_STAT_ROVR\n");
			dev_err(dev->dev, "Receive overrun\n");
			err |= OMAP_I2C_STAT_ROVR;
			omap_i2c_ack_stat(dev, OMAP_I2C_STAT_ROVR);
			break;
			}	
			if (stat & OMAP_I2C_STAT_XUDF) 
			{		
			rtdm_printk("OMAP_I2C_STAT_XUDF\n");
			dev_err(dev->dev, "Transmit underflow\n");
			err |= OMAP_I2C_STAT_XUDF;
			omap_i2c_ack_stat(dev, OMAP_I2C_STAT_XUDF);
			break;
			}

//	mask = omap_i2c_read_reg(dev, OMAP_I2C_IE_REG);
//        stat = omap_i2c_read_reg(dev, OMAP_I2C_STAT_REG);
//        rtdm_printk("mask=%ud\n",mask);
//        rtdm_printk("stat=%ud\n",stat);
//	rtdm_printk("suspend\n");
//	suspend(dev->dev);
	}
//	while(0);
	while(stat);

	omap_i2c_complete_cmd(dev,err);
out:
//	 err = rtdm_irq_enable(&dev->irq_handle);
//       if(err<0)
//           rtdm_printk("error in rtdm_irq_enable\n");
	rtdm_lock_put_irqrestore(&dev->lock,context1);
	err = rtdm_irq_enable(&dev->irq_handle);
         if(err<0)
              rtdm_printk("error in rtdm_irq_enable\n");
//	  spin_unlock_irqrestore(&dev->lock1,flags);	
//	  mask = omap_i2c_read_reg(dev, OMAP_I2C_IE_REG);
//        stat = omap_i2c_read_reg(dev, OMAP_I2C_STAT_REG);
//        rtdm_printk("isr ended :mask=%ud\n",mask);
//        rtdm_printk("isr ended :stat=%ud\n",stat);
//	  rtdm_printk("irq handled\n");
	return RTDM_IRQ_HANDLED;
}
 	               
//static int rtdm_my_isr_2(rtdm_irq_t *irq_context)
//{
//	MY_DEV *dev = dev_id;
//	unsigned long timeout,systime,systime1;
//	u16 w,err;
//	int ret;

//	rtdm_printk("........omap_i2c_isr started..........\n");

//	irqreturn_t ret = IRQ_HANDLED;

//	MY_DEV *dev = rtdm_irq_get_arg(irq_context,struct omap_i2c_dev );

//	rtdm_printk("address of local struct MY_DEV=%d\n",dev);


//	 err = rtdm_irq_disable(&dev->irq_handle);
//       if(err<0)
//                      rtdm_printk("error in rtdm_irq_enable\n");

//	ret=omap_i2c_isr_thread(dev);
//	err = rtdm_irq_enable(&dev->irq_handle);
  //     if(err<0)
//        	      rtdm_printk("error in rtdm_irq_enable\n");
	
	
//	suspend(dev->dev);
//	ret=omap_i2c_isr_thread(dev);
/*	
	u16 mask;
	u16 stat;
	
	
//	rtdm_printk("address of private struct=%d\n",dev);
	rtdm_lock_get(&dev->lock_1);

	mask = omap_i2c_read_reg(dev,OMAP_I2C_IE_REG);
	stat = omap_i2c_read_reg(dev,OMAP_I2C_STAT_REG);

	if(stat & mask)
	{

	        //check for stop condition
      if(dev->b_hw && dev->stop)
        {
                systime = rtdm_clock_read();

                timeout = systime + OMAP_I2C_TIMEOUT;

                u16 con = omap_i2c_read_reg(dev, OMAP_I2C_CON_REG);
              rtdm_printk("read from OMAP_I2C_CON_REG\n");

                while(con & OMAP_I2C_CON_STT)
                {

                con = omap_i2c_read_reg(dev,OMAP_I2C_CON_REG);
                systime1 = rtdm_clock_read();

                        if(systime1 > timeout)
                        {
                        dev_err(dev->dev,"controller time out waiting for start condition to finish\n");
                        return -ETIMEDOUT;
                        }
                        cpu_relax();
                }
        w |= OMAP_I2C_CON_STP;
        w &= ~OMAP_I2C_CON_STT;
        omap_i2c_write_reg(dev, OMAP_I2C_CON_REG , w);
        }

	err = rtdm_irq_disable(&dev->irq_handle);
        if(err<0)
                rtdm_printk("error in rtdm_irq_enable\n");

	ret=omap_i2c_isr_thread(dev);

	err = rtdm_irq_enable(&dev->irq_handle);
         if(err<0)
              rtdm_printk("error in rtdm_irq_enable\n");


	}

	rtdm_lock_put(&dev->lock_1);
*/
//	ret=RTDM_IRQ_HANDLED;
//	return ret;
//}

//static int rtdm_my_isr_2(rtdm_irq_t *irq_context)
//{



//return ret;
//}

#ifdef CONFIG_OF
static struct omap_i2c_bus_platform_data omap3_pdata = {
	.rev = OMAP_I2C_IP_VERSION_1,
	.flags = OMAP_I2C_FLAG_BUS_SHIFT_2,
};

static struct omap_i2c_bus_platform_data omap4_pdata = {
	.rev = OMAP_I2C_IP_VERSION_2,
};

static const struct of_device_id omap_i2c_of_match[] = {
	{
		.compatible = "ti,omap4-i2c",
		.data = &omap4_pdata,
	},
	{
		.compatible = "ti,omap3-i2c",
		.data = &omap3_pdata,
	},
	{ },
};
MODULE_DEVICE_TABLE(of, omap_i2c_of_match);
#endif

static struct rtdm_device i2c_device= {

        .struct_version         = RTDM_DEVICE_STRUCT_VER,
        .device_flags           = RTDM_NAMED_DEVICE,
        .context_size           = sizeof(MY_DEV),
        .device_name            = DEVICE_NAME,
        .proc_name              = DEVICE_NAME,
        .open_nrt               = i2c_open_nrt,

        .ops={
                .close_nrt      =	i2c_close_nrt,
                .read_rt        =	i2c_rd_rt,
                .write_rt       =	i2c_wr_rt,
		.ioctl_rt  	=	i2c_ioctl_rt,
        },

        .device_class           =RTDM_CLASS_SERIAL,
        .device_sub_class       =2015,
        .profile_version        =1,
        .driver_name            =DRIVER_NAME,
        .driver_version         =RTDM_DRIVER_VER(0,0,1),
        .peripheral_name        ="RTDM I2C MASTER",
        .provider_name          ="JAY KOTHARI",
};


#define OMAP_I2C_SCHEME(rev)		((rev & 0xc000) >> 14)

#define OMAP_I2C_REV_SCHEME_0_MAJOR(rev) (rev >> 4)
#define OMAP_I2C_REV_SCHEME_0_MINOR(rev) (rev & 0xf)

#define OMAP_I2C_REV_SCHEME_1_MAJOR(rev) ((rev & 0x0700) >> 7)
#define OMAP_I2C_REV_SCHEME_1_MINOR(rev) (rev & 0x1f)
#define OMAP_I2C_SCHEME_0			0
#define OMAP_I2C_SCHEME_1			1

static int omap_i2c_probe(struct platform_device *pdev)
{
	printk("omap_i2c_probe started\n");
	MY_DEV			*dev;
	struct resource		*mem;
	const struct omap_i2c_bus_platform_data *pdata = pdev->dev.platform_data;
	struct device_node	*node = pdev->dev.of_node;
	const struct of_device_id *match;
	int irq;
	int r,err;
	int ret;
	u32 rev;
	u16 minor, major, scheme;
	u16 addr,mask,stat;
	struct rtdm_device *rdev;	
	void __iomem 		*mem_1;
//	rtdm_printk("address of local struct MY_DEV=%d",dev);

	/* NOTE: driver uses the static register mapping */
	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!mem) {
		dev_err(&pdev->dev, "no mem resource?\n");
		return -ENODEV;
	}
//	printk("platform_get_resource\n");

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(&pdev->dev, "no irq resource?\n");
		return irq;
	}
//	printk("platform_get_irq\n");

	dev = devm_kzalloc(&pdev->dev, sizeof(MY_DEV), GFP_KERNEL);
	if (!dev) {
		dev_err(&pdev->dev, "Menory allocation failed\n");
		return -ENOMEM;
	}

//	printk("devm_kzalloc\n");

	rtdm_printk("address of local struct=%d\n",dev);


	dev->base = devm_request_and_ioremap(&pdev->dev, mem);
	if (!dev->base) {
		dev_err(&pdev->dev, "I2C region already claimed\n");
		return -ENOMEM;
	}
	printk("dev_request_and_ioremap\n");

	rtdm_printk("clock enabling.......i2c1\n");
	mem_1 = ioremap(SOC_PRCM_REGS, SOC_PRCM_SIZE);
	
	if(!mem) 
         {
            printk (KERN_ERR "HI: ERROR: Failed to remap memory for GPIO Bank 2 IRQ pin configuration.\n");
            return 0;
         }

	int retval;
	retval=ioread32(mem_1 + 0x48);
        rtdm_printk("value of clock i2c enable retval=%d\n",retval);
	iowrite32(CM_PER_I2C1_CLKCTRL, mem_1 + 0x48);
	retval = ioread32(mem_1 + 0x48);
	rtdm_printk("value of clock i2c enable retval=%d\n",retval);	
	rtdm_printk("clock enable for i2c1\n");

	rdev=kzalloc(sizeof(struct rtdm_device),GFP_KERNEL);
	rdev = &dev->rtdm_dev;
	memcpy(rdev, &i2c_device, sizeof(struct rtdm_device));	
	ret=rtdm_dev_register(rdev);
	if(ret<0)       
     	{
                    printk("RTDM device not registered\n"); 
        }

	rdev->device_data =  devm_kzalloc(&pdev->dev, sizeof(MY_DEV), GFP_KERNEL);

	rdev->device_data = dev;


	match = of_match_device(of_match_ptr(omap_i2c_of_match), &pdev->dev);
	if(match) 
	{
		u32 freq = 100000; /* default to 100000 Hz */
		pdata = match->data;
		dev->flags = pdata->flags;
		of_property_read_u32(node, "clock-frequency", &freq);
		/* convert DT freq value in Hz into kHz for speed */
		dev->speed = freq / 1000;
	} else if (pdata != NULL) 
	{
		dev->speed = pdata->clkrate;
		dev->flags = pdata->flags;
		dev->set_mpu_wkup_lat = pdata->set_mpu_wkup_lat;
	}
//	printk("of_match_device\n");
	dev->pins = devm_pinctrl_get_select_default(&pdev->dev);
	if (IS_ERR(dev->pins)) 
	{
		if (PTR_ERR(dev->pins) == -EPROBE_DEFER)
			return -EPROBE_DEFER;
	
		dev_warn(&pdev->dev, "did not get pins for i2c error: %li\n",
			 PTR_ERR(dev->pins));
		dev->pins = NULL;
	}
	printk("dev->pins\n");
	
	dev->dev = &pdev->dev;
	dev->irq = irq;
	
//	rtdm_printk("irq=%d\n",irq);
	

//	rtdm_lock_init(&dev->lock);
//	rtdm_lock_init(&dev->lock_1);
//	spin_lock_init(&dev->lock1);
	
//	rtdm_printk("rtdm_lock_init\n");
	
	platform_set_drvdata(pdev, dev);

//	rtdm_event_init(&dev->w_event,0);
//	rtdm_printk("rtdm_event_init done\n");

	
	dev->reg_shift = (dev->flags >> OMAP_I2C_FLAG_BUS_SHIFT__SHIFT) & 3;
	rtdm_printk("dev->reg_shift\n");

	
//	r = resume(dev->dev);
//	if (IS_ERR_VALUE(r))
//	{	rtdm_printk("error in resume\n");
//		goto err_free_mem;
//	}

//	pm_runtime_enable(dev->dev);
//        pm_runtime_set_autosuspend_delay(dev->dev, OMAP_I2C_PM_TIMEOUT);
//        pm_runtime_use_autosuspend(dev->dev);

//        r = pm_runtime_get_sync(dev->dev);
//        if (IS_ERR_VALUE(r))
//                goto err_free_mem;

//	 rtdm_printk("prob finished omap_i2c_init\n");
	
	/*
	 * Read the Rev hi bit-[15:14] ie scheme this is 1 indicates ver2.
	 * On omap1/3/2 Offset 4 is IE Reg the bit [15:14] is 0 at reset.
	 * Also since the omap_i2c_read_reg uses reg_map_ip_* a
	 * raw_readw is done.
	 */

        printk("base address=%x\n",dev->base);

        addr = (u16 )(dev->base + 0x04);
	
        printk("base address=%x\n",addr);
        rev = ioread16(dev->base + 0x04);
        printk("__raw_readw\n");
	
	scheme = OMAP_I2C_SCHEME(rev);
	switch (scheme) 
	{
	case OMAP_I2C_SCHEME_0:
		rtdm_printk("OMAP_I2C_SCHEME_0\n");
		dev->regs = (u8 *)reg_map_ip_v1;
		dev->rev = omap_i2c_read_reg(dev, OMAP_I2C_REV_REG);
		minor = OMAP_I2C_REV_SCHEME_0_MAJOR(dev->rev);
		major = OMAP_I2C_REV_SCHEME_0_MAJOR(dev->rev);
		break;
	case OMAP_I2C_SCHEME_1:
		 rtdm_printk("OMAP_I2C_SCHEME_1\n");

		/* FALLTHROUGH */
	default:
		 rtdm_printk("default\n");
		dev->regs = (u8 *)reg_map_ip_v2;
		rev = (rev << 16) |
			omap_i2c_read_reg(dev, OMAP_I2C_IP_V2_REVNB_LO);
		minor = OMAP_I2C_REV_SCHEME_1_MINOR(rev);
		major = OMAP_I2C_REV_SCHEME_1_MAJOR(rev);
		dev->rev = rev;
	}
	
	dev->errata = 0;
	
	if (dev->rev >= OMAP_I2C_REV_ON_2430 &&
			dev->rev < OMAP_I2C_REV_ON_4430_PLUS)
		dev->errata |= I2C_OMAP_ERRATA_I207;
	
	if (dev->rev <= OMAP_I2C_REV_ON_3430_3530)
		dev->errata |= I2C_OMAP_ERRATA_I462;
	
	if (!(dev->flags & OMAP_I2C_FLAG_NO_FIFO)) {
		u16 s;
	
		/* Set up the fifo size - Get total size */
		s = (omap_i2c_read_reg(dev, OMAP_I2C_BUFSTAT_REG) >> 14) & 0x3;
		dev->fifo_size = 0x8 << s;
	
		/*
		 * Set up notification threshold as half the total available
		 * size. This is to ensure that we can handle the status on int
		 * call back latencies.
		 */
	
		dev->fifo_size = (dev->fifo_size / 2);
	
		if (dev->rev < OMAP_I2C_REV_ON_3630)
			dev->b_hw = 1; /* Enable hardware fixes */
	
		/* calculate wakeup latency constraint for MPU */
		if (dev->set_mpu_wkup_lat != NULL)
			dev->latency = (1000000 * dev->fifo_size) /
				       (1000 * dev->speed / 8);
	}
	
	/* reset ASAP, clearing any IRQs */
	omap_i2c_init(dev);
	printk("omap_i2c_init :\n");

	if(dev->rev < OMAP_I2C_OMAP1_REV_2)
	{
	r = devm_request_irq(&pdev->dev,dev->irq,my_isr_1,0,pdev->name, dev);
	}
	else
	{
	r = devm_request_irq(&pdev->dev,dev->irq,my_isr_2,0, pdev->name , dev);
	}
//	printk("OMAP_i2c_isr completeed\n");


	if(dev->rev < OMAP_I2C_OMAP1_REV_2)
              err=rtdm_irq_request(&dev->irq_handle,dev->irq,rtdm_my_isr_1,0,pdev->name,dev);
        else
              err=rtdm_irq_request(&dev->irq_handle,dev->irq,rtdm_my_isr_2,0,pdev->name,dev);

        if(err<0)
        {
              rtdm_printk("error in requesting irq\n");
              dev_err(dev->dev, "failure requesting irq %i\n", dev->irq);
              goto err_unuse_clocks;
             return err;   //do some diffrent approach
        }

//	err=irq_set_irq_type(dev->irq,IRQ_TYPE_EDGE_BOTH);//set irq type
//	if (err < 0)
//		{
//		rtdm_printk("error in irq_set_irq_type\n");
//		}
	
//	rtdm_printk("rtdm_irq_request for irq\n");
//	if(dev->rev < OMAP_I2C_OMAP1_REV_2)
//		err=rtdm_irq_request(&dev->irq_handle,dev->irq,omap_i2c_omap1_isr,RTDM_IRQTYPE_SHARED ,pdev->name,dev);
//	else
//		err=rtdm_irq_request(&dev->irq_handle,dev->irq,omap_i2c_isr,RTDM_IRQTYPE_SHARED ,pdev->name,dev);

//	if(err<0)
//	{
//            rtdm_printk("error in requesting irq\n");
//		dev_err(dev->dev, "failure requesting irq %i\n", dev->irq);
//		goto err_unuse_clocks;
		
//              return err;   //do some diffrent approach
//	}
//

//	else rtdm_printk("rtdm_irq_request complete\n");

//	 err = rtdm_irq_enable(&dev->irq_handle);
//	if(err<0)
//		rtdm_printk("error in rtdm_irq_enable\n");

//	irq_set_irq_type(dev->irq,IRQ_TYPE_LEVEL_LOW);//set irq type
//	rtdm_printk("rtdm_irq_request done\n");
	
	rtdm_printk("address of local struct MY_DEV=%d\n",dev);
	rtdm_printk("prob finished\n");
	
//        pm_runtime_mark_last_busy(dev->dev);
//        pm_runtime_put_autosuspend(dev->dev);

        return 0;

err_unuse_clocks:
        omap_i2c_write_reg(dev, OMAP_I2C_CON_REG, 0);
//        pm_runtime_put(dev->dev);
//        pm_runtime_disable(&pdev->dev);
err_free_mem:
        platform_set_drvdata(pdev, NULL);

        return r;
}

static int omap_i2c_remove(struct platform_device *pdev)
{
	rtdm_printk("omap_i2c_remove\n");
	struct omap_i2c_dev	*dev = platform_get_drvdata(pdev);
	int ret;

	rtdm_printk("omap_i2c_remove");
	
	platform_set_drvdata(pdev, NULL);

//	i2c_del_adapter(&dev->adapter);

//	ret = resume(&pdev->dev);
//	if (IS_ERR_VALUE(ret))
//		return ret;

	omap_i2c_write_reg(dev, OMAP_I2C_CON_REG, 0);
//	pm_runtime_put(&pdev->dev);

//	pm_runtime_disable(&pdev->dev);

	rtdm_printk("omap_i2c_remove end\n");

	rtdm_event_destroy(&dev->w_event);

	return 0;
}

//#ifdef CONFIG_PM
//#ifdef CONFIG_PM_RUNTIME
//static int omap_i2c_runtime_suspend(struct device *dev)
//{
//	rtdm_printk("suspend start\n");
  //      struct platform_device *pdev = to_platform_device(dev);
//        struct omap_i2c_dev *_dev = platform_get_drvdata(pdev);

//        _dev->iestate = omap_i2c_read_reg(_dev, OMAP_I2C_IE_REG);

//        omap_i2c_write_reg(_dev, OMAP_I2C_IE_REG, 0);

//        if (_dev->rev < OMAP_I2C_OMAP1_REV_2) {
//                omap_i2c_read_reg(_dev, OMAP_I2C_IV_REG); /* Read clears */
//        } else {
//                omap_i2c_write_reg(_dev, OMAP_I2C_STAT_REG, _dev->iestate);
//
//                /* Flush posted write */
//                omap_i2c_read_reg(_dev, OMAP_I2C_STAT_REG);
//        }
//	rtdm_printk("suspend end\n");
//        return 0;
//}

//static int omap_i2c_runtime_resume(struct device *dev)
//{
//	rtdm_printk("resume start\n");
//        struct platform_device *pdev = to_platform_device(dev);
//        struct omap_i2c_dev *_dev = platform_get_drvdata(pdev);
//
//        if (!_dev->regs)
//                return 0;
//
//      __omap_i2c_init(_dev);
//
//	rtdm_printk("resume end\n");
//
//        return 0;
//}
//#endif /* CONFIG_PM_RUNTIME */

//static struct dev_pm_ops omap_i2c_pm_ops = {
//        SET_RUNTIME_PM_OPS(omap_i2c_runtime_suspend,
//                           omap_i2c_runtime_resume, NULL)
//};

//#define OMAP_I2C_PM_OPS (&omap_i2c_pm_ops)
//#else
//#define OMAP_I2C_PM_OPS NULL
//#endif /* CONFIG_PM */

struct platform_driver omap_i2c_driver = {
	.probe		= omap_i2c_probe,
	.remove		= omap_i2c_remove,
	.driver		= {
		.name	= DRIVER_NAME,
		.owner	= THIS_MODULE,
//		.pm	= OMAP_I2C_PM_OPS,
		.of_match_table = of_match_ptr(omap_i2c_of_match),
	},
};

/* I2C may be needed to bring up other drivers */
static int __init omap_i2c_init_driver(void)
{
	int ret;
	rtdm_printk("omap_i2c_init_driver function\n");
	return platform_driver_register(&omap_i2c_driver);
}

//subsys_initcall(omap_i2c_init_driver);
module_init(omap_i2c_init_driver);

static void __exit omap_i2c_exit_driver(void)
{
	rtdm_printk("omap_i2c_exit_driver exit\n");

	platform_driver_unregister(&omap_i2c_driver);

	rtdm_dev_unregister(&i2c_device, 1000); 

}
module_exit(omap_i2c_exit_driver);

MODULE_AUTHOR("JAY KOTHARI <jaikothari10@gmail.com>");
MODULE_DESCRIPTION("TI OMAP I2C bus adapter");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:omap_i2c");


