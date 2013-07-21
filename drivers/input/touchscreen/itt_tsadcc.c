/*
 *  Atmel Touch Screen Driver
 *
 *  Copyright (c) 2008 ATMEL
 *  Copyright (c) 2008 Dan Liang
 *  Copyright (c) 2008 TimeSys Corporation
 *  Copyright (c) 2008 Justin Waters
 *
 *  Based on touchscreen code from Atmel Corporation.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */
#include <linux/init.h>
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <mach/board.h>
#include <linux/platform_data/atmel.h>
#include <mach/cpu.h>
#include <mach/at91_pio.h>

/* Register definitions based on AT91SAM9RL64 preliminary draft datasheet */

#define ATMEL_TSADCC_CR		0x00	/* Control register */
#define   ATMEL_TSADCC_SWRST	(1 << 0)	/* Software Reset*/
#define	  ATMEL_TSADCC_START	(1 << 1)	/* Start conversion */

#define ATMEL_TSADCC_MR		0x04	/* Mode register */
#define	  ATMEL_TSADCC_TSAMOD	(3    <<  0)	/* ADC mode */
#define	    ATMEL_TSADCC_TSAMOD_ADC_ONLY_MODE	(0x0)	/* ADC Mode */
#define	    ATMEL_TSADCC_TSAMOD_TS_ONLY_MODE	(0x1)	/* Touch Screen Only Mode */
#define	  ATMEL_TSADCC_LOWRES	(1    <<  4)	/* Resolution selection */
#define	  ATMEL_TSADCC_SLEEP	(1    <<  5)	/* Sleep mode */
#define	  ATMEL_TSADCC_PENDET	(1    <<  6)	/* Pen Detect selection */
#define	  ATMEL_TSADCC_PRES	(1    <<  7)	/* Pressure Measurement Selection */
#define	  ATMEL_TSADCC_PRESCAL	(0x3f <<  8)	/* Prescalar Rate Selection */
#define	  ATMEL_TSADCC_EPRESCAL	(0xff <<  8)	/* Prescalar Rate Selection (Extended) */
#define	  ATMEL_TSADCC_STARTUP	(0x7f << 16)	/* Start Up time */
#define	  ATMEL_TSADCC_SHTIM	(0xf  << 24)	/* Sample & Hold time */
#define	  ATMEL_TSADCC_PENDBC	(0xf  << 28)	/* Pen Detect debouncing time */

#define ATMEL_TSADCC_TRGR	0x08	/* Trigger register */
#define	  ATMEL_TSADCC_TRGMOD	(7      <<  0)	/* Trigger mode */
#define	    ATMEL_TSADCC_TRGMOD_NONE		(0 << 0)
#define     ATMEL_TSADCC_TRGMOD_EXT_RISING	(1 << 0)
#define     ATMEL_TSADCC_TRGMOD_EXT_FALLING	(2 << 0)
#define     ATMEL_TSADCC_TRGMOD_EXT_ANY		(3 << 0)
#define     ATMEL_TSADCC_TRGMOD_PENDET		(4 << 0)
#define     ATMEL_TSADCC_TRGMOD_PERIOD		(5 << 0)
#define     ATMEL_TSADCC_TRGMOD_CONTINUOUS	(6 << 0)
#define   ATMEL_TSADCC_TRGPER	(0xffff << 16)	/* Trigger period */

#define ATMEL_TSADCC_TSR	0x0C	/* Touch Screen register */
#define	  ATMEL_TSADCC_TSFREQ	(0xf <<  0)	/* TS Frequency in Interleaved mode */
#define	  ATMEL_TSADCC_TSSHTIM	(0xf << 24)	/* Sample & Hold time */

#define ATMEL_TSADCC_CHER	0x10	/* Channel Enable register */
#define ATMEL_TSADCC_CHDR	0x14	/* Channel Disable register */
#define ATMEL_TSADCC_CHSR	0x18	/* Channel Status register */
#define	  ATMEL_TSADCC_CH(n)	(1 << (n))	/* Channel number */

#define ATMEL_TSADCC_SR		0x1C	/* Status register */
#define	  ATMEL_TSADCC_EOC(n)	(1 << ((n)+0))	/* End of conversion for channel N */
#define	  ATMEL_TSADCC_OVRE(n)	(1 << ((n)+8))	/* Overrun error for channel N */
#define	  ATMEL_TSADCC_DRDY	(1 << 16)	/* Data Ready */
#define	  ATMEL_TSADCC_GOVRE	(1 << 17)	/* General Overrun Error */
#define	  ATMEL_TSADCC_ENDRX	(1 << 18)	/* End of RX Buffer */
#define	  ATMEL_TSADCC_RXBUFF	(1 << 19)	/* TX Buffer full */
#define	  ATMEL_TSADCC_PENCNT	(1 << 20)	/* Pen contact */
#define	  ATMEL_TSADCC_NOCNT	(1 << 21)	/* No contact */

#define ATMEL_TSADCC_LCDR	0x20	/* Last Converted Data register */
#define	  ATMEL_TSADCC_DATA	(0x3ff << 0)	/* Channel data */

#define ATMEL_TSADCC_IER	0x24	/* Interrupt Enable register */
#define ATMEL_TSADCC_IDR	0x28	/* Interrupt Disable register */
#define ATMEL_TSADCC_IMR	0x2C	/* Interrupt Mask register */
#define ATMEL_TSADCC_CDR0	0x30	/* Channel Data 0 */
#define ATMEL_TSADCC_CDR1	0x34	/* Channel Data 1 */
#define ATMEL_TSADCC_CDR2	0x38	/* Channel Data 2 */
#define ATMEL_TSADCC_CDR3	0x3C	/* Channel Data 3 */
#define ATMEL_TSADCC_CDR4	0x40	/* Channel Data 4 */
#define ATMEL_TSADCC_CDR5	0x44	/* Channel Data 5 */

#define ATMEL_TSADCC_XPOS	0x50
#define ATMEL_TSADCC_Z1DAT	0x54
#define ATMEL_TSADCC_Z2DAT	0x58

#define PRESCALER_VAL(x)	((x) >> 8)

#define ADC_DEFAULT_CLOCK	100000
#define ZTHRESHOLD		3200
#define ATMEL_TOUCHSCREEN_TIMEOUT	5000000
#define ATMEL_MASTER_CLOCK          (133333200UL)
#define ATMEL_ADS_CLOCK             300000
#define ATMEL_ADS_STARTUP           500
#define ATMEL_ADS_SHTIM             0xf
#define ATMEL_ADS_DEBOUNCE          10
#define TOUCH_PRESSURE_LIMIT        2500
#define TOUCH_NOISE_LIMIT           0x0A

struct atmel_tsadcc {
	struct input_dev	*input;
	char			phys[32];
	struct clk		*clk;
	int			irq;
	unsigned int		prev_absx;
	unsigned int		prev_absy;
	unsigned int		prev_absz;

	struct at91_tsadcc_data board;
	unsigned char		bufferedmeasure;
    unsigned char       pendbc;
    unsigned char       reported;
    unsigned char       measurenumber;
    unsigned int        measurementX[5];
    unsigned int        measurementY[5];
    unsigned int        measurementZ[5];
	unsigned int        buttonDown;
};

static void __iomem		*tsc_base;

#define atmel_tsadcc_read(reg)		__raw_readl(tsc_base + (reg))
#define atmel_tsadcc_write(reg, val)	__raw_writel((val), tsc_base + (reg))

static void atmel_tsadcc_dump_conf(struct platform_device *pdev)
{
	dev_info(&pdev->dev, "--- configuration ---\n");
	dev_info(&pdev->dev, "Mode Register: %#x\n", atmel_tsadcc_read(ATMEL_TSADCC_MR));
	dev_info(&pdev->dev, "Trigger Register: %#x\n", atmel_tsadcc_read(ATMEL_TSADCC_TRGR));
	dev_info(&pdev->dev, "ADC Channel Status Register: %#x\n", atmel_tsadcc_read(ATMEL_TSADCC_CHSR));
	dev_info(&pdev->dev, "---------------------\n");
}

static irqreturn_t atmel_tsadcc_interrupt(int irq, void *dev)
{ 
	struct atmel_tsadcc	*ts_dev = (struct atmel_tsadcc *)dev;
	struct input_dev	*input_dev = ts_dev->input;
	struct at91_tsadcc_data *pdata = &ts_dev->board;

	unsigned int status;
	unsigned int reg, pres, z1dat, z2dat, temp, i, j, lowSampNum, HiSampNum;
	unsigned int nSamp[5];
	unsigned int X, Y, Z;

	status = atmel_tsadcc_read(ATMEL_TSADCC_SR);
	status &= atmel_tsadcc_read(ATMEL_TSADCC_IMR);
	if (status & ATMEL_TSADCC_NOCNT) {
		/* Contact lost */
		reg = atmel_tsadcc_read(ATMEL_TSADCC_MR) | (ts_dev->pendbc << 28);

		atmel_tsadcc_write(ATMEL_TSADCC_MR, reg);
		atmel_tsadcc_write(ATMEL_TSADCC_TRGR, ATMEL_TSADCC_TRGMOD_NONE);
		atmel_tsadcc_write(ATMEL_TSADCC_IDR,
				   ATMEL_TSADCC_EOC(3) | ATMEL_TSADCC_NOCNT);
		atmel_tsadcc_write(ATMEL_TSADCC_IER, ATMEL_TSADCC_PENCNT);
		atmel_tsadcc_write(ATMEL_TSADCC_IMR, ATMEL_TSADCC_PENCNT);

		if (ts_dev->reported == 1)
		{
			input_report_abs(input_dev, ABS_X, ts_dev->prev_absx);
			input_report_abs(input_dev, ABS_Y, ts_dev->prev_absy);
			input_report_abs(input_dev, ABS_PRESSURE, ts_dev->prev_absz);
			input_report_key(input_dev, BTN_TOUCH, 1);
			input_sync(input_dev);

			input_report_key(input_dev, BTN_TOUCH, 0);
			input_report_abs(input_dev, ABS_PRESSURE, 0);
			input_sync(input_dev);
		}
		ts_dev->reported = 0;
		ts_dev->measurenumber = 0;
		ts_dev->buttonDown = false;

	} else if (status & ATMEL_TSADCC_PENCNT) {
		/* Pen detected */
		reg = atmel_tsadcc_read(ATMEL_TSADCC_MR);
		reg &= ~ATMEL_TSADCC_PENDBC;
        ts_dev->reported = 0;
        ts_dev->measurenumber = 0;
        for (i = 0; i < 5; i++)
        {
            ts_dev->measurementX[i]=0;
            ts_dev->measurementY[i]=0;
        }
		ts_dev->buttonDown = true;
		ts_dev->prev_absx = 0;
		ts_dev->prev_absy = 0;
		ts_dev->prev_absz = 0;

		atmel_tsadcc_write(ATMEL_TSADCC_IDR, ATMEL_TSADCC_PENCNT);
		atmel_tsadcc_write(ATMEL_TSADCC_MR, reg);
		atmel_tsadcc_write(ATMEL_TSADCC_IER,
				   ATMEL_TSADCC_EOC(3) | ATMEL_TSADCC_NOCNT);
		atmel_tsadcc_write(ATMEL_TSADCC_IMR,
				   ATMEL_TSADCC_EOC(3) | ATMEL_TSADCC_NOCNT);
		atmel_tsadcc_write(ATMEL_TSADCC_TRGR,
				   ATMEL_TSADCC_TRGMOD_PERIOD | (0x09FF << 16));

		/* Software controlled triggering */
		/* atmel_tsadcc_write(ATMEL_TSADCC_CR, 2); */

	} else if (status & ATMEL_TSADCC_EOC(3)) {
		/* Conversion finished, make new measurement */
		ts_dev->measurementX[ts_dev->measurenumber] = atmel_tsadcc_read(ATMEL_TSADCC_CDR3) << 10;
		ts_dev->measurementX[ts_dev->measurenumber] /= atmel_tsadcc_read(ATMEL_TSADCC_CDR2);

		ts_dev->measurementY[ts_dev->measurenumber] = atmel_tsadcc_read(ATMEL_TSADCC_CDR1) << 10;
		ts_dev->measurementY[ts_dev->measurenumber] /= atmel_tsadcc_read(ATMEL_TSADCC_CDR0);

		z1dat = atmel_tsadcc_read(ATMEL_TSADCC_Z1DAT);
		z2dat = atmel_tsadcc_read(ATMEL_TSADCC_Z2DAT);

		if (z1dat != 0)
		{
			pres = (atmel_tsadcc_read(ATMEL_TSADCC_XPOS) * ((z2dat/z1dat)-1) * 1000) / 1024;
			ts_dev->measurementZ[ts_dev->measurenumber] =  pres;
			ts_dev->measurenumber++;
		}
		else
		{
			// Do nothing, sample ignored
		}

		if (ts_dev->measurenumber >= 4)
		{
			Z = 0;
	        for (i = 0; i < 5; i++)
	        {
				Z += ts_dev->measurementZ[i];
			}
			Z /= 5;
			if (pres < TOUCH_PRESSURE_LIMIT)
			{
                /* throw out lowest sample */
                temp = ts_dev->measurementX[0];
                lowSampNum = 0;

                for (i = 0; i < 4; i++)
                {
                    if (temp > ts_dev->measurementX[i+1])
                    {
                        temp = ts_dev->measurementX[i+1];
                        lowSampNum = i+1;
                    }
                }
                ts_dev->measurementX[lowSampNum] = 0;

                /* throw out highest sample */
                temp = ts_dev->measurementX[0];
                HiSampNum = 0;

                for (i = 0; i < 4; i++)
                {
                    if (temp < ts_dev->measurementX[i+1])
                    {
                        temp = ts_dev->measurementX[i+1];
                        HiSampNum = i+1;
                    }
                }
                ts_dev->measurementX[HiSampNum] = 0;

                j = 0;

                for (i =0; i < 5; i++)
                {
                    if(ts_dev->measurementX[i])
                    {
                        /* only three remaining samples */
                        nSamp[j++] = ts_dev->measurementX[i];
                    }                
                }

                if ( j > 2 &&
                        (abs(nSamp[0] - nSamp[1]) < TOUCH_NOISE_LIMIT) &&
                        (abs(nSamp[1] - nSamp[2]) < TOUCH_NOISE_LIMIT) &&
                        (abs(nSamp[2] - nSamp[0]) < TOUCH_NOISE_LIMIT))

                {
                    /* good sample obtained */
                    X = (nSamp[0] + nSamp[1] + nSamp[2]) / 3;
                }
                else
                {
                    X = 0;
                    Y = 0;
                }

                /* X coordinate sampled successfull calculate Y coordinate. */
                if(X != 0)
                {//AH
                    /* throw out lowest sample */
                    temp = ts_dev->measurementY[0];
                    lowSampNum = 0;

                    for (i = 0; i < 4; i++)
                    {
                        if (temp > ts_dev->measurementY[i+1])
                        {
                            temp = ts_dev->measurementY[i+1];
                            lowSampNum = i+1;
                        }
                    }

                    ts_dev->measurementY[lowSampNum] = 0;


                    /* throw out highest sample */
                    temp = ts_dev->measurementY[0];
                    HiSampNum = 0;

                    for (i = 0; i < 4; i++)
                    {
                        if (temp < ts_dev->measurementY[i+1])
                        {
                            temp = ts_dev->measurementY[i+1];
                            HiSampNum = i+1;
                        }
                    }
                    ts_dev->measurementY[HiSampNum] = 0;

                    j = 0;
                    for (i = 0; i < 5; i++)
                    {
                        if(ts_dev->measurementY[i])
                        {
                            /* only three remaining samples */
                            nSamp[j++] = ts_dev->measurementY[i];
                        }
                    }

                    if (j > 2 &&
                            (abs(nSamp[0] - nSamp[1]) < TOUCH_NOISE_LIMIT) &&
                            (abs(nSamp[1] - nSamp[2]) < TOUCH_NOISE_LIMIT) &&
                            (abs(nSamp[2] - nSamp[0]) < TOUCH_NOISE_LIMIT))
                    {
                        /* good sample obtained */
                        Y = (nSamp[0] + nSamp[1] + nSamp[2]) / 3;
                        ts_dev->buttonDown = true;
                    }
                    else
                    {
                        /* Y coordinate sample failed set sample to default. */
                        X = 0;
                        Y = 0;
                    }
 
					if (X != 0 && Y != 0)
					{
						ts_dev->prev_absx = X;
						ts_dev->prev_absy = Y;
						ts_dev->prev_absz = Z;
/*
						input_report_abs(input_dev, ABS_X, X);
						input_report_abs(input_dev, ABS_Y, Y);
						input_report_abs(input_dev, ABS_PRESSURE, Z);
						input_report_key(input_dev, BTN_TOUCH, ts_dev->buttonDown);
						input_sync(input_dev); */
						ts_dev->reported = 1;
					}
				}
			}
			ts_dev->measurenumber = 0;
		}

		/* Software controlled triggering */
		/* atmel_tsadcc_write(ATMEL_TSADCC_CR, 2); */
    }

	return IRQ_HANDLED;
}

#if defined(CONFIG_OF)
static int __devinit atmel_of_init_tsadcc(struct device_node *np,
				struct at91_tsadcc_data *pdata,
				struct platform_device *pdev)
{
	u32 val;

	if (of_property_read_u32(np, "atmel,tsadcc_clock", &val) == 0)
		pdata->adc_clock = val;

	if (of_property_read_u32(np, "atmel,filtering_average", &val) == 0) {
		if (val > 0x03) {
			dev_err(&pdev->dev, "invalid touch average setting, 0x%02x\n",
				val);
			return -EINVAL;
		}
		pdata->filtering_average = (u8)val;
	}

	if (of_property_read_u32(np, "atmel,pendet_debounce", &val) == 0) {
		if (val > 0x0f) {
			dev_err(&pdev->dev, "invalid pen detect debounce, 0x%02x\n",
				val);
			return -EINVAL;
		}
		pdata->pendet_debounce = (u8)val;
	}

	if (of_property_read_u32(np, "atmel,pendet_sensitivity", &val) == 0) {
		if (val > 0x03) {
			dev_err(&pdev->dev, "invalid pen detective sensitivity setting, 0x%02x\n",
				val);
			return -EINVAL;
		}
		pdata->pendet_sensitivity = (u8)val;
	}

	if (of_property_read_u32(np, "atmel,ts_sample_hold_time", &val) == 0) {
		if (val > 0x0f) {
			dev_err(&pdev->dev, "invalid ts sample hold time, 0x%02x\n",
				val);
			return -EINVAL;
		}
		pdata->ts_sample_hold_time = (u8)val;
	}

	return 0;
}
#else
static int __devinit atmel_of_init_tsadcc(struct device_node *np,
				struct at91_tsadcc_data *pdata,
				struct platform_device *pdev)
{
	return -EINVAL;
}
#endif

/*
 * The functions for inserting/removing us as a module.
 */

static int __devinit atmel_tsadcc_probe(struct platform_device *pdev)
{
	struct atmel_tsadcc	*ts_dev;
	struct input_dev	*input_dev;
	struct resource		*res;
	struct at91_tsadcc_data *pdata;
	int		err = 0;
	unsigned int	prsc;
	unsigned int	reg, ghi_mr = 0;
	unsigned int uDummy, Val = 1, Exp = 0;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "no mmio resource defined.\n");
		return -ENXIO;
	}

	/* Allocate memory for device */
	ts_dev = kzalloc(sizeof(struct atmel_tsadcc), GFP_KERNEL);
	if (!ts_dev) {
		dev_err(&pdev->dev, "failed to allocate memory.\n");
		return -ENOMEM;
	}
	platform_set_drvdata(pdev, ts_dev);
	pdata = &ts_dev->board;

	input_dev = input_allocate_device();
	if (!input_dev) {
		dev_err(&pdev->dev, "failed to allocate input device.\n");
		err = -EBUSY;
		goto err_free_mem;
	}

	ts_dev->irq = platform_get_irq(pdev, 0);
	if (ts_dev->irq < 0) {
		dev_err(&pdev->dev, "no irq ID is designated.\n");
		err = -ENODEV;
		goto err_free_dev;
	}

	if (!request_mem_region(res->start, resource_size(res),
				"atmel tsadcc regs")) {
		dev_err(&pdev->dev, "resources is unavailable.\n");
		err = -EBUSY;
		goto err_free_dev;
	}

	tsc_base = ioremap(res->start, resource_size(res));
	if (!tsc_base) {
		dev_err(&pdev->dev, "failed to map registers.\n");
		err = -ENOMEM;
		goto err_release_mem;
	}

	err = request_irq(ts_dev->irq, atmel_tsadcc_interrupt, 0,
			pdev->dev.driver->name, ts_dev);
	if (err) {
		dev_err(&pdev->dev, "failed to allocate irq.\n");
		goto err_unmap_regs;
	}

	ts_dev->clk = clk_get(&pdev->dev, "tsc_clk");
	if (IS_ERR(ts_dev->clk)) {
		dev_err(&pdev->dev, "failed to get ts_clk\n");
		err = PTR_ERR(ts_dev->clk);
		goto err_free_irq;
	}

	ts_dev->input = input_dev;
	ts_dev->bufferedmeasure = 0;

	snprintf(ts_dev->phys, sizeof(ts_dev->phys),
		 "%s/input0", dev_name(&pdev->dev));

	input_dev->name = "atmel touch screen controller";
	input_dev->phys = ts_dev->phys;
	input_dev->dev.parent = &pdev->dev;

	__set_bit(EV_ABS, input_dev->evbit);
	input_set_abs_params(input_dev, ABS_X, 0, 800, 0, 0);
	input_set_abs_params(input_dev, ABS_Y, 0, 480, 0, 0);
	input_set_abs_params(input_dev, ABS_PRESSURE, 0, TOUCH_PRESSURE_LIMIT, 0, 0);
	input_set_capability(input_dev, EV_KEY, BTN_TOUCH);

	/* clk_enable() always returns 0, no need to check it */
	clk_enable(ts_dev->clk);

	prsc = clk_get_rate(ts_dev->clk);
	dev_info(&pdev->dev, "Master clock is set at: %d Hz\n", prsc);

	if (pdev->dev.of_node) {
		err = atmel_of_init_tsadcc(pdev->dev.of_node, pdata, pdev);
		if (err)
			goto err_fail;
	} else {
		if (!pdev->dev.platform_data)
			goto err_fail;
		else
			memcpy(pdata, pdev->dev.platform_data, sizeof(*pdata));
	}

	atmel_tsadcc_write(PIO_IDR,  (1 << 20)|(1 << 21)|(1 << 22)|(1 << 23));
	atmel_tsadcc_write(PIO_PUDR, (1 << 20)|(1 << 21)|(1 << 22)|(1 << 23));
	atmel_tsadcc_write(ATMEL_TSADCC_IDR, 0xFFFF);

	atmel_tsadcc_write(ATMEL_TSADCC_CR, ATMEL_TSADCC_SWRST);

	ghi_mr = atmel_tsadcc_read(ATMEL_TSADCC_MR);
	ghi_mr = (ghi_mr & ~ATMEL_TSADCC_TSAMOD) | ATMEL_TSADCC_TSAMOD_TS_ONLY_MODE;
	ghi_mr |= ATMEL_TSADCC_PENDET /*| ATMEL_TSADCC_SLEEP*/;

	/* 300kHz. */
	uDummy = (ATMEL_MASTER_CLOCK << 4) / (ATMEL_ADS_CLOCK << 5) - 1;
	ghi_mr = (ghi_mr & ~ATMEL_TSADCC_PRESCAL) | ((uDummy << 8) & 0xff00);

	uDummy = (ATMEL_ADS_STARTUP * ATMEL_ADS_CLOCK) / (8 * 1000000);
	ghi_mr = (ghi_mr & ~ATMEL_TSADCC_STARTUP) | (uDummy << 16);
    ghi_mr = (ghi_mr & ~ATMEL_TSADCC_SHTIM) | (ATMEL_ADS_SHTIM << 24);
	atmel_tsadcc_write(ATMEL_TSADCC_TSR, uDummy << 24);

	uDummy = ATMEL_ADS_DEBOUNCE * (ATMEL_ADS_CLOCK / 1000);
	while (Val < uDummy)
	{
		Val *= 2;
		Exp++;
	}
	ts_dev->pendbc = Exp;
	ghi_mr = (ghi_mr & ~ATMEL_TSADCC_PENDBC) | ((Exp & 0xf) << 28);
	ghi_mr = (ghi_mr & ~0x00000080) | 0x80;

	atmel_tsadcc_write(ATMEL_TSADCC_MR, ghi_mr);
	atmel_tsadcc_write(ATMEL_TSADCC_TRGR, ATMEL_TSADCC_TRGMOD_NONE);
	atmel_tsadcc_read(ATMEL_TSADCC_SR);
	atmel_tsadcc_write(ATMEL_TSADCC_IER, ATMEL_TSADCC_PENCNT);
	atmel_tsadcc_write(ATMEL_TSADCC_IMR, ATMEL_TSADCC_PENCNT);

	atmel_tsadcc_dump_conf(pdev);

	/* All went ok, so register to the input system */
	err = input_register_device(input_dev);
	if (err)
		goto err_fail;

	return 0;

err_fail:
	clk_disable(ts_dev->clk);
	clk_put(ts_dev->clk);
err_free_irq:
	free_irq(ts_dev->irq, ts_dev);
err_unmap_regs:
	iounmap(tsc_base);
err_release_mem:
	release_mem_region(res->start, resource_size(res));
err_free_dev:
	input_free_device(input_dev);
err_free_mem:
	kfree(ts_dev);
	return err;
}

static int __devexit atmel_tsadcc_remove(struct platform_device *pdev)
{
	struct atmel_tsadcc *ts_dev = dev_get_drvdata(&pdev->dev);
	struct resource *res;

	free_irq(ts_dev->irq, ts_dev);

	input_unregister_device(ts_dev->input);

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	iounmap(tsc_base);
	release_mem_region(res->start, resource_size(res));

	clk_disable(ts_dev->clk);
	clk_put(ts_dev->clk);

	kfree(ts_dev);

	return 0;
}

#if defined(CONFIG_OF)
static const struct of_device_id atmel_tsaddcc_dt_ids[] = {
	{ .compatible = "atmel,at91sam9x5-tsadcc"},
	{ /* sentinel */ }
};

MODULE_DEVICE_TABLE(of, atmel_tsaddcc_dt_ids);
#endif

static struct platform_driver atmel_tsadcc_driver = {
	.probe		= atmel_tsadcc_probe,
	.remove		= __devexit_p(atmel_tsadcc_remove),
	.driver		= {
		.name	= "atmel_tsadcc",
		.of_match_table = of_match_ptr(atmel_tsaddcc_dt_ids),
	},
};
module_platform_driver(atmel_tsadcc_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Atmel TouchScreen Driver");
MODULE_AUTHOR("Dan Liang <dan.liang@atmel.com>");

