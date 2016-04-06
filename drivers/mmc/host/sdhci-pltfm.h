/*
 * Copyright 2010 MontaVista Software, LLC.
 *
 * Author: Anton Vorontsov <avorontsov@ru.mvista.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
/* 
 * Includes Intel Corporation's changes/modifications dated: 2012. 
 * Changed/modified portions - Copyright © 2012 , Intel Corporation.   
 */ 

#ifndef _DRIVERS_MMC_SDHCI_PLTFM_H
#define _DRIVERS_MMC_SDHCI_PLTFM_H

#include <linux/clk.h>
#include <linux/types.h>
#include <linux/mmc/sdhci-pltfm.h>

struct sdhci_pltfm_host {
	struct clk *clk;
	void *priv; /* to handle quirks across io-accessor calls */
};

extern struct sdhci_pltfm_data sdhci_cns3xxx_pdata;
extern struct sdhci_pltfm_data sdhci_esdhc_imx_pdata;
extern struct sdhci_pltfm_data sdhci_dove_pdata;
extern struct sdhci_pltfm_data sdhci_tegra_pdata;
extern struct sdhci_pltfm_data sdhci_puma6_pdata;

#endif /* _DRIVERS_MMC_SDHCI_PLTFM_H */
