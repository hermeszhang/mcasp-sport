/* -*- Mode: C; tab-width: 8; indent-tabs-mode: t; c-basic-offset: 8 -*-
 *
 * AM335x MCASP to ADSP-21262 SPORT driver.
 *
 * Copyright (C) GeoTechnologies 2016.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 */
#ifndef GTSPORT_MCASP_SETUP_H
#define GTSPORT_MCASP_SETUP_H

#include "davinci-mcasp.h"

void gtsport_mcasp_setup(struct mcasp_dev *mcasp);
void gtsport_mcasp_rx_start(struct mcasp_dev *mcasp);
void gtsport_mcasp_rx_stop(struct mcasp_dev *mcasp);

#endif /* GTSPORT_MCASP_SETUP_H */
