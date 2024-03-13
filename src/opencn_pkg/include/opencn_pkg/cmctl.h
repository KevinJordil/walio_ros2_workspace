/* SPDX-License-Identifier: GPL-2.0
 * Copyright (C) 2021 Jean-Pierre Miceli <jean-pierre.miceli@heig-vd.ch>
 */

#ifndef CMCTL_H
#define CMCTL_H

/* Error codes */
typedef enum {
	CMCTL_SUCCESS = 0,
	CMCTL_PIN_NOT_EXIST,
	CMCTL_PIN_TYPE_ERROR,
	CMCTL_CONNECTION_FAILED,
	CMCTL_NO_CONNECTION,
} cmctl_error_t;

#endif /* CMCTL_H */