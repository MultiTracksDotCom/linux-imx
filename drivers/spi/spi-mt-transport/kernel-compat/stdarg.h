/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Compat shim only -- NOT part of the portable core (../../../.. relative to here).
 *
 * The portable core targets hosted C11, so it includes the standard
 * <stdarg.h>. Under the kernel's -nostdinc build this cross-compiler's own
 * freestanding headers aren't on the search path either, so <stdarg.h>
 * doesn't resolve at all -- linux/stdarg.h is the kernel's own sanctioned
 * replacement for exactly this case. Only reached via this module's own
 * Makefile (ccflags-y += -I$(src)/kernel-compat), so it
 * cannot shadow <stdarg.h> anywhere else in the kernel tree.
 */
#include <linux/stdarg.h>
