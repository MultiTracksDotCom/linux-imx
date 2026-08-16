/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Compat shim only -- NOT part of the vendored core (see ../PROVENANCE.md).
 *
 * The vendored core targets hosted C11, so it includes the standard
 * <stdbool.h> for bool/true/false. Under the kernel's -nostdinc build this
 * cross-compiler's own freestanding headers aren't on the search path
 * either, so <stdbool.h> doesn't resolve at all. linux/types.h (bool) and
 * linux/stddef.h (true/false) are the kernel's equivalents. Only reached via
 * this module's own Makefile (ccflags-y += -I$(srctree)/$(src)/core/kernel-compat),
 * so it cannot shadow <stdbool.h> anywhere else in the kernel tree.
 */
#include <linux/stddef.h>
#include <linux/types.h>
