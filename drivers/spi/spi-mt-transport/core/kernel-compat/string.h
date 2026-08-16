/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Compat shim only -- NOT part of the vendored core (see ../PROVENANCE.md).
 *
 * The vendored core targets hosted C11 (its STM32/host-native builds use a
 * real libc), so it includes the standard <string.h> for memcpy/memset/
 * memcmp. The kernel build has no hosted libc and doesn't provide a bare
 * <string.h> -- linux/string.h is the kernel's equivalent, with compatible
 * signatures for the functions the core actually uses. This shim lets
 * <string.h> resolve to it without hand-editing the vendored .c files
 * themselves. Only reached via this module's own Makefile
 * (ccflags-y += -I$(src)/core/kernel-compat), so it cannot shadow <string.h>
 * anywhere else in the kernel tree.
 */
#include <linux/string.h>
