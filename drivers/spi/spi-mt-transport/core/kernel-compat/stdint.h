/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Compat shim only -- NOT part of the vendored core (see ../PROVENANCE.md).
 *
 * The vendored core targets hosted C11, so it includes the standard
 * <stdint.h> for the fixed-width int types. Under the kernel's -nostdinc
 * build this cross-compiler's own freestanding headers aren't on the search
 * path either, so <stdint.h> doesn't resolve at all. linux/types.h already
 * provides int8_t/uint8_t/.../int64_t/uint64_t (via asm-generic/int-ll64.h)
 * with identical signedness/width. Only reached via this module's own
 * Makefile (ccflags-y += -I$(srctree)/$(src)/core/kernel-compat), so it
 * cannot shadow <stdint.h> anywhere else in the kernel tree.
 */
#include <linux/types.h>
