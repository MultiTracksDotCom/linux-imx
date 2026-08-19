# SPDX-License-Identifier: GPL-2.0
#
# IMPORTANT -- this module only builds inside the imx8mmini-bb-evk Yocto BSP
# pipeline, not from a plain checkout of this kernel tree built directly
# (e.g. a bare `make -C . M=drivers/spi/spi-mt-transport modules`). The
# core/ referenced below is deliberately NOT committed to this repo (see
# core/'s absence and the .gitignore in this directory) -- it's the
# portable protocol core, whose single source of truth is the firmware
# repo's firmware-common/spi-transport/{src,inc}/. Yocto's
# meta-mt-transport-evk/recipes-kernel/linux-imx/linux-imx_%.bbappend
# fetches it separately and stages it into core/ via a do_patch[postfuncs]
# hook *before* this Makefile ever runs. Enabling CONFIG_SPI_MT_TRANSPORT
# outside that pipeline will fail with a generic "No rule to make target
# core/spi_transport.o" -- that's expected, not a bug; get the core staged
# first (or just build via bitbake, which always does this automatically).
obj-$(CONFIG_SPI_MT_TRANSPORT) += spi-mt-transport.o
spi-mt-transport-y := \
	core/spi_transport.o \
	core/spi_transport_channel.o \
	core/spi_transport_frame.o \
	core/spi_transport_crc16.o \
	core/spi_transport_hw.o \
	spi_transport_os_linux.o \
	spi_transport_hw_linux.o \
	spi_mt_transport_drv.o

# $(src) alone resolves against $(objtree) under Yocto's out-of-tree (O=)
# kernel builds; these headers only exist in the source tree, so the
# $(srctree)/ prefix is required or the compile can't find them.
ccflags-y += -I$(srctree)/$(src)/kernel-compat -I$(srctree)/$(src)/core/include
