# SPDX-License-Identifier: GPL-2.0
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
# kernel builds; these headers live only in the source tree, so the
# $(srctree)/ prefix is required or the compile can't find them.
ccflags-y += -I$(srctree)/$(src)/core/kernel-compat -I$(srctree)/$(src)/core/include
