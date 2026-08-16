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

ccflags-y += -I$(src)/core/kernel-compat -I$(src)/core/include
