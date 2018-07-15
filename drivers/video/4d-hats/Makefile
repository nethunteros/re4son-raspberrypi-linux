EXTRA_CFLAGS += -I$(src)

KERNEL_GE_4_1 := $(shell [ $(VERSION) -ge 4 -a $(PATCHLEVEL) -ge 1 ] && echo true)
ifeq ($(KERNEL_GE_4_1),true)
	obj-$(CONFIG_4DPI_SPI_DMA)        += spi-bcm2835_dma.o
else
	obj-$(CONFIG_4DPI_SPI_DMA)        += spi-bcm2708_dma.o
endif

obj-$(CONFIG_FB_HATS)             += 4dpi.o

ifeq ($(CONFIG_LOCALVERSION),"-Re4son-v7")
        4dpi-objs                       := 4d-hats.o compress-v7.o
else
        4dpi-objs                       := 4d-hats.o compress-v6.o
endif

obj-$(CONFIG_4DPI_ADS7846)        += 4dpi_touch.o
obj-$(CONFIG_4DPI_AR1020I2C)      += ar1020-i2c.o
obj-$(CONFIG_4DPI_FT5X06)         += edt-ft5x06.o
obj-$(CONFIG_4DPI_PWMBL)          += pwmbl.o
