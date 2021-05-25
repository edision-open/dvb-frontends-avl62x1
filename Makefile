# Makefile -- For the linux kernel driver module

#
# By default, the build is done against the running linux kernel source.
# To build against a different kernel source tree, set KDIR:
#
#	make KDIR=/path/to/kernel/source

ifneq ($(KERNELRELEASE),)

ccflags-y += -I$(src)/common
ccflags-y += -I$(src)/sdk_src
ccflags-y += -DCONFIG_MEDIA_TUNER_AV201X=1

obj-m += avl6261.o

avl6261-objs := \
	av201x.o \
	common/avl_bsp_linux.o \
	common/avl_lib.o \
	sdk_src/avl62x1_api.o \
	sdk_src/avl62x1_lib.o \
	avl62x1.o

else

KDIR	?= /lib/modules/$(shell uname -r)/build

all:
	$(MAKE) -C $(KDIR) M=$$PWD modules

clean:
	$(MAKE) -C $(KDIR) M=$$PWD clean

endif
