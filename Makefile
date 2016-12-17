export ARCH          = arm
export CROSS_COMPILE = arm-linux-gnueabi-

obj-m := gtsport.o
gtsport-objs += gtsport-dev.o mcasp-setup.o

PWD   := $(shell pwd)
KDIR  := $(PWD)/../beagle-linux-3.8.13

.PHONY: module
module:
	$(MAKE) -C $(KDIR) SUBDIRS=$(PWD) modules

clean:
	$(MAKE) -C $(KDIR) SUBDIRS=$(PWD) clean

upload: module
	scp gtsport.ko root@192.168.7.2:
