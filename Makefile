obj-m	:=dabpci.o

KERNELDIR ?= /lib/modules/$(shell uname -r)/build
PWD		  := $(shell pwd)

all:
	$(MAKE) -C $(KERNELDIR) M=$(PWD)

clean:
	 *~ core .depend .*.cmd *.ko *.mod.c .tmp_versions Module.symvers modules.order

