obj-m	:=Tongshi_DAB.o

KERNELDIR ?= /lib/modules/$(shell uname -r)/build
PWD		  := $(shell pwd)

all:
	$(MAKE) -C $(KERNELDIR) M=$(PWD)

clean:
	rm -rf *~ *.o core .depend .*.cmd *.ko *.mod.c .tmp_versions Module.symvers modules.order

