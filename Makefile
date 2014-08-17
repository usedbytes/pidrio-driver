MODULES := pidrio.o

obj-m := $(MODULES)

MAKEARCH := $(MAKE) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE)

all: modules
modules:
	$(MAKEARCH) -C $(KDIR) M=${shell pwd} modules

clean:
	$(MAKEARCH) -C $(KDIR) M=${shell pwd} clean
