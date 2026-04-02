ccflags-y += -I$(srctree)/drivers/misc/cix/sensor/core
ccflags-y += -I$(srctree)/drivers/misc/cix/sensor/scp/include

obj-$(CONFIG_CIX_KERNEL_SENSORHUB) += cix_sensorhub_mod.o
cix_sensorhub_mod-objs += sensor_list.o sensorhub_ipi.o cix_sensorhub.o