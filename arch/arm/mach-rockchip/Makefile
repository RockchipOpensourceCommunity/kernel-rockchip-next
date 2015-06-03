CFLAGS_platsmp.o := -march=armv7-a
obj-y += cpu.o

obj-$(CONFIG_ARCH_ROCKCHIP) += rockchip.o
obj-$(CONFIG_PM_SLEEP) += pm.o sleep.o
obj-$(CONFIG_SMP) += headsmp.o platsmp.o
obj-$(CONFIG_RK_VCODEC) += vcodec_service.o
