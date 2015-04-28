Compile kernel for popmetal:

export ARCH=arm
export CROSS_COMPILE=/path/to/your/toolchain/...

make zImage

make rk3288-popmetal.dtb

cat arch/arm/boot/zImage arch/arm/boot/dts/rk3288-popmetal.dtb > arch/arm/boot/kernel

./mkkrnlimg arch/arm/boot/kernel kernel.img

then Download the kernel.img to your board.
