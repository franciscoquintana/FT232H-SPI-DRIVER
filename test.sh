sudo rmmod spi_ft232h_usb
make clean
make all
sudo insmod spi-ft232h-usb.ko
