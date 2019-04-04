sudo rmmod spi_ft232h_usb
make clean
make all
sudo insmod spi-ft232h-usb.ko
sudo chown -R root:gpio /sys/class/gpio/gpio4/ && sudo chmod -R 770 /sys/class/gpio/gpio4/
