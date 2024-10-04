import gpio
import i2c
import bmp180

main:
  bus := i2c.Bus
    --sda=gpio.Pin 21
    --scl=gpio.Pin 22

  device := bus.device bmp180.I2C-ADDRESS

  driver := bmp180.Driver device

  print "$driver.read-temperature C"
  print "$driver.read-pressure Pa"
