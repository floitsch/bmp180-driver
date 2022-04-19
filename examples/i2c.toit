import gpio
import i2c
import bmp180

main:
  bus := i2c.Bus
    --sda=gpio.Pin 21
    --scl=gpio.Pin 22

  device := bus.device bmp180.I2C_ADDRESS

  driver := bmp180.Driver device

  print "$driver.read_temperature C"
  print "$driver.read_pressure Pa"
