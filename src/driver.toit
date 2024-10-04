// Copyright (C) 2022 David Lao. All rights reserved.
// Use of this source code is governed by a MIT-style license that can be found
// in the LICENSE file.

import binary
import serial.device as serial
import serial.registers as serial

I2C-ADDRESS ::= 0x77

//
// Driver for the Bosch BMP180 environmental sensor with I2C interface
//
class Driver:
  static CAL-COEF-AC1_         ::= 0xAA
  static CAL-COEF-AC2_         ::= 0xAC
  static CAL-COEF-AC3_         ::= 0xAE
  static CAL-COEF-AC4_         ::= 0xB0
  static CAL-COEF-AC5_         ::= 0xB2
  static CAL-COEF-AC6_         ::= 0xB4
  static CAL-COEF-B1_          ::= 0xB6
  static CAL-COEF-B2_          ::= 0xB8
  static CAL-COEF-MB_          ::= 0xBA
  static CAL-COEF-MC_          ::= 0xBC
  static CAL-COEF-MD_          ::= 0xBE

  static REG-CONTROL_          ::= 0xF4
  static REG-TEMPDATA_         ::= 0xF6
  static REG-PRESSUREDATA_     ::= 0xF6
  static REG-READTEMP-CMD_     ::= 0x2E
  static REG-READPRESSURE-CMD_ ::= 0x34

  static REG-CHIPID_           ::= 0xD0

  reg_/serial.Registers ::= ?

  ac1 := null
  ac2 := null
  ac3 := null
  ac4 := null
  ac5 := null
  ac6 := null
  b1  := null
  b2  := null
  mb  := null
  mc  := null
  md  := null

  // hardware pressure sampling accuracy modes
  // 0 = ultra low power, 1 = standard, 2 = high resolution, 3 ultra high resolution
  oversampling := 1

  constructor dev/serial.Device:
    reg_ = dev.registers

    tries := 5
    while (reg_.read-u8 REG-CHIPID_) != 0x55:
      tries--
      if tries == 0: throw "INVALID_CHIP"
      sleep --ms=1

    // read calibration coefficients
    ac1 = reg_.read-i16-be CAL-COEF-AC1_
    ac2 = reg_.read-i16-be CAL-COEF-AC2_
    ac3 = reg_.read-i16-be CAL-COEF-AC3_
    ac4 = reg_.read-u16-be CAL-COEF-AC4_
    ac5 = reg_.read-u16-be CAL-COEF-AC5_
    ac6 = reg_.read-u16-be CAL-COEF-AC6_
    b1  = reg_.read-i16-be CAL-COEF-B1_
    b2  = reg_.read-i16-be CAL-COEF-B2_
    mb  = reg_.read-i16-be CAL-COEF-MB_
    mc  = reg_.read-i16-be CAL-COEF-MC_
    md  = reg_.read-i16-be CAL-COEF-MD_

  // Reads the temperature and returns it in degrees Celsius
  read-temperature:
    ut := read-raw-temperature_
    b5 := compute-b5_ ut
    t := (b5 + 8) / 16.0
    t = t / 10.0
    return t

  // Reads the barometric pressure and returns it in Pascals
  read-pressure:
    ut := read-raw-temperature_
    b5 := compute-b5_ ut
    up := read-raw-pressure_

    b6 := b5 - 4000
    x1 := (b2 * ((b6 * b6) >> 12)) >> 11
    x2 := (ac2 * b6) >> 11
    x3 := x1 + x2
    b3 := (((ac1 * 4 + x3) << oversampling) + 2) / 4

    x1 = (ac3 * b6) >> 13
    x2 = (b1 * ((b6 * b6) >> 12)) >> 16
    x3 = ((x1 + x3) + 2) >> 2
    b4 := (ac4 * (x3 + 32768)) >> 15
    b7 := (up - b3) * (50000 >> oversampling)

    p := null
    if b7 < 0x80000000:
      p = (b7 * 2) / b4
    else:
      p = (b7 / b4) * 2

    x1 = (p >> 8) * (p >> 8)
    x1 = (x1 * 3038) >> 16
    x2 = (-7357 * p) >> 16

    p = p + ((x1 + x2 + 3791) >> 4)
    return p

  // compute b5
  compute-b5_ ut:
    x1 := (ut - ac6) * ac5 >> 15
    x2 := (mc << 11) / (x1 + md)
    return x1 + x2

  // read uncompensated temperature value
  read-raw-temperature_ :
    reg_.write-u8 REG-CONTROL_ REG-READTEMP-CMD_
    sleep --ms=5
    ut := reg_.read-u16-be REG-TEMPDATA_
    return ut

  // read uncompensated pressure value
  read-raw-pressure_ :
    reg_.write-u8 REG-CONTROL_ (REG-READPRESSURE-CMD_ + (oversampling << 6))
    sleep --ms=5
    up := reg_.read-u24-be REG-PRESSUREDATA_
    up >>= (8 - oversampling)
    return up
