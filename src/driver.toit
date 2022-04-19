// Copyright (C) 2022 David Lao. All rights reserved.
// Use of this source code is governed by a MIT-style license that can be found
// in the LICENSE file.

import binary
import serial.device as serial
import serial.registers as serial

I2C_ADDRESS ::= 0x77

//
// Driver for the Bosch BMP180 environmental sensor with I2C interface
//
class Driver:
  static CAL_COEF_AC1_         ::= 0xAA
  static CAL_COEF_AC2_         ::= 0xAC
  static CAL_COEF_AC3_         ::= 0xAE
  static CAL_COEF_AC4_         ::= 0xB0
  static CAL_COEF_AC5_         ::= 0xB2
  static CAL_COEF_AC6_         ::= 0xB4
  static CAL_COEF_B1_          ::= 0xB6
  static CAL_COEF_B2_          ::= 0xB8
  static CAL_COEF_MB_          ::= 0xBA
  static CAL_COEF_MC_          ::= 0xBC
  static CAL_COEF_MD_          ::= 0xBE

  static REG_CONTROL_          ::= 0xF4
  static REG_TEMPDATA_         ::= 0xF6
  static REG_PRESSUREDATA_     ::= 0xF6
  static REG_READTEMP_CMD_     ::= 0x2E
  static REG_READPRESSURE_CMD_ ::= 0x34

  static REG_CHIPID_           ::= 0xD0

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
    while (reg_.read_u8 REG_CHIPID_) != 0x55:
      tries--
      if tries == 0: throw "INVALID_CHIP"
      sleep --ms=1

    // read calibration coefficients
    ac1 = reg_.read_i16_be CAL_COEF_AC1_
    ac2 = reg_.read_i16_be CAL_COEF_AC2_
    ac3 = reg_.read_i16_be CAL_COEF_AC3_
    ac4 = reg_.read_u16_be CAL_COEF_AC4_
    ac5 = reg_.read_u16_be CAL_COEF_AC5_
    ac6 = reg_.read_u16_be CAL_COEF_AC6_
    b1  = reg_.read_i16_be CAL_COEF_B1_
    b2  = reg_.read_i16_be CAL_COEF_B2_
    mb  = reg_.read_i16_be CAL_COEF_MB_
    mc  = reg_.read_i16_be CAL_COEF_MC_
    md  = reg_.read_i16_be CAL_COEF_MD_

  // Reads the temperature and returns it in degrees Celsius 
  read_temperature:
    ut := read_raw_temperature_
    b5 := compute_b5_ ut
    t := (b5 + 8) / 16.0
    t = t / 10.0
    return t

  // Reads the barometric pressure and returns it in Pascals
  read_pressure:
    ut := read_raw_temperature_
    b5 := compute_b5_ ut 
    up := read_raw_pressure_

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
  compute_b5_ ut:
    x1 := (ut - ac6) * ac5 >> 15
    x2 := (mc << 11) / (x1 + md)
    return x1 + x2
  
  // read uncompensated temperature value
  read_raw_temperature_ :
    reg_.write_u8 REG_CONTROL_ REG_READTEMP_CMD_
    sleep --ms=5
    ut := reg_.read_u16_be REG_TEMPDATA_
    return ut

  // read uncompensated pressure value
  read_raw_pressure_ :
    reg_.write_u8 REG_CONTROL_ (REG_READPRESSURE_CMD_ + (oversampling << 6))
    sleep --ms=5
    up := reg_.read_u24_be REG_PRESSUREDATA_
    up >>= (8 - oversampling)
    return up
