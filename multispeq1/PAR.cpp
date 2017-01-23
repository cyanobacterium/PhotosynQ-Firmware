
// code related to the PAR/color sensor

#include <Arduino.h>
#include "src/TCS3471.h"              // color sensor
#include "src/Adafruit_TCS34725.h"              // color sensor
#include "serial.h"
#include "defines.h"
#include "eeprom.h"

// external function declarations
void i2cWrite(byte address, byte count, byte* buffer);
void i2cRead(byte address, byte count, byte* buffer);

// global variables
extern float light_intensity;
extern float light_intensity_averaged;
extern float light_intensity_raw;
extern float light_intensity_raw_averaged;
extern float r;
extern float r_averaged;
extern float g;
extern float g_averaged;
extern float b;
extern float b_averaged;

//static TCS3471 *par_sensor=0;
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_700MS, TCS34725_GAIN_1X);


// initialize the PAR/color sensor

void PAR_init()
{
  /*
    // color sensor init

    if (par_sensor == 0)
       par_sensor = new TCS3471(i2cWrite, i2cRead);

    par_sensor->setWaitTime(200.0);
    par_sensor->setIntegrationTime(700.0);
    par_sensor->setGain(TCS3471_GAIN_1X);
    par_sensor->enable();
  */
}  // PAR_init()

void PAR_init_constant()
{
  /*
    // color sensor init

    if (par_sensor == 0)
     par_sensor = new TCS3471(i2cWrite, i2cRead);

    par_sensor->enableInterrupt();
    // range is from 2.4ms up to 614.4ms, parameter is float and in milliseconds,
    par_sensor->setIntegrationTime(600);
    // range is from 2.4ms up to 7400ms, from 2.4ms up to 614.4ms step is 2.4ms, from 614.4ms up step is  28.8ms
    // if set to anything less than 2.4ms, wait time is disabled
    par_sensor->setWaitTime(10.0);
    // naked chip under regular ambient lighting works ok with 1x gain
    par_sensor->setGain(TCS3471_GAIN_1X);
    // if C(lear) channel goes above this value, interrupt will be generated
    // range is from 0-65535, 16 full bits
    par_sensor->interruptHighThreshold(32768);
    // range is from 0-65535
    // this will ensure that interrupt is generated all the time
    par_sensor->interruptLowThreshold(32767);
    par_sensor->interruptPersistence(2);
    par_sensor->clearInterrupt();
    par_sensor->enable();

  */
  analogWriteResolution(12);
}  // PAR_init_constant()


uint16_t par_to_dac (float _par, uint16_t _pin) {                                             // convert dac value to par, in form y = mx2+ rx + b where y is the dac value
  //  int dac_value = _par * _par * eeprom->par_to_dac_slope1[_pin] + _par * eeprom->par_to_dac_slope2[_pin] + eeprom->par_to_dac_yint[_pin];
  double a = _par * _par * _par * _par * eeprom->par_to_dac_slope1[_pin] / 1000000000;
  double b = _par * _par * _par * eeprom->par_to_dac_slope2[_pin] / 1000000000;
  double c = _par * _par * eeprom->par_to_dac_slope3[_pin] / 1000000000;
  double d = _par * eeprom->par_to_dac_slope4[_pin];
  double e = eeprom->par_to_dac_yint[_pin];
  int dac_value = a + b + c + d + e;
  if (_par == 0) {                                                                           // regardless of the calibration, force a PAR of zero to lights off
    dac_value = 0;
  }
  dac_value = constrain(dac_value, 0, 4095);
  return dac_value;
}

float light_intensity_raw_to_par (float _light_intensity_raw, float _r, float _g, float _b) {
  int par_value = eeprom->light_slope_all * _light_intensity_raw + _r * eeprom->light_slope_r + _g * eeprom->light_slope_g + _b * eeprom->light_slope_b + eeprom->light_yint;

  if (par_value < 0)                                                                             // there may be cases when the output could be less than zero.  In those cases, set to zero and mark a flag
    par_value = 0;
  return par_value;
}

int get_light_intensity(int _averages) {

  uint16_t clear, red, green, blue;
  tcs.getRawData(&red, &green, &blue, &clear);

  r = red;
  g = green;
  b = blue;
  light_intensity_raw = clear;
  light_intensity = light_intensity_raw_to_par(light_intensity_raw, r, g, b);

  r_averaged += r / _averages;
  g_averaged += g / _averages;
  b_averaged += b / _averages;

  light_intensity_raw_averaged += light_intensity_raw / _averages;
  light_intensity_averaged += light_intensity / _averages;

  return light_intensity;

  /*

    r = par_sensor->readRData();
    g = par_sensor->readGData();
    b = par_sensor->readBData();
    light_intensity_raw = par_sensor->readCData();
    light_intensity = light_intensity_raw_to_par(light_intensity_raw, r, g, b);

    r_averaged += r / _averages;
    g_averaged += g / _averages;
    b_averaged += b / _averages;

    light_intensity_raw_averaged += light_intensity_raw / _averages;
    light_intensity_averaged += light_intensity / _averages;
  */
  //  return light_intensity;
  return 1;

} // get_light_intensity()


volatile bool colorAvailable = true;

void TCS3471Int()
{
  colorAvailable = true;
}

void constant_light () {

  // make sure that the light used in the constant light device is set to 12 bits
  analogWriteFrequency(40, 14648.437);                                                           // set analog frequency
  analogWriteResolution(12);                                                           // set analog frequency

  ///Note, control loop not stable at low values,  need to check how int(float) is working, also check all limits
  //such as targethigh and target low.

  int targetlow = 0;
  int targethigh = 0;

  int par_set = 0;
  int out_index = 0;
  int dac_set = 0;

  while (dac_set != -1) {
    if ((out_index) > 4095)out_index = 0;
    analogWrite(DACT, out_index);
    int input_value = Serial_Input_Long("+", 3);
//    Serial_Print_Line(input_value);
    if (input_value != 0) {
      if (input_value == -1) break;
      else if (input_value > 5000 || input_value < -1) {
        Serial_Print(",\"error\": \"out of range\"}]");
        break;
      }
      else if (input_value != -1) {
        par_set = input_value;
        dac_set = par_to_dac(par_set,1);
        targetlow = dac_set - 1;
        targethigh = dac_set + 1;
      }
    }
    if (out_index < targetlow)
    {
      out_index = out_index + 1;   //removed the ^2 term to ratio, removed + int(targetlow/clearVal) JB
      analogWrite(A14, out_index);
    }

    if (out_index > targethigh)
    {
      out_index = out_index - 1;  ////removed the ^2 term to ratio, removed + int(targetlow/clearVal) JB
      analogWrite(A14, out_index);
    }
  }
  analogWrite(DACT, 0);
  Serial_Print("}]");
}


