
// If your name isn't Jon, don't touch this file

// reasonably generic utility functions
// put function prototypes in util.h

#include "defines.h"
#include "eeprom.h"
#include "utility/crc32.h"
#include "DAC.h"
#include "util.h"
#include "serial.h"

unsigned int read_once(unsigned char address);
void program_once(unsigned char address, unsigned int value);

int jz_test_mode = 0;


void start_watchdog(int minutes)
{
  if (minutes < 1)
    minutes = 1;
  WDOG_UNLOCK = WDOG_UNLOCK_SEQ1;
  WDOG_UNLOCK = WDOG_UNLOCK_SEQ2;
  delayMicroseconds(1); // Need to wait a bit..
  WDOG_TOVALL = 0; // The next 2 lines sets the time-out value. This is the value that the watchdog timer compare itself to.
  WDOG_TOVALH = minutes;     // approximate
  WDOG_PRESC = 0;
  WDOG_STCTRLH = (WDOG_STCTRLH_WAITEN | WDOG_STCTRLH_STOPEN | WDOG_STCTRLH_WDOGEN | WDOG_STCTRLH_ALLOWUPDATE); // enable
}

void stop_watchdog()
{
  WDOG_UNLOCK = WDOG_UNLOCK_SEQ1;
  WDOG_UNLOCK = WDOG_UNLOCK_SEQ2;
  delayMicroseconds(1); // Need to wait a bit..
  WDOG_STCTRLH = WDOG_STCTRLH_ALLOWUPDATE;               // disable
}

void feed_watchdog()
{
  noInterrupts();
  WDOG_REFRESH = 0xA602;
  WDOG_REFRESH = 0xB480;
  interrupts()
}


// qsort uint16_t comparison function (tri-state) - needed for median16()

static int uint16_cmp(const void *a, const void *b)
{
  const uint16_t *ia = (const uint16_t *)a; // casting pointer types
  const uint16_t *ib = (const uint16_t *)b;

  if (*ia == *ib)
    return 0;

  if (*ia > *ib)
    return 1;
  else
    return -1;
}

// return the median value of an array of 16 bit unsigned values
// note: this also sorts the array
// percentile is normally .50

uint16_t median16(uint16_t array[], const int n, const float percentile)
{
  qsort(array, (size_t) n, sizeof(uint16_t), uint16_cmp);
  return (array[(int) roundf(n * percentile)]);
}


// return the stdev value of an array of 16 bit unsigned values

float stdev16(uint16_t val[], const int count)
{
  // calc stats
  double mean = 0, delta = 0, m2 = 0, variance = 0, stdev = 0, n = 0;

  for (int i = 0; i < count; i++) {
    ++n;
    delta = val[i] - mean;
    mean += delta / n;
    m2 += (delta * (val[i] - mean));
  } // for
  variance = m2 / (count - 1);  // (n-1):Sample Variance  (n): Population Variance
  stdev =  sqrt(variance);        // Calculate standard deviation
  //Serial_Printf("single pulse stdev = %.2f, mean = %.2f AD counts\n", stdev, mean);
  //Serial_Printf("bits (95%%) = %.2f\n", (15 - log(stdev * 2) / log(2.0))); // 2 std dev from mean = 95%
  return stdev;
} // stdev16()


// check a json protocol for validity (matching [] and {})
// return 1 if OK, otherwise 0
// also check CRC value if present

int check_protocol(char *str)
{
  int bracket = 0, curly = 0;
  char *ptr = str;

  while (*ptr != 0) {
    switch (*ptr) {
      case '[':
        ++bracket;
        break;
      case ']':
        --bracket;
        break;
      case '{':
        ++curly;
        break;
      case '}':
        --curly;
        break;
    } // switch
    ++ptr;
  } // while

  if (bracket != 0 || curly != 0)  // unbalanced - can't be correct
    return 0;

  // check CRC - 8 hex digits immediately after the closing ]
  ptr = strrchr(str, ']'); // find last ]
  if (!ptr)                        // no ] found - how can that be?
    return 0;

  ++ptr;                      // char after last ]

  if (!isxdigit(*(ptr)))      // hex digit follows last ] ?
    return 1;                    // no CRC so report OK

  // CRC is there - check it

  crc32_init();     // find crc of json (from first [ to last ])
  crc32_buf (str, ptr - str);

  // note: must be exactly 8 upper case hex digits
  if (strncmp(int32_to_hex (crc32_value()), ptr, 8) != 0) {
    return 0;                 // bad CRC
  }

  *ptr = 0;                   // remove the CRC

  return 1;                   // CRC is OK
} // check_protocol()



// Battery check:
// 0 - just read voltage (quick)
// 1 - level while flashing the 4 IR LEDs at 250 mA each for awhile
// This should run just before any new protocol - if it’s too low, report to the user
// return 1 if low, otherwise 0

int battery_low(int load)         // 0 for no load, 1 to flash LEDs to create load
{
  int value = battery_level(load);

  if (load) {
    if (value < BAT_MIN_LOADED)
      return 1;   // too low
  } else {
    if (value < BAT_MIN)
      return 1;   // too low
  } // if

  return 0;

} // battery_low()

#define R1 680                // resistor divider for power measurement
#define R2 2000

int battery_level(int load)
{
  uint32_t initial_value = 0;

  // enable bat measurement
  pinMode(BATT_ME, OUTPUT);
  digitalWriteFast(BATT_ME, LOW);

  delayMicroseconds(300);    // has a slow filter circuit
  
  // find voltage before high load
  for (int i = 0 ; i < 100; ++i)
    initial_value += analogRead(BATT_TEST);  // test A10 analog input
  initial_value /= 100;

  //Serial_Printf("initial AD = %d\n",initial_value);

  uint32_t value = initial_value;

  // TODO verify that load effects voltage
  
  if (load) {   // flash LEDs if needed to create load

    // set DAC values to 1/4 of full output to create load
    DAC_set(5, 4096 / 4);
    DAC_set(6, 4096 / 4);
    DAC_set(8, 4906 / 4);
    DAC_set(9, 4906 / 4);
    DAC_set(10, 4906 / 4);
    DAC_change();
    delay(1);       // stabilize

    // turn on 4 LEDs
    digitalWriteFast(PULSE5, 1);
    digitalWriteFast(PULSE6, 1);
    digitalWriteFast(PULSE8, 1);
    digitalWriteFast(PULSE9, 1);
    digitalWriteFast(PULSE10, 1);

    delay(20);          // there a slow filter on the circuit

    // value after load
    value = 0;
    for (int i = 0 ; i < 100; ++i)
      value += analogRead(BATT_TEST);  // test A10 analog input
    value /= 100;

    // turn off 4 LEDs
    digitalWriteFast(PULSE5, 0);
    digitalWriteFast(PULSE6, 0);
    digitalWriteFast(PULSE8, 0);
    digitalWriteFast(PULSE9, 0);
    digitalWriteFast(PULSE10, 0);

    //Serial_Printf("loaded bat = %d counts %fV\n", value, value * (REF_VOLTAGE / 65536));
  }  // if

  // set Bat_meas pin to high impedance
  pinMode(BATT_ME, INPUT); 

  int milli_volts = 1000 * ((value / 65536.) * REF_VOLTAGE) / ((float)R1 / (R1 + R2));

  return milli_volts;
}

// find percentage of battery remaining

int battery_percent(int load)
{
  int v = battery_level(load);   // get mv, test without load

  // Serial_Printf("level = %d, load = %d\n",v,load);

  float min_level;
  
  if (load)
     min_level = BAT_MIN_LOADED * 1000;     // consider this min charge level on a lithium battery when loaded
  else
     min_level = BAT_MIN * 1000;            // consider this min charge level on a lithium battery when loaded
      
  const float max_level = BAT_MAX * 1000;   // consider this fully charged

  v = round(((v - min_level) / (max_level - min_level)) * 100);     // express as %
  v = constrain(v,0,100);

  // Serial_Printf("v = %d\n",v);

  return v;
}

// return 1 if the accelerometer values haved changed
#define ACCEL_CHANGE 60
void MMA8653FC_read(int *axeXnow, int *axeYnow, int *axeZnow);

int accel_changed()
{
  int x, y, z;
  static int prev_x, prev_y, prev_z;
  int changed = 0;

  MMA8653FC_read(&x, &y, &z);

  if (abs(x - prev_x) > ACCEL_CHANGE || abs(y - prev_y) > ACCEL_CHANGE || abs(z - prev_z) > ACCEL_CHANGE)
    changed = 1;

  prev_x = x;        // always update this so slow change won't trigger it
  prev_y = y;
  prev_z = z;

  return changed;
}  // accel_changed()

const unsigned long SHUTDOWN = 240 * 1000;   // power down after X ms of inactivity
static unsigned long last_activity = millis();

// record that we have seen serial port activity (used with powerdown())
void activity() {
  last_activity = millis();
}

// if not on USB and there hasn't been any activity for x seconds, then power down BLE and sleep

void powerdown() {

  if ((millis() - last_activity > SHUTDOWN /* && !Serial */) || battery_low(0)) {   // if USB is active, no timeout sleep

    // TODO - turn off unneeded peripherals or some pins to high impedance/floating?
    // TODO put accelerometer into lowest power mode
    // TODO - have accelerometer create interrupt to wake up

    accel_changed();     // update values with current

    // wake up if the device has changed orientation
    // remain in sleep if battery is low

    for (;;) {
      while (battery_low(0)) 
        sleep_mode(60000);    // sleep much longer for low bat

      // note: Accel runs down to 2V - ie, battery is fine
      if (accel_changed() && !battery_low(0))     //       Accel requires ~2ms from power on.  So leave it powered.
        break;
      else
        sleep_mode(200);          // sleep for 200 ms (can be much longer if accel interrupts on movement)

    } // for

    // note, peripherals are now in an unknown state
    // calling setup() + turn on peripherals might also work and would preserve ram contents (allowing hibernate in more places)

    // reboot to turn everything on and re-intialize peripherals
#define CPU_RESTART_ADDR ((uint32_t *)0xE000ED0C)
#define CPU_RESTART_VAL 0x5FA0004
    *CPU_RESTART_ADDR = CPU_RESTART_VAL;

  } // if
}  // powerdown()


//#define USE_HIBERNATE  // doesn't work, you have to edit the library source code
#include <Snooze.h>

static SnoozeBlock config;

// enter sleep mode for n milliseconds
void sleep_mode(const int n)
{
  // Set Low Power Timer wake up in milliseconds.
  config.setTimer(n);      // milliseconds

#ifdef USE_HIBERNATE
  Snooze.hibernate( config );
#else
  Snooze.deepSleep( config );
#endif

} // sleep_mode()


// print message for every I2C device on the bus
// original author unknown

#include <i2c_t3.h>

void scan_i2c(void)
{
  byte error, address;
  int nDevices;

  Serial.println("Scanning...");

  nDevices = 0;
  for (address = 1; address < 127; address++ )
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.print(address, HEX);
      Serial.println("  !");

      nDevices++;
    }
    else if (error == 4)
    {
      Serial.print("Unknow error at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.println(address, HEX);
    }
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");

} // scan_i2c()

#if 0

int conv2d(const char* p) {
  int v = 0;
  if ('0' <= *p && *p <= '9')
    v = *p - '0';
  return 10 * v + *++p - '0';
}

#include <Time.h>

// get the compiled time and use it to set the system time and the RTC

void timefromcompiler(void) {
  const char *date = __DATE__;
  const char *time = __TIME__;

  int _days, _month = 1, _year, _hour, _minute, _second;
  uint32_t _ticks;

  //Day
  _days = conv2d(date + 4);

  //Month
  switch (date[0]) {
    case 'J':
      if (date[1] == 'a')  // Jan
        _month = 1;
      else if (date[2] == 'n')  // June
        _month = 6;
      else
        _month = 7;  // July
      break;
    case 'F': _month = 2; break;
    case 'A': _month = date[2] == 'r' ? 4 : 8; break;
    case 'M': _month = date[2] == 'r' ? 3 : 5; break;
    case 'S': _month = 9; break;
    case 'O': _month = 10; break;
    case 'N': _month = 11; break;
    case 'D': _month = 12; break;
  }

  //Year
  _year = conv2d(date + 9);

  //Time
  _hour = conv2d(time);
  _minute = conv2d(time + 3);
  _second = conv2d(time + 6);

  // This sets the system time (NOT the Teensy RTC Clock)
  // set your seperated date/time variables out as normal and update system time FIRST
  setTime(_hour, _minute, _second, _days, _month, _year);

  // now we can use the system time to update the Teensy's RTC bits
  // This sets the RTC Clock from system time - epoch style, just like it wants :)
  Teensy3Clock.set(now());

  Serial_Printf("Set RTC to: %d-%d-%dT%d:%d:%d.000Z\n", year(), month(), day(), hour(), minute(), second());
}

#endif




