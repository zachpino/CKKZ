# Sensors and Actuators

Let's wire-up some sensors and build some plant datasets!

-----

### Raspberry Pi Pinout

![pinout](https://www.raspberrypi-spy.co.uk/wp-content/uploads/2012/06/raspberry_pi_3_model_b_plus_gpio.jpg)

-----

### Setting Up Interfaces

Open the Raspberry Pi menu -> Raspberry Pi Configuration and visit the "Interfaces" tab. Enable...

- SSH
- VNC
- SPI
- I2C

Also, under the "Localisation" tab, make sure all settings match US standards to prevent weird keyboard behavior.

Download and configure [RealVNC](https://www.realvnc.com/en/connect/download/viewer/windows/) to enable remote Raspberry Pi management.

-----

### Updating Raspberry Pi System Software

In Terminal, enter the following three commands in sequence, hitting *enter* after each. Terminal will spew out some progress data as it fetches updates from various code repositories. Some of these fetches might fail, but the update process will be successful so long as the final line reports success. 

```
sudo apt-get update
sudo apt-get upgrade
```

It is necessary to reboot the Pi before continuing, to load the new system software.

```
sudo reboot now
```

-----

### Adafruit Base Sensor Libraries for I2C

Conveniently, the Adafruit company provides a foundational library for data collection and instrumentation for the I2C protocol. 

Again, one line at a time, hitting *enter* after each. Process updates will be printed throughout, which can usually be safely ignored.

```
pip3 install RPI.GPIO
pip3 install adafruit-blinka
```

For more information on these libraries, consults this [Adafruit Tutorial](https://learn.adafruit.com/circuitpython-on-raspberrypi-linux/installing-circuitpython-on-raspberry-pi).

-----

### I2C

Moving on to individual components, let us consider the I2C data exchange protocol, used by nearly all contemporary microcontroller interfacing components in hobbyist and production contexts.

I2C is a four wire data protocol.

- VIN : Voltage Source (usually specified as 3.3 volts (*3V3*) in microcontroller contexts)
- GND : Voltage Sink
- SCL : Serial Clock, the signals that modulate which component should speak when
- SDA : Serial Data, the actual data is shared on this pin

Because of the variability in how any individual sensor manipulates electrical signals, hardware developers use simplifying exchange protocols for more interoperability. The [I2C digital communication standard](https://learn.sparkfun.com/tutorials/i2c), short for "inter-integrated circuits" and pronounced "eye-squared-sea," was invented in 1982 to aid in the digital communication between a fast 'master' device and many slower attached 'slave' devices — solving the timing problems exhibited by its competitor standard, [SPI](https://en.wikipedia.org/wiki/Serial_Peripheral_Interface). Each device in an I2C system needs to have a unique address, and up to 111 unique devices can be connected in the same circuit, *on the same 4 pins*. This expands microcontroller capabilities considerably.

(BTW the offensive and anachronistic naming convention unfortunately chosen by the I2C creators is currently being attacked by most [progressive computer scientists](https://www.vice.com/en_us/article/8x7akv/masterslave-terminology-was-removed-from-python-programming-language). Let's hope it soon gets replaced fully!)

I2C is an easy to implement standard that uses pins labels `SDA` and `SCL` for all communication. `SDA` carries the *da*ta between master and slave, and `SCL` keeps the *cl*ock of those transfers, so that each can speak and listen at the right times. On our Raspberry Pis, I2C connections happen on the appropriately labeled `SDA` and `SCL` pins, which you can reference in the Raspberry Pi pinout above.

Each sensor/actuator in an I2C network must have a unique address to prevent data collisions. Many I2C sensors allow configurable addresses amongst a narrow set of options (usually changed through slicing traces on the board). Not all boards have this ability, and so multiple identical sensors in the same network is sometimes a serious challenge. 

---

### SI7021 Temperature and Humidity Sensor

![temp humidity](https://cdn-shop.adafruit.com/970x728/3251-00.jpg)

The [Si7021](https://www.adafruit.com/product/3251) reads temperature and humidity with automatic relative humidity adjustment, plus or minus 3%. The data may require calibration, but it will be relatively accurate to plus or minus .4 degrees Celsius from -10 to 85 degrees Celsius.

Consult the [datasheet](https://cdn-learn.adafruit.com/assets/assets/000/035/931/original/Support_Documents_TechnicalDocs_Si7021-A20.pdf) for additional technical info.

First, let's install the Adafruit library.

```
sudo pip3 install adafruit-circuitpython-si7021
```

And the simplest Python code to get a reading out of it...

```python
#necessary Python modules
import time
import board
import busio
import adafruit_si7021
 
# Create library object using our Bus I2C port
i2c = busio.I2C(board.SCL, board.SDA)
si7021 = adafruit_si7021.SI7021(i2c)
 
# Loop forever 
while True:
    print("Temperature: " + str(si7021.temperature) + "C")
    print("Humidity: " + str(si7021.relative_humidity) + "%")
    time.sleep(1)
```

-----

### VEML 7700 Light Sensor

![light](https://cdn-shop.adafruit.com/970x728/4162-04.jpg)

Light measurement is hard, since we are often interested in sensing the combination of a variety of different wavelengths. This particular sensor, the [VEML 7700](https://www.adafruit.com/product/4162#technical-details), is unique in being calibrated to real world brightness units (Lux), as well as sensing into the near IR and UV. 

Consult the [datasheet](https://www.vishay.com/docs/84286/veml7700.pdf) for additional technical info and sensitivity diagrams across wavelengths.


Library install...

```
sudo pip3 install adafruit-circuitpython-veml7700
```

Simple code...

```python
import time
import board
import busio
import adafruit_veml7700

# Create library object using our Bus I2C port
i2c = busio.I2C(board.SCL, board.SDA)
veml7700 = adafruit_veml7700.VEML7700(i2c)

#Loop forever
while True:
    print("Ambient light:", str(veml7700.light))
    time.sleep(1)
```

---

### SGP30 Air Quality Sensor

![Gas](https://cdn-shop.adafruit.com/970x728/3709-00.jpg)

The SGP30 is an advanced sensor that uses the oxidation of tin (rusting!) process alongside an optical fallback to detect specifically-sized molecules in the air, which comport well with indoor air quality factors. Notionally, it measures carbon dioide equivalence (eCO2) in parts-per-million, but is also reports data on [volatile organic compounds](https://en.wikipedia.org/wiki/Volatile_organic_compound) which the human nose would detect as *bad smells*. These sensors are often used for chemical plant monitoring and pollution regulation.

Consult the [datasheet](https://www.vishay.com/docs/84286/veml7700.pdf) for additional technical info and sensitivity diagrams across wavelengths.

[Datasheet](https://www.sensirion.com/fileadmin/user_upload/customers/sensirion/Dokumente/9_Gas_Sensors/Datasheets/Sensirion_Gas_Sensors_SGP30_Datasheet.pdf) for more info!

Unfortunately, this sensor needs time to not only warm up its "hot plate " (the region of sensor where the rusting happens), but also need time to [calibrate to its environment](https://learn.adafruit.com/adafruit-sgp30-gas-tvoc-eco2-mox-sensor/). The code below automatically re-calibrates over time, and takes around 12 hours of continuous reading to fully acclimate.

Library install...

```
sudo pip3 install adafruit-circuitpython-sgp30
```

Simple code...

```python
#import modules
import time
import board
import busio
import adafruit_sgp30
 
#Setup I2C communication
i2c = busio.I2C(board.SCL, board.SDA, frequency=100000)
 
# Create library object on our I2C port
sgp30 = adafruit_sgp30.Adafruit_SGP30(i2c)

# Read out serial number of individual sensor 
#print("SGP30 serial #", [hex(i) for i in sgp30.serial])
 
#turn on sensor and establish default calibration baseline. 
sgp30.iaq_init()
sgp30.set_iaq_baseline(0x8973, 0x8AAE)

#timer 
elapsed_sec = 0
 
#loop forever
while True:
    print("eCO2: " + str(sgp30.eCO2) + " ppm | TVOC:" + str(sgp30.TVOC) + " ppb")

    time.sleep(1)
    elapsed_sec += 1
    #recalibrate and see new baseline values every 10 seconds
    if elapsed_sec > 10:
        elapsed_sec = 0
        print(
            "**** Baseline values: eCO2 = 0x%x, TVOC = 0x%x"
            % (sgp30.baseline_eCO2, sgp30.baseline_TVOC)
        )
```

-----
 
### ADS1115 Analog to Digital Converter

![adc](https://cdn-shop.adafruit.com/970x728/1085-02.jpg)

The [ADS1115](https://www.adafruit.com/product/1085) is a boring, simple, commodity utility component for converting variable analog voltage to a digital pulse, with variable gain up to 16x.

![mq](https://lh3.googleusercontent.com/proxy/zulfNK3CxtEnR2mUiojNxQctZKkzoidtiL8x0-0BAg5YPF2QmFDz_cyp9iz487VwyMrSgSEbENF7jdC5otMsGNeG96EIpbNvRcWq4UeAf2Ohi0lZjsQ)

We will use this component to read the data out of our MQ-# analog gas sensors.

Install library...

```
sudo pip3 install adafruit-circuitpython-ads1x15
```

Simple code...

```python
#import modules
import board
import busio
import time
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn

#setup i2c and sensor 
i2c = busio.I2C(board.SCL, board.SDA)
ads = ADS.ADS1115(i2c)

#adjust gain, 1,2,4,8,16
ads.gain = 8

while True:
	#take converted reading
	analogReading = AnalogIn(ads, ADS.P0)

	#print data
	print(analogReading.value, analogReading.voltage)

	time.sleep(1)

```

-----

### STEMMA Soil Moisture Sensor

![stemma](https://cdn-shop.adafruit.com/970x728/4026-01.jpg)

The [STEMMA](https://www.adafruit.com/product/4026) is a *capacitive*, rather than resistive, sensor to detect the amount of conductive material in variably dry volume. This component is not waterproof on its top, and so cannot survive much splash! 

Check out the [datasheet](http://ww1.microchip.com/downloads/en/devicedoc/atmel-42242-sam-d10_datasheet.pdf) for the capacitive controller, which basically measures how effective its environment is as a battery. Weird.

Library...

```
sudo pip3 install adafruit-circuitpython-seesaw
```

Simple code...

```python
# import modules
import time
from board import SCL, SDA
import busio
from adafruit_seesaw.seesaw import Seesaw
 
#setup communication bus and sensor
i2c_bus = busio.I2C(SCL, SDA)
stemma = Seesaw(i2c_bus, addr=0x36)
 
#loop forever
while True:
    # read moisture level through capacitive touch pad
    touch = stemma.moisture_read()
 
    # read temperature from the temperature sensor
    temp = stemma.get_temp()
 
    print("temp: " + str(temp) + "  moisture: " + str(touch))
    time.sleep(1)
```

-----

### L293D Motor Driver

![motors](https://rees-fbcb.kxcdn.com/7541-thickbox_default/l293d-powerdip-16-stepper-motor-controller-driver-ic-ic008.jpg)

Motors are simple components, that need complicated controllers to do anything useful. The L293D is used by just about every motorized system, as it is super cheap, reliable, and power-draw conscientious. It is, however, a computer itself and adds a lot of complexity to projects. No library needed.

Fancy code from [this page](https://learn.adafruit.com/adafruit-raspberry-pi-lesson-9-controlling-a-dc-motor/software) for variable speed and directional control of any motor...

```python
import RPi.GPIO as io
io.setmode(io.BCM)

in1_pin = 4
in2_pin = 17

io.setup(in1_pin, io.OUT)
io.setup(in2_pin, io.OUT)

def set(property, value):
    try:
        f = open("/sys/class/rpi-pwm/pwm0/" + property, 'w')
        f.write(value)
        f.close()	
    except:
        print("Error writing to: " + property + " value: " + value)
 
set("delayed", "0")
set("mode", "pwm")
set("frequency", "500")
set("active", "1")

def clockwise():
    io.output(in1_pin, True)    
    io.output(in2_pin, False)

def counter_clockwise():
    io.output(in1_pin, False)
    io.output(in2_pin, True)

clockwise()

while True:
    cmd = raw_input("Command, f/r 0..9, E.g. f5 :")
    direction = cmd[0]
    if direction == "f":
        clockwise()
    else: 
        counter_clockwise()
    speed = int(cmd[1]) * 11
    set("duty", str(speed))
```

-----


