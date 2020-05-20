picoReflow 
==========

This README and project are being updated as of May 2019, more alterations and uploads as per as-built prototype.

Turns a Orange Pi Zero into a universal & web-enabled Reflow Oven Controller.

Uses a custom PCB with IO expanders, watchdog and thermocouple ICs via 1-wire, SPI and i2c.

**Standard Interface**

![Image](https://apollo.open-resource.org/_media/mission:resources:picoreflow_webinterface.jpg)

**Curve Editor**

![Image](https://apollo.open-resource.org/_media/mission:resources:picoreflow_webinterface_edit.jpg)

## Hardware

  * Orange Pi Zero H2+
  * 4x MAX 31855 1-wire K-Type Thermocouple
  * 1x MPC23008  8 channel GPIO Expander 
  * 8x AC zero-crossing Solid-State-Relays
  * 8x ACS7xx Alegro +-5A Isolated Current Sensor IC 
  * 1x MPC xxxx 8 channel SPI ADC
  * 1x Ti xxx Hardware (programmable) watchdog
  * 1x LT4320 Ideal Diode Bridge Controller (24Vac -> DC)
  * 1x Relay switched 12V output

### To update/fix on PCB 0.1

  * Power 1-wire from 3.3v OUTPUT of Orange Pi
  * Switch MOSI/MISO pins on Orange Pi breakout
  * Add one more AC solid state relays
  * Add two AC Return pins per "block"
  * Add DC motor driver/relay
  * Update OLED display connector pin-out
  * Change to thermocouple mini blocks instead of terminals
  * Change to terminal blocks to socket + connector (top entry)
  * Update overlay to show isolation zones
  
### To update/fix in cabling diagram

  * Add thermal fuse
  * Add front panel deadman display

## Installation

### Dependencies

We've tried to keep external dependencies to a minimum to make it easily
deployable on any flavor of open-source operating system. If you deploy it
successfully on any other OS, please update this:

#### Currently tested versions

  * greenlet-0.4.2
  * bottle-0.12.4
  * gevent-1.0
  * gevent-websocket-0.9.3

#### Ubuntu/Raspbian

    $ sudo apt-get install python-pip python-dev libevent-dev
    $ sudo pip install ez-setup
    $ sudo pip install greenlet bottle gevent gevent-websocket

#### Gentoo

    $ emerge -av dev-libs/libevent dev-python/pip
    $ pip install ez-setup
    $ pip install greenlet bottle gevent gevent-websocket

#### Raspberry PI deployment

If you want to deploy the code on a PI for production:

    $ pip install RPi.GPIO

This **only applies to non-Raspbian installations**, since Raspbian ships
RPi.GPIO with the default installation.

If you also want to use the in-kernel SPI drivers with a MAX31855 sensor:

    $ sudo pip install Adafruit-MAX31855

### Clone repo

    $ git clone https://github.com/ArakniD/picoReflow.git
    $ cd picoReflow

## Configuration

All parameters are defined in config.py, just copy the example and review/change to your mind's content.

    $ cp config.py.EXAMPLE config.py

## Usage

### Server Startup

    $ ./picoReflowd.py

### Autostart Server onBoot
If you want the server to autostart on boot, run the following commands

    sudo cp /home/pi/picoReflow/lib/init/reflow /etc/init.d/
    sudo chmod +x /etc/init.d/reflow
    sudo update-rc.d reflow defaults

### Client Access

Open Browser and goto http://127.0.0.1:8080 (for local development) or the IP
of your PI and the port defined in config.py (default 8080).

## License

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

## Support & Contact

Please raise issues for This fork of picroReflow here; Otherwise base package
issues should also go back to the root project :)

### Original works/issues

Please use the issue tracker for project related issues.

More info: https://apollo.open-resource.org/mission:resources:picoreflow
