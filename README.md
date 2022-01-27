# dump978-rb

This is the original [FlightAware](https://github.com/flightaware/dump978) and [mutability](https://github.com/mutability/dump978) UAT decoder, just renamed.

It is a reimplementation in C++, loosely based on the demodulator from
https://github.com/mutability/dump978.

For prebuilt Raspbian packages, see https://flightaware.com/adsb/piaware/install

## Overview

dump978-rb is the main binary. It talks to the SDR, demodulates UAT data,
and provides the data in a variety of ways - either as raw messages or as
json-formatted decoded messages, and either on a network port or to stdout.

## Building as a package

```
$ sudo apt-get install \
  build-essential \
  debhelper \
  dh-systemd \
  libboost-system-dev \
  libboost-program-options-dev \
  libboost-regex-dev \
  libboost-filesystem-dev \
  libsoapysdr-dev

$ dpkg-buildpackage -b
$ sudo dpkg -i ../dump978-rb_*.deb
```

## Building from source

 1. Ensure SoapySDR and Boost are installed
 2. 'make'

## Installing the SoapySDR driver module

You will want at least one SoapySDR driver installed. For rtlsdr, try

```
$ sudo apt-get install soapysdr-module-rtlsdr
```

## Third-party code

Third-party source code included in libs/:

 * fec - from Phil Karn's fec-3.0.1 library (see fec/README)
 * json.hpp - JSON for Modern C++ v3.5.0 - https://github.com/nlohmann/json
