#!/bin/bash

# Copy interfaces from ../ros2/src/raros_interfaces to ./extra_packages/raros_interfaces
./copy_interfaces.sh

# install dependencies
pio pkg install

# ensure clean environment
pio run -t clean_microros

# Build & upload the application. Change the upload port to match your device.
pio run --target upload --upload-port /dev/ttyACM0