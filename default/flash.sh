#!/bin/sh

avrdude -p m328p -c avrispmkII -P usb -U flash:w:ap_ppm_encoder.hex
