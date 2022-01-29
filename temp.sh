#!/bin/sh

# fetches the status from silviapid
# format is:
# String( targetTemp ) + "|"
# + String( currentTemp ) + "|"
# + String( isHeating ) + "|"
# + String( millis() ) + "|"
# + String( isStandby )
# e.g. '100.00|99.96|0|2692644|0'
#
# and separates the current temperature value

curl -s http://silviapid/ajax_get_temp | cut -d\| -f2

# example: log the temperature every second until ^C
# while true; do ./temp.sh; sleep 1; done
