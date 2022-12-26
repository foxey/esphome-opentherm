[![build](https://github.com/foxey/esphome-opentherm/actions/workflows/build.yml/badge.svg)](https://github.com/foxey/esphome-opentherm/actions/workflows/build.yaml)

# Work in progress!

This component is not actually working until this notice has been removed.

# What does this implement?

This component provides support for opentherm devices such as:
* [DIYLess](https://diyless.com/)' Master OpenTherm Shield
* [Ihor Melnyk](http://ihormelnyk.com/opentherm_adapter)' OpenTherm adapter

Those are typically connected to an ESP8266 or ESP32

The functional aspect (OpenTherm communications) is heavily based on [ihormelnyk/opentherm_library](https://github.com/ihormelnyk/opentherm_library).

The goal of this component is not to provide a full-blown climate device, but rather expose a
bunch of OpenTherm data and functionality. To make use of this data and functionality is up to the user. 
This could be by - for example - using the exposed enities in ESPHome/HA automations or by using the 
exposed entities in other components (e.g. a combination of [PID Climate](https://esphome.io/components/climate/pid.html)
and a few [Template Outputs](https://esphome.io/components/output/template.html))

As this is my first component, I'm also looking for constructive criticism on how to enhance this
component where needed.

## Notes

This component is a fork of [khenderick/esphome-opentherm](https://github.com/khenderick/esphome-opentherm) and adapted to support operating in gateway mode. It is connected both to an OpenTherm master device (typically a thermostat) and an OpenTherm slave device (typically a CH boiler) and forwards all messages from the master to the slave.

[@hendrick](https://github.com/khenderick)'s component is currently also in an [open PR to esphome](https://github.com/esphome/esphome/pull/3921),
but has currently a low chance of acceptance since it uses a ~32ms delay in `loop()`. This should be
rewritten to work async.

Feel free to help me out and open a PR with improvements.

## Example usage:

```bash
esphome compile example_opentherm.yaml && esphome upload example_opentherm.yaml
```
