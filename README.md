# ESPHome EMU-2 Energy Monitor

The ESPHome emu2_meter component exposes metrics from the [EMU-2](https://www.rainforestautomation.com/rfa-z105-2-emu-2-2/) energy monitor by Rainforest Automation. The EMU-2 energy monitor acts like a USB serial device (CDC-ACM), so the microcontroller running the component needs to be capable of running in USB host mode. The component was tested on ESP32-S3 microcontroller.

The component uses the [emu2-data-parser](https://github.com/mayo/emu2-data-parser) tag parser to interpret the XML generated by the monitor into structured metrics.

Follow the ESPHome [External Components](https://esphome.io/components/external_components.html) to use this component. See the [energy_monitor.yaml](examples/energy_monitor.yaml) example for reference.
