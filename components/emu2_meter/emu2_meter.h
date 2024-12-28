#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"

extern "C" {
    #include "emu2_parser.h"
}

namespace esphome {
namespace emu2_meter {

class Emu2Meter : public Component {
  public:
    void setup() override;
    void loop() override;
    void dump_config() override;
    void on_shutdown() override;
    float get_setup_priority() const override { return setup_priority::HARDWARE; }

    // void update() override;

    void set_total_active_energy_sensor(sensor::Sensor *total_active_energy_sensor) { total_active_energy_sensor_ = total_active_energy_sensor; }
    void set_import_active_energy_sensor(sensor::Sensor *import_active_energy_sensor) { import_active_energy_sensor_ = import_active_energy_sensor; }
    void set_export_active_energy_sensor(sensor::Sensor *export_active_energy_sensor) { export_active_energy_sensor_ = export_active_energy_sensor; }
    void set_maximum_demand_active_power_sensor(sensor::Sensor *maximum_demand_active_power_sensor) { maximum_demand_active_power_sensor_ = maximum_demand_active_power_sensor; }
    void set_price_sensor(sensor::Sensor *price_sensor) { price_sensor_ = price_sensor; }
    void set_rssi_sensor(sensor::Sensor *rssi_sensor) { rssi_sensor_ = rssi_sensor; }

    void update_active_energy(double import_active_energy, double export_active_energy);
    void update_maximum_demand_active_power(double maximum_demand_active_power);
    void update_price(double price);
    void update_rssi(uint8_t rssi);

  protected:
    sensor::Sensor *total_active_energy_sensor_{nullptr};
    sensor::Sensor *import_active_energy_sensor_{nullptr};
    sensor::Sensor *export_active_energy_sensor_{nullptr};
    sensor::Sensor *maximum_demand_active_power_sensor_{nullptr};
    sensor::Sensor *price_sensor_{nullptr};
    sensor::Sensor *rssi_sensor_{nullptr};

    enum ErrorCode {
      NONE = 0,
      COMMUNICATION_FAILED,
    } error_code_{NONE};
};

}  // namespace emu2
}  // namespace esphome
