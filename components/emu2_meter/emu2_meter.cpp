#include "emu2_meter.h"

#include "esphome/core/defines.h"
#include "esphome/core/version.h"
#include "esphome/core/log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "usb/usb_host.h"
#include "usb/cdc_acm_host.h"

extern "C" {
    #include "xml_parser.h"
    #include "emu2_parser.h"
    #include "murmur3.h"
}

#define USB_HOST_PRIORITY   (20)

//these need to define the device we've looking for
#define USB_DEVICE_VID      (0x04b4)
#define USB_DEVICE_PID      (0x0003) // 0x303A:0x4001 (TinyUSB CDC device)
#define USB_DEVICE_DUAL_PID (0x0003) // 0x303A:0x4002 (TinyUSB Dual CDC device)

// Change these values to match your needs
// #define BAUDRATE     (115200)
// #define STOP_BITS    (0)      // 0: 1 stopbit, 1: 1.5 stopbits, 2: 2 stopbits
// #define PARITY       (0)      // 0: None, 1: Odd, 2: Even, 3: Mark, 4: Space
// #define DATA_BITS    (8)

#define TX_TIMEOUT_MS       (9000)
#define BUFFER_SIZE       (512)

namespace esphome {
namespace emu2_meter {

static SemaphoreHandle_t device_disconnected_sem;

static XmlParser xml_parser;
static Emu2Parser emu2_parser;

cdc_acm_dev_hdl_t cdc_dev = NULL;

static const char *const TAG = "emu2_meter";

/**
 * @brief Data received callback
 *
 * @param[in] data     Pointer to received data
 * @param[in] data_len Length of received data in bytes
 * @param[in] arg      Argument we passed to the device open function
 * @return
 *   true:  We have processed the received data
 *   false: We expect more data
 */
static bool handle_rx(const uint8_t *data, size_t data_len, void *arg) {
    // ESP_LOGI(TAG, "Data received");
    // ESP_LOG_BUFFER_HEXDUMP(TAG, data, data_len, ESP_LOG_INFO);
    // printf("%.*s\n", data_len, data);

    xml_parser_process(&xml_parser, (char *)data, data_len);

    return true;
}

/**
 * @brief Device event callback
 *
 * Apart from handling device disconnection it doesn't do anything useful
 *
 * @param[in] event    Device event type and data
 * @param[in] user_ctx Argument we passed to the device open function
 */
static void handle_event(const cdc_acm_host_dev_event_data_t *event, void *user_ctx) {
    switch (event->type) {
        case CDC_ACM_HOST_ERROR:
            ESP_LOGE(TAG, "CDC-ACM error has occurred, err_no = %i", event->data.error);
            break;
        case CDC_ACM_HOST_DEVICE_DISCONNECTED:
            ESP_LOGI(TAG, "Device suddenly disconnected");
            ESP_ERROR_CHECK(cdc_acm_host_close(event->data.cdc_hdl));
            cdc_dev = NULL;
            xSemaphoreGive(device_disconnected_sem);
            break;
        case CDC_ACM_HOST_SERIAL_STATE:
            ESP_LOGI(TAG, "Serial state notif 0x%04X", event->data.serial_state.val);
            break;
        case CDC_ACM_HOST_NETWORK_CONNECTION:
        default:
            ESP_LOGW(TAG, "Unsupported CDC event: %i", event->type);
            break;
    }
}

/**
 * @brief USB Host library handling task
 *
 * @param arg Unused
 */
static void usb_lib_task(void *arg) {
    while (1) {
        // Start handling system events
        uint32_t event_flags;
        usb_host_lib_handle_events(portMAX_DELAY, &event_flags);

        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) {
            ESP_ERROR_CHECK(usb_host_device_free_all());
        }
        if (event_flags & USB_HOST_LIB_EVENT_FLAGS_ALL_FREE) {
            ESP_LOGI(TAG, "USB: All devices freed");
            // Continue handling USB events to allow device reconnection
        }
    }
}



static void handle_tag(char *tag, int is_terminating, void *userdata) {
  Emu2Parser *emu2_parser = (Emu2Parser *)userdata;
//   uint32_t hash = murmur3_32(tag, strlen(tag), 42);
//   printf("Tag: %s, hash: %08lx, end: %d\n", tag, hash, is_terminating);

  emu2_parser_process_tag(emu2_parser, tag, is_terminating);
}

static void handle_data(char *data, void *userdata) {
  Emu2Parser *emu2_parser = (Emu2Parser *)userdata;

  // printf("Data: %s\n", data);
  emu2_parser_process_data(emu2_parser, data);
}

void handle_metric(Emu2Metric metric, void *userdata) {
    Emu2Meter *meter = (Emu2Meter *)userdata;

    printf("Metric:\n  meter_mac_id = %#16llx\n  device_mac_id = %#16llx\n  type = %d\n", metric.meter_mac_id,  metric.device_mac_id, metric.tag);
     switch (metric.tag) {
      case EMU2_METRIC_InstantaneousDemand:
        printf("InstantaneousDemand:\n  demand = %d\n  multiplier = %d\n  divisor = %d\n  digits_left = %d\n  digits_right = %d\n  suppress_leading_zero = %d\n",
          metric.instantaneous_demand.demand,
          metric.instantaneous_demand.multiplier,
          metric.instantaneous_demand.divisor,
          metric.instantaneous_demand.digits_left,
          metric.instantaneous_demand.digits_right,
          metric.instantaneous_demand.suppress_leading_zero);

          meter->update_maximum_demand_active_power(metric.instantaneous_demand.demand * (float)metric.instantaneous_demand.multiplier / (float)metric.instantaneous_demand.divisor);
          // meter  total_energy
        break;

      case EMU2_METRIC_CurrentSummationDelivered:
        printf("CurrentSummationDelivered:\n  summation_delivered = %lld\n  summation_received = %lld\n  multiplier = %d\n  divisor = %d\n  digits_left = %d\n  digits_right = %d\n  suppress_leading_zero = %d\n",
          metric.current_summation_delivered.summation_delivered,
          metric.current_summation_delivered.summation_received,
          metric.current_summation_delivered.multiplier,
          metric.current_summation_delivered.divisor,
          metric.current_summation_delivered.digits_left,
          metric.current_summation_delivered.digits_right,
          metric.current_summation_delivered.suppress_leading_zero);

          meter->update_active_energy(
            (double)metric.current_summation_delivered.summation_delivered * (double)metric.current_summation_delivered.multiplier / (double)metric.current_summation_delivered.divisor,
            (double)metric.current_summation_delivered.summation_received * (double)metric.current_summation_delivered.multiplier / (double)metric.current_summation_delivered.divisor
          );
        break;

      case EMU2_METRIC_PriceCluster:
        printf("PriceCluster:\n  price = %d\n  trailing_digits = %d\n  currency = %d\n  tier = %d\n  start_time = %d\n  duration = %d\n  rate_label = %s\n",
          metric.price_cluster.price,
          metric.price_cluster.trailing_digits,
          metric.price_cluster.currency,
          metric.price_cluster.tier,
          metric.price_cluster.start_time,
          metric.price_cluster.duration,
          metric.price_cluster.rate_label);

          meter->update_price(
            (float)metric.price_cluster.price / (float)pow(10, metric.price_cluster.trailing_digits)
          );
        break;

      case EMU2_METRIC_ConnectionStatus:
        printf("ConnectionStatus:\n  status = %d\n  link_strength = %d\n  channel = %d\n  short_addr = %d\n  ext_pan_id = %lld\n",
          metric.connection_status.status,
          metric.connection_status.link_strength,
          metric.connection_status.channel,
          metric.connection_status.short_addr,
          metric.connection_status.ext_pan_id);

          meter->update_rssi(metric.connection_status.link_strength);
        break;
    }
}

void Emu2Meter::setup() {
    device_disconnected_sem = xSemaphoreCreateBinary();
    assert(device_disconnected_sem);

    const usb_host_config_t host_config = {
        .skip_phy_setup = false,
        .intr_flags = ESP_INTR_FLAG_LEVEL1,
    };
    ESP_LOGI(TAG, "Installing USB host");
    ESP_ERROR_CHECK(usb_host_install(&host_config));
    ESP_LOGI(TAG, "Installed USB host");


    // Create a task that will handle USB library events
    BaseType_t task_created = xTaskCreate(usb_lib_task, "usb_lib", 4096, xTaskGetCurrentTaskHandle(), USB_HOST_PRIORITY, NULL);
    // BaseType_t task_created = xTaskCreate(usb_lib_task, "usb_lib", 4096, NULL, 10, NULL);
    assert(task_created == pdTRUE);

    ESP_LOGI(TAG, "Installing CDC-ACM driver");
    ESP_ERROR_CHECK(cdc_acm_host_install(NULL));

    emu2_parser_init(&emu2_parser, handle_metric, NULL, this);
    xml_parser_init(&xml_parser, handle_tag, handle_data, NULL, &emu2_parser);

}

void Emu2Meter::loop() {

    const cdc_acm_host_device_config_t dev_config = {
        .connection_timeout_ms = TX_TIMEOUT_MS,
        .out_buffer_size = BUFFER_SIZE,
        .in_buffer_size = BUFFER_SIZE,
        .event_cb = handle_event,
        .data_cb = handle_rx,
        .user_arg = NULL,
    };

    if (cdc_dev == NULL) {
        while(true) {
            // Open USB device from tusb_serial_device example example. Either single or dual port configuration.
            ESP_LOGI(TAG, "Opening CDC ACM device 0x%04X:0x%04X...", USB_DEVICE_VID, USB_DEVICE_PID);
            esp_err_t err = cdc_acm_host_open(USB_DEVICE_VID, USB_DEVICE_PID, 0, &dev_config, &cdc_dev);
            if (ESP_OK != err) {
                ESP_LOGI(TAG, "Failed to open device");
                continue;
            }

            vTaskDelay(pdMS_TO_TICKS(100));

            break;
        }

        ESP_LOGI(TAG, "Opened device");
    }

    // cdc_acm_line_coding_t line_coding = {
    //     .dwDTERate = EXAMPLE_BAUDRATE,
    //     .bCharFormat = EXAMPLE_STOP_BITS,
    //     .bParityType = EXAMPLE_PARITY,
    //     .bDataBits = EXAMPLE_DATA_BITS,
    // };



    // esp_err_t err = mdns_init();
    // if (err != ESP_OK) {
    //     ESP_LOGW(TAG, "mDNS init failed: %s", esp_err_to_name(err));
    //     this->mark_failed();
    //     return;
    // }

    // mdns_hostname_set(this->hostname_.c_str());
    // mdns_instance_name_set(this->hostname_.c_str());

    // for (const auto &service : this->services_) {
    //     std::vector<mdns_txt_item_t> txt_records;
    //     for (const auto &record : service.txt_records) {
    //     mdns_txt_item_t it{};
    //     // dup strings to ensure the pointer is valid even after the record loop
    //     it.key = strdup(record.key.c_str());
    //     it.value = strdup(record.value.c_str());
    //     txt_records.push_back(it);
    //     }
    //     err = mdns_service_add(nullptr, service.service_type.c_str(), service.proto.c_str(), service.port,
    //                         txt_records.data(), txt_records.size());

    //     // free records
    //     for (const auto &it : txt_records) {
    //     delete it.key;    // NOLINT(cppcoreguidelines-owning-memory)
    //     delete it.value;  // NOLINT(cppcoreguidelines-owning-memory)
    //     }

    //     if (err != ESP_OK) {
    //     ESP_LOGW(TAG, "Failed to register mDNS service %s: %s", service.service_type.c_str(), esp_err_to_name(err));
    //     }
    // }
}

void Emu2Meter::on_shutdown() {
// mdns_free();
    // delay(40);  // Allow the mdns packets announcing service removal to be sent
}


// void Emu2Meter::update() {
//   float energy = this->qmp6988_data_.pressure / 100;
//   float demand = this->qmp6988_data_.temperature;

//   ESP_LOGD(TAG, "Temperature=%.2f°C, Pressure=%.2fhPa", temperature, pressurehectopascals);

//   if (this->total_active_energy_sensor_ != nullptr) {
//     this->total_active_energy_sensor_->publish_state(temperature);
//   }

//   if (this->maximum_demand_active_sensor_ != nullptr) {
//     this->maximum_demand_active_sensor_->publish_state(pressurehectopascals);
//   }

//   if (this->rssi_sensor_ != nullptr) {
//     this->rssi_sensor_->publish_state(pressurehectopascals);
//   }
// }

void Emu2Meter::dump_config() {
  // ESP_LOGCONFIG(TAG, "mDNS:");
  // ESP_LOGCONFIG(TAG, "  Hostname: %s", this->hostname_.c_str());
  // ESP_LOGV(TAG, "  Services:");
  // for (const auto &service : this->services_) {
  //   ESP_LOGV(TAG, "  - %s, %s, %d", service.service_type.c_str(), service.proto.c_str(), service.port);
  //   for (const auto &record : service.txt_records) {
  //     ESP_LOGV(TAG, "    TXT: %s = %s", record.key.c_str(), record.value.c_str());
  //   }
  // }

  // ESP_LOGCONFIG(TAG, "BMP280:");
  // LOG_I2C_DEVICE(this);
  // switch (this->error_code_) {
  //   case COMMUNICATION_FAILED:
  //     ESP_LOGE(TAG, "Communication with BMP280 failed!");
  //     break;
  //   case WRONG_CHIP_ID:
  //     ESP_LOGE(TAG, "BMP280 has wrong chip ID! Is it a BME280?");
  //     break;
  //   case NONE:
  //   default:
  //     break;
  // }
  // ESP_LOGCONFIG(TAG, "  IIR Filter: %s", iir_filter_to_str(this->iir_filter_));
  // LOG_UPDATE_INTERVAL(this);

  // LOG_SENSOR("  ", "Temperature", this->temperature_sensor_);
  // ESP_LOGCONFIG(TAG, "    Oversampling: %s", oversampling_to_str(this->temperature_oversampling_));
  // LOG_SENSOR("  ", "Pressure", this->pressure_sensor_);
  // ESP_LOGCONFIG(TAG, "    Oversampling: %s", oversampling_to_str(this->pressure_oversampling_));
}

void Emu2Meter::update_active_energy(double import_active_energy, double export_active_energy) {
  this->import_active_energy_sensor_->publish_state(import_active_energy);
  this->export_active_energy_sensor_->publish_state(export_active_energy);
  this->total_active_energy_sensor_->publish_state(import_active_energy - export_active_energy);
}

void Emu2Meter::update_maximum_demand_active_power(double maximum_demand_active_power) {
  this->maximum_demand_active_power_sensor_->publish_state(maximum_demand_active_power);
}

void Emu2Meter::update_price(double price) {
  this->price_sensor_->publish_state(price);
};

void Emu2Meter::update_rssi(uint8_t rssi) {
  this->rssi_sensor_->publish_state(rssi);
};

}  // namespace emu2_meter
}  // namespace esphome
