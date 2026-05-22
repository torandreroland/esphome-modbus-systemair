#include "ventilation_climate.h"

#include <cmath>

#include "esphome/core/log.h"

namespace esphome
{
  namespace ventilation_climate
  {

    static const char *const TAG = "ventilation_climate";

    static constexpr uint16_t REGISTER_FAN_SPEED = 100;
    static constexpr uint16_t REGISTER_TARGET_TEMPERATURE_WRITE = 207;

    VentilationClimate::RegisterItem::RegisterItem(VentilationClimate *owner, RegisterKind kind, uint16_t address)
        : owner_(owner), kind_(kind)
    {
      this->register_type = modbus::ModbusRegisterType::HOLDING;
      this->sensor_value_type = modbus::helpers::SensorValueType::U_WORD;
      this->start_address = address;
      this->bitmask = 0xFFFFFFFF;
      this->offset = 0;
      this->register_count = 1;
      this->skip_updates = 0;
      this->force_new_range = false;
    }

    void VentilationClimate::RegisterItem::parse_and_publish(const std::vector<uint8_t> &data)
    {
      auto raw = static_cast<uint16_t>(modbus_controller::payload_to_float(data, *this));
      this->owner_->parse_register_(this->kind_, raw);
    }

    void VentilationClimate::register_modbus_items(modbus_controller::ModbusController *parent)
    {
      this->parent_ = parent;
      parent->add_sensor_item(&this->fan_speed_item_);
      parent->add_sensor_item(&this->target_temperature_item_);
      parent->add_sensor_item(&this->current_temperature_item_);
      parent->add_sensor_item(&this->rotor_relay_item_);
    }

    void VentilationClimate::setup()
    {
      if (!this->fan_mode.has_value())
        this->fan_mode = climate::CLIMATE_FAN_OFF;
      this->publish_climate_state_();
    }

    void VentilationClimate::dump_config() { LOG_CLIMATE("", "Ventilation Climate", this); }

    climate::ClimateTraits VentilationClimate::traits()
    {
      auto traits = climate::ClimateTraits();
      traits.add_supported_mode(climate::CLIMATE_MODE_OFF);
      traits.add_supported_mode(climate::CLIMATE_MODE_HEAT);
      traits.add_supported_fan_mode(climate::CLIMATE_FAN_OFF);
      traits.add_supported_fan_mode(climate::CLIMATE_FAN_LOW);
      traits.add_supported_fan_mode(climate::CLIMATE_FAN_MEDIUM);
      traits.add_supported_fan_mode(climate::CLIMATE_FAN_HIGH);
      traits.add_feature_flags(climate::CLIMATE_SUPPORTS_CURRENT_TEMPERATURE);
      traits.add_feature_flags(climate::CLIMATE_SUPPORTS_ACTION);
      traits.set_visual_min_temperature(this->min_temperature_);
      traits.set_visual_max_temperature(this->max_temperature_);
      traits.set_visual_target_temperature_step(this->target_temperature_step_);
      traits.set_visual_current_temperature_step(0.1f);
      return traits;
    }

    void VentilationClimate::control(const climate::ClimateCall &call)
    {
      if (call.get_target_temperature().has_value())
        this->write_target_temperature_(*call.get_target_temperature());

      if (call.get_mode().has_value())
        this->write_hvac_mode_(*call.get_mode());

      if (call.get_fan_mode().has_value())
        this->write_fan_mode_(*call.get_fan_mode());

      this->publish_climate_state_();
    }

    void VentilationClimate::parse_register_(RegisterKind kind, uint16_t raw)
    {
      switch (kind)
      {
      case RegisterKind::FAN_SPEED:
        switch (raw)
        {
        case 1:
          this->fan_mode = climate::CLIMATE_FAN_LOW;
          break;
        case 2:
          this->fan_mode = climate::CLIMATE_FAN_MEDIUM;
          break;
        case 3:
          this->fan_mode = climate::CLIMATE_FAN_HIGH;
          break;
        case 0:
        default:
          this->fan_mode = climate::CLIMATE_FAN_OFF;
          break;
        }
        break;

      case RegisterKind::TARGET_TEMPERATURE:
      {
        float target = raw / 10.0f;
        this->has_target_temperature_readback_ = true;
        if (target > 0.0f)
        {
          this->mode = climate::CLIMATE_MODE_HEAT;
          this->target_temperature = target;
          this->internal_target_temperature_ = target;
        }
        else
        {
          this->mode = climate::CLIMATE_MODE_OFF;
          this->target_temperature = this->internal_target_temperature_;
        }
        break;
      }

      case RegisterKind::CURRENT_TEMPERATURE:
      {
        int32_t signed_raw = raw;
        if (signed_raw > 32767)
          signed_raw -= 65535;
        this->current_temperature = signed_raw / 10.0f;
        break;
      }

      case RegisterKind::ROTOR_RELAY:
        if (this->mode == climate::CLIMATE_MODE_OFF)
        {
          this->action = climate::CLIMATE_ACTION_OFF;
        }
        else
        {
          this->action = raw == 1 ? climate::CLIMATE_ACTION_HEATING : climate::CLIMATE_ACTION_IDLE;
        }
        break;
      }

      this->publish_climate_state_();
    }

    void VentilationClimate::publish_climate_state_()
    {
      if (!this->has_target_temperature_readback_)
        this->target_temperature = this->internal_target_temperature_;

      if (this->mode == climate::CLIMATE_MODE_OFF)
        this->action = climate::CLIMATE_ACTION_OFF;

      this->publish_state();
    }

    void VentilationClimate::write_hvac_mode_(climate::ClimateMode mode)
    {
      switch (mode)
      {
      case climate::CLIMATE_MODE_OFF:
        this->mode = climate::CLIMATE_MODE_OFF;
        this->action = climate::CLIMATE_ACTION_OFF;
        this->write_register_(REGISTER_TARGET_TEMPERATURE_WRITE, 0);
        break;
      case climate::CLIMATE_MODE_HEAT:
        this->mode = climate::CLIMATE_MODE_HEAT;
        this->write_register_(REGISTER_TARGET_TEMPERATURE_WRITE,
                              this->target_temperature_to_register_(this->internal_target_temperature_));
        break;
      default:
        ESP_LOGW(TAG, "Unsupported HVAC mode requested");
        break;
      }
    }

    void VentilationClimate::write_target_temperature_(float target_temperature)
    {
      this->internal_target_temperature_ = target_temperature;
      this->target_temperature = target_temperature;

      if (this->mode == climate::CLIMATE_MODE_HEAT)
      {
        this->write_register_(REGISTER_TARGET_TEMPERATURE_WRITE,
                              this->target_temperature_to_register_(target_temperature));
      }
    }

    uint16_t VentilationClimate::target_temperature_to_register_(float target_temperature) const
    {
      return static_cast<uint16_t>(std::round(target_temperature - (this->min_temperature_ - 1.0f)));
    }

    void VentilationClimate::write_fan_mode_(climate::ClimateFanMode fan_mode)
    {
      uint16_t fan_speed = 0;
      switch (fan_mode)
      {
      case climate::CLIMATE_FAN_LOW:
        fan_speed = 1;
        break;
      case climate::CLIMATE_FAN_MEDIUM:
        fan_speed = 2;
        break;
      case climate::CLIMATE_FAN_HIGH:
        fan_speed = 3;
        break;
      case climate::CLIMATE_FAN_OFF:
        fan_speed = 0;
        break;
      default:
        ESP_LOGW(TAG, "Unsupported fan mode requested");
        return;
      }

      this->fan_mode = fan_mode;
      this->write_register_(REGISTER_FAN_SPEED, fan_speed);
    }

    void VentilationClimate::write_register_(uint16_t address, uint16_t value)
    {
      if (this->parent_ == nullptr)
        return;

      auto command = modbus_controller::ModbusCommandItem::create_write_single_command(this->parent_, address, value);
      command.on_data_func = [this, command](modbus::ModbusRegisterType register_type, uint16_t start_address,
                                             const std::vector<uint8_t> &data)
      {
        this->parent_->on_write_register_response(command.register_type, start_address, data);
      };
      this->parent_->queue_command(command);
    }

  } // namespace ventilation_climate
} // namespace esphome
