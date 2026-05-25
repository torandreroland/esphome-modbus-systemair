#pragma once

#include "esphome/components/climate/climate.h"
#include "esphome/components/modbus_controller/modbus_controller.h"
#include "esphome/core/component.h"

namespace esphome
{
  namespace ventilation_climate
  {

    class VentilationClimate : public climate::Climate, public Component
    {
    public:
      void setup() override;
      void dump_config() override;

      void set_parent(modbus_controller::ModbusController *parent) { this->parent_ = parent; }
      void register_modbus_items(modbus_controller::ModbusController *parent);
      void set_min_temperature(float min_temperature) { this->min_temperature_ = min_temperature; }
      void set_max_temperature(float max_temperature) { this->max_temperature_ = max_temperature; }
      void set_target_temperature_step(float target_temperature_step)
      {
        this->target_temperature_step_ = target_temperature_step;
      }

    protected:
      enum class RegisterKind : uint8_t
      {
        FAN_SPEED,
        TARGET_TEMPERATURE,
        CURRENT_TEMPERATURE,
        ROTOR_RELAY,
      };

      class RegisterItem : public modbus_controller::SensorItem
      {
      public:
        RegisterItem(VentilationClimate *owner, RegisterKind kind, uint16_t address);

        void parse_and_publish(const std::vector<uint8_t> &data) override;

      protected:
        VentilationClimate *owner_;
        RegisterKind kind_;
      };

      climate::ClimateTraits traits() override;
      void control(const climate::ClimateCall &call) override;

      void parse_register_(RegisterKind kind, uint16_t raw);
      void publish_climate_state_();
      void write_hvac_mode_(climate::ClimateMode mode);
      void write_fan_mode_(climate::ClimateFanMode fan_mode);
      void write_target_temperature_(float target_temperature);
      uint16_t target_temperature_to_register_(float target_temperature) const;
      void write_register_(uint16_t address, uint16_t value);

      modbus_controller::ModbusController *parent_{nullptr};

      float min_temperature_{10.0f};
      float max_temperature_{30.0f};
      float target_temperature_step_{1.0f};
      float internal_target_temperature_{18.0f};
      bool has_target_temperature_readback_{false};

      RegisterItem fan_speed_item_{this, RegisterKind::FAN_SPEED, 100};
      RegisterItem target_temperature_item_{this, RegisterKind::TARGET_TEMPERATURE, 207};
      RegisterItem current_temperature_item_{this, RegisterKind::CURRENT_TEMPERATURE, 213};
      RegisterItem rotor_relay_item_{this, RegisterKind::ROTOR_RELAY, 351};
    };

  } // namespace ventilation_climate
} // namespace esphome
