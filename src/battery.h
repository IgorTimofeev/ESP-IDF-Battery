#pragma once

#include <limits>
#include <esp_timer.h>
#include <esp_log.h>
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"

namespace YOBA {
	class Battery {
		public:
			Battery(
				const adc_unit_t adcUnit,
				adc_oneshot_unit_handle_t* adcOneshotUnitHandle,
				const adc_channel_t adcChannel,
				const uint16_t voltageMin,
				const uint16_t voltageMax,
				const uint32_t voltageDividerR1,
				const uint32_t voltageDividerR2
			) :
				_unit(adcUnit),
				_unitHandle(adcOneshotUnitHandle),
				_channel(adcChannel),
				_voltageMin(voltageMin),
				_voltageMax(voltageMax),
				_voltageDividerR1(voltageDividerR1),
				_voltageDividerR2(voltageDividerR2)
			{

			}

			void setup() const {
				constexpr adc_oneshot_chan_cfg_t channelConfig = {
					.atten = ADC_ATTEN_DB_12,
					.bitwidth = ADC_BITWIDTH_12,
				};

				ESP_ERROR_CHECK(adc_oneshot_config_channel(*_unitHandle, _channel, &channelConfig));

				const adc_cali_line_fitting_config_t calibrationConfig = {
					.unit_id = _unit,
					.atten = ADC_ATTEN_DB_12,
					.bitwidth = ADC_BITWIDTH_DEFAULT,
					.default_vref = ADC_CALI_LINE_FITTING_EFUSE_VAL_DEFAULT_VREF
				};

				ESP_ERROR_CHECK(adc_cali_create_scheme_line_fitting(&calibrationConfig, const_cast<adc_cali_handle_t*>(&_caliHandle)));
			}

			void tick() {
				// Multisampling
				int sample;

				ESP_ERROR_CHECK(adc_oneshot_get_calibrated_result(*_unitHandle, _caliHandle, _channel, &sample));

				_sampleSum += sample;
				_sampleIndex++;

				if (_sampleIndex < _sampleCount)
					return;

				_voltage = _sampleSum / _sampleCount;

				_sampleSum = 0;
				_sampleIndex = 0;

				// Restoring real battery voltage based on dividers
				_voltage = _voltage * (_voltageDividerR1 + _voltageDividerR2) / _voltageDividerR2;

//					ESP_LOGI("Battery", "Avg: %lu, mapped: %d", _sampleSum / _sampleCount, _voltage);
			}

			uint8_t getCharge() const {
				if (_voltage <= _voltageMin) {
					return 0;
				}
				else if (_voltage >= _voltageMax) {
					return 0xFF;
				}
				else {
					return static_cast<uint8_t>((_voltage - _voltageMin) * 0xFF / (_voltageMax - _voltageMin));
				}
			}

			uint16_t getVoltage() const {
				return _voltage;
			}

		private:
			adc_unit_t _unit;
			adc_oneshot_unit_handle_t* _unitHandle;
			adc_channel_t _channel;
			adc_cali_handle_t _caliHandle {};

			uint16_t _voltageMin;
			uint16_t _voltageMax;
			uint32_t _voltageDividerR1;
			uint32_t _voltageDividerR2;

			constexpr static uint8_t _sampleCount = 8;
			uint32_t _sampleSum = 0;
			uint8_t _sampleIndex = 0;
			uint16_t _voltage = 0;
	};
}