#pragma once

#include <esp_log.h>
#include <esp_adc/adc_oneshot.h>
#include <esp_adc/adc_cali.h>

namespace YOBA {
	class Battery {
		public:
			Battery(
				const adc_unit_t ADCUnit,
				adc_oneshot_unit_handle_t* ADCOneshotUnit,
				const adc_channel_t ADCChannel,

				const uint16_t voltageMin,
				const uint16_t voltageMax,
				const uint32_t voltageDividerR1,
				const uint32_t voltageDividerR2
			) :
				_ADCUnit(ADCUnit),
				_ADCOneshotUnit(ADCOneshotUnit),
				_ADCChannel(ADCChannel),

				_voltageMin(voltageMin),
				_voltageMax(voltageMax),
				_voltageDividerR1(voltageDividerR1),
				_voltageDividerR2(voltageDividerR2)
			{

			}

			void setup() {
				adc_oneshot_chan_cfg_t channelConfig {};
				channelConfig.atten = ADC_ATTEN_DB_12;
				channelConfig.bitwidth = ADC_BITWIDTH_12;
				ESP_ERROR_CHECK(adc_oneshot_config_channel(*_ADCOneshotUnit, _ADCChannel, &channelConfig));

				#ifdef ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
					adc_cali_curve_fitting_config_t fittingConfig {};
					fittingConfig.unit_id = _ADCUnit;
					fittingConfig.atten = ADC_ATTEN_DB_12;
					fittingConfig.bitwidth = ADC_BITWIDTH_DEFAULT;
					ESP_ERROR_CHECK(adc_cali_create_scheme_curve_fitting(&fittingConfig, &_caliHandle));

				#else
					adc_cali_line_fitting_config_t fittingConfig {};
					fittingConfig.unit_id = _ADCUnit;
					fittingConfig.atten = ADC_ATTEN_DB_12;
					fittingConfig.bitwidth = ADC_BITWIDTH_DEFAULT;
					fittingConfig.default_vref = ADC_CALI_LINE_FITTING_EFUSE_VAL_DEFAULT_VREF;
					ESP_ERROR_CHECK(adc_cali_create_scheme_line_fitting(&fittingConfig, &_caliHandle));

				#endif
			}

			void tick() {
				// Multisampling
				int sample;

				ESP_ERROR_CHECK(adc_oneshot_get_calibrated_result(*_ADCOneshotUnit, _caliHandle, _ADCChannel, &sample));

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
			adc_unit_t _ADCUnit;
			adc_oneshot_unit_handle_t* _ADCOneshotUnit;
			adc_channel_t _ADCChannel;

			uint16_t _voltageMin;
			uint16_t _voltageMax;
			uint32_t _voltageDividerR1;
			uint32_t _voltageDividerR2;

			adc_cali_handle_t _caliHandle {};

			constexpr static uint8_t _sampleCount = 8;
			uint32_t _sampleSum = 0;
			uint8_t _sampleIndex = 0;
			uint16_t _voltage = 0;
	};
}