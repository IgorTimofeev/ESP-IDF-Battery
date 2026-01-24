#pragma once

#include <algorithm>

#include <esp_log.h>
#include <esp_timer.h>
#include <esp_adc/adc_oneshot.h>
#include <esp_adc/adc_cali.h>

namespace YOBA {
	template<
		adc_unit_t ADCUnit,
		adc_channel_t ADCChannel,

		uint32_t voltageMinMV,
		uint32_t voltageMaxMV,
		uint32_t voltageDividerR1,
		uint32_t voltageDividerR2,

		uint16_t tickRateHz = 1,
		uint8_t multisamplingThreshold = 8
	>
	class Battery {
		public:
			Battery(
				adc_oneshot_unit_handle_t* ADCOneshotUnit
			) :
				_ADCOneshotUnit(ADCOneshotUnit)
			{
				// static_assert(voltageOnDividerMaxMV <= 3300, "Retard alert: output voltage is too high for ADC reading");
			}

			void setup() {
				adc_oneshot_chan_cfg_t channelConfig {};
				channelConfig.atten = ADC_ATTEN_DB_12;
				channelConfig.bitwidth = ADC_BITWIDTH_12;
				ESP_ERROR_CHECK(adc_oneshot_config_channel(*_ADCOneshotUnit, ADCChannel, &channelConfig));

				#ifdef ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
					adc_cali_curve_fitting_config_t fittingConfig {};
					fittingConfig.unit_id = ADCUnit;
					fittingConfig.atten = ADC_ATTEN_DB_12;
					fittingConfig.bitwidth = ADC_BITWIDTH_DEFAULT;
					ESP_ERROR_CHECK(adc_cali_create_scheme_curve_fitting(&fittingConfig, &_caliHandle));

				#else
					adc_cali_line_fitting_config_t fittingConfig {};
					fittingConfig.unit_id = ADCUnit;
					fittingConfig.atten = ADC_ATTEN_DB_12;
					fittingConfig.bitwidth = ADC_BITWIDTH_DEFAULT;
					fittingConfig.default_vref = ADC_CALI_LINE_FITTING_EFUSE_VAL_DEFAULT_VREF;
					ESP_ERROR_CHECK(adc_cali_create_scheme_line_fitting(&fittingConfig, &_caliHandle));

				#endif
			}

			uint8_t getCharge() const {
				if (_voltage <= voltageMinMV) {
					return 0;
				}

				if (_voltage >= voltageMaxMV) {
					return 0xFF;
				}

				return static_cast<uint8_t>((_voltage - voltageMinMV) * 0xFF / (voltageMaxMV - voltageMinMV));
			}

			uint16_t getVoltage() const {
				return _voltage;
			}

			void tick() {
				if (esp_timer_get_time() < _tickTime)
					return;

				int sample;
				const auto error = adc_oneshot_get_calibrated_result(*_ADCOneshotUnit, _caliHandle, ADCChannel, &sample);

				// Timeout one same oneshot unit?
				if (error != ESP_OK) {
					ESP_ERROR_CHECK_WITHOUT_ABORT(error);
					return;
				}

				_sampleSum += sample;
				_sampleIndex++;

				if (_sampleIndex < multisamplingThreshold)
					return;

				auto voltage = _sampleSum / multisamplingThreshold;

				_sampleSum = 0;
				_sampleIndex = 0;

				// ESP_LOGI("bat", "voltage before: %d", voltage);

				// Restoring real battery voltage
				voltage = voltageMinMV + (voltageMaxMV - voltageMinMV) * voltage / voltageOnDividerMaxMV;

				// ESP_LOGI("bat", "voltage after: %d", voltage);

				_voltage = static_cast<uint16_t>(voltage);

				_tickTime = esp_timer_get_time() + 1'000'000 / (tickRateHz * multisamplingThreshold);
			}

		private:
			constexpr static uint16_t voltageOnDividerMaxMV = voltageMaxMV * voltageDividerR2 / (voltageDividerR1 + voltageDividerR2);

			adc_oneshot_unit_handle_t* _ADCOneshotUnit;

			adc_cali_handle_t _caliHandle {};
			uint32_t _sampleSum = 0;
			uint8_t _sampleIndex = 0;
			uint16_t _voltage = 0;
			int64_t _tickTime = 0;
	};
}