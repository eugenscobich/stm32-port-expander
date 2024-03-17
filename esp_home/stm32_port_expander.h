#include "esphome.h"

static const char *logTag = "Stm32PortExpander";

#define get_stm32_port_expander(constructor) static_cast<Stm32PortExpander *>(constructor.get_component(0))
#define stm32_port_expander_float_output(parent, pin) get_stm32_port_expander(parent)->get_float_output(pin)
#define stm32_port_expander_binary_output(parent, pin) get_stm32_port_expander(parent)->get_binary_output(pin)
#define stm32_port_expander_binary_sensor(parent, pin) get_stm32_port_expander(parent)->get_binary_sensor(pin)
#define stm32_port_expander_analog_sensor(parent, pin) get_stm32_port_expander(parent)->get_analog_sensor(pin)

class Stm32PortExpander;

using namespace esphome;

class Stm32PortExpanderFloatOutput: public FloatOutput {
public:
	Stm32PortExpanderFloatOutput(Stm32PortExpander *parent, uint8_t pin) {
		this->parent_ = parent;
		this->pin_ = pin;
		this->value_ = 101;
	}
	void write_state(float state) override;
	uint8_t get_pin() {
		return this->pin_;
	}

protected:
	Stm32PortExpander *parent_;
	uint8_t pin_;
	uint8_t value_;

	friend class Stm32PortExpander;
};

class Stm32PortExpanderBinaryOutput: public BinaryOutput {
public:
	Stm32PortExpanderBinaryOutput(Stm32PortExpander *parent, uint8_t pin) {
		this->parent_ = parent;
		this->pin_ = pin;
		this->state_ = 3;
	}
	void write_state(bool state) override;
	uint8_t get_pin() {
		return this->pin_;
	}

protected:
	Stm32PortExpander *parent_;
	uint8_t pin_;
	uint8_t state_;

	friend class Stm32PortExpander;
};

class Stm32PortExpanderBinarySensor: public BinarySensor {
public:
	Stm32PortExpanderBinarySensor(Stm32PortExpander *parent, uint8_t pin) {
		this->parent_ = parent;
		this->pin_ = pin;
	}
	uint8_t get_pin() {
		return this->pin_;
	}

protected:
	Stm32PortExpander *parent_;
	uint8_t pin_;
	bool state_;

	friend class Stm32PortExpander;
};

class Stm32PortExpanderAnalogSensor: public Sensor {
public:
	Stm32PortExpanderAnalogSensor(Stm32PortExpander *parent, uint8_t pin) {
		this->parent_ = parent;
		this->pin_ = pin;
	}
	uint8_t get_pin() {
		return this->pin_;
	}

protected:
	Stm32PortExpander *parent_;
	uint8_t pin_;
	uint8_t value_;

	friend class Stm32PortExpander;
};

// ===================== Stm32PortExpander ===================== //

class Stm32PortExpander: public Component, public I2CDevice {
public:
	Stm32PortExpander(I2CBus *bus, uint8_t address, unsigned long pull_timeout = 1000) {
		set_i2c_address(address);
		set_i2c_bus(bus);
		this->pull_timeout_ = pull_timeout;
		this->current_pull_timeout_ = millis();
	}

	void setup() override {
		ESP_LOGCONFIG(logTag, "Setting up Stm32PortExpander at %#02X ...", address_);
	}

	void loop() override {
		if (this->retry_timeout_ == 0 || this->retry_timeout_ < millis()) {
			if (this->current_pull_timeout_ + this->pull_timeout_ < millis()) {
				this->current_pull_timeout_ = millis();
				for (Stm32PortExpanderBinarySensor *sensor : this->binary_sensor_pins_) {
					uint8_t pinNumber = sensor->get_pin();
					if (ERROR_OK != this->read_register(pinNumber, &this->data_, 1)) {
						ESP_LOGE(logTag, "Error reading binary sensor at pin[%d].", pinNumber);
						this->retry_timeout_ = millis() + 5000;
						return;
					}
					ESP_LOGD(logTag, "Successful received binary sensor value[%d] for pin[%d].", data_, pinNumber);
					sensor->publish_state(data_ == 1);
				}

				for (Stm32PortExpanderAnalogSensor *sensor : this->analog_sensor_pins_) {
					uint8_t pinNumber = sensor->get_pin();
					if (ERROR_OK != this->read_register(pinNumber, &this->data_, 1)) {
						ESP_LOGE(logTag, "Error reading analog sensor at pin[%d].", pinNumber);
						this->retry_timeout_ = millis() + 5000;
						return;
					}
					ESP_LOGD(logTag, "Successful received analog sensor value[%d] for pin[%d].", data_, pinNumber);
					sensor->publish_state(data_);
				}
				this->retry_timeout_ = 0;
			}
		}
	}

	FloatOutput* get_float_output(uint8_t pin) {
		Stm32PortExpanderFloatOutput *output = new Stm32PortExpanderFloatOutput(this, pin);
		float_output_pins_.push_back(output);
		return output;
	}

	BinaryOutput* get_binary_output(uint8_t pin) {
		Stm32PortExpanderBinaryOutput *output = new Stm32PortExpanderBinaryOutput(this, pin);
		binary_output_pins_.push_back(output);
		return output;
	}

	BinarySensor* get_binary_sensor(uint8_t pin) {
		Stm32PortExpanderBinarySensor *sensor = new Stm32PortExpanderBinarySensor(this, pin);
		binary_sensor_pins_.push_back(sensor);
		return sensor;
	}

	Sensor* get_analog_sensor(uint8_t pin) {
		Stm32PortExpanderAnalogSensor *sensor = new Stm32PortExpanderAnalogSensor(this, pin);
		analog_sensor_pins_.push_back(sensor);
		return sensor;
	}

	void write_value(uint8_t pin, uint8_t value) {
		ESP_LOGD(logTag, "Writing value[%d] to pin[%d]", value, pin);
		ErrorCode status = this->write_register(pin, &value, 1);
		if (status == ERROR_OK) {
			ESP_LOGI(logTag, "Written successful value[%d] to pin[%d]", value, pin);
		} else {
			ESP_LOGE(logTag, "Fail to write value[%d] to pin[%d]", value, pin);
		}
	}

protected:
	uint8_t data_;
	unsigned long retry_timeout_ = 0;
	unsigned long current_pull_timeout_= 0;
	unsigned long pull_timeout_ = 0;
	std::vector<Stm32PortExpanderFloatOutput*> float_output_pins_;
	std::vector<Stm32PortExpanderBinaryOutput*> binary_output_pins_;
	std::vector<Stm32PortExpanderBinarySensor*> binary_sensor_pins_;
	std::vector<Stm32PortExpanderAnalogSensor*> analog_sensor_pins_;
};

void Stm32PortExpanderFloatOutput::write_state(float state) {
	this->value_ = uint8_t(state * 100);
	ESP_LOGD("Stm32PortExpanderFloatOutput", "FloatOutput: pin[%d], raw_value[%f] and value[%d]", pin_, state, value_);
	this->parent_->write_value(this->pin_, this->value_);
}

void Stm32PortExpanderBinaryOutput::write_state(bool state) {
	this->state_ = state ? 1 : 0;
	ESP_LOGD("Stm32PortExpanderBinaryOutput", "BinaryOutput: pin[%d], state[%b]", pin_, state_);
	this->parent_->write_value(this->pin_, (uint8_t) this->state_);
}
