const int VOLTAGE_SENSOR_PIN = 9; // A10 on XIAO ESP32-S3 corresponds to GPIO9
const unsigned long voltagePrintInterval = 1000; // 1000 milliseconds = 1 second

// --- Voltage Divider and ADC Configuration ---
// IMPORTANT: Verify R1 and R2 match your physical connections!
// R1 is the resistor from your actual voltage source (Vin) to the ADC pin.
// R2 is the resistor from the ADC pin to Ground.
const float R1_OHMS = 10000.0f; // e.g., 10k Ohm
const float R2_OHMS = 3840.0f;  // e.g., 3.84k Ohm

const float ADC_VREF = 3.3f;       // ADC reference voltage for ESP32 (usually 3.3V)
const float ADC_MAX_VALUE = 4095.0f; // ESP32 has a 12-bit ADC (2^12 - 1)

inline float readBatteryVoltage() {
  int sensorValue = analogRead(VOLTAGE_SENSOR_PIN); // Read the analog value (0-4095)
    
    // Calculate voltage at the ADC pin
    float v_at_adc_pin = sensorValue * (ADC_VREF / ADC_MAX_VALUE);
    
    // Calculate the actual input voltage before the divider
    // Vin = V_at_adc_pin * (R1 + R2) / R2
    float v_in_actual = 0.0f;
    if (R2_OHMS > 0.001f) { // Avoid division by zero
          v_in_actual = v_at_adc_pin * (R1_OHMS + R2_OHMS) / R2_OHMS;
    }
    return v_in_actual;
    /*
    Serial.print("A10(GPIO1) Raw: ");
    Serial.print(sensorValue);
    Serial.print(" | V_adc: ");
    Serial.print(v_at_adc_pin, 2); // Voltage at ADC pin, 2 decimal places
    Serial.print(" V | Vin_actual: ");
    Serial.print(v_in_actual, 2);   // Calculated actual input voltage, 2 decimal places
    Serial.println(" V");*/
}