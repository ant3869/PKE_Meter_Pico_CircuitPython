"""

example:

from emf_processing import EMFProcessor

# Example initialization and data processing
emf_processor = EMFProcessor()

# Adjusting sensitivity and signal gain dynamically
emf_processor.set_sensitivity(1.2)  # Increase sensitivity
emf_processor.set_signal_gain(1.5)  # Increase signal gain

# Process the EMF data with updated settings
raw_emf_data = [[100, 200, -100], [110, 210, -110], ...]  # Example raw EMF data
processed_data = emf_processor.prepare_data(raw_emf_data)

trend, anomalies = emf_processor.get_trend_and_anomalies(processed_data)
print("Trend:", trend)
print("Anomalies:", anomalies)


"""

import math

class EMFProcessor:
    def __init__(self, max_expected_magnitude=1000, base_window_size=10, threshold=2):
        self.max_expected_magnitude = max_expected_magnitude
        self.base_window_size = base_window_size
        self.threshold = threshold
        self.sensitivity = 1.0
        self.signal_gain = 1.0
        self.calibration_params = {
            'offset_x': 0, 'scale_x': 1,
            'offset_y': 0, 'scale_y': 1,
            'offset_z': 0, 'scale_z': 1
        }

    def set_sensitivity(self, sensitivity):
        self.sensitivity = sensitivity

    def set_signal_gain(self, gain):
        self.signal_gain = gain

    def normalize_data(self, raw_data):
        normalized = []
        for sample in raw_data:
            calibrated_sample = [
                (sample[i] - self.calibration_params[f'offset_{axis}']) * self.calibration_params[f'scale_{axis}']
                for i, axis in enumerate(['x', 'y', 'z'])
            ]
            magnitude = math.sqrt(sum(x**2 for x in calibrated_sample)) / self.max_expected_magnitude
            normalized.append(min(max(magnitude * self.sensitivity, 0), 1))
        return normalized

    def moving_average_filter(self, data):
        return [sum(data[i:i+self.base_window_size]) / self.base_window_size for i in range(len(data)-self.base_window_size+1)]

    def exponential_moving_average(self, data, alpha=0.3):
        ema = [data[0]]
        for i in range(1, len(data)):
            ema.append(alpha * data[i] + (1 - alpha) * ema[-1])
        return [val * self.signal_gain for val in ema]

    def prepare_data(self, raw_data):
        """Prepares data by normalizing, filtering, and applying EMA."""
        if not raw_data:
            return []
        normalized_data = self.normalize_data(raw_data)
        filtered_data = self.moving_average_filter(normalized_data)
        ema_data = self.exponential_moving_average(filtered_data)
        return ema_data

    def calculate_trend(self, data):
        if len(data) < 2:
            return 0
        x = range(len(data))
        x_mean = sum(x) / len(x)
        y_mean = sum(data) / len(data)
        numerator = sum((xi - x_mean) * (yi - y_mean) for xi, yi in zip(x, data))
        denominator = sum((xi - x_mean) ** 2 for xi in x)
        if denominator == 0:
            return 0
        return numerator / denominator

    def detect_anomalies(self, data):
        if len(data) < 2:
            return []
        mean = sum(data) / len(data)
        variance = sum((xi - mean) ** 2 for xi in data) / len(data)
        stddev = math.sqrt(variance)
        return [i for i, x in enumerate(data) if abs(x - mean) > self.threshold * stddev]

    def process_emf_data(self, raw_data):
        """Main method to process EMF data and optionally calculate trend or detect anomalies."""
        processed_data = self.prepare_data(raw_data)
        return processed_data

    def get_trend_and_anomalies(self, processed_data):
        """Calculates the trend and detects anomalies on processed data."""
        trend = self.calculate_trend(processed_data) if processed_data else 0
        anomalies = self.detect_anomalies(processed_data) if processed_data else []
        return trend, anomalies
