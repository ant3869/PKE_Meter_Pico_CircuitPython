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
        if not isinstance(sensitivity, (int, float)) or sensitivity <= 0:
            raise ValueError("Sensitivity must be a positive number.")
        self.sensitivity = sensitivity

    def set_signal_gain(self, gain):
        if not isinstance(gain, (int, float)) or gain <= 0:
            raise ValueError("Gain must be a positive number.")
        self.signal_gain = gain

    def normalize_data(self, raw_data):
        if not all(isinstance(sample, list) and len(sample) == 3 for sample in raw_data):
            raise ValueError("Each raw data sample must be a list of three numeric values.")
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
        if len(data) < self.base_window_size:
            return data  # Return the original data if it's shorter than the window size
        return [sum(data[i:i+self.base_window_size]) / self.base_window_size for i in range(len(data)-self.base_window_size+1)]

    def exponential_moving_average(self, data, alpha=0.3):
        if not 0 < alpha < 1:
            raise ValueError("Alpha must be between 0 and 1.")
        if not data:
            return []
        ema = [data[0]]  # Initialize EMA with the first data point
        for i in range(1, len(data)):
            ema.append(alpha * data[i] + (1 - alpha) * ema[-1])
        return [val * self.signal_gain for val in ema]

    def prepare_data(self, raw_data):
        if not isinstance(raw_data, list) or not raw_data:
            raise ValueError("Raw data must be a non-empty list.")
        normalized_data = self.normalize_data(raw_data)
        filtered_data = self.moving_average_filter(normalized_data)
        ema_data = self.exponential_moving_average(filtered_data)
        return ema_data

    def calculate_trend(self, data):
        if len(data) < 2:
            return 0  # Not enough data to calculate a trend
        x = range(len(data))
        x_mean = sum(x) / len(x)
        y_mean = sum(data) / len(data)
        numerator = sum((xi - x_mean) * (yi - y_mean) for xi, yi in zip(x, data))
        denominator = sum((xi - x_mean) ** 2 for xi in x)
        return numerator / denominator if denominator != 0 else 0

    def detect_anomalies(self, data):
        if len(data) < 2:
            return []  # Not enough data to detect anomalies
        mean = sum(data) / len(data)
        variance = sum((xi - mean) ** 2 for xi in data) / len(data)
        stddev = math.sqrt(variance)
        return [i for i, x in enumerate(data) if abs(x - mean) > self.threshold * stddev]

    def get_trend_and_anomalies(self, processed_data):
        if not processed_data:
            return 0, []  # Return defaults if there's no processed data
        trend = self.calculate_trend(processed_data)
        anomalies = self.detect_anomalies(processed_data)
        return trend, anomalies
    
    def get_composite_emf_score(self, processed_data):
        """Calculates a composite EMF score from processed data."""
        if not processed_data:
            return 0  # Return the lowest score if there's no data
        
        # Calculate the average magnitude of the processed data
        avg_magnitude = sum(processed_data) / len(processed_data)
        
        # Calculate trend and anomalies
        trend, anomalies = self.get_trend_and_anomalies(processed_data)
        trend_score = (trend / max(abs(trend), 1)) * 0.5 + 0.5  # Normalize trend to 0-1 range
        
        # Calculate anomaly score (assuming more anomalies lower the score due to noise)
        anomaly_score = 1 - (len(anomalies) / len(processed_data)) if processed_data else 1
        
        # Composite score combines average magnitude with trend and anomaly impacts
        # This is a basic formula; adjust based on empirical data or desired weighting
        composite_score = avg_magnitude * trend_score * anomaly_score
        
        # Normalize and map to 0-1100 range, then clamp
        normalized_score = min(max(self.map_to_range(composite_score), 0), 1100)
        return normalized_score
    
    def map_to_range(self, score, min_score=0, max_score=1, target_min=0, target_max=1100):
        """Maps a score from its original range to the target range (0-1100) and clamps the value."""
        # Assuming score is already normalized to min_score-max_score range; adjust if not
        return ((score - min_score) / (max_score - min_score)) * (target_max - target_min) + target_min
