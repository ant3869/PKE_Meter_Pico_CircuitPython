"""

This method follows the steps outlined in the original Madgwick paper 
but adapts them for use without NumPy. It's important to ensure that 
sensor data (ax, ay, az, gx, gy, gz, mx, my, mz) is pre-processed to 
match the expected input format and scale. This might involve converting 
raw sensor values to the units expected by the algorithm (e.g., meters 
per second squared for acceleration, radians per second for gyroscopic rates, 
and normalized values for the magnetometer).

Fine-tuning parameters such as beta and validating the algorithm's performance 
with actual LSM9DS1 data will be crucial to ensure accurate orientation estimation. 
Testing and iterative adjustments may be needed based on the specific application 
requirements and the operating environment's characteristics.

"""

import math

class Madgwick:
    def __init__(self, beta=0.1):
        GyroMeasError = math.pi * (40.0 / 180.0)
        self.beta = math.sqrt(3.0 / 4.0) * GyroMeasError
        self.q = [1.0, 0.0, 0.0, 0.0]  # Quaternion representing the initial orientation
        self._roll = 0
        self._pitch = 0
        self._yaw = 0

    def computeOrientation(self):
        q = self.q
        # Conversion of quaternion to Euler angles (roll, pitch, and yaw)
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (q[0] * q[1] + q[2] * q[3])
        cosr_cosp = 1 - 2 * (q[1] ** 2 + q[2] ** 2)
        self._roll = math.atan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2 * (q[0] * q[2] - q[3] * q[1])
        if abs(sinp) >= 1:
            self._pitch = math.copysign(math.pi / 2, sinp)
        else:
            self._pitch = math.asin(sinp)

        # Yaw (z-axis rotation)
        siny_cosp = 2 * (q[0] * q[3] + q[1] * q[2])
        cosy_cosp = 1 - 2 * (q[2] ** 2 + q[3] ** 2)
        self._yaw = math.atan2(siny_cosp, cosy_cosp)

        # Convert radians to degrees
        self._roll = math.degrees(self._roll)
        self._pitch = math.degrees(self._pitch)
        self._yaw = math.degrees(self._yaw)

    def quaternionMul(self, q1, q2):
        # Quaternion multiplication without NumPy
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2
        w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
        x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
        y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
        z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
        return [w, x, y, z]

    def normalizeq(self, q):
        # Quaternion normalization
        norm = math.sqrt(sum(x ** 2 for x in q))
        return [x / norm for x in q]

    # Update this method to handle magnetometer readings from LSM9DS1
    def update(self, ax, ay, az, gx, gy, gz, mx, my, mz, dt):
        # Convert gyroscope degrees/sec to radians/sec
        gx, gy, gz = map(math.radians, [gx, gy, gz])
    
        # Normalize accelerometer measurement
        norm = math.sqrt(ax * ax + ay * ay + az * az)
        if norm == 0: return  # avoid division by zero
        ax, ay, az = ax / norm, ay / norm, az / norm
    
        # Normalize magnetometer measurement
        norm = math.sqrt(mx * mx + my * my + mz * mz)
        if norm == 0: return  # avoid division by zero
        mx, my, mz = mx / norm, my / norm, mz / norm
    
        # Auxiliary variables to avoid repeated calculations
        q1, q2, q3, q4 = self.q
        _2q1mx = 2 * q1 * mx
        _2q1my = 2 * q1 * my
        _2q1mz = 2 * q1 * mz
        _2q2mx = 2 * q2 * mx
        _2q1 = 2 * q1
        _2q2 = 2 * q2
        _2q3 = 2 * q3
        _2q4 = 2 * q4
        _2q1q3 = 2 * q1 * q3
        _2q3q4 = 2 * q3 * q4
        q1q1 = q1 * q1
        q1q2 = q1 * q2
        q1q3 = q1 * q3
        q1q4 = q1 * q4
        q2q2 = q2 * q2
        q2q3 = q2 * q3
        q2q4 = q2 * q4
        q3q3 = q3 * q3
        q3q4 = q3 * q4
        q4q4 = q4 * q4
    
        # Reference direction of Earth's magnetic field
        hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4
        hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4
        _2bx = math.sqrt(hx * hx + hy * hy)
        _2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4
        _4bx = 2 * _2bx
        _4bz = 2 * _2bz
    
        # Gradient descent algorithm corrective step
        s1 = -_2q3 * (2 * q2q4 - _2q1q3 - ax) + _2q2 * (2 * q1q1 + 2 * q3q3 - az) - _2bz * q3 * (_2bx * (0.5 - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5 - q2q2 - q3q3) - mz)
        s2 = _2q4 * (2 * q2q4 - _2q1q3 - ax) + _2q1 * (2 * q1q1 + 2 * q3q3 - az) - 4 * q2 * (1 - 2 * q2q2 - 2 * q3q3 - az) + _2bz * q4 * (_2bx * (0.5 - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5 - q2q2 - q3q3) - mz)
        s3 = -_2q1 * (2 * q2q4 - _2q1q3 - ax) + _2q4 * (2 * q1q1 + 2 * q3q3 - az) - 4 * q3 * (1 - 2 * q2q2 - 2 * q3q3 - az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5 - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5 - q2q2 - q3q3) - mz)
        s4 = _2q2 * (2 * q2q4 - _2q1q3 - ax) + _2q3 * (2 * q1q1 + 2 * q3q3 - az) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5 - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5 - q2q2 - q3q3) - mz)
        norm = math.sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4)  # normalize step magnitude
        if norm == 0: return  # avoid division by zero
        s1 /= norm
        s2 /= norm
        s3 /= norm
        s4 /= norm
    
        # Apply feedback step
        qDot1 = 0.5 * (-q2 * gx - q3 * gy - q4 * gz) - self.beta * s1
        qDot2 = 0.5 * (q1 * gx + q3 * gz - q4 * gy) - self.beta * s2
        qDot3 = 0.5 * (q1 * gy - q2 * gz + q4 * gx) - self.beta * s3
        qDot4 = 0.5 * (q1 * gz + q2 * gy - q3 * gx) - self.beta * s4
    
        # Integrate to get new quaternion
        q1 += qDot1 * dt
        q2 += qDot2 * dt
        q3 += qDot3 * dt
        q4 += qDot4 * dt
        norm = math.sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4)  # normalize quaternion
        self.q = [q1 / norm, q2 / norm, q3 / norm, q4 / norm]
    
        self.computeOrientation()  # Update Euler angles

    # Properties for roll, pitch, and yaw to ensure they can be read but not arbitrarily set
    @property
    def roll(self):
        return self._roll

    @property
    def pitch(self):
        return self._pitch

    @property
    def yaw(self):
        return self._yaw

    # The beta property ensures that the beta value is within an acceptable range
    @property
    def beta(self):
        return self._beta

    @beta.setter
    def beta(self, value):
        if 0 <= value <= 1:
            self._beta = value
        else:
            raise ValueError("Beta value must be between 0 and 1.")