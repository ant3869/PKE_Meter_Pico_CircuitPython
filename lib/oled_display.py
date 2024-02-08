"""

# Initialize the display manager
display_manager = DisplayManager()

# To draw a graph with a new value and baseline
new_value = 100  # Example EMF value
baseline = 50  # Example baseline
display_manager.draw_graph(new_value, baseline)

"""

import board
import busio
from adafruit_ssd1306 import SSD1306_I2C

class DisplayManager:
    def __init__(self, width=128, height=64, sda=board.GP4, scl=board.GP5):
        self.i2c = busio.I2C(scl, sda)
        self.oled = SSD1306_I2C(width, height, self.i2c)
        self.width = width
        self.height = height
        self.graph_baseline_y = height - 6
        self.max_graph_value = height - 62
        self.scale_factor = 0.5
        self.bar_width = 100
        self.bar_height = 5
        self.bar_x = 10
        self.bar_y = 30
        self.y_history = [self.height // 6] * self.width
    
    def draw_line(self, x0, y0, x1, y1, color):
        x0, y0, x1, y1 = map(int, (x0, y0, x1, y1))
        dx = abs(x1 - x0)
        sx = 1 if x0 < x1 else -1
        dy = -abs(y1 - y0)
        sy = 1 if y0 < y1 else -1
        err = dx + dy
        while True:
            self.oled.pixel(x0, y0, color)
            if x0 == x1 and y0 == y1:
                break
            e2 = 2 * err
            if e2 >= dy:
                err += dy
                x0 += sx
            if e2 <= dx:
                err += dx
                y0 += sy
    
    def draw_graph(self, value, baseline):
        self.oled.fill(0)
        self.oled.text('EMF: {:.2f}'.format(value), 0, 0, 1)
        self.oled.text('Base: {:.2f}'.format(baseline), 0, 10, 1)
    
        if self.y_history and len(self.y_history) > 1:
            for x in range(1, len(self.y_history)):
                y0 = self.graph_baseline_y - (self.y_history[x - 1] * self.scale_factor)
                y1 = self.graph_baseline_y - (self.y_history[x] * self.scale_factor)            
                y0 = max(0, min(self.height, y0))
                y1 = max(0, min(self.height, y1))
                self.draw_line(x - 1, y0, x, y1, 1)
    
        self.oled.show()
