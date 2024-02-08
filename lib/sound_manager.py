"""

# Usage examples:

# Initialize SoundManager
sound_manager = SoundManager()

# Adjust volume
sound_manager.set_volume(50)  # Set volume to 50%

# Play a single track
sound_manager.play_track(2)  # Play track number 2

# Stop playback
sound_manager.stop_playback()

# Loop a single track
sound_manager.loop_track(3)  # Continuously loop track number 3

# Control sound based on EMF reading
EMF_MAX_ADC = 1100  # Example maximum value for your ADC reading
track_settings = {'low': 1, 'medium': 3, 'high': 5}  # Mapping of EMF reading ranges to track numbers
emf_reading = 450  # Example value, replace with actual EMF reading from your sensor
sound_manager.sound_control(emf_reading, EMF_MAX_ADC, track_settings)

"""

import board
import busio
from dfplayer import DFPlayer

class SoundManager:
    def __init__(self, player_tx=board.GP0, player_rx=board.GP1, player_baud=9600, player_vol=80):
        self.uart = busio.UART(tx=player_tx, rx=player_rx, baudrate=player_baud)
        self.dfplayer = DFPlayer(uart=self.uart)
        self.dfplayer.set_volume(player_vol)
        self.last_played_track = None

    def set_volume(self, volume):
        """Set the volume of the DFPlayer."""
        self.dfplayer.set_volume(volume)

    def play_track(self, track_number):
        """Play a specific track."""
        self.dfplayer.play(track=track_number)
        self.last_played_track = track_number

    def stop_playback(self):
        """Stop any current playback."""
        self.dfplayer.stop()

    def loop_track(self, track_number):
        """Loop a specific track."""
        self.play_track(track_number)
        self.dfplayer.loop()

    def sound_control(self, emf_reading, emf_max_adc, track_settings):
        track_to_play = None

        if emf_reading < (emf_max_adc / 3):
            track_to_play = track_settings['low']  # Track number for low EMF reading
        elif emf_reading < (2 * emf_max_adc / 3):
            track_to_play = track_settings['medium']  # Track number for medium EMF reading
        else:
            track_to_play = track_settings['high']  # Track number for high EMF reading

        # Only play if not the last played track
        if track_to_play != self.last_played_track:
            self.play_track(track_to_play)
