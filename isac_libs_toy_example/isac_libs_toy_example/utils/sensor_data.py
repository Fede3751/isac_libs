import time
from datetime import datetime

class SensorData:
    def __init__(self,  strength : float, in_range : bool = False, misuration_ts : float = None, req_cache = None) -> None:
        """
        Initialize SensorData object with temperature, blood pressure, and heartbeat data.
        
        Args:
        in_range: If an agent is in range
        misuration_ts (float, optional): Timestamp of measurement, in Unix time. Defaults to current time.
        """

        self.misuration_ts = misuration_ts if misuration_ts else time.time()
        self.in_range = in_range
        self.history = []  # To log all historical measurements
        self.strength = strength
        self.req_cache = req_cache

        # Log initial data
        self._log_data()

    def _log_data(self):
        """Private method to log sensor data."""
        self.history.append({
            'in_range': self.in_range
        })

    def get_timestamp(self) -> str:
        """Return the formatted timestamp."""
        return datetime.fromtimestamp(self.misuration_ts).strftime('%Y-%m-%d %H:%M:%S')


    def log_new_reading(self, in_range: float, misuration_ts: float = None) -> None:
        """
        Logs a new sensor reading.
        """
        # Update sensor readings
        self.in_range = in_range
        self.misuration_ts = misuration_ts if misuration_ts else time.time()

        #self._validate_data()

        self._log_data()

    def __repr__(self) -> str:
        """Provide a string representation of the sensor data."""
        return(f"SensorData(Position: {self.position}, In Range: {self.in_range})")

    def get_history(self):
        """Return the history of sensor readings."""
        return [
            {
                'in_range': entry['in_range']
            }
            for entry in self.history
        ]
    
    def to_dict(self):
        """Convert the SensorData object to a dictionary for serialization."""
        return {
            'in_range': self.in_range,
            'timestamp': self.misuration_ts,
            'position': self.position
        }
    

