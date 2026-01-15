from typing import Callable, List, Dict, Any

from rosgraph_msgs.msg import Clock


DEFAULT_SCHEDULER_RATE = 10

class EventScheduler():
    
    """
    Class used to schedule timed events following a ROS2 Clock interface.

        rate:
            specify the rate at which events are checked, set by default to 10 checks
            per simulation seconds
    """

    def __init__(self, rate=DEFAULT_SCHEDULER_RATE):
        self.current_time = 0
        self.rate = rate
        self.scheduled_events = []

    def routine(self, clock: Clock):
        """
        Main method to update the scheduler. This method should be called every time
        the simulation clock updates.

            clock:
                the new time in the simulation
        """
        new_time = clock.clock.sec + clock.clock.nanosec * 10**(-9)

        # Before running through events, check for rate. This allows
        # to run through all the events at a desired frequency, without
        # wasting resources
        if new_time - self.current_time > 1/self.rate:
            self.current_time = new_time
        else:
            return

        # Loop through all the scheduled events
        for event in self.scheduled_events:
            # If the event time has passed, execute it
            if event['time'] + event['scheduled_time'] < self.current_time:
                event['callback'](*event['args'])

                if event['repeat']:
                    event['scheduled_time'] = self.current_time
                else:
                    self.scheduled_events.remove(event)

    # Schedule a new event in the scheduler.
    def schedule_event(self, time: float, callback: Callable, repeat=True, args: list = []):
        """
        Schedule a new event in the EvenScheduler.

            time:
                timeout to the event in seconds
            callback:
                the function to execute when the time expires
            repeat:
                if True, the event is repeated at the frequency given by time
            args:
                argument to pass to the callback function
        """

        new_event = {
            'time': time,
            'callback': callback,
            'repeat': repeat,
            'args': args,
            'scheduled_time': self.current_time,
        }

        self.scheduled_events.append(new_event)

