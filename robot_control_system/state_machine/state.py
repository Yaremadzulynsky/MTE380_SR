"""
state_machine/state.py

Base class for all robot states.
"""

from __future__ import annotations
from typing import TYPE_CHECKING, Optional

if TYPE_CHECKING:
    from state_machine.hardware.robot       import Robot
    from state_machine.vision.line_detector import LineDetector
    from state_machine.odometry             import Odometry


class State:
    """
    Base class for a robot state.

    Subclasses override enter / tick / exit.  tick() returns the name of the
    next state to transition to, or None to remain in the current state.
    """

    #: Unique name used to register and look up this state in the machine.
    name: str = ''

    def enter(self, robot: 'Robot', detector: Optional['LineDetector'],
              odometry: Optional['Odometry'],
              target_heading: Optional[float] = None) -> None:
        """Called once when the machine transitions into this state."""

    def tick(self, robot: 'Robot', detector: Optional['LineDetector'],
             odometry: Optional['Odometry'],
             target_heading: Optional[float] = None) -> Optional[str]:
        """
        Called every tick by the state machine.

        Parameters
        ----------
        target_heading : float | None
            World-frame heading the robot should drive toward (radians),
            computed by the state machine from line angle + lateral correction.
            None when no line is detected.

        Returns
        -------
        str
            Name of the state to transition to.
        None
            Stay in the current state.
        """
        return None

    def exit(self, robot: 'Robot', detector: Optional['LineDetector'],
             odometry: Optional['Odometry'],
             target_heading: Optional[float] = None) -> None:
        """Called once when the machine transitions out of this state."""
