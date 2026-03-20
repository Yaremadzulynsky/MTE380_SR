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
              odometry: Optional['Odometry']) -> None:
        """Called once when the machine transitions into this state."""

    def tick(self, robot: 'Robot', detector: Optional['LineDetector'],
             odometry: Optional['Odometry']) -> Optional[str]:
        """
        Called every tick by the state machine.

        Returns
        -------
        str
            Name of the state to transition to.
        None
            Stay in the current state.
        """
        return None

    def exit(self, robot: 'Robot', detector: Optional['LineDetector'],
             odometry: Optional['Odometry']) -> None:
        """Called once when the machine transitions out of this state."""
