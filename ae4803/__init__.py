"""AE4803 flight simulator - standalone Python port of AE4803Sim.slx.

The Simulink model remains the authority for the lab; this package reproduces
its equations block-for-block so results are comparable, while adding a proper
fixed-step real-time loop and direct (TARGET-free) joystick input.
"""

__version__ = "1.0.0"

from .params import Params                                   # noqa: F401
from .model import (FlightModel, AutopilotConfig,             # noqa: F401
                    AutoStopConfig, Outputs)
from .controls import PilotCommands                           # noqa: F401
