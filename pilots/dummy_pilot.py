from __future__ import annotations

from .pilot import Pilot


class DummyPilot(Pilot):
    """
    Pilot for TrajectoryVehicleModel.
    """

    def step(self, *args, **kwargs) -> tuple[float, float]:  # noqa: ARG002
        """
        Return value has no effect in the simulation core.
        """
        return (0.0, 0.0)
