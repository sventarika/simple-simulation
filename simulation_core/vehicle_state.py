from dataclasses import asdict, dataclass, field


@dataclass
class VehicleState:
    x: float = field(default=None)
    y: float = field(default=None)
    delta: float = field(default=None)
    v: float = field(default=None)
    theta: float = field(default=None)
    theta_rate: float = field(default=None)
    slip_angle: float = field(default=None)

    length: float = field(default=None)
    width: float = field(default=None)

    def asdict(self, filter_non_available: bool = False) -> dict:
        d = asdict(self)
        if filter_non_available:
            d_filtered = {k: v for k, v in d.items() if v is not None}
        else:
            d_filtered = d

        return d_filtered

    @property
    def keys(self) -> list:
        return list(self.asdict().keys())

    @property
    def values(self) -> list:
        return list(self.asdict().values())
