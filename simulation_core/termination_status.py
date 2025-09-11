from dataclasses import dataclass, field, asdict


@dataclass
class TerminationStatus:
    is_offroad: bool = field(default=False)
    is_collision: bool = field(default=False)
    is_time_out: bool = field(default=False)
    is_goal_reached: bool = field(default=False)
    is_standstill: bool = field(default=False)

    collision_obj_id: int = field(default=None)

    def asdict(self) -> dict:
        return asdict(self)

    def is_terminated(self) -> bool:
        termination_factors = (self.is_offroad, self.is_collision, self.is_time_out, self.is_goal_reached, self.is_standstill)
        return any(termination_factors)
