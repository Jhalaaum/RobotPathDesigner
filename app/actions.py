"""Qt-free mission actions. Distances in mm, angles clockwise in degrees.
Motor speeds are motor degrees/sec; these are not chassis speeds.
"""
from dataclasses import dataclass, asdict
import ast
import json
import math
from typing import ClassVar, Optional, Tuple, Union


def number(value):
    if type(value) not in (int, float) or not math.isfinite(value):
        raise ValueError('Expected a finite number')
    return value


def side_check(side):
    if side not in ('left', 'right'):
        raise ValueError('Side must be left or right')


@dataclass(frozen=True)
class Drive:
    distance: float
    method: str = 'gyro'
    kind: ClassVar[str] = 'drive'
    def __post_init__(self):
        number(self.distance)
        if self.method not in ('gyro', 'motor'):
            raise ValueError('Unknown drive method')


@dataclass(frozen=True)
class Turn:
    angle: float
    kind: ClassVar[str] = 'turn'
    def __post_init__(self):
        number(self.angle)


@dataclass(frozen=True)
class LineTrack:
    distance: float
    side: str = 'right'
    kind: ClassVar[str] = 'line_track'
    def __post_init__(self):
        number(self.distance)
        side_check(self.side)


@dataclass(frozen=True)
class TrackToIntersection(LineTrack):
    """Planner-only: known nominal chassis travel to a REAL track junction.

    Emits a sensor-controlled stop in Pybricks; distance is only for desktop
    prediction/collision checking, not a robot stop distance. Do not use it if
    there is an unmodelled earlier intersection along the tracking section.
    """
    kind: ClassVar[str] = 'track_to_intersection'


@dataclass(frozen=True)
class TrackUntilIntersection:
    side: str = 'right'
    count: int = 1
    kind: ClassVar[str] = 'intersection'
    def __post_init__(self):
        side_check(self.side)
        if type(self.count) is not int or self.count < 1:
            raise ValueError('Intersection count must be a positive integer')


@dataclass(frozen=True)
class MotorAction:
    port: str
    angle: Optional[float] = None
    speed: float = 500
    wait: bool = True
    mode: str = 'relative'
    duration_ms: Optional[int] = None
    kind: ClassVar[str] = 'motor'
    def __post_init__(self):
        if self.port not in ('A','B','C','D','E','F'):
            raise ValueError('Invalid motor port')
        number(self.speed)
        if self.speed == 0 or type(self.wait) is not bool:
            raise ValueError('Invalid motor speed or wait flag')
        if self.mode in ('relative','absolute'):
            number(self.angle)
            if self.duration_ms is not None:
                raise ValueError('Angle mode cannot take duration')
        elif self.mode == 'timed':
            if type(self.duration_ms) is not int or self.duration_ms < 0 or self.angle is not None:
                raise ValueError('Timed motor action requires nonnegative integer milliseconds, no angle')
        elif self.mode == 'stall':
            if self.angle is not None or self.duration_ms is not None or not self.wait:
                raise ValueError('Stall mode requires blocking operation without angle or duration')
        else:
            raise ValueError('Unknown motor mode')


@dataclass(frozen=True)
class Wait:
    duration_ms: int
    kind: ClassVar[str] = 'wait'
    def __post_init__(self):
        if type(self.duration_ms) is not int or self.duration_ms < 0:
            raise ValueError('Wait requires nonnegative integer milliseconds')


Action = Union[Drive, Turn, LineTrack, TrackToIntersection, TrackUntilIntersection, MotorAction, Wait]
_TYPES = {cls.kind: cls for cls in (Drive, Turn, LineTrack, TrackToIntersection, TrackUntilIntersection, MotorAction, Wait)}


@dataclass(frozen=True)
class Program:
    actions: Tuple[Action, ...] = ()
    def __post_init__(self):
        object.__setattr__(self, 'actions', tuple(self.actions))
        if any(type(a) not in _TYPES.values() for a in self.actions):
            raise ValueError('Unsupported action')
    def to_json(self):
        return json.dumps({'version': 1, 'actions': [dict(kind=a.kind, **asdict(a)) for a in self.actions]}, indent=2, allow_nan=False)
    @classmethod
    def from_json(cls, source):
        data = json.loads(source)
        if not isinstance(data, dict) or data.get('version') != 1 or not isinstance(data.get('actions'), list):
            raise ValueError('Unsupported action program format')
        result = []
        for item in data['actions']:
            if not isinstance(item, dict) or item.get('kind') not in _TYPES:
                raise ValueError('Unknown action kind')
            try:
                result.append(_TYPES[item['kind']](**{k:v for k,v in item.items() if k != 'kind'}))
            except TypeError as exc:
                raise ValueError(f'Invalid action fields: {exc}') from exc
        return cls(tuple(result))


def fmt(value):
    """Compact output for editable competition code (not a measurement claim)."""
    value = round(float(value), 2)
    return str(int(value)) if value.is_integer() else f'{value:.2f}'.rstrip('0').rstrip('.')


def to_pybricks(program, motors=None):
    """Minimal helper calls. Motor aliases must refer to existing motors."""
    motors = motors or {}
    lines = []
    for a in program.actions:
        if isinstance(a, Drive):
            lines.append(f'{a.method}DriveStraight({fmt(a.distance)})')
        elif isinstance(a, Turn):
            lines.append(f'turn({fmt(a.angle)})')
        elif isinstance(a, TrackToIntersection):
            side = 'side="left"' if a.side == 'left' else ''
            lines.append(f'singleLineTrackTillIntersection({side})')
        elif isinstance(a, LineTrack):
            side = ', side="left"'  if a.side == 'left' else ''
            lines.append(f'singleLineTrackByDist({fmt(a.distance)}{side})')
        elif isinstance(a, TrackUntilIntersection):
            if a.count != 1:
                raise ValueError('Existing Pybricks helper supports only one intersection')
            side = 'side="left"' if a.side == 'left' else ''
            lines.append(f'singleLineTrackTillIntersection({side})')
        elif isinstance(a, Wait):
            lines.append(f'wait({a.duration_ms})')
        elif isinstance(a, MotorAction):
            alias = motors.get(a.port)
            if not isinstance(alias, str) or not alias.isidentifier() or __import__('keyword').iskeyword(alias):
                raise ValueError(f'Missing valid motor alias for port {a.port}')
            method = {'relative':'run_angle','absolute':'run_target','timed':'run_time','stall':'run_until_stalled'}[a.mode]
            args = [fmt(a.speed)]
            if a.mode != 'stall':
                args.append(fmt(a.duration_ms if a.mode == 'timed' else a.angle))
                if not a.wait:
                    args.append('wait=False')
            lines.append(f'{alias}.{method}({", ".join(args)})')
    return '\n'.join(lines) + ('\n' if lines else '')


def from_pybricks(source, motors=None):
    """Strict AST parser for the minimal action dialect; never executes code."""
    aliases = {v:k for k,v in (motors or {}).items()}
    result = []
    for stmt in ast.parse(source).body:
        try:
            if isinstance(stmt, ast.Pass):
                continue
            if not isinstance(stmt, ast.Expr) or not isinstance(stmt.value, ast.Call):
                raise ValueError('Expected a standalone action call')
            call = stmt.value
            args = [ast.literal_eval(a) for a in call.args]
            kwargs = {}
            for k in call.keywords:
                if k.arg is None or k.arg in kwargs:
                    raise ValueError('Unsupported keyword expansion or duplicate argument')
                kwargs[k.arg] = ast.literal_eval(k.value)
            if isinstance(call.func, ast.Name):
                name = call.func.id
                if name in ('gyroDriveStraight','motorDriveStraight'):
                    if kwargs or len(args) != 1: raise ValueError('Expected one distance')
                    action = Drive(args[0], 'gyro' if name.startswith('gyro') else 'motor')
                elif name == 'turn': action = Turn(*args, **kwargs)
                elif name == 'singleLineTrackByDist': action = LineTrack(*args, **kwargs)
                elif name == 'singleLineTrackTillIntersection':
                    action = TrackUntilIntersection(*args, **kwargs)
                    if action.count != 1: raise ValueError('Existing helper supports one intersection')
                elif name == 'wait': action = Wait(*args, **kwargs)
                else: raise ValueError(f'Unsupported command {name}')
            elif isinstance(call.func, ast.Attribute) and isinstance(call.func.value, ast.Name) and call.func.value.id in aliases:
                method = call.func.attr
                mode = {'run_angle':'relative','run_target':'absolute','run_time':'timed','run_until_stalled':'stall'}.get(method)
                if mode is None or len(args) != (1 if mode == 'stall' else 2) or set(kwargs)-{'wait'} or (mode == 'stall' and kwargs):
                    raise ValueError('Unsupported motor call')
                action = MotorAction(aliases[call.func.value.id], speed=args[0], mode=mode, angle=args[1] if mode in ('relative','absolute') else None, duration_ms=args[1] if mode == 'timed' else None, **kwargs)
            else: raise ValueError('Unsupported call target')
            result.append(action)
        except (ValueError, TypeError, SyntaxError) as exc:
            raise ValueError(f'Line {stmt.lineno}: {exc}') from exc
    return Program(tuple(result))


def to_movements(program, intersection_distance_mm=200.0, auto_intersections=False):
    """Adapter for the current Qt canvas. Reject non-navigation actions rather than lose them."""
    movements, warnings = [], []
    for i, a in enumerate(program.actions, 1):
        if isinstance(a, Turn):
            movements.append({'type':'turn','angle':float(a.angle)})
        elif isinstance(a, TrackToIntersection):
            # Preserve known *nominal* planning geometry when applying to GUI.
            # Actual sensor stop is variable; do not force auto re-detection.
            movements.append(dict(type='line', distance=float(a.distance),
                                  stop='Intersection', side=a.side,
                                  estimated=True, source_line=i,
                                  auto_detect=False))
            warnings.append(f'Action {i}: junction stop uses nominal {a.distance:.1f} mm for preview only.')
        elif isinstance(a, TrackUntilIntersection):
            if a.count != 1: raise ValueError('Canvas adapter currently supports one intersection per action')
            if number(intersection_distance_mm) <= 0: raise ValueError('Positive intersection estimate required')
            movements.append(dict(type='line', distance=float(intersection_distance_mm), stop='Intersection', side=a.side, estimated=True, source_line=i, auto_detect=bool(auto_intersections)))
            if not auto_intersections: warnings.append(f'Action {i}: intersection distance estimated at {intersection_distance_mm:g} mm.')
        elif isinstance(a, (Drive, LineTrack)):
            if abs(a.distance) < .001:
                warnings.append(f'Action {i}: zero-distance movement omitted.')
                continue
            movements.append(dict(type='line' if isinstance(a, LineTrack) else 'straight',distance=float(a.distance),stop='Distance',side=getattr(a,'side','right'),estimated=False))
        else: raise ValueError(f'Action {i}: {a.kind} cannot yet be represented by the waypoint canvas')
    if not any(m['type'] != 'turn' for m in movements): raise ValueError('At least one drive or line-track command is needed to draw a route.')
    return movements, warnings
