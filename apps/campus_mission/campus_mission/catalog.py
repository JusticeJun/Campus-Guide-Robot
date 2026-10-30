"""Map-bound places with atomic updates and map/world coordinate conversion."""
import hashlib
import io
import json
import math
import os
import threading
from pathlib import Path

import numpy as np
from PIL import Image
from pydantic import BaseModel, ConfigDict, Field, field_validator
import yaml


class Pose(BaseModel):
    model_config = ConfigDict(extra='forbid', allow_inf_nan=False)
    x: float
    y: float
    yaw: float


class Place(BaseModel):
    model_config = ConfigDict(extra='forbid')
    id: str = Field(pattern=r'^[a-z][a-z0-9_]{0,63}$')
    name: str = Field(min_length=1, max_length=80)
    aliases: list[str] = Field(default_factory=list, max_length=20)
    category: str = Field(default='landmark', max_length=40)
    observation_pose: Pose
    note: str = Field(default='', max_length=500)
    source: str = 'operator'
    approximate: bool = True
    tasks: list[str] = Field(default_factory=lambda: ['navigate', 'photograph', 'inspect'])

    @field_validator('tasks')
    @classmethod
    def tasks_valid(cls, value):
        if not value or not set(value) <= {'navigate', 'photograph', 'inspect'}:
            raise ValueError('지원하지 않는 임무 종류입니다.')
        return value


class Catalog:
    def __init__(self, map_yaml, places_file):
        self.map_yaml = Path(map_yaml).expanduser().resolve()
        self.path = Path(places_file).expanduser().resolve()
        self.lock = threading.RLock()
        meta = yaml.safe_load(self.map_yaml.read_text())
        self.meta = meta
        self.image = Image.open(self.map_yaml.parent / meta['image']).convert('L')
        self.width, self.height = self.image.size
        self.resolution = float(meta['resolution'])
        self.origin = [float(v) for v in meta['origin']]
        if self.resolution <= 0 or not all(math.isfinite(v) for v in self.origin):
            raise ValueError('지도 metadata가 올바르지 않습니다.')
        # Hash content + interpretation, never a multi-GB RTAB database or path.
        header = {k: meta.get(k, default) for k, default in (
            ('resolution', None), ('origin', None), ('negate', 0),
            ('occupied_thresh', .65), ('free_thresh', .196), ('mode', 'trinary'))}
        self.fingerprint = hashlib.sha256(json.dumps(header, sort_keys=True).encode()
                                          + self.image.tobytes()).hexdigest()
        a = np.array(self.image)
        prob = a / 255. if meta.get('negate', 0) else (255 - a) / 255.
        self.occupancy = np.where(prob > meta['occupied_thresh'], 100,
                                 np.where(prob < meta['free_thresh'], 0, -1)).astype(np.int8)
        self.grid_hash = hashlib.sha256(np.flipud(self.occupancy).tobytes()).hexdigest()
        buf = io.BytesIO()
        self.image.save(buf, format='PNG')
        self.png = buf.getvalue()
        self.map_id = self.map_yaml.parent.name
        self.places = {}
        if self.path.exists():
            data = yaml.safe_load(self.path.read_text())
            if data.get('map_fingerprint') != self.fingerprint or data.get('frame_id') != 'map':
                raise ValueError('장소 파일과 지도가 다릅니다. 새 지도용 장소 파일을 등록하세요.')
            self.map_id = data['map_id']
            for row in data.get('places', []):
                p = Place.model_validate(row)
                self.validate_pose(p.observation_pose)
                if p.id in self.places:
                    raise ValueError('중복 장소 ID')
                self.places[p.id] = p

    def world_to_pixel(self, pose):
        dx, dy = pose.x-self.origin[0], pose.y-self.origin[1]
        c, s = math.cos(self.origin[2]), math.sin(self.origin[2])
        return ((c*dx+s*dy)/self.resolution-.5,
                self.height-.5-(-s*dx+c*dy)/self.resolution)

    def validate_pose(self, pose):
        u, v = self.world_to_pixel(pose)
        x, y = int(round(u)), int(round(v))
        if not (0 <= x < self.width and 0 <= y < self.height):
            raise ValueError('좌표가 지도 범위 밖입니다.')
        if self.occupancy[y, x] != 0:
            raise ValueError('목적지 중심은 지도에서 확인된 빈 공간이어야 합니다.')

    def describe(self):
        with self.lock:
            return {'map_id': self.map_id, 'fingerprint': self.fingerprint,
                    'frame_id': 'map', 'width': self.width, 'height': self.height,
                    'resolution': self.resolution, 'origin': self.origin,
                    'places': [p.model_dump() for p in self.places.values()]}

    def save(self, place):
        self.validate_pose(place.observation_pose)
        with self.lock:
            updated = dict(self.places)
            updated[place.id] = place
            data = {'map_id': self.map_id, 'map_fingerprint': self.fingerprint,
                    'frame_id': 'map', 'places': [p.model_dump() for p in updated.values()]}
            self.path.parent.mkdir(parents=True, exist_ok=True)
            temp = self.path.with_suffix('.yaml.tmp')
            temp.write_text(yaml.safe_dump(data, allow_unicode=True, sort_keys=False))
            os.replace(temp, self.path)
            self.places = updated


def path_score(path):
    """Prefer short forward routes; explicit reverse and gear changes cost more."""
    score, previous = 0., None
    for a, b in zip(path, path[1:]):
        dx, dy = b['x']-a['x'], b['y']-a['y']
        distance = math.hypot(dx, dy)
        if distance < 1e-5:
            continue
        reverse = dx*math.cos(a['yaw'])+dy*math.sin(a['yaw']) < 0
        score += distance * (3 if reverse else 1)
        if previous is not None and previous != reverse:
            score += 1
        previous = reverse
    return score


def heading_candidates(start, target):
    approach = math.atan2(target.y-start['y'], target.x-start['x'])
    values = [approach, target.yaw, approach+math.pi/2, approach-math.pi/2, approach+math.pi]
    result = []
    for v in values:
        v = math.atan2(math.sin(v), math.cos(v))
        if not any(abs(math.atan2(math.sin(v-x), math.cos(v-x))) < .08 for x in result):
            result.append(v)
    return result
