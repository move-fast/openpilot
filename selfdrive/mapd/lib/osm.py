import overpy
from .geo import DIRECTION, distance_and_bearing, absoule_delta_with_direction, bearing_delta


ACCEPTABLE_BEARING_DELTA = 5.0


class OSM():
  def __init__(self):
    self.api = overpy.Overpass()

  def fetch_road_ways_around_location(self, location, radius):
      lat, lon = location

      # fetch all ways and nodes on this ways around location
      around_str = f'{str(radius)},{str(lat)},{str(lon)}'
      q = """
          way(around:""" + around_str + """)
            [highway]
            [highway!~"^(footway|path|bridleway|steps|cycleway|construction|bus_guideway|escape|service)$"];
          (._;>;);
          out;
          """
      return self.api.query(q).ways


class NodeRelation():
  def __init__(self, node, location, bearing):
    self.node = node
    self.update(location, bearing)

  def __repr__(self):
    return f'NodeRelation: n: {self.node.id}, distance: {self.distance:.4f}, bearing_delta: {self.bearing_delta:.4f}, \
      direction: {self.direction}'

  def update(self, location, bearing):
    self.distance, self.bearing_to_node = distance_and_bearing(location, (self.node.lat, self.node.lon))
    self.bearing_delta, self.direction = absoule_delta_with_direction(bearing_delta(bearing, self.bearing_to_node))


class WayRelation():
  def __init__(self, way, location=None, bearing=None):
    self.way = way
    self.update_bounding_box()

    self.ahead_idx = None
    self.behind_idx = None
    self.valid = False
    self.direction = DIRECTION.NONE
    self._speed_limit = None

    if location is not None and bearing is not None:
      self.update(location, bearing)

  def __repr__(self):
    return f'Way: {self.way.id}, ahead: {self.ahead_idx}, behind: {self.behind_idx}, {self.direction}, ok: {self.valid}'

  def update(self, location, bearing):
    if self.is_location_in_bbox(location):
      # TODO: This can perhaps be done more efficient by starting from the closest node.
      for idx, node in enumerate(self.way.nodes):
        node_relation = NodeRelation(node, location, bearing)

        if abs(node_relation.bearing_delta) > ACCEPTABLE_BEARING_DELTA:
          continue

        if node_relation.direction == DIRECTION.AHEAD:
          self.ahead_idx = idx
          if self.behind_idx is not None:
            break
        elif node_relation.direction == DIRECTION.BEHIND:
          self.behind_idx = idx
          if self.ahead_idx is not None:
            break

    self.validate()

  def validate(self):
    if self.ahead_idx is None or self.behind_idx is None or abs(self.ahead_idx - self.behind_idx) > 1:
      self.valid = False
      self.direction = DIRECTION.NONE
      return

    self.valid = True
    self.direction = DIRECTION.FORWARD if self.ahead_idx - self.behind_idx > 0 else DIRECTION.BACKWARD

  def update_bounding_box(self):
    nodes = self.way.nodes
    node = nodes.pop(0)

    max_lat = min_lat = node.lat
    max_lon = min_lon = node.lon

    for node in nodes:
      max_lat = max(max_lat, node.lat)
      max_lon = max(max_lon, node.lon)
      min_lat = min(min_lat, node.lat)
      min_lon = min(min_lon, node.lon)

    self.bbox = min_lat, min_lon, max_lat, max_lon

  def is_location_in_bbox(self, location):
    lat, lon = location
    min_lat, min_lon, max_lat, max_lon = self.bbox
    return lat >= min_lat and lat <= max_lat and lon >= min_lon and lon <= max_lon

  @property
  def speed_limit(self):
    if self._speed_limit is not None:
      return self._speed_limit

    limit = self.way.tags.get("maxspeed")
    if limit is None:
      if self.direction == DIRECTION.FORWARD:
        limit = self.way.tags.get("maxspeed:forward")
      elif self.direction == DIRECTION.BACKWARD:
        limit = self.way.tags.get("maxspeed:backward")

    self._speed_limit = limit
    return self._speed_limit
