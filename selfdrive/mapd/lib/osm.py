import overpy
from common.numpy_fast import interp
from .geo import DIRECTION, distance_and_bearing, absoule_delta_with_direction, bearing_delta, bearing, distance


_ACCEPTABLE_BEARING_DELTA_V = [40., 20, 10, 5]
_ACCEPTABLE_BEARING_DELTA_BP = [.03, .1, .2, .3]


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
  """A class that represent the relationship of an OSM node and a given `location` and `bearing` of a driving vehicle.
  """
  def __init__(self, node, location, bearing):
    self.node = node
    self.update(location, bearing)

  def __repr__(self):
    return f'NodeRelation: n: {self.node.id}, distance: {self.distance:.4f}, bearing_delta: {self.bearing_delta:.4f}, \
      direction: {self.direction}'

  def update(self, location, bearing):
    """Will update the associated node with a given `location` and `bearing`.
       Specifically it will find the distance, bearing delta and direction between the node and the given location.
    """
    self.distance, self.bearing_to_node = distance_and_bearing(location, (self.node.lat, self.node.lon))
    self.bearing_delta, self.direction = absoule_delta_with_direction(bearing_delta(bearing, self.bearing_to_node))


class WayRelation():
  """A class that represent the relationship of an OSM way and a given `location` and `bearing` of a driving vehicle.
  """
  def __init__(self, way, location=None, bearing=None):
    self.way = way
    self.update_bounding_box()
    self.reset()
    self._internode_distances = [None] * (len(way.nodes) - 1)
    self._lenght = None

    if location is not None and bearing is not None:
      self.update(location, bearing)

  def __repr__(self):
    return f'Way: {self.way.id}, ahead: {self.ahead_idx}, behind: {self.behind_idx}, {self.direction}, ok: {self.valid}'

  def reset(self):
    self.ahead_idx = None
    self.behind_idx = None
    self.valid = False
    self.direction = DIRECTION.NONE
    self._speed_limit = None
    self._located_way_bearing = None

  def update(self, location, bearing):
    """Will update and validate the associated way with a given `location` and `bearing`.
       Specifically it will find the nodes behind and ahead of the current location and bearing.
       If no proper fit to the way geometry, the way relation is marked as invalid.
    """
    self.reset()
    if self.is_location_in_bbox(location):
      # TODO: This can perhaps be done more efficient by starting from the closest node.
      for idx, node in enumerate(self.way.nodes):
        node_relation = NodeRelation(node, location, bearing)
        if abs(node_relation.bearing_delta) > \
           interp(node_relation.distance, _ACCEPTABLE_BEARING_DELTA_BP, _ACCEPTABLE_BEARING_DELTA_V):
          continue

        if node_relation.direction == DIRECTION.AHEAD:
          self.ahead_idx = idx
          self.distance_to_node_ahead = node_relation.distance
          if self.behind_idx is not None:
            break
        elif node_relation.direction == DIRECTION.BEHIND:
          self.behind_idx = idx
          if self.ahead_idx is not None:
            break
    # Validate
    if self.ahead_idx is None or self.behind_idx is None or abs(self.ahead_idx - self.behind_idx) > 1:
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
    """Indicates if a given location is contained in the bounding box surrounding the way.
    """
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

  @property
  def ref(self):
    return self.way.tags.get("ref", None)

  @property
  def located_bearing(self):
    """Returns the exact bearing of the portion of way we are currentluy located at.
    """
    if self._located_way_bearing is not None:
      return self._located_way_bearing

    if not self.valid:
      return None

    ahead_node = self.way.nodes[self.ahead_idx]
    behind_node = self.way.nodes[self.behind_idx]

    self._located_way_bearing = bearing((behind_node.lat, behind_node.lon), (ahead_node.lat, ahead_node.lon))
    return self._located_way_bearing

  def located_bearing_delta(self, bearing):
    """Returns the delta between the given bearing and the exact
       bearing of the portion of way we are currentluy located at.
    """
    if self.located_bearing is None:
      return None
    return bearing_delta(bearing, self.located_bearing)

  def internode_distance(self, idx):
    d = self._internode_distances[idx]
    if d is not None:
      return d
    pointA = (self.way.nodes[idx].lat, self.way.nodes[idx].lon)
    pointB = (self.way.nodes[idx + 1].lat, self.way.nodes[idx + 1].lon)
    d = distance(pointA, pointB)
    self._internode_distances[idx] = d
    return d

  @property
  def lenght(self):
    if self._lenght is None:
      distances = [self.internode_distance(idx) for idx in range(len(self._internode_distances))]
      self._lenght = sum(distances)
    return self._lenght

  @property
  def distance_to_end(self):
    if not self.valid:
      return None
    if self.direction == DIRECTION.FORWARD:
      indices = range(self.ahead_idx, len(self.way.nodes) - 1)
    else:  # BACKWARDS
      indices = range(self.ahead_idx)
    distances = [self.internode_distance(idx) for idx in indices]
    return sum(distances) + self.distance_to_node_ahead

  @property
  def last_node(self):
    """Returns the last node on the way considering the traveling direction
    """
    if not self.valid:
      return None
    return self.way.nodes[-1] if self.direction == DIRECTION.FORWARD else self.way.nodes[0]

  def edge_on_node(self, node_id):
    """Indicates if the associated way starts or ends in the node with `node_id`
    """
    return self.way.nodes[0].id == node_id or self.way.nodes[-1].id == node_id


class WayCollection():
  """A collection of WayRelations to use for maps data analysis.
  """
  def __init__(self, ways):
    self.ways = ways
    self.way_relations = list(map(lambda way: WayRelation(way), ways))
    self.reset()

  def reset(self):
    self.location = None
    self.bearing = None
    self.located_way_relations = []

  def locate(self, location, bearing):
    """Updates the driving location inside the collection based on given `location` and `bearing` for the
    driving vehicle.
    In practice repopulates the located_way_relations list with all valid way_relations and sorted by best bearing
    match to driving direction.
    """
    if location is None or bearing is None or (self.location == location and self.bearing == bearing):
      return

    self.reset()
    self.location = location
    self.bearing = bearing

    for wr in self.way_relations:
      wr.update(location, bearing)

    self.located_way_relations = list(filter(lambda wr: wr.valid, self.way_relations))
    self.located_way_relations.sort(key=lambda wr: wr.located_bearing_delta(bearing))

  @property
  def current(self):
    """Returns the current way relation if any based on current `location` and `bearing`
    """
    # the best matching way relation is the first one on the located_way_relations list.
    return self.located_way_relations[0] if len(self.located_way_relations) else None

  @property
  def next(self):
    """Returns the next way relation if any based on current `location` and `bearing`
    """
    current = self.current
    if current is None:
      return None

    last_node = current.last_node
    possible_next_wr = list(filter(lambda wr: wr.edge_on_node(last_node.id), self.way_relations))
    possibles = len(possible_next_wr)

    if possibles == 0:
      return None
    if possibles == 1 or current.ref is None:
      return possible_next_wr[0]

    return next((wr for wr in possible_next_wr if wr.ref == current.ref), possible_next_wr[0])
