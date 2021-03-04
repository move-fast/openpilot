from .geo import DIRECTION, distance_and_bearing, absoule_delta_with_direction, bearing_delta, bearing, distance
from common.numpy_fast import interp


_ACCEPTABLE_BEARING_DELTA_V = [40., 20, 10, 5]
_ACCEPTABLE_BEARING_DELTA_BP = [.03, .1, .2, .3]


class WayRelation():
  """A class that represent the relationship of an OSM way and a given `location` and `bearing` of a driving vehicle.
  """
  def __init__(self, way, location=None, bearing=None):
    self.way = way
    self.update_bounding_box()
    self.reset_location_variables()
    self.direction = DIRECTION.NONE
    self._speed_limit = None
    if location is not None and bearing is not None:
      self.update(location, bearing)

  def __repr__(self):
    return f'id: {self.id}, ahead: {self.ahead_idx}, behind: {self.behind_idx}, {self.direction}, active: {self.active}'

  def reset_location_variables(self):
    self.location = None
    self.bearing = None
    self.active = False
    self.ahead_idx = None
    self.behind_idx = None
    self._active_way_bearing = None

  @property
  def id(self):
    return self.way.id

  def update(self, location, bearing):
    """Will update and validate the associated way with a given `location` and `bearing`.
       Specifically it will find the nodes behind and ahead of the current location and bearing.
       If no proper fit to the way geometry, the way relation is marked as invalid.
    """
    self.reset_location_variables()
    if not self.is_location_in_bbox(location):
      return
    # TODO: This can perhaps be done more efficient by starting from the closest node.
    for idx, node in enumerate(self.way.nodes):
      distance_to_node, bearing_to_node = distance_and_bearing(location, (node.lat, node.lon))
      delta, direction = absoule_delta_with_direction(bearing_delta(bearing, bearing_to_node))
      if abs(delta) > interp(distance_to_node, _ACCEPTABLE_BEARING_DELTA_BP, _ACCEPTABLE_BEARING_DELTA_V):
        continue
      if direction == DIRECTION.AHEAD:
        self.ahead_idx = idx
        self.distance_to_node_ahead = distance_to_node
        if self.behind_idx is not None:
          break
      elif direction == DIRECTION.BEHIND:
        self.behind_idx = idx
        if self.ahead_idx is not None:
          break
    # Validate
    if self.ahead_idx is None or self.behind_idx is None or abs(self.ahead_idx - self.behind_idx) > 1:
      self.reset_location_variables()
      return
    self.active = True
    self.location = location
    self.bearing = bearing
    self._speed_limit = None
    self.direction = DIRECTION.FORWARD if self.ahead_idx - self.behind_idx > 0 else DIRECTION.BACKWARD

  def update_direction_from_starting_node(self, start_node_id):
    self._speed_limit = None
    if self.way.nodes[0].id == start_node_id:
      self.direction = DIRECTION.FORWARD
    elif self.way.nodes[-1].id == start_node_id:
      self.direction = DIRECTION.BACKWARD
    else:
      self.direction = DIRECTION.NONE

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
  def active_bearing(self):
    """Returns the exact bearing of the portion of way we are currentluy located at.
    """
    if self._active_way_bearing is not None:
      return self._active_way_bearing
    if not self.active:
      return None
    ahead_node = self.way.nodes[self.ahead_idx]
    behind_node = self.way.nodes[self.behind_idx]
    self._active_way_bearing = bearing((behind_node.lat, behind_node.lon), (ahead_node.lat, ahead_node.lon))
    return self._active_way_bearing

  def active_bearing_delta(self, bearing):
    """Returns the delta between the given bearing and the exact
       bearing of the portion of way we are currentluy located at.
    """
    if self.active_bearing is None:
      return None
    return bearing_delta(bearing, self.active_bearing)

  @property
  def node_behind(self):
    return self.way.nodes[self.behind_idx] if self.behind_idx is not None else None

  @property
  def node_ahead(self):
    return self.way.nodes[self.ahead_idx] if self.ahead_idx is not None else None

  @property
  def last_node(self):
    """Returns the last node on the way considering the traveling direction
    """
    if self.direction == DIRECTION.FORWARD:
      return self.way.nodes[-1]
    if self.direction == DIRECTION.BACKWARD:
      return self.way.nodes[0]
    return None

  def edge_on_node(self, node_id):
    """Indicates if the associated way starts or ends in the node with `node_id`
    """
    return self.way.nodes[0].id == node_id or self.way.nodes[-1].id == node_id

  def next_wr(self, way_relations):
    """Returns a tuple with the next way relation (if any) based on `location` and `bearing` and a copy of
    the `way_relations` list excluding the found next way relation. (to help with recursion)
    """
    if self.direction not in [DIRECTION.FORWARD, DIRECTION.BACKWARD]:
      return None, way_relations
    possible_next_wr = list(filter(lambda wr: wr.id != self.id and wr.edge_on_node(self.last_node.id), way_relations))
    possibles = len(possible_next_wr)
    if possibles == 0:
      return None, way_relations
    if possibles == 1 or self.ref is None:
      next_wr = possible_next_wr[0]
    else:
      next_wr = next((wr for wr in possible_next_wr if wr.ref == self.ref), possible_next_wr[0])
    next_wr.update_direction_from_starting_node(self.last_node.id)
    updated_way_relations = list(filter(lambda wr: wr.id != next_wr.id, way_relations))
    return next_wr, updated_way_relations
