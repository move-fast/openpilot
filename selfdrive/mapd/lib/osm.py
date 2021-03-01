import overpy
from copy import deepcopy
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
    self.reset_location_variables()
    self.direction = DIRECTION.NONE
    self._speed_limit = None
    self._internode_distances = [None] * (len(way.nodes) - 1)
    self._lenght = None

    if location is not None and bearing is not None:
      self.update(location, bearing)

  def __repr__(self):
    return f'Way: {self.way.id}, ahead: {self.ahead_idx}, behind: {self.behind_idx}, {self.direction}, on: {self.valid}'

  def reset_location_variables(self):
    self.valid = False
    self.ahead_idx = None
    self.behind_idx = None
    self._located_way_bearing = None

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
      self.reset_location_variables()
      return

    self.valid = True
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
      return self.lenght
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

  def has_exact_state(self, way_relation):
    """ Indicates if this instance is an exact match to `way_relation` on `id` and location in way.
    """
    return self.same_direction(way_relation) and self.ahead_idx == way_relation.ahead_idx \
        and self.distance_to_node_ahead == way_relation.distance_to_node_ahead

  def same_direction(self, way_relation):
    """ Indicates if this instance and `way_relation` refer to the same way in the same direction
    """
    return way_relation is not None and self.id == way_relation.id and self.direction == way_relation.direction


class SpeedLimitAhead():
  """And object representing a speed limited road section ahead.
  provides the start and end distance and the speed limit value
  """
  def __init__(self, start, end, value):
    self.start = start
    self.end = end
    self.value = value

  def __repr__(self):
    return f'from: {self.start}, to: {self.end}, limit: {self.value}'


class Route():
  """A set of consecutive way relations forming a default driving route ahead.
  """
  def __init__(self):
    self.ordered_way_relations = []

  def __repr__(self):
    return f'{self.ordered_way_relations}'

  @property
  def current(self):
    return self.ordered_way_relations[0] if len(self.ordered_way_relations) else None

  def update(self, current, way_relations):
    # Nothing to update if `current` is None or if nothing has changed since last update.
    if current is None or current.has_exact_state(self.current):
      return
    # If route is already populated and only change is the location inside the current way relation,
    # then only update first element in route.
    if len(self.ordered_way_relations) > 0 and current.same_direction(self.current):
      self.ordered_way_relations[0] = current
      return
    # otherwise update the whole route.
    self.ordered_way_relations = []
    wr = current
    while wr is not None:
      self.ordered_way_relations.append(deepcopy(wr))
      wr, way_relations = wr.next_wr(way_relations)

  @property
  def speed_limits_ahead(self):
    """Returns and array of SpeedLimitAhead objects for the actual route
    """
    way_count = len(self.ordered_way_relations)
    if way_count == 0:
      return []

    wr = self.ordered_way_relations[0]
    section_start = 0
    section_distance = wr.distance_to_end
    section_speed_limit = wr.speed_limit

    if way_count == 1:
      return [SpeedLimitAhead(0, section_distance, section_speed_limit)]

    limits_ahead = []
    for wr in self.ordered_way_relations[1:]:
      speed_limit = wr.speed_limit

      if speed_limit == section_speed_limit:
        section_distance += wr.distance_to_end
      else:
        # Close section
        limits_ahead.append(SpeedLimitAhead(section_start, section_start + section_distance, section_speed_limit))
        # New section
        section_speed_limit = speed_limit
        section_start = section_start + section_distance
        section_distance = wr.distance_to_end

    limits_ahead.append(SpeedLimitAhead(section_start, section_start + section_distance, section_speed_limit))

    return limits_ahead


class WayCollection():
  """A collection of WayRelations to use for maps data analysis.
  """
  def __init__(self, ways):
    self.ways = ways
    self.way_relations = list(map(lambda way: WayRelation(way), ways))
    self._route = Route()
    self.reset()

  def reset(self):
    self.location = None
    self.bearing = None
    self._current = None

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

    located_way_relations = list(filter(lambda wr: wr.valid, self.way_relations))
    located_way_relations.sort(key=lambda wr: wr.located_bearing_delta(bearing))

    # the best matching way relation is the first one on the located_way_relations list.
    # Assign to current and reset location variables for the rest.
    self._current = located_way_relations[0] if len(located_way_relations) else None
    if len(located_way_relations) > 1:
      for wr in located_way_relations[1:]:
        wr.reset_location_variables()

  @property
  def current(self):
    """Returns the current way relation if any based on current `location` and `bearing`
    """
    return self._current

  @property
  def route(self):
    """Provides a route ahead using the way relations on the collection considering as starting point on
    the current way relation
    """
    self._route.update(self.current, self.way_relations)
    return self._route

  def speed_limits_ahead(self, distance=500):
    """Provides a dictionary of speed limits ahead from current location to given `distance`.
    Keys are the distances where the speed limit value changes ahead.
    When speed limit is not available for a specific section of the road ahead, `None` is returned as value.
    """
    return None
