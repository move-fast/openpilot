from .geo import DIRECTION
from copy import deepcopy
from .RouteNode import RouteNode


class SpeedLimitSection():
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
  """A set of consecutive way relations forming a default driving route.
  """
  def __init__(self, current, way_relations):
    self._reset()
    if not current.active:
      return
    wr = current
    while wr is not None:
      self._ordered_way_relations.append(deepcopy(wr))
      wr, way_relations = wr.next_wr(way_relations)
    self._build_node_list()
    self._locate()

  def __repr__(self):
    return f'{self._ordered_way_relations}'

  def _reset(self):
    self._ordered_way_relations = []
    self._limits_ahead = None
    self._ahead_idx = None
    self._distance_to_node_ahead = None

  def _build_node_list(self):
    route_nodes = []
    # Compile list of nodes from all way relations in order including speed limit from way relation.
    for wr in self._ordered_way_relations:
      wr_route_nodes = [RouteNode(nd, wr.speed_limit) for nd in wr.way.nodes]
      if wr.direction == DIRECTION.BACKWARD:
        wr_route_nodes.reverse()
      for rn in wr_route_nodes:
        if len(route_nodes) == 0 or rn.id != route_nodes[-1].id:
          route_nodes.append(rn)
        elif rn.id == route_nodes[-1].id:
          # We overwrite the adjacent nodes from ways with the one from the further way since this contains
          # the speed limit we need.
          route_nodes[-1] = rn
    # Update all nodes xy, distance, bearing and tangent angle
    count = len(route_nodes)
    ref_node = route_nodes[0].node
    for idx, rn in enumerate(route_nodes):
      rn.update_xy(ref_node)
      if idx < count - 1:
        rn.update_distance_and_bearing_to_next_node(route_nodes[idx + 1].node)
      if idx <= 1:
        continue
      route_nodes[idx - 1].update_tangent_angle(route_nodes[idx - 2].xy, rn.xy)
    # update all nodes curvature
    for idx, rn in enumerate(route_nodes):
      if idx == 0 or idx == count - 1:
        continue
      rn.update_curvature(route_nodes[idx - 1], route_nodes[idx + 1])
    self._route_nodes = route_nodes

  def _locate(self):
    """Will resolve the index in the route nodes list for the node ahead of the current location.
    It updates as well the distance from the current location to the node ahead.
    """
    current = self.current_wr
    if current is None:
      return
    node_ahead_id = current.node_ahead.id
    self._distance_to_node_ahead = current.distance_to_node_ahead
    start_idx = self._ahead_idx if self._ahead_idx is not None else 0
    self._ahead_idx = None
    for idx in range(start_idx, len(self._route_nodes)):
      if self._route_nodes[idx].id == node_ahead_id:
        self._ahead_idx = idx
        break

  @property
  def valid(self):
    return self.current_wr is not None

  @property
  def current_wr(self):
    return self._ordered_way_relations[0] if len(self._ordered_way_relations) else None

  def update(self, location, bearing):
    """Will update the route structure based on the given `location` and `bearing` assuming progress on the route
    on the original direction. If direction has changed or active point on the route can not be found, the route
    will become invalid.
    """
    if len(self._ordered_way_relations) == 0 or location is None or bearing is None:
      return
    if self.current_wr.location == location and self.current_wr.bearing == bearing:
      return
    # Transverse the way relations on the actual order until we find an active one. From there, rebuild the route
    # with the way relations remaining ahead.
    for idx, wr in enumerate(self._ordered_way_relations):
      active_direction = wr.direction
      wr.update(location, bearing)
      if not wr.active:
        continue
      if wr.direction == active_direction:
        # We have now the current wr, Repopulate from here till the end and locate
        new = self._ordered_way_relations[idx:]
        self._reset()
        self._ordered_way_relations = new
        self._locate()
        return
      # Driving direction on the route has changed. stop.
      break
    # if we got here, there is no new active way relation or driving direction has changed. Reset.
    self._reset()

  @property
  def speed_limits_ahead(self):
    """Returns and array of SpeedLimitSection objects for the actual route
    """
    if self._limits_ahead is not None:
      return self._limits_ahead
    nodes_count = len(self._route_nodes)
    if nodes_count == 0:
      return []
    # create speed limits sections combining all continious nodes that have same speed limit value.
    rn = self._route_nodes[0]
    section_start = 0
    section_distance = self._distance_to_node_ahead
    section_speed_limit = rn.speed_limit
    if self._ahead_idx == nodes_count - 1:
      limits_ahead = [SpeedLimitSection(0, section_distance, section_speed_limit)]
    else:
      limits_ahead = []
      for rn in self._route_nodes[self._ahead_idx:]:
        speed_limit = rn.speed_limit
        if speed_limit == section_speed_limit:
          section_distance += rn.distance_to_next_node
        else:
          # Close section
          limits_ahead.append(SpeedLimitSection(section_start, section_start + section_distance, section_speed_limit))
          # New section
          section_speed_limit = speed_limit
          section_start = section_start + section_distance
          section_distance = rn.distance_to_next_node
      # Clse final section
      limits_ahead.append(SpeedLimitSection(section_start, section_start + section_distance, section_speed_limit))
    self._limits_ahead = limits_ahead
    return limits_ahead

  @property
  def curvatures_ahed(self):
    """Provides a list of ordered tuples by distance including the distance ahead and the curvature.
    """
    data_list = []
    distance = self._distance_to_node_ahead
    for rn in self._route_nodes[self._ahead_idx:]:
      data_list.append((distance, rn.curvature))
      distance += rn.distance_to_next_node
    return data_list
