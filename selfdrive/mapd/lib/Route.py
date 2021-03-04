from .geo import DIRECTION
from copy import deepcopy
from .NodeReference import NodeReference


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
  """A set of consecutive way relations forming a default driving route ahead.
  """
  def __init__(self):
    self.ordered_way_relations = []
    self._node_references = None

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
    self._node_references = None
    wr = current
    while wr is not None:
      self.ordered_way_relations.append(deepcopy(wr))
      wr, way_relations = wr.next_wr(way_relations)

  @property
  def speed_limits_ahead(self):
    """Returns and array of SpeedLimitSection objects for the actual route
    """
    way_count = len(self.ordered_way_relations)
    if way_count == 0:
      return []

    wr = self.ordered_way_relations[0]
    section_start = 0
    section_distance = wr.distance_to_end
    section_speed_limit = wr.speed_limit

    if way_count == 1:
      return [SpeedLimitSection(0, section_distance, section_speed_limit)]

    limits_ahead = []
    for wr in self.ordered_way_relations[1:]:
      speed_limit = wr.speed_limit

      if speed_limit == section_speed_limit:
        section_distance += wr.distance_to_end
      else:
        # Close section
        limits_ahead.append(SpeedLimitSection(section_start, section_start + section_distance, section_speed_limit))
        # New section
        section_speed_limit = speed_limit
        section_start = section_start + section_distance
        section_distance = wr.distance_to_end

    limits_ahead.append(SpeedLimitSection(section_start, section_start + section_distance, section_speed_limit))

    return limits_ahead

  @property
  def node_references(self):
    if self._node_references is None:
      node_references = []

      for wr in self.ordered_way_relations:
        wr_node_references = [NodeReference(nd) for nd in wr.way.nodes]

        if wr.direction == DIRECTION.BACKWARD:
          wr_node_references.reverse()

        for nr in wr_node_references:
          if len(node_references) == 0 or nr.id != node_references[-1].id:
            node_references.append(nr)

      ref_node = node_references[0].node

      for idx, nr in enumerate(node_references):
        nr.update_xy(ref_node)

        if idx <= 1:
          continue

        node_references[idx - 1].update_tangent_angle(node_references[idx - 2].xy, nr.xy)

      count = len(node_references)

      for idx, nr in enumerate(node_references):
        if idx == 0 or idx == count - 1:
          continue

        nr.update_curvature(node_references[idx - 1], node_references[idx + 1])

      self._node_references = node_references
    return self._node_references

  @property
  def xy_path(self):
    return [nr.xy for nr in self.node_references]

  @property
  def curvatures(self):
    def curv_data(nr):
      return nr.x, nr.y, nr.tangent_angle, nr.curvature

    return [curv_data(nr) for nr in self.node_references]
