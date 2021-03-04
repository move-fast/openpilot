from .geo import three_point_tangent_angle, three_point_curvature, xy, distance_and_bearing


class RouteNode():
  def __init__(self, node, speed_limit=None):
    self.node = node
    self._ref_node_id = None  # The id of the node used as refernce for xy calculation
    self.x = None
    self.y = None
    self.tangent_angle = None
    self.curvature = None
    self.distance_to_next_node = 0
    self.bearing_to_next_node = None
    self.speed_limit = speed_limit

  def __repr__(self):
    return f'id: {self.id}, (x, y): {self.x}, {self.y}'

  @property
  def id(self):
    return self.node.id

  @property
  def xy(self):
    return self.x, self.y

  def update_xy(self, ref_node):
    if self._ref_node_id == ref_node.id:
      return
    self._ref_node_id = ref_node.id
    ref_point = (ref_node.lat, ref_node.lon)
    point = (self.node.lat, self.node.lon)
    self.x, self.y = xy(ref_point, point)

  def update_distance_and_bearing_to_next_node(self, next_node):
    self.distance_to_next_node, self.bearing_to_next_node = \
        distance_and_bearing((self.node.lat, self.node.lon), (next_node.lat, next_node.lon))

  def update_tangent_angle(self, prev_xy, next_xy):
    if self.x is None or self.y is None:
      return
    self.tangent_angle = three_point_tangent_angle((self.x, self.y), prev_xy, next_xy)

  def update_curvature(self, prev_nr, next_nr):
    if self.tangent_angle is None or self.x is None or self.y is None:
      return
    self.curvature = three_point_curvature(self.xy, prev_nr.xy, next_nr.xy,
                                           self.tangent_angle, prev_nr.tangent_angle, next_nr.tangent_angle)
