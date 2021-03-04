from .WayRelation import WayRelation
from .Route import Route


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
