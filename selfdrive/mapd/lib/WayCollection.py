from .WayRelation import WayRelation
from .Route import Route
import uuid


class WayCollection():
  """A collection of WayRelations to use for maps data analysis.
  """
  def __init__(self, ways):
    self.id = uuid.uuid4()
    self.way_relations = list(map(lambda way: WayRelation(way), ways))

  def get_route(self, location, bearing):
    """Provides the best route found in the way collection based on provided `location` and `bearing`
    """
    if location is None or bearing is None:
      return None
    for wr in self.way_relations:
      wr.update(location, bearing)
    active_way_relations = list(filter(lambda wr: wr.active, self.way_relations))
    active_way_relations.sort(key=lambda wr: wr.active_bearing_delta(bearing))
    if len(active_way_relations) == 0:
      return None
    # the best matching way relation is the first one on the active_way_relations list. Consider it current.
    # reset location for remaining located wr.
    current = active_way_relations[0]
    if len(active_way_relations) > 1:
      for wr in active_way_relations[1:]:
        wr.reset_location_variables()
    return Route(current, self.way_relations, self.id)
