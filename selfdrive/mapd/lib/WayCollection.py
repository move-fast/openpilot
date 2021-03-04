from .WayRelation import WayRelation
from .Route import Route


class WayCollection():
  """A collection of WayRelations to use for maps data analysis.
  """
  def __init__(self, ways):
    self.way_relations = list(map(lambda way: WayRelation(way), ways))

  def get_route(self, location, bearing):
    """Provides the best route found in the way collection based on provided `location` and `bearing`
    """
    if location is None or bearing is None:
      return

    for wr in self.way_relations:
      wr.update(location, bearing)

    located_way_relations = list(filter(lambda wr: wr.valid, self.way_relations))
    located_way_relations.sort(key=lambda wr: wr.located_bearing_delta(bearing))

    # the best matching way relation is the first one on the located_way_relations list. Consider it current.
    # reset location for remaining located wr.
    current = located_way_relations[0] if len(located_way_relations) else None
    if len(located_way_relations) > 1:
      for wr in located_way_relations[1:]:
        wr.reset_location_variables()

    return Route(current, self.way_relations)
