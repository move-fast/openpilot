from lib.osm import WayRelation, OSM
from decimal import Decimal
import sys


if __name__ == '__main__':
  location = (Decimal(sys.argv[1]), Decimal(sys.argv[2]))
  bearing = float(sys.argv[4])

  # 1. Get ways around location
  osm = OSM()
  ways = osm.fetch_road_ways_around_location(location, float(sys.argv[3]))

  # Find valid ways
  way_relations = map(lambda way: WayRelation(way, location, bearing), ways)
  valid_way_relations = list(filter(lambda wr: wr.valid, way_relations))

  # Pick the one with lowest bearing delta between our bearing and the real bearing of the road where we are located.
  if len(valid_way_relations) == 0:
    print('No valid ways found for given loaction and bearing.')
  else:
    valid_way_relations.sort(key=lambda wr: wr.located_bearing_delta(bearing))
    wr = valid_way_relations[0]
    print(f'Best way: {wr}')
    print(f'Speed Limit: {wr.speed_limit}')
