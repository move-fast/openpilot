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
  valid_way_relations = filter(lambda wr: wr.valid, way_relations)

  for wr in list(valid_way_relations):
    print(wr)
    print(wr.speed_limit)
