from lib.osm import WayRelation, OSM, WayCollection
from decimal import Decimal
import sys


# TEST: python speed.py 52.273948132602584 13.91490391150784 1000 313


if __name__ == '__main__':
  location = (Decimal(sys.argv[1]), Decimal(sys.argv[2]))
  bearing = float(sys.argv[4])

  # 1. Get ways around location
  osm = OSM()
  ways = osm.fetch_road_ways_around_location(location, float(sys.argv[3]))

  # 2. Create the collection
  way_collection = WayCollection(ways)

  # 3. Find the current way
  way_collection.locate(location, bearing)
  wr = way_collection.current

  # 4. Output
  print('_____ GIVEN DIRECTION')
  if wr is None:
    print('No valid ways found for given loaction and bearing.')
  else:
    print(f'Best way: {wr}')
    print(f'Speed Limit: {wr.speed_limit}')
    print(f'Distance To End: {wr.distance_to_end}')
    print(f'Next way: {way_collection.next}')

  # 4. Update on the oposit direction for testing
  way_collection.locate(location, bearing - 180)
  wr = way_collection.current

  # 5. Output in oposit direction
  print('_____REVERSE DIRECTION')
  if wr is None:
    print('No valid ways found for given loaction and bearing.')
  else:
    print(f'Best way: {wr}')
    print(f'Speed Limit: {wr.speed_limit}')
    print(f'Distance To End: {wr.distance_to_end}')
    print(f'Next way: {way_collection.next}')
