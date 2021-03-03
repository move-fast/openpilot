from lib.osm import WayRelation, OSM, WayCollection
from decimal import Decimal
import sys
import csv


# TEST: python speed.py 52.273948132602584 13.91490391150784 1000 313


def csv_out_xy_path(name, xy_path):
  with open(f'{name}.csv', 'w', newline='') as results_csv:
    csv_writer = csv.writer(results_csv, delimiter=',')
    csv_writer.writerow([
        'X', 'Y'
    ])
    for xy in xy_path:
      csv_writer.writerow(xy)


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
    print(f'Route Ahead: {way_collection.route}')
    print(f'Limits Ahead: {way_collection.route.speed_limits_ahead}')
    print(f'Node Refs: {way_collection.route.xy_path}')
    csv_out_xy_path('forward', way_collection.route.xy_path)

  # 4. Update on the oposit direction for testing
  way_collection.locate(location, bearing - 180)
  wr = way_collection.current

  # 5. Output in oposit direction
  print('      ')
  print('_____REVERSE DIRECTION')
  if wr is None:
    print('No valid ways found for given loaction and bearing.')
  else:
    print(f'Best way: {wr}')
    print(f'Speed Limit: {wr.speed_limit}')
    print(f'Distance To End: {wr.distance_to_end}')
    print(f'Route Ahead: {way_collection.route}')
    print(f'Limits Ahead: {way_collection.route.speed_limits_ahead}')
    print(f'Node Refs: {way_collection.route.xy_path}')
    csv_out_xy_path('backward', way_collection.route.xy_path)
