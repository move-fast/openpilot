from lib.osm import OSM
from lib.WayCollection import WayCollection
from decimal import Decimal
import sys
import csv


# TEST: python speed.py 52.273948132602584 13.91490391150784 1000 313


def csv_out_curvatures(name, curv_data):
  with open(f'{name}.csv', 'w', newline='') as results_csv:
    csv_writer = csv.writer(results_csv, delimiter=',')
    csv_writer.writerow([
        'X', 'Y', 'tan', 'cur'
    ])
    for curv in curv_data:
      csv_writer.writerow(curv)


if __name__ == '__main__':
  location = (Decimal(sys.argv[1]), Decimal(sys.argv[2]))
  bearing = float(sys.argv[4])

  # 1. Get ways around location
  osm = OSM()
  ways = osm.fetch_road_ways_around_location(location, float(sys.argv[3]))

  # 2. Create the collection
  way_collection = WayCollection(ways)

  # 3. Find the current route
  route = way_collection.get_route(location, bearing)

  # 4. Output
  print('_____ GIVEN DIRECTION')
  if route is None:
    print('No valid routes found for given loaction and bearing.')
  else:
    print(f'Current way: {route.current_wr}')
    print(f'Speed Limit: {route.current_wr.speed_limit}')
    print(f'Route Ahead: {route}')
    print(f'Limits Ahead: {route.speed_limits_ahead}')
    print(f'curvatures: {route.curvatures_ahed}')
    # csv_out_curvatures('forward', route.curvatures)

  # 4. Update on the oposit direction for testing
  route = way_collection.get_route(location, bearing - 180)

  # 5. Output in oposit direction
  print('      ')
  print('_____REVERSE DIRECTION')
  if route is None:
    print('No valid routes found for given loaction and bearing.')
  else:
    print(f'Current way: {route.current_wr}')
    print(f'Speed Limit: {route.current_wr.speed_limit}')
    print(f'Route Ahead: {route}')
    print(f'Limits Ahead: {route.speed_limits_ahead}')
    print(f'curvatures: {route.curvatures_ahed}')
    # csv_out_curvatures('backward', route.curvatures)
