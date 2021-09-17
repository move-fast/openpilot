import overpy
import math


class DPOSM():
  def __init__(self):
    self.api = overpy.Overpass()

  def build_way_query(self, lat, lon, heading, radius=50):
    print("lat: %s lon: %s heading: %s" % (lat, lon, heading))
    """Builds a query to find all highways within a given radius around a point"""
    a = 111132.954*math.cos(float(lat)/180*3.141592)
    b = 111132.954 - 559.822 * math.cos( 2 * float(lat)/180*3.141592) + 1.175 * math.cos( 4 * float(lat)/180*3.141592)
    heading = math.radians(-heading + 90)
    lat = lat+math.sin(heading)*radius/2/b
    lon = lon+math.cos(heading)*radius/2/a
    pos = "  (around:%f,%f,%f)" % (radius, lat, lon)
    lat_lon = "(%f,%f)" % (lat, lon)
    q = """(
      way
      """ + pos + """
      [highway][highway!~"^(footway|path|corridor|bridleway|steps|cycleway|construction|bus_guideway|escape|service|track)$"];
      >;);out;""" + """is_in""" + lat_lon + """;area._[admin_level~"[24]"];
      convert area ::id = id(), admin_level = t['admin_level'],
      name = t['name'], "ISO3166-1:alpha2" = t['ISO3166-1:alpha2'];out;
      """
    return q

  def fetch_road_ways_around_location(self, lat, lon, heading, radius=50):
    q = self.build_way_query(lat, lon, heading, radius)
    try:
      ways = self.api.query(q).ways
    except Exception as e:
      print(f'Exception while querying OSM:\n{e}')
      ways = []

    return ways

if __name__ == "__main__":
  osm = OSM()
  print(osm.build_way_query(-27.6256875, 153.0617238, 0, 3000))
