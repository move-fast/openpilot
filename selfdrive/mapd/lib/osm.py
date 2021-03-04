import overpy


class OSM():
  def __init__(self):
    self.api = overpy.Overpass()

  def fetch_road_ways_around_location(self, location, radius):
      lat, lon = location

      # fetch all ways and nodes on this ways around location
      around_str = f'{str(radius)},{str(lat)},{str(lon)}'
      q = """
          way(around:""" + around_str + """)
            [highway]
            [highway!~"^(footway|path|bridleway|steps|cycleway|construction|bus_guideway|escape|service)$"];
          (._;>;);
          out;
          """
      return self.api.query(q).ways
