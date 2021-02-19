import overpy
import sys
import csv
import math
import simplejson as json

#  pip install simplejson
#  pip install overpy
#  python overpass_speed.py 37.7833 -122.4167 500

earth_radius = 6378e3  # meters


def coords_by_bearing_distance(coordinates, bearing, distance):
    """
    compute destination coordinates by initial coordinates, distance and bearing
    see https://www.movable-type.co.uk/scripts/latlong.html
    """
    lat, lon = coordinates
    angular_distance = distance / earth_radius
    lat_rad = math.radians(lat)
    lon_rad = math.radians(lon)
    bearing_rad = math.radians(bearing)
    lat_fwd_rad = math.asin(
        math.sin(lat_rad) * math.cos(angular_distance) +
        math.cos(lat_rad) * math.sin(angular_distance) * math.cos(bearing_rad))
    lon_fwd_rad = lon_rad + math.atan2(
        math.sin(bearing_rad) * math.sin(angular_distance) * math.cos(lat_rad),
        math.cos(angular_distance) - math.sin(lat_rad) * math.sin(lat_fwd_rad))
    lat_fwd = math.degrees(lat_fwd_rad)
    lon_fwd = math.degrees(lon_fwd_rad)

    return lat_fwd, lon_fwd


def bearing_between_coords(coordinates_1, coordinates_2):
    """
    compute bearing betweein two coordinates
    see https://www.movable-type.co.uk/scripts/latlong.html
    """
    lat_1, lon_1 = coordinates_1
    lat_2, lon_2 = coordinates_2
    lat_rad_1 = math.radians(lat_1)
    lon_rad_1 = math.radians(lon_1)
    lat_rad_2 = math.radians(lat_2)
    lon_rad_2 = math.radians(lon_2)
    y = math.sin(lon_rad_2 - lon_rad_1) * math.cos(lat_rad_2)
    x = math.cos(lat_rad_1) * math.sin(lat_rad_2) - math.sin(
        lat_rad_1) * math.cos(lat_rad_2) * math.cos(lon_rad_2 - lon_rad_1)
    bearing = math.degrees(math.atan2(y, x))

    return bearing  # degrees


def maxspeed(coordinates, bearing, distance):
    lat, lon = coordinates
    api = overpy.Overpass()

    coordinates_fwd = coords_by_bearing_distance(coordinates, bearing,
                                                 distance)
    print('(%f, %f)' % (coordinates_fwd))

    bearing = bearing_between_coords(coordinates, coordinates_fwd)
    print(bearing)

    # fetch all ways and nodes
    q = """
        (
            way(around:""" + str(distance) + """,""" + str(
        lat) + """,""" + str(lon) + """) ["maxspeed"];
            way(around:""" + str(distance) + """,""" + str(
            lat) + """,""" + str(lon) + """) ["maxspeed:forward"];
            way(around:""" + str(distance) + """,""" + str(
                lat) + """,""" + str(lon) + """) ["maxspeed:backward"];
        );
        (._;>;);
        out body;
        """
    result = api.query(q)
    results_list = []
    with open('nodes.csv', 'w', newline='') as results_csv:
        csv_writer = csv.writer(results_csv, delimiter=',')
        csv_writer.writerow([
            'Lat', 'Long', 'speed_limit', "speed_limit:forward",
            "speed_limit:backward"
        ])
        for way in result.ways:
            road = {}
            road["name"] = way.tags.get("name", "n/a")
            road["speed_limit"] = way.tags.get("maxspeed", "n/a")
            road["speed_limit:forward"] = way.tags.get("maxspeed:forward",
                                                       "n/a")
            road["speed_limit:backward"] = way.tags.get(
                "maxspeed:backward", "n/a")
            nodes = []
            for node in way.nodes:
                nodes.append((node.lat, node.lon))
                csv_writer.writerow([
                    float(node.lat),
                    float(node.lon), road["speed_limit"],
                    road["speed_limit:forward"], road["speed_limit:backward"]
                ])
            road["nodes"] = nodes
            results_list.append(road)
    return results_list


if __name__ == '__main__':
    results = maxspeed((float(sys.argv[1]), float(sys.argv[2])),
                       float(sys.argv[3]), float(sys.argv[4]))
    print(json.dumps(results))
