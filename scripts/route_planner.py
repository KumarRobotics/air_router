#!/usr/bin/env python3
import os
import pdb
import argparse
import cv2
import sys
import yaml
import random
import time
import pprint
import utm
import numpy as np
import heapq
import rospy
import rospkg

SCALE = 1

""" Route planner uses standard coordinates (m) to plan the mission. These can be
obtained from UTM (for GPS files) or as absolute coordinates for simulation
files """


class Mission():
    """ Get a mission file and convert it to standard coordinates """
    def __init__(self, mission_file, mission_file_format):
        self.mission_file = mission_file
        with open(mission_file, 'r') as f:
            yml = yaml.load(f, Loader=yaml.FullLoader)

        if mission_file_format == "QGC":
            self.offset = np.array([0, 0])
            # Check that the file is a valid QGC mission file
            if (yml['fileType'] != "Plan" or
                    yml['version'] != 1 or
                    yml['groundStation'] != "QGroundControl"):
                sys.exit(f"Error: {mission_file} is not a valid QGC plan file")

            # Get mission in utm
            mission = yml["mission"]["items"]
            for m in mission:
                if m["params"][4] is not None and m["params"][5] is not None:
                    m["params"][4:6] = utm.from_latlon(m["params"][4],
                                                       m["params"][5])[:2] - self.offset

            # Get rally points in utm
            self.rally = [utm.from_latlon(x, y)[:2]-self.offset
                          for [x, y, _] in
                          yml["rallyPoints"]["points"]]

            # Extract waypoints from the mission
            # Waypoints are a dictionary where the key is the waypoint numer,
            # and the value is the standard coordinates
            self.waypoints = {i: d["params"][4:6]
                              for i, d in enumerate(mission)
                              if d["command"] == 16}

            # Get fence in UTM
            self.fence = yml["geoFence"]["polygons"][0]
            for p in self.fence["polygon"]:
                p[0], p[1] = utm.from_latlon(p[0], p[1])[:2] - self.offset
            if not self.fence["inclusion"]:
                sys.exit("Error: the geoFence is not an inclusion zone")

            # noFly is a list of polygons, with the coordinates of the noFly
            # zones
            self.noFly = [i["polygon"] for i in yml["geoFence"]["polygons"]
                          if not i["inclusion"]]
            for nf in self.noFly:
                for p in nf:
                    p[0], p[1] = utm.from_latlon(p[0], p[1])[:2] - self.offset
        elif mission_file_format == "Sim":
            self.noFly = [i["polygon"] for i in yml["geoFence"]["polygons"]
                          if not i["inclusion"]]
            self.rally = None
            self.waypoints = yml["waypoints"]
            self.fence = yml["geoFence"]["polygons"][0]
            if not self.fence["inclusion"]:
                sys.exit("Error: the geoFence is not an inclusion zone")

        else:
            sys.exit(f"Error: {mission_file_format} is not a valid format")


class Path_planner():
    def __init__(self, mission, map_name):
        # Check that mission is a valid object
        assert isinstance(mission, Mission)
        self.mission = mission

        # The origin of the quad is an offset from the current utm position
        # self.offset = np.array([float(offset_x), float(offset_y)])
        self.offset = np.array([0, 0])

        # Store the last path for visualization purposes
        self.last_start = None
        self.last_end = None
        self.last_path = None

        # Get the image from semantics_manager
        pkg = rospkg.RosPack()
        path = pkg.get_path('semantics_manager')
        map_yaml_path = os.path.join(path, "maps", map_name, "map_config.yaml")
        with open(map_yaml_path, 'r') as f:
            map_yaml = yaml.load(f, Loader=yaml.FullLoader)
        image = os.path.join(path, "maps", map_name, map_yaml["viz"])

        # Get resolution (px/m) and image origin from the yaml
        self.resolution = map_yaml["resolution"]
        self.image_origin_px_x = map_yaml["image_origin_px_x"]
        self.image_origin_px_y = map_yaml["image_origin_px_y"]

        # Origin of the image in UTM
        # img_lat, img_long = [39.943254, -75.202198]
        # self.img_utm = utm.from_latlon(img_lat, img_long)[:2] - self.offset
        # img_bottom_lat, img_bottom_long = [39.939762, -75.1982]
        # img_utm_bottom = utm.from_latlon(img_bottom_lat, img_bottom_long)[:2] - self.offset
        # self.corners_latlong = np.array([self.img_utm, img_utm_bottom ])

        # Keep picture for displaying purposes
        img = cv2.imread(image)
        self.img = cv2.resize(img, (0,0), fx=SCALE, fy=SCALE)

        # Get the scale of the image
        # self.scale_east = self.img.shape[1]/(img_utm_bottom[0] - self.img_utm[0])
        # self.scale_north = self.img.shape[0]/(img_utm_bottom[1] - self.img_utm[1])
        # print(f"Scale: {self.scale_east}, {self.scale_north}")
        self.generateGraph()

    def scale_points(self, utms_x, utms_y):
        """ scale_points can take an utm point and return the corresponding
        x,y coordinates in the image"""
        if isinstance(utms_x, float):
            utms_x = [utms_x]
            utms_y = [utms_y]
        x = []
        y = []
        for utm_x, utm_y in zip(utms_x, utms_y):
            x_, y_ = [int(utm_x*self.resolution)+self.image_origin_px_x,
                      -int(utm_y*self.resolution)+self.image_origin_px_y]
            x.append(x_)
            y.append(y_)
        if len(x) == 1:
            return x[0], y[0]
        return x, y

    def getDistance(self, p1, p2):
        """Gets the distance in meters from two points p0 and p2, both points in
        standard coordinates """
        return np.linalg.norm(np.array(p1) - np.array(p2))

    def draw_poly_nofly(self, img, filled=False):
        if len(img.shape) == 2:
            color = 255
        else:
            color = (0, 0, 255)
        for item in self.mission.noFly:
            p1 = [i[0] for i in item]
            p2 = [i[1] for i in item]
            x, y = self.scale_points(p1, p2)
            # Close the polygon
            x.append(x[0])
            y.append(y[0])
            pts = np.array([x, y]).T
            pts = pts.reshape((-1, 1, 2))
            if filled:
                cv2.fillPoly(img, [pts], color, 1)
            else:
                cv2.polylines(img, [pts], True, color, 1)
        return img

    def generateGraph(self):
        # create a set with all the possible waypoints
        points = {}
        for i in self.mission.waypoints:
            points[i] = {"latlon": self.mission.waypoints[i],
                         "neigh": {j: None for j in self.mission.waypoints.keys() if
                                                    j != i}}

        # Remove points that are too far away from each other
        for i in points:
            for j in points[i]["neigh"].copy():
                d = self.getDistance(points[i]["latlon"], points[j]["latlon"])
                if d > 100:
                    del points[i]["neigh"][j]

        # Remove points that intersect with noFly zones
        polygon_mask = np.zeros(self.img.shape[:2], dtype=np.uint8)
        self.draw_poly_nofly(polygon_mask, filled=True)

        # TODO: check that all the routes are within the fence

        for i in points:
            for j in points[i]["neigh"].copy():
                lat, long = points[i]["latlon"]
                lat2, long2 = points[j]["latlon"]
                x, y = self.scale_points(lat, long)
                x2, y2 = self.scale_points(lat2, long2)
                line_mask = np.zeros(self.img.shape[:2], dtype=np.uint8)
                cv2.line(line_mask, (x, y), (x2, y2), 255, 1)
                intersection = cv2.bitwise_and(line_mask, polygon_mask)
                # Check if the line intersects with the noFly zone
                if cv2.countNonZero(intersection) > 0:
                    del points[i]["neigh"][j]

        # Fill the neighbor distances
        for i in points:
            for j in points[i]["neigh"]:
                d = self.getDistance(points[i]["latlon"], points[j]["latlon"])
                points[i]["neigh"][j] = d
        self.graph = points

        # Calculate paths for all the nodes
        # Not very efficient but it is a small graph
        for r in self.graph:
            _, self.graph[r]["path"] = self.dijstra(r)

    def planRoute(self, start_latlon, end_latlon):
        """ Returns the route for """
        self.last_start = start_latlon
        self.last_end = end_latlon

        # Check that the points are within the allowed geofence
        fence_mask = np.zeros(self.img.shape[:2], dtype=np.uint8)
        fence = np.array(self.mission.fence["polygon"])
        fence_x, fence_y = self.scale_points(fence[:, 0], fence[:, 1])
        fence = np.array([fence_x, fence_y]).T
        fence = fence.reshape((-1, 1, 2))
        fence_mask = cv2.fillPoly(fence_mask, [fence], 255, 1)
        # Invert image as we want black the points outside the fence
        fence_mask = cv2.bitwise_not(fence_mask)
        point_mask = np.zeros(self.img.shape[:2], dtype=np.uint8)
        point_mask = cv2.circle(point_mask, self.scale_points(start_latlon[0],
                                                               start_latlon[1]),
                                1, 255, -1)
        point_mask = cv2.circle(point_mask, self.scale_points(end_latlon[0],
                                                              end_latlon[1]),
                                1, 255, -1)
        if cv2.countNonZero(cv2.bitwise_and(fence_mask, point_mask)) > 0:
            print("Start or end point is outside the allowed geofence")
            return None

        # Find the closest waypoint to the start and end
        dstart = np.inf
        dend = np.inf
        for r in self.graph:
            d = self.getDistance(start_latlon, self.graph[r]["latlon"])
            if d < dstart:
                dstart = d
                start_node = r
            d = self.getDistance(end_latlon, self.graph[r]["latlon"])
            if d < dend:
                dend = d
                end_node = r

        # If start and end node are the same, return an empty list
        if start_node == end_node:
            return []

        self.last_path = self.graph[start_node]["path"][end_node]
        return self.last_path

    def dijstra(self, start):
        dist = {node: float('inf') for node in self.graph}
        dist[start] = 0
        pq = [(0, start)]
        predecessors = {node: None for node in self.graph}
        while pq:
            (distance, node) = heapq.heappop(pq)
            if distance > dist[node]:
                continue
            for neighbor, weight in self.graph[node]["neigh"].items():
                new_distance = dist[node] + weight
                if new_distance < dist[neighbor]:
                    dist[neighbor] = new_distance
                    predecessors[neighbor] = node
                    heapq.heappush(pq, (new_distance, neighbor))
        paths = {node: [] for node in self.graph}
        for node in self.graph:
            if predecessors[node] is None:
                continue
            current_node = node
            while current_node is not None:
                paths[node].insert(0, current_node)
                current_node = predecessors[current_node]
        return (dist, paths)

    def display_points(self, waypoints=False, noFly=False, rally=False,
                       routes=False, plan=False):
        old_x = None
        old_y = None
        img = self.img.copy()
        if waypoints:
            for i in self.mission.waypoints:
                wp = self.mission.waypoints[i]
                # print(f"Lat: {lat:.6f}, Lon: {lon:.6f}")
                x, y = self.scale_points(wp[0], wp[1])
                img = cv2.circle(img, (x, y), 2, (0, 0, 255), -1)
                img = cv2.putText(img, str(i), (x, y),
                                  cv2.FONT_HERSHEY_SIMPLEX, .25, (0, 0, 255))

                if old_x is not None and old_y is not None:
                    img = cv2.line(img, (old_x, old_y), (x, y), (0, 255, 0), 1)
                old_x = x
                old_y = y

        if rally:
            for item in self.rally:
                lat = item[0]
                lon = item[1]
                x, y = self.scale_points(lat, lon)
                img = cv2.circle(img, (x, y), 2, (255, 0, 255), -1)
                # img = cv2.putText(img, str(i), (x, y),
                #                   cv2.FONT_HERSHEY_SIMPLEX, .25, (0, 0, 255))

        if noFly:
            img = self.draw_poly_nofly(img, filled=False)
            fence = np.array(self.mission.fence["polygon"])
            fence_x, fence_y = self.scale_points(fence[:, 0], fence[:, 1])
            fence = np.array([fence_x, fence_y]).T
            fence = fence.reshape((-1, 1, 2))
            # Draw polygon for the fence
            img = cv2.polylines(img, [fence], True, (0, 255, 255), 1)

        if routes:
            for route in self.graph:
                for neigh in self.graph[route]["neigh"]:
                    lat, long = self.graph[route]["latlon"]
                    lat2, long2 = self.graph[neigh]["latlon"]
                    x, y = self.scale_points(lat, long)
                    x2, y2 = self.scale_points(lat2, long2)
                    cv2.line(img, (x, y), (x2, y2), (255, 255, 0), 1)
                img = cv2.putText(img, str(route), (x, y),
                                  cv2.FONT_HERSHEY_SIMPLEX, .5, (0, 0, 255))

        if plan:
            if self.last_end is not None:
                last_x_end, last_y_end = self.scale_points(self.last_end[0],
                                         self.last_end[1])
                img = cv2.circle(img, (last_x_end, last_y_end),
                                 2, (0, 255, 0), -1)

            if self.last_start is not None:
                last_x_start, last_y_start = self.scale_points(self.last_start[0],
                                         self.last_start[1])
                img = cv2.circle(img, (last_x_start, last_y_start),
                                 2, (0, 255, 0), -1)

            if self.last_path is not None:
                for i, node in enumerate(self.last_path):
                    lat, long = self.graph[node]["latlon"]
                    x, y = self.scale_points(lat, long)
                    img = cv2.circle(img, (x, y), 2, (0, 255, 0), -1)
                    if i > 0:
                        lat, long = self.graph[self.last_path[i-1]]["latlon"]
                        x2, y2 = self.scale_points(lat, long)
                        cv2.line(img, (x, y), (x2, y2), (0, 255, 0), 1)
                    if i == 0:
                        cv2.line(img, (x, y),
                                 (last_x_start, last_y_start), (0, 255, 0), 1)
                # Plot a line connecting the end and start
                cv2.line(img, (x, y), (last_x_end, last_y_end), (0, 255, 0), 1)

        cv2.namedWindow('Pennovation', cv2.WINDOW_NORMAL)
        cv2.imshow('Pennovation', img)
        cv2.waitKey(0)


if __name__ == "__main__":
    # read program arguments
    parser = argparse.ArgumentParser(
            prog=f"{os.path.basename(__file__)}",
            description='Read a yaml file and plot waypoints')
    parser.add_argument('--map_name',
                        help='Map name to use from semantics_manager',
                        required=True)
    args = parser.parse_args()

    # Create a mission from the mission file
    pkg = rospkg.RosPack()
    path = pkg.get_path('semantics_manager')
    map_yaml_path = os.path.join(path, "maps", args.map_name, "map_config.yaml")
    with open(map_yaml_path, "r") as f:
        map_yaml = yaml.load(f, Loader=yaml.FullLoader)
    mission_file = os.path.join(path, "maps", args.map_name, map_yaml["quad_plan"])
    m = Mission(mission_file, map_yaml["quad_plan_format"])

    # Create a path planner object
    q = Path_planner(m, args.map_name)

    # a = {r:{"neigh": list(q.graph[r]["neigh"].keys()),
    #         "utm": [str(j) for j in utm.from_latlon(q.graph[r]["latlon"][0],
    #                                q.graph[r]["latlon"][1])
    #                 ]} for r in q.graph}
    # pprint.pprint(a)
    # # save a into a yaml
    # with open("test.yaml", "w") as f:
    #     f.write(yaml.dump(a))
    # sys.exit()

    i = 0
    while (i < 10):
        start_latlon = [random.uniform(-130, 130),
                        random.uniform(-100, 100)]
        end_latlon =   [random.uniform(-130, 130),
                        random.uniform(-100, 100)]
        print(f"Start: {start_latlon} - End: {end_latlon}")
        route = q.planRoute(start_latlon, end_latlon)
        if route is not None:
            q.display_points(noFly=True, routes=True, plan=True)
            i += 1
