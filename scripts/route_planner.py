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

SCALE = .3


class Path_planner():
    def __init__(self, filename, image):
        # Check that the file is a proper Path_planner file
        with open(filename, 'r') as f:
            yml = yaml.load(f, Loader=yaml.FullLoader)

        # Checks to perform in the input file
        if (yml['fileType'] != "Plan" or
                yml['version'] != 1 or
                yml['groundStation'] != "QGroundControl"):
            sys.exit(f"Error: {filename} is not a valid QGC plan file")

        self.filename = filename
        self.mission = yml["mission"]["items"]
        self.rally = yml["rallyPoints"]["points"]
        self.waypoints = {i:d for i, d in enumerate(self.mission) if d["command"] == 16}
        self.fence = yml["geoFence"]["polygons"][0]
        if not self.fence["inclusion"]:
            sys.exit("Error: the geoFence is not an inclusion zone")
        self.noFly = [i for i in yml["geoFence"]["polygons"]
                      if not i["inclusion"]]

        # Store the last path for visualization purposes
        self.last_start = None
        self.last_end = None
        self.last_path = None

        # self.origin_lat, self.origin_long, _ = self.mission['plannedHomePosition']
        img_lat, img_long = [39.943254, -75.202198]
        [self.img_utm_east, self.img_utm_north, _, _] = utm.from_latlon(img_lat, img_long)
        img_bottom_lat, img_bottom_long = [39.939762, -75.1982]
        [img_bottom_utm_east, img_bottom_utm_north, _, _] = utm.from_latlon(img_bottom_lat,
                                                                      img_bottom_long)
        self.corners_latlong = np.array([[img_lat, img_long],
                                         [img_bottom_lat, img_bottom_long]])

        # Keep picture for displaying purposes
        img = cv2.imread(image)
        img = cv2.rotate(img, cv2.ROTATE_180)
        img = cv2.flip(img, 1)
        self.img = cv2.resize(img, (0,0), fx=SCALE, fy=SCALE)

        # Get the scale of the image
        self.scale_east = self.img.shape[1]/(img_bottom_utm_east - self.img_utm_east)
        self.scale_north = self.img.shape[0]/(img_bottom_utm_north - self.img_utm_north)
        self.generateGraph()

    def scale_points(self, lats, longs):
        """ scale_points can take a single latitude/longitude or a list, and
        converts it to the corresponding x,y coordinate in the image"""
        if isinstance(lats, float):
            lats = [lats]
            longs = [longs]
        x = []
        y = []
        for lat, lon in zip(lats, longs):
            utm_east, utm_north, _, _ = utm.from_latlon(lat, lon)
            x_, y_ = [int((utm_east - self.img_utm_east)*self.scale_east),
                    int((utm_north - self.img_utm_north)*self.scale_north)]
            x.append(x_)
            y.append(y_)
        if len(x) == 1:
            return x[0], y[0]
        return x, y

    def getDistance(self, p1, p2):
        """Gets the distance in meters from two points p1 and p2, each point has
        latitude and longitude"""
        p_1_e, p_1_n, _, _ = utm.from_latlon(p1[0], p1[1])
        p_2_e, p_2_n, _, _ = utm.from_latlon(p2[0], p2[1])
        return np.sqrt((p_1_e - p_2_e)**2 + (p_1_n - p_2_n)**2)

    def draw_poly_nofly(self, img, filled=False):
        if len(img.shape) == 2:
            color = 255
        else:
            color = (0, 0, 255)
        for item in self.noFly:
            lats = [i[0] for i in item["polygon"]]
            longs = [i[1] for i in item["polygon"]]
            x, y = self.scale_points(lats, longs)
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
        for i in self.waypoints:
            points[i] = {"latlon": self.waypoints[i]["params"][4:6],
                         "neigh": {j: None for j in self.waypoints.keys() if
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
        fence = np.array(self.fence["polygon"])
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

        if start_node == end_node:
            print("Start and end are the same")
            return None

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

    def display_points(self, mission=False, noFly=False, rally=False,
                       routes=False, plan=False):
        old_x = None
        old_y = None
        img = self.img.copy()
        if mission:
            for i, item in enumerate(self.mission):
                if "params" not in item.keys():
                    continue
                lat = item['params'][4]
                lon = item['params'][5]
                # check if lat or lon are None
                if lat is None or lon is None or lat == 0 or lon == 0:
                    continue
                # print(f"Lat: {lat:.6f}, Lon: {lon:.6f}")
                x, y = self.scale_points(lat, lon)
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
    parser.add_argument('--filename', help='YAML file to read', required=True)
    parser.add_argument('--image', help='Image to display', required=True)
    args = parser.parse_args()

    # Check if the file exists
    if not os.path.isfile(args.filename):
        sys.exit(f"Error: {args.filename} does not exist")

    # Check that image exists
    if not os.path.isfile(args.image):
        sys.exit(f"Error: {args.image} does not exist")

    # Check that image exists
    q = Path_planner(args.filename, "../images/pennovation2.png")

    q.display_points(mission=True)

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
        start_latlon = [random.uniform(q.corners_latlong[0, 0],
                                       q.corners_latlong[1, 0]),
                        random.uniform(q.corners_latlong[0, 1],
                                       q.corners_latlong[1, 1])]
        end_latlon = [random.uniform(q.corners_latlong[0, 0],
                                     q.corners_latlong[1, 0]),
                      random.uniform(q.corners_latlong[0, 1],
                                     q.corners_latlong[1, 1])]
        route = q.planRoute(start_latlon, end_latlon)
        if route is not None:
            q.display_points(noFly=True, rally=True, routes=True, plan=True)
            i += 1
