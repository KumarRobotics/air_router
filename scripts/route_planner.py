#!/usr/bin/env python3
import os
import pdb
import argparse
import cv2
import sys
import yaml
import random
import pprint
import utm
import numpy as np
import heapq
import rospy
import threading

""" Route planner uses standard coordinates (m) to plan the mission. These can
be obtained from UTM (for GPS files) or as absolute coordinates for simulation
files """
# FIXME(fclad): The file refers to "latlon" when using standard coordinates.


class Mission():
    """ Get a mission file and convert it to standard coordinates """
    def __init__(self, mission_file, mission_file_format, origin=None):
        self.mission_file = mission_file
        with open(mission_file, 'r') as f:
            yml = yaml.load(f, Loader=yaml.FullLoader)

        if mission_file_format == "QGC":
            # Origin should be a vector with 2 elements, with the UTM
            # coordinates of the origin of the mission. These will be
            # subtracted to the UTM coordinates of the waypoints to
            # scale the mission
            self.origin = origin
            assert self.origin is not None
            assert len(self.origin) == 2
            assert isinstance(self.origin, np.ndarray)

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
                                                       m["params"][5])[:2] - self.origin

            # Get rally points in utm
            self.rally = [utm.from_latlon(x, y)[:2] - self.origin
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
                p[0], p[1] = utm.from_latlon(p[0], p[1])[:2] - self.origin
            if not self.fence["inclusion"]:
                sys.exit("Error: the geoFence is not an inclusion zone")

            # noFly is a list of polygons, with the coordinates of the noFly
            # zones
            self.noFly = [i["polygon"] for i in yml["geoFence"]["polygons"]
                          if not i["inclusion"]]
            for nf in self.noFly:
                for p in nf:
                    p[0], p[1] = utm.from_latlon(p[0], p[1])[:2] - self.origin
        elif mission_file_format == "Sim":
            self.noFly = [i["polygon"] for i in yml["geoFence"]["polygons"]
                          if not i["inclusion"]]
            self.rally = None
            self.waypoints = {w: [yml["waypoints"][w][0], yml["waypoints"][w][1]]
                              for w in yml["waypoints"]}
            self.altitude = {w: yml["waypoints"][w][2] for w in yml["waypoints"]}
            self.fence = yml["geoFence"]["polygons"][0]
            if not self.fence["inclusion"]:
                sys.exit("Error: the geoFence is not an inclusion zone")

        else:
            sys.exit(f"Error: {mission_file_format} is not a valid format")


class Path_planner():
    def __init__(self, map_path, max_edge_length):
        # Store the last path for visualization purposes
        self.last_start = None
        self.last_end = None
        self.last_path = None
        # Create a lock for the visualization objects
        self.lock = threading.Lock()
        self.max_edge_length = max_edge_length
        self.map_path = map_path

        # Check that max eddge len is a positive number below 500
        if not isinstance(self.max_edge_length, int):
            pdb.set_trace()
            sys.exit("Error: max_edge_lenght must be an integer")
        if self.max_edge_length > 500 or self.max_edge_length < 1:
            sys.exit("Error: max_edge_length must be between 1 and 500")

        # Get the image from semantics_manager
        with open(map_path, 'r') as f:
            map_yaml = yaml.load(f, Loader=yaml.FullLoader)
        # Image is in the same folder as the yaml
        image = os.path.join(os.path.dirname(map_path), map_yaml["color"])

        # Get resolution (px/m) and image origin pixels from the yaml
        self.resolution = map_yaml["img_resolution"]
        self.image_origin_px_x = map_yaml["image_origin_px_x"]
        self.image_origin_px_y = map_yaml["image_origin_px_y"]

        # Keep picture for displaying purposes
        img = cv2.imread(image)

        # Resize image to a height of 1000px if its height is not 1000px
        # if img.shape[0] != 1000:
        #     old_height = img.shape[0]
        #     img = cv2.resize(img, (int(img.shape[1] * 1000 / img.shape[0]),
        #                            int(1000)))
        #     self.scale_factor = 1000 / old_height
        # else:
        #     self.scale_factor = 1
        self.scale_factor = 1
        self.img = img

        # # Reduce the contrast of the image
        # clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        # cl1 = clahe.apply(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY))
        # # make the image brighter
        # self.img = cv2.cvtColor(cl1, cv2.COLOR_GRAY2BGR)
        # self.img = cv2.convertScaleAbs(self.img, alpha=2.5, beta=0)
        # self.img = cv2.cvtColor(cl1, cv2.COLOR_GRAY2BGR)

        # Desaturate the image
        self.img = cv2.cvtColor(self.img, cv2.COLOR_BGR2HSV)
        self.img[:, :, 1] = self.img[:, :, 1] * 0.4
        self.img[:, :, 2] = self.img[:, :, 2] * .5
        self.img = cv2.cvtColor(self.img, cv2.COLOR_HSV2BGR)
        # clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        # cl1 = clahe.apply(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY))
        # # make the image brighter
        # self.img = cv2.cvtColor(cl1, cv2.COLOR_GRAY2BGR)
        # self.img = cv2.convertScaleAbs(self.img, alpha=2.5, beta=0)
        # self.img = cv2.cvtColor(cl1, cv2.COLOR_GRAY2BGR)


        # Create mission object. Only QGC missions have GPS coordinates for the
        # origin
        mission_file = os.path.join(os.path.dirname(map_path),
                                    map_yaml["quad_plan"])
        mission_file_format = map_yaml["quad_plan_format"]
        if map_yaml["quad_plan_format"] == "Sim":
            self.mission = Mission(mission_file, mission_file_format)
        elif map_yaml["quad_plan_format"] == "QGC":
            origin = utm.from_latlon(map_yaml["gps_origin_lat"],
                                     map_yaml["gps_origin_long"])[:2]
            self.origin = np.array(origin)
            self.mission = Mission(mission_file, map_yaml["quad_plan_format"],
                                   self.origin)
        else:
            sys.exit("Invalid mission format")

        # Generate the graph and calculate costs with Dijkstra
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
            x.append(int(x_*self.scale_factor))
            y.append(int(y_*self.scale_factor))
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
            color = (00, 50, 200)
        # Draw all the no fly zones
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
                cv2.polylines(img, [pts], True, color, 3)
        # Draw the fence
        fence = np.array(self.mission.fence["polygon"])
        fence_x, fence_y = self.scale_points(fence[:, 0], fence[:, 1])
        fence = np.array([fence_x, fence_y]).T
        fence = fence.reshape((-1, 1, 2))
        if filled:
            # We want the negative of the fence here
            mask = np.full(img.shape[:2], 255, dtype=np.uint8)
            cv2.fillPoly(mask, [fence], 0, 1)
            cv2.add(img, mask, img)
        else:
            img = cv2.polylines(img, [fence], True, color, 4)
        return img

    def generateGraph(self):
        # create a set with all the possible waypoints
        points = {}
        for i in self.mission.waypoints:
            points[i] = {"latlon": self.mission.waypoints[i],
                         "neigh": {j: None for j
                                   in self.mission.waypoints.keys() if j != i}}

        # Remove points that are too far away from each other
        for i in points:
            for j in points[i]["neigh"].copy():
                d = self.getDistance(points[i]["latlon"], points[j]["latlon"])
                if d > self.max_edge_length:
                    del points[i]["neigh"][j]

        # Create a mask for the no-fly zone
        polygon_mask = np.zeros(self.img.shape[:2], dtype=np.uint8)
        polygon_mask = self.draw_poly_nofly(polygon_mask, filled=True)

        # Dilate the mask so that the lines are not too close to the fence
        kernel_size = int(self.resolution*self.scale_factor)*5
        self.polygon_mask = cv2.dilate(polygon_mask,
                                       np.ones((kernel_size, kernel_size),
                                               np.uint8),
                                       iterations=1)
        # Display polygon mask
        # cv2.imshow("mask", self.polygon_mask)
        # cv2.waitKey(0)

        for i in points:
            for j in points[i]["neigh"].copy():
                lat, long = points[i]["latlon"]
                lat2, long2 = points[j]["latlon"]
                x, y = self.scale_points(lat, long)
                x2, y2 = self.scale_points(lat2, long2)
                line_mask = np.zeros(self.img.shape[:2], dtype=np.uint8)
                cv2.line(line_mask, (x, y), (x2, y2), 255, 3)
                intersection = cv2.bitwise_and(line_mask, self.polygon_mask)
                # Check if the line intersects with the noFly zone
                if cv2.countNonZero(intersection) > 0:
                    del points[i]["neigh"][j]

            # Check that we have at least one route to all the waypoints
            # shutdown node otherwise
            if len(points[i]["neigh"]) == 0:
                rospy.logerr("No route to waypoint {}".format(i))
                rospy.signal_shutdown("No route to waypoint {}".format(i))
                # return

        # Fill the neighbor distances
        for i in points:
            for j in points[i]["neigh"]:
                d = self.getDistance(points[i]["latlon"], points[j]["latlon"])
                points[i]["neigh"][j] = d
        self.graph = points

        # Calculate paths for all the nodes
        # Not very efficient but it is a small graph
        for r in self.graph:
            _, self.graph[r]["path"] = self.dijkstra(r)

    def planRoute(self, start_latlon, end_latlon):
        """ Returns the route for """
        # Check that the start and end are within the map boundaries
        start_x, start_y = self.scale_points(start_latlon[0], start_latlon[1])
        end_x, end_y = self.scale_points(end_latlon[0], end_latlon[1])

        if start_x < 0 or start_x > self.img.shape[1] or \
                start_y < 0 or start_y > self.img.shape[0]:
            rospy.logerr("Start point outside image range")
            return None
        if end_x < 0 or end_x > self.img.shape[1] or \
                end_y < 0 or end_y > self.img.shape[0]:
            rospy.logerr("End point outside image range")
            return None

        # Check that the points are within the allowed geofence
        fence_mask = np.zeros(self.img.shape[:2], dtype=np.uint8)
        fence = np.array(self.mission.fence["polygon"])
        fence_x, fence_y = self.scale_points(fence[:, 0], fence[:, 1])
        fence = np.array([fence_x, fence_y]).T
        fence = fence.reshape((-1, 1, 2))
        fence_mask = cv2.fillPoly(fence_mask, [fence], 255, 1)
        # Invert image as we want black the points outside the fence
        fence_mask = cv2.bitwise_not(fence_mask)
        # IMPORTANT: Do no check for the no-fly zones here because the target
        # robot may be inside one!

        point_mask = np.zeros(self.img.shape[:2], dtype=np.uint8)
        point_mask = cv2.circle(point_mask, (start_x, start_y),
                                10, 255, 2)
        point_mask = cv2.circle(point_mask, (end_x, end_y),
                                10, 255, 2)
        if cv2.countNonZero(cv2.bitwise_and(fence_mask, point_mask)) > 0:
            rospy.logerr("Start or end point is outside the allowed geofence")
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

        with self.lock:
            self.last_start = start_latlon
            self.last_end = end_latlon
            # If start and end node are the same, return a list with a single
            # item
            if start_node == end_node:
                self.last_path = [start_node]
            else:
                self.last_path = self.graph[start_node]["path"][end_node]

            # Check if we have 3 waypoints or more in our path and see if we
            # can skip the first one (avoids going backwards and then
            # move forward)
            if len(self.last_path) >= 2:
                # Create lines for the start and first waypoint, and the first
                # and second waypoints
                l1_1 = np.array(start_latlon)
                l1_2 = self.graph[self.last_path[0]]["latlon"]
                l1 = l1_2 - l1_1
                l2_1 = np.array(self.graph[self.last_path[0]]["latlon"])
                l2_2 = np.array(self.graph[self.last_path[1]]["latlon"])
                l2 = l2_1 - l2_2
                # Get the angle between the two lines
                angle = np.arccos(np.dot(l1, l2) / (np.linalg.norm(l1) *
                                                    np.linalg.norm(l2)))
                if angle < np.pi / 2:
                    lat, long = start_latlon
                    lat2, long2 = l2_2
                    x, y = self.scale_points(lat, long)
                    x2, y2 = self.scale_points(lat2, long2)
                    line_mask = np.zeros(self.img.shape[:2], dtype=np.uint8)
                    cv2.line(line_mask, (x, y), (x2, y2), 255, 5)
                    intersection = cv2.bitwise_and(line_mask,
                                                   self.polygon_mask)
                    # Check if the line intersects with the noFly zone
                    if cv2.countNonZero(intersection) == 0:
                        rospy.loginfo("Removing first waypoint")
                        self.last_path.pop(0)

        return self.last_path.copy()

    def dijkstra(self, start):
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

    def display_points(self, get_image=False, origin=False, waypoints=False,
                       noFly=False, rally=False, routes=False, plan=False):
        old_x = None
        old_y = None
        img = self.img.copy()
        # Make img rgba
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGBA)
        if origin:
            # Display the origin in the image
            img = cv2.circle(img, (int(self.image_origin_px_x*self.scale_factor),
                                   int(self.image_origin_px_y*self.scale_factor)),
                             thickness=2, radius=10, color=(20, 20, 20))
        if waypoints:
            for i in self.mission.waypoints:
                wp = self.mission.waypoints[i]
                # print(f"Lat: {lat:.6f}, Lon: {lon:.6f}")
                x, y = self.scale_points(wp[0], wp[1])
                if old_x is not None and old_y is not None:
                    img = cv2.line(img, (old_x, old_y), (x, y), (0, 255, 0), 2)
                img = cv2.circle(img, (x, y), 10, (0, 255, 0), -1)
                img = cv2.putText(img, str(i), (x+10, y-10),
                                  cv2.FONT_HERSHEY_SIMPLEX, 1, (200, 200, 200), 2)
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
            overlay = img.copy()
            for route in self.graph:
                for neigh in self.graph[route]["neigh"]:
                    lat, long = self.graph[route]["latlon"]
                    lat2, long2 = self.graph[neigh]["latlon"]
                    x, y = self.scale_points(lat, long)
                    x2, y2 = self.scale_points(lat2, long2)
                    cv2.line(overlay, (x, y), (x2, y2), (0, 255, 0), 2)
            alpha = 0.25
            result = cv2.addWeighted(overlay, alpha, img, 1 - alpha, 0)
            img = result

            for route in self.graph:
                for neigh in self.graph[route]["neigh"]:
                    lat, long = self.graph[route]["latlon"]
                    lat2, long2 = self.graph[neigh]["latlon"]
                    x, y = self.scale_points(lat, long)
                img = cv2.circle(img, (x, y), 10, (100, 255, 0), -1)
                # img = cv2.putText(img, str(route), (x, y),
                #                   cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

        if plan:
            with self.lock:
                if self.last_end is not None and self.last_start is not None \
                        and self.last_path is not None:
                    last_x_end, last_y_end = self.scale_points(self.last_end[0],
                                                               self.last_end[1])
                    img = cv2.circle(img, (last_x_end, last_y_end),
                                     10, (128, 255, 255), -1)

                    last_x_start, last_y_start = self.scale_points(self.last_start[0],
                                                                   self.last_start[1])
                    img = cv2.circle(img, (last_x_start, last_y_start),
                                     10, (0, 255, 255), -1)

                    for i, node in enumerate(self.last_path):
                        lat, long = self.graph[node]["latlon"]
                        x, y = self.scale_points(lat, long)
                        img = cv2.circle(img, (x, y), 10, (0, 255, 255), -1)
                        if i > 0:
                            lat, long = self.graph[self.last_path[i-1]]["latlon"]
                            x2, y2 = self.scale_points(lat, long)
                            cv2.line(img, (x, y), (x2, y2), (0, 255, 255), 3)
                        if i == 0:
                            cv2.line(img, (x, y),
                                     (last_x_start, last_y_start),
                                     (0, 255, 255), 3)
                    # Plot a line connecting the end and the last node
                    # cv2.line(img, (x, y), (last_x_end, last_y_end),
                    #          (0, 255, 255), 3)
            for route in self.graph:
                for neigh in self.graph[route]["neigh"]:
                    lat, long = self.graph[route]["latlon"]
                    lat2, long2 = self.graph[neigh]["latlon"]
                    x, y = self.scale_points(lat, long)
                # img = cv2.putText(img, str(route), (x, y),
                #                   cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

        # print the max edge length and map at the top
        cv2.putText(img, f"Max Edge Length: {self.max_edge_length}m",
                    (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        if not get_image:
            cv2.namedWindow('Pennovation', cv2.WINDOW_NORMAL)
            cv2.imshow('Pennovation', img)
            cv2.waitKey(0)
        else:
            # make img rgb
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            return img


if __name__ == "__main__":
    # read program arguments
    parser = argparse.ArgumentParser(
            prog=f"{os.path.basename(__file__)}",
            description='Read a yaml file and plot waypoints')
    parser.add_argument('--map_name',
                        help='Map name to use from semantics_manager',
                        required=True)
    parser.add_argument('--max_edge_length',
                        help='Max edge length for the graph',
                        required=False, default=100, type=int)
    args = parser.parse_args()

    import rospkg
    rospack = rospkg.RosPack()
    semantics_path = rospack.get_path('semantics_manager')
    map_path = os.path.join(semantics_path, "maps", args.map_name, "map_config.yaml")
    max_edge_length = args.max_edge_length

    print(f"Map path: {map_path}")
    print(f"Max edge length: {args.max_edge_length}")

    # Create a path planner object
    q = Path_planner(map_path, max_edge_length)
    q.display_points(waypoints=True, noFly=True, origin=True)

    i = 0
    for j in range(100):
        start_latlon = [random.uniform(-130, 130),
                        random.uniform(-100, 100)]
        end_latlon = [random.uniform(-130, 130),
                      random.uniform(-100, 100)]
        print(f"Start: {start_latlon} - End: {end_latlon}")
        # Check if start or end are within no fly zones
        route = q.planRoute(start_latlon, end_latlon)
        if route is not None:
            q.display_points(noFly=True, routes=True, plan=True)
            i += 1
            if i == 3:
                break
