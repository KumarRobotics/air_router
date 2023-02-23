#!/usr/bin/env python3
import os
import pdb
import argparse
import cv2
import sys
import yaml
import pprint
import utm
import numpy as np

class QGC():
    def __init__(self, filename, image):
        # Check that the file is a proper QGC file
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
        self.noFly = [i for i in yml["geoFence"]["polygons"] if not i["inclusion"]]
        # self.origin_lat, self.origin_long, _ = self.mission['plannedHomePosition']

        img_lat, img_long = [39.943269, -75.202298]
        [self.img_utm_east, self.img_utm_north, _, _] = utm.from_latlon(img_lat, img_long)
        img_bottom_lat, img_bottom_long = [39.939762, -75.198247]
        [img_bottom_utm_east, img_bottom_utm_north, _, _] = utm.from_latlon(img_bottom_lat,
                                                                      img_bottom_long)

        # Display pennovation picture
        img = cv2.imread(image)
        # Rotate 180 deg
        img = cv2.rotate(img, cv2.ROTATE_180)
        # Flip horizontally
        img = cv2.flip(img, 1)
        # scale the image to 1/4th of its original size
        self.img = cv2.resize(img, (0,0), fx=0.25, fy=0.25)
        self.scale_east = self.img.shape[1]/(img_bottom_utm_east - self.img_utm_east)
        self.scale_north = self.img.shape[0]/(img_bottom_utm_north - self.img_utm_north)

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

    def generateRoutes(self):
        # create a set with all the possible rally points
        points = {}
        for i, p in enumerate(self.rally):
            points[i] = {"latlon": p[:2], "neigh": set([j for j in
                                                    range(len(self.rally)) if
                                                    j != i])}

        # Remove points that are too far away from each other
        for i in points:
            for j in points[i]["neigh"].copy():
                d = self.getDistance(points[i]["latlon"], points[j]["latlon"])
                if d > 100:
                    print(f"Removing {j} from {i} because distance is {d:.2f}m")
                    points[i]["neigh"].remove(j)

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
                    print(f"Removing {j} from {i} because it intersects with noFly zone")
                    points[i]["neigh"].remove(j)
        self.routes = points


    def display_points(self, mission=False, noFly=False, rally=False,
                       routes=False):
        old_x = None
        old_y = None
        img = self.img.copy()
        if mission:
            for i, item in enumerate(self.mission):
                if not "params" in item.keys():
                    continue
                lat = item['params'][4]
                lon = item['params'][5]
                # check if lat or lon are None
                if lat is None or lon is None or lat == 0 or lon == 0:
                    continue
                # print(f"Lat: {lat:.6f}, Lon: {lon:.6f}")
                x, y = self.scale_points(lat, lon)
                img = cv2.circle(img, (x, y), 2, (0, 0, 255), -1)
                # img = cv2.putText(img, str(i), (x, y),
                #                   cv2.FONT_HERSHEY_SIMPLEX, .25, (0, 0, 255))

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
            for route in self.routes:
                for neigh in self.routes[route]["neigh"]:
                    lat, long = self.routes[route]["latlon"]
                    lat2, long2 = self.routes[neigh]["latlon"]
                    x, y = self.scale_points(lat, long)
                    x2, y2 = self.scale_points(lat2, long2)
                    cv2.line(img, (x, y), (x2, y2), (255, 255, 0), 1)


        cv2.imshow('Pennovation', img)
        cv2.waitKey(0)


if __name__ == "__main__":
    # read program arguments
    parser = argparse.ArgumentParser(
            prog = f"{os.path.basename(__file__)}",
            description='Read a yaml file and plot waypoints')
    parser.add_argument('--filename', help='YAML file to read')
    args = parser.parse_args()

    # Check if the file exists
    if not os.path.isfile(args.filename):
        sys.exit(f"Error: {args.filename} does not exist")
    q = QGC(args.filename, "../images/pennovation2.png")
    q.generateRoutes()
    q.display_points(noFly=True, rally=True, routes=True)


