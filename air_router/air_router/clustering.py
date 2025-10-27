import numpy as np
import math

class Clustering():
    def __init__(self):
        pass 

    def kmeans(self, k, points):
        # Initialize centroids to random waypoint locations
        centroids = points[np.random.choice(points.shape[0], size=k, replace=False)]

        # Book keeping variables
        iteration_count = 0
        regression_count = 0
        previous_shift = 10000000000.0

        # What we haven't converged onto a solution...
        while regression_count < 3 and iteration_count < k:
            # Create empty clusters
            clusters = [[] for _ in range(k)]

            # For each each waypoint i
            for i in range(len(points)):
                # Guess that i belongs to cluster 0
                closestIndex = 0
                midDist = self.distance(points[i], centroids[0])

                # Search for closest centroid
                for j in range(len(centroids)):
                    dist = self.distance(points[i], centroids[j])
                    # Is this a closer centroid?
                    if dist < midDist:
                        # Found better fit
                        midDist = dist
                        closestIndex = j

                # Add point to cluster of closest centroid
                clusters[closestIndex].append(points[i].tolist())

            # Recalculate centroids as the mean of each cluster
            newCentroids = []
            largest_shift = 0
            for i in range(k):
                newCentroid = self.calculateCentroid(clusters[i])
                newCentroids.append(newCentroid)
                shift_dist = self.distance(newCentroid, centroids[i])
                # Is this the biggest shift?
                if largest_shift < shift_dist:
                    largest_shift = shift_dist

            # Save our updated cluster points
            centroids = newCentroids

            # Did we make meaningful prgress?
            if largest_shift > previous_shift:
                # No... noted
                regression_count += 1
            previous_shift = largest_shift

            print(f"Created new clusters, largest shift = {largest_shift}")
            iteration_count += 1

        # If we made it this far... then we converged!
        print(f"Clustering converged after {iteration_count} iterations!")
        return centroids


    def distance(self, point_a, point_b):
        return math.sqrt((point_a[0] - point_b[0])**2 + (point_a[1] - point_b[1])**2)

    def calculateCentroid(self, cluster):
        sum_x = sum(point[0] for point in cluster)
        sum_y = sum(point[1] for point in cluster)
        new_x = sum_x / len(cluster)
        new_y = sum_y / len(cluster)
        return [new_x, new_y]

