import numpy as np
import math

class Clustering():
    def __init__(self):
        pass 
    
    def kmeans(self, k, points):
        # Initialize centroids
        # centroids ← list of k starting centroids
        centroids = points[np.random.choice(points.shape[0], size=k, replace=False)]

        # converged ← false
        converged = False

        # while converged == false do
        count = 0
        while not converged and count < 20:
            # clusters ← list of k empty lists
            clusters = [[] for _ in range(k)]

            # for i ← 0 to length(points) - 1 do
            for i in range(len(points)):
                # Guess that i belongs to cluster 0
                closestIndex = 0
                midDist = self.distance(points[i], centroids[0])
                # Search for closest cluster
                for j in range(len(centroids)):
                    dist = self.distance(points[i], centroids[j])
                    if dist < midDist:
                        # Found better fit
                        midDist = dist
                        closestIndex = j
                # Add point to closest cluster
                clusters[closestIndex].append(points[i].tolist())

            # Recalculate centroids as the mean of each cluster
            newCentroids = []
            converged = True
            largest_shift = 0
            for i in range(k):
                newCentroid = self.calculateCentroid(clusters[i])
                newCentroids.append(newCentroid)
                shift_dist = self.distance(newCentroid, centroids[i])
                if shift_dist > 3.0:
                    converged = False
                if largest_shift < shift_dist:
                    largest_shift = shift_dist
            centroids = newCentroids
            print(f"Created new clusters, largest shift = {largest_shift}")
            count += 1

        # If we made it this far... then we converged!
        print("Clustering converged!")
        return centroids
    

    def distance(self, point_a, point_b):
        return math.sqrt((point_a[0] - point_b[0])**2 + (point_a[1] - point_b[1])**2)

    def calculateCentroid(self, cluster):
        sum_x = sum(point[0] for point in cluster)
        sum_y = sum(point[1] for point in cluster)
        new_x = sum_x / len(cluster)
        new_y = sum_y / len(cluster)
        return [new_x, new_y]

