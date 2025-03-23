import numpy as np
import matplotlib.pyplot as plt

def plot_data():
    points = np.load('points.npy')
    segments = np.load('segments.npy')

    # Plot points
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    point_x = points[:, 0]
    point_y = points[:, 1]
    ax.scatter(point_x, point_y)

    plt.figure()
    plt.plot(segments[:, 0])
    plt.xlabel('Temps')
    plt.ylabel('Longueur du segment')

    plt.show()

if __name__ == '__main__':
    plot_data()
