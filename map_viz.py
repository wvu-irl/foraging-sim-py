import numpy as np
from foraging_map import MapLayer

def displayMap(map_obj, plt, fig, ax):
    img_shape = map_obj.map_shape + (3,)
    img = np.ones(img_shape, dtype=np.uint8) * 255
    for x in range(map_obj.map_shape[0]):
        for y in range(map_obj.map_shape[1]):
            if map_obj.map[MapLayer.ROBOT, x, y] > 0: # Mark robot locations as red
                img[x, y, :] = np.array([255, 0, 0], dtype=np.uint8)
            elif map_obj.map[MapLayer.OBSTACLE, x, y] > 0: # Mark obstacle locations as black
                img[x, y, :] = np.array([0, 0, 0], dtype=np.uint8)
            elif map_obj.map[MapLayer.FOOD, x, y] > 0: # Mark food locations as green
                img[x, y, :] = np.array([0, 255, 0], dtype=np.uint8)
            elif map_obj.map[MapLayer.HOME, x, y] > 0: # Mark home locations as blue
                img[x, y, :] = np.array([0, 0, 255], dtype=np.unit8)

    ax.cla()
    ax.imshow(img)
    plt.pause(0.001)
