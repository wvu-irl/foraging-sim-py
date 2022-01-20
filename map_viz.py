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
            elif map_obj.map[MapLayer.FOOD, x, y] == 1: # Mark food with heading 1 as bright green
                img[x, y, :] = np.array([0, 255, 0], dtype=np.uint8)
            elif map_obj.map[MapLayer.FOOD, x, y] == 3: # Mark food with heading 3 as medium-bright green
                img[x, y, :] = np.array([0, 155, 0], dtype=np.uint8)
            elif map_obj.map[MapLayer.FOOD, x, y] == 5: # Mark food with heading 5 as medium-dark green
                img[x, y, :] = np.array([0, 100, 0], dtype=np.uint8)
            elif map_obj.map[MapLayer.FOOD, x, y] == 7: # Mark food with heading 7 as dark green
                img[x, y, :] = np.array([0, 55, 0], dtype=np.uint8)
            elif map_obj.map[MapLayer.HOME, x, y] > 0: # Mark home locations as blue
                img[x, y, :] = np.array([0, 0, 255], dtype=np.uint8)

    ax.cla()
    ax.imshow(np.swapaxes(img, 0, 1), origin='lower')
    plt.pause(0.001)
