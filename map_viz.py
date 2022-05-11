import numpy as np
from foraging_map import MapLayer

def displayMap(obj, plt, fig, ax):
    img_shape = obj.map.map_shape + (3,)
    img = np.ones(img_shape, dtype=np.uint8) * 255
    for x in range(obj.map.map_shape[0]):
        for y in range(obj.map.map_shape[1]):
            if obj.map.map[MapLayer.ROBOT, x, y] > 0: # Mark robot locations with color corresponding to personality type
                robot_id = obj.map.map[MapLayer.ROBOT, x, y] - 1
                robot_personality = obj.robot_personality_list[robot_id]
                if obj.true_robot_states[robot_id].battery < 1e-3: # If robot battery is dead, make dark gray
                    robot_color = [100, 100, 100]  # Gray
                #elif robot_personality == 0:
                #    robot_color = [0, 0, 155]  # Dark Blue
                elif robot_personality in [0, 1, 4, 7, 10]:
                    robot_color = [255, 0, 0]   # Red
                elif robot_personality in [2, 5, 8, 11]:
                    robot_color = [0, 0, 255]   # Blue
                elif robot_personality in [3, 6, 9, 12]:
                    robot_color = [255, 0, 255] # Purple
                #elif robot_personality == 4:
                #    #robot_color = [255, 128, 0] # Orange
                #    robot_color = [255, 0, 0]   # Red
                #elif robot_personality == 5:
                #    #robot_color = [150, 0, 255] # Violet
                #    robot_color = [0, 0, 255]   # Blue
                #elif robot_personality == 6:
                #    #robot_color = [60, 30, 0]   # Brown
                #    robot_color = [255, 0, 255] # Purple
                #elif robot_personality == 7:
                #    robot_color = [86, 0, 45]   # Burgandy
                #elif robot_personality == 8:
                #    robot_color = [255, 200, 0] # Yellow
                img[x, y, :] = np.array(robot_color, dtype=np.uint8)
            elif obj.map.map[MapLayer.OBSTACLE, x, y] > 0: # Mark obstacle locations as black
                img[x, y, :] = np.array([0, 0, 0], dtype=np.uint8)
            elif obj.map.map[MapLayer.FOOD, x, y] == 1: # Mark food with heading 1 as bright green
                img[x, y, :] = np.array([0, 255, 0], dtype=np.uint8)
            elif obj.map.map[MapLayer.FOOD, x, y] == 3: # Mark food with heading 3 as medium-bright green
                img[x, y, :] = np.array([0, 155, 0], dtype=np.uint8)
            elif obj.map.map[MapLayer.FOOD, x, y] == 5: # Mark food with heading 5 as medium-dark green
                img[x, y, :] = np.array([0, 100, 0], dtype=np.uint8)
            elif obj.map.map[MapLayer.FOOD, x, y] == 7: # Mark food with heading 7 as dark green
                img[x, y, :] = np.array([0, 55, 0], dtype=np.uint8)
            elif obj.map.map[MapLayer.HOME, x, y] > 0: # Mark home locations as black
                img[x, y, :] = np.array([0, 0, 0], dtype=np.uint8)

    ax.cla()
    ax.imshow(np.swapaxes(img, 0, 1), origin='lower')
    #ax.imshow(img, origin='lower')
    ax.set_xticks(np.arange(0, img_shape[0], 1))
    ax.set_yticks(np.arange(0, img_shape[1], 1))
    ax.set_xticks(np.arange(-0.5, img_shape[0], 1), minor=True)
    ax.set_yticks(np.arange(-0.5, img_shape[1], 1), minor=True) 
    ax.grid(which='minor', color='k', linestyle='-', linewidth=1)
    plt.pause(0.001)
