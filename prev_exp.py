import numpy as np

class PrevExpData:
    def allocate(self, num_trials, num_robots, num_time_steps):
        self.x = np.empty((num_trials, num_robots, num_time_steps), dtype=np.int)
        self.y = np.empty_like(self.x)
        self.has_food = np.empty_like(self.x, dtype=np.bool)
        self.battery = np.empty_like(self.x)
        self.last_successful_food_x = np.empty_like(self.x)
        self.last_successful_food_y = np.empty_like(self.x)
        self.last_failed_food_x = np.empty_like(self.x)
        self.last_failed_food_y = np.empty_like(self.x)
        self.last_approach_dir = np.empty_like(self.x)
   
    def record(self, obj, t):
        i = obj.trial_num
        for j in range(obj.num_robots):
            self.x[i, j, t] = obj.robot[j].states.x
            self.y[i, j, t] = obj.robot[j].states.y
            self.has_food[i, j, k] = obj.robot[j].states.has_food
            self.battery[i, j, k] = obj.robot[j].states.battery
            self.last_successful_food_x[i, j, k] = obj.robot[j].states.last_successful_food_x
            self.last_successful_food_y[i, j, k] = obj.robot[j].states.last_successful_food_y
            self.last_failed_food_x[i, j, k] = obj.robot[j].states.last_failed_food_x
            self.last_failed_food_y[i, j, k] = obj.robot[j].states.last_failed_food_y
            self.last_approach_dir[i, j, k] = obj.robot[j].states.last_approach_dir

    def save(self, filename):
        np.savez(filename,\
                x = self.x,\
                y = self.y,\
                has_food = self.has_food,\
                battery = self.battery,\
                last_successful_food_x = self.last_successful_food_x,\
                last_successful_food_y = self.last_successful_food_y,\
                last_failed_food_x = self.last_failed_food_x,\
                last_failed_food_y = self.last_failed_food_y,\
                last_approach_dir = self.last_approach_dir)

    def load(self, filename):
        data = np.load(filename)
        self.x = data["x"]
        self.y = data["y"]
        self.has_food = data["has_food"]
        self.battery = data["battery"]
        self.last_successful_food_x = data["last_successful_food_x"]
        self.last_successful_food_y = data["last_successful_food_y"]
        self.last_failed_food_x = data["last_failed_food_x"]
        self.last_failed_food_y = data["last_failed_food_y"]
        self.last_approach_dir = data["last_approach_dir"]
