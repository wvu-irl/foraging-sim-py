import numpy as np

class PrevExpData:
    def allocate(self, num_trials, num_robots, num_time_steps, robot_id, personality):
        self.x = np.empty((num_trials, num_robots, num_time_steps), dtype=np.int)
        self.y = np.empty_like(self.x)
        self.has_food = np.empty_like(self.x, dtype=np.bool)
        self.battery = np.empty_like(self.x)
        self.last_successful_food_x = np.empty_like(self.x)
        self.last_successful_food_y = np.empty_like(self.x)
        self.last_failed_food_x = np.empty_like(self.x)
        self.last_failed_food_y = np.empty_like(self.x)
        self.last_approach_dir = np.empty_like(self.x)
        if len(robot_id) != num_robots:
            raise RuntimeError("robot_id list not same length as num_robots in PrevExpData")
        elif len(personality) != num_robots:
            raise RuntimeError("personality list not same length as num_robots in PrevExpData")
        self.robot_id = np.array(robot_id, dtype=np.int)
        self.personality = np.array(personality, dtype=np.int)
        self.last_trial_written = np.array([-1], dtype=np.int)
 
    def record(self, obj, t):
        i = obj.trial_num
        for j in range(obj.num_robots):
            self.x[i, j, t] = obj.robot[j].states.x
            self.y[i, j, t] = obj.robot[j].states.y
            self.has_food[i, j, t] = obj.robot[j].states.has_food
            self.battery[i, j, t] = obj.robot[j].states.battery
            self.last_successful_food_x[i, j, t] = obj.robot[j].states.last_successful_food_x
            self.last_successful_food_y[i, j, t] = obj.robot[j].states.last_successful_food_y
            self.last_failed_food_x[i, j, t] = obj.robot[j].states.last_failed_food_x
            self.last_failed_food_y[i, j, t] = obj.robot[j].states.last_failed_food_y
            self.last_approach_dir[i, j, t] = obj.robot[j].states.last_approach_dir

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
                last_approach_dir = self.last_approach_dir,\
                robot_id = self.robot_id,\
                personality = self.personality,\
                last_trial_written = self.last_trial_written)

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
        self.robot_id = data["robot_id"]
        self.personality = data["personality"]
        self.last_trial_written = data["last_trial_written"]
