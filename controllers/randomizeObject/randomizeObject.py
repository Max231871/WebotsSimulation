from controller import Supervisor
import random
import time
import math

class RoadRandomizer(Supervisor):
    def __init__(self):
        super(RoadRandomizer, self).__init__()
        self.time_step = int(self.getBasicTimeStep())
        self.roads = []
        
        # DEF names for trigger objects
        self.teleportation_trigger_def_1 = "redEnd"  # Object that triggers teleportation
        self.teleportation_trigger_def_2 = "car"     # Object that must contact TriggerObject1
        
        # DEF names of objects to teleport upon contact
        self.teleport_objects = ["redEnd", "greenStart", "car"]
        
        # Initialize roads and trigger objects
        self.initialize_roads()
        self.object1 = self.getFromDef(self.teleportation_trigger_def_1)
        self.object2 = self.getFromDef(self.teleportation_trigger_def_2)
        
        # Cooldown to prevent multiple teleportations
        self.teleport_cooldown = 0
        self.cooldown_steps = 60  # e.g., 60 steps = ~2 seconds if time_step=32ms

    def initialize_roads(self):
        """
        Dynamically collects all road segments with DEF names like Road_0, Road_1, etc.
        """
        road_type = "Road"
        index = 0
        while True:
            road_def = f"{road_type}_{index}"
            road = self.getFromDef(road_def)
            if road:
                self.roads.append(road)
                index += 1
            else:
                break

    def get_random_road_position(self):
        """
        Selects a random road segment and returns its translation position.
        """
        if not self.roads:
            return [0, 0, 0]  # Default position to avoid crashing
        selected_road = random.choice(self.roads)
        road_position = selected_road.getField("translation").getSFVec3f()
        return road_position

    def teleport_object(self, object_def, position):
        """
        Teleports a specific object to the given position.
        """
        obj = self.getFromDef(object_def)
        if obj and obj != self.getFromDef("car"):
            obj.getField("translation").setSFVec3f(position)

    def get_position(self, obj):
        """
        Retrieves the translation position of a given object.
        """
        return obj.getField("translation").getSFVec3f()

    def check_proximity(self, pos1, pos2, threshold=3):
        """
        Checks if two positions are within a specified threshold distance.
        """
        distance = ((pos1[0] - pos2[0]) ** 2 + 
                    (pos1[1] - pos2[1]) ** 2 + 
                    (pos1[2] - pos2[2]) ** 2) ** 0.5
        return distance < threshold, distance
    def rotation_readjustment(self):
        car = self.getFromDef("car")

        car_position = car.getField("translation").getSFVec3f()

        # Get the new position of redEnd after teleportation
        red_end = self.getFromDef("redEnd")
        red_end_position = red_end.getField("translation").getSFVec3f()

        # Compute the direction vector from the car to redEnd in the x-z plane
        dir_x = red_end_position[0] - car_position[0]
        dir_y = red_end_position[1] - car_position[1]

        # Calculate the yaw angle required to face redEnd
        yaw_angle = math.atan2(dir_y, dir_x)  # Angle in radians

        # Set the car's rotation to face redEnd
        # Rotate only around the y-axis by changing only the angle (4th parameter)
        car.getField("rotation").setSFRotation([0, 0, 1, yaw_angle])

    def run(self):
        """
        Main loop that checks for contact and teleports objects when contact is detected.
        """
        time_initial = time.time()   # Collect initial time to be used as t=0
        rotation_cooldown = time.time()
        while self.step(self.time_step) != -1:
            if self.object1 and self.object2:
                pos1 = self.get_position(self.object1)
                pos2 = self.get_position(self.object2)
                distanceAway = [0,0]
                distanceAway[0] = pos2[0]-pos1[0]
                distanceAway[1] = pos2[1]-pos1[1]
                #print(f"Distance from goal: [{distanceAway[0]:.2f}, {distanceAway[1]:.2f}] meters")

                # Check proximity and get distance
                in_proximity, distance = self.check_proximity(pos1, pos2, threshold=5)
                if time.time()-rotation_cooldown > 2:
                   # self.rotation_readjustment()
                    rotation_cooldown = time.time()
                if in_proximity:
                    elapsed_time = round((time.time() - time_initial), 2)  # Calculate time it took to reach destination
                    print(f"It took {elapsed_time}s to reach destination.")
                    time_initial = time.time()

                    # Reset the car's velocities to stop it
                    car = self.getFromDef("car")
                    car.resetPhysics()
                    
                    if self.teleport_cooldown == 0:
                        # Teleport objects
                        for obj_def in self.teleport_objects:
                            new_position = self.get_random_road_position()
                            self.teleport_object(obj_def, new_position)
                        green_start = self.getFromDef("greenStart")

                        
                        car_position = green_start.getField("translation").getSFVec3f()
                        car.getField("translation").setSFVec3f(car_position)
                       # self.rotation_readjustment()
                        while get_position(green_start) == get_position(self.getFromDef('redEnd')):
                            new_position = self.get_random_road_position()
                            self.teleport_object('greenStart', new_position)
                            self.teleport_object('car', new_position)
                        self.teleport_cooldown = self.cooldown_steps  # Start cooldown

                else:
                    if self.teleport_cooldown > 0:
                        self.teleport_cooldown -= 1
            else:
                pass

# Create and run the controller
controller = RoadRandomizer()
controller.run()
