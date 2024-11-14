from controller import Supervisor
import random
import time
import math
import sys
import os

class inputKeep(Supervisor):
    def __init__(self):
        super(inputKeep, self).__init__()
        self.time_step = int(self.getBasicTimeStep())
        current_dir = os.path.dirname(os.path.abspath(__file__))
        self.cars = []
        self.initialize_car()
    def import_variables(self):
        data = []

#        else:
#            data.append("False")
        robot_def_name = "redEnd"
        robot_node = self.getFromDef(robot_def_name)
        custom_data_field = robot_node.getField('customData')
        
        if custom_data_field is not None:
            data.append(custom_data_field.getSFString())
        else:
            data.append("-1")
        return data
    def initialize_cars(self):
        """
        Dynamically collects all road segments with DEF names like Road_0, Road_1, etc.
        """
        car_type = "car"
        index = 0
        while True:
            car_def = f"{car_type}_{index}"
            car = self.getFromDef(car_def)
            if car:
                print(car_def)
                self.cars.append(car)
                index += 1
            else:
                break      
    def initialize_car(self):
        car_def = "car"
        car = self.getFromDef(car_def)
        self.cars.append(car)
        

    def run(self):
        while self.step(self.time_step) != -1:
            self.import_variables()

            

# Create and run the controller
controller = inputKeep()
controller.run()
