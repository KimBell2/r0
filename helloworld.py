from DrivingInterface.drive_controller import DrivingController
import time
import numpy as np

class DrivingClient(DrivingController):
    st = time.time()
    ed = time.time()
    limited_speed=200

    def __init__(self):
        # =========================================================== #
        #  Area for member variables =============================== #
        # =========================================================== #
        # Editing area starts from here
        #

        self.is_debug = True

        #
        # Editing area ends
        # ==========================================================#
        super().__init__()
    
    def control_driving(self, car_controls, sensing_info):
        gap = self.ed - self.st

        self.st = time.time()

        # =========================================================== #
        # Area for writing code about driving rule ================= #
        # =========================================================== #
        # Editing area starts from here
        #
        
        if self.is_debug:
            print("=========================================================")
            print("[MyCar] to middle: {}".format(sensing_info.to_middle))

                #print("[MyCar] collided: {}".format(sensing_info.collided))
            print("[MyCar] car speed: {} km/h".format(sensing_info.speed))

            print("[MyCar] is moving forward: {}".format(sensing_info.moving_forward))
            print("[MyCar] moving angle: {}".format(sensing_info.moving_angle))
            #print("[MyCar] lap_progress: {}".format(sensing_info.lap_progress))

            print("[MyCar] track_forward_angles: {}".format(sensing_info.track_forward_angles))
            #print("[MyCar] track_forward_obstacles: {}".format(sensing_info.track_forward_obstacles))
            #print("[MyCar] opponent_cars_info: {}".format(sensing_info.opponent_cars_info))
            #print("[MyCar] distance_to_way_points: {}".format(sensing_info.distance_to_way_points))
            print("gap : ", gap)
            print("width : ", (self.half_road_limit-1.25)*2)
            
            print("=========================================================")

        ###########################################################################

        if sensing_info.speed<=90:
            car_controls.throttle=1
        if sensing_info.speed>=90:
            car_controls.throttle=0
            
        cal_steering = ((sensing_info.moving_angle*0.02)*-1)+((sensing_info.to_middle*0.02)*-1)
        cal_steering = cal_steering + (cal_steering*(sensing_info.speed/180)/2)*-1
        
        if abs(cal_steering) >= 1:
            cal_steering = cal_steering/abs(cal_steering)

        car_controls.steering = cal_steering
        
        if self.is_debug:
            print("[MyCar] steering:{}, throttle:{}, brake:{}".format(car_controls.steering, car_controls.throttle, car_controls.brake))
            print("limited_speed : ",self.limited_speed)
            print("cal_steering : ",cal_steering)
            #self.is_debug = False


        self.ed = time.time()

        #
        # Editing area ends
        # ==========================================================#
        return car_controls


    # ============================
    # If you have NOT changed the <settings.json> file
    # ===> player_name = ""
    #
    # If you changed the <settings.json> file
    # ===> player_name = "My car name" (specified in the json file)  ex) Car1
    # ============================
    def set_player_name(self):
        player_name = ""
        return player_name


if __name__ == '__main__':
    print("[MyCar] Start Bot!")
    client = DrivingClient()
    return_code = client.run()
    print("[MyCar] End Bot!")

    exit(return_code)
