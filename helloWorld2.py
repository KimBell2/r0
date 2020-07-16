from DrivingInterface.drive_controller import DrivingController
import time
import numpy as np
from math import *

class DrivingClient(DrivingController):
    st = time.time()
    ed = time.time()


    def __init__(self):
        # =========================================================== #
        #  Area for member variables =============================== #
        # =========================================================== #
        # Editing area starts from here
        #
        self.is_debug = False
        self.limited_speed = 300
        self.index = 1
        self.err = 0
        self.steer=0
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
            
            print("=========================================================")

        ###########################################################################

        # Moving straight forward
        # car_controls.steering = 0
        # car_controls.throttle = 1
        # car_controls.brake = 0

        # 0으로 나누기 주의
        # 인접한 리스트의 차이가 클 때 속도 변경 필요

        self.limited_speed = self.get_max_speed(sensing_info)

        if sensing_info.speed<self.limited_speed:
            car_controls.throttle=1
        if sensing_info.speed>=self.limited_speed+15:
            car_controls.brake=1
        if sensing_info.speed>self.limited_speed+15:
            car_controls.throttle = -1
            

        # if sensing_info.speed > 120:
        #     cur_speed=2
        # elif sensing_info.speed > 80:
        #     cur_speed=1
        # else:
        #     cur_speed=0

        # 속도가 너무 빠를 때 핸들 어떻게 꺾을지
        #-1 = 자동차 폭 2m

        
        dx=sensing_info.speed*1.3
        dy=sensing_info.to_middle*-1

        if max(np.abs(sensing_info.track_forward_angles))>30:
            arrNum = np.where(np.abs(sensing_info.track_forward_angles)>30)[0][0]
            dx = max(arrNum*10,0)
            dy = (self.half_road_limit-1)*np.sign(sensing_info.track_forward_angles[arrNum])*-1-sensing_info.to_middle
        
        # print(sensing_info.track_forward_angles)
        #print('분산 : ', np.var(sensing_info.track_forward_angles))
        #print('표준편차 : ', np.std(sensing_info.track_forward_angles))
        #print('angle : ', sensing_info.track_forward_angles)

        #dx = sensing_info.speed*1.3-x
        #dy = sensing_info.to_middle*-1-y


        # plusAngle = 0
        # if np.where(abs(self.gapForwardAngle[:self.index])>15)[0].size == 0:
        #     dx = sensing_info.speed
        #     dy = sensing_info.to_middle*-1
        # else:
        #     dist = np.where(abs(self.gapForwardAngle[:self.index])>15)[0][0]
        #     if abs(sensing_info.track_forward_angles[cur_speed]) > 15:
        #         plusAngle = sensing_info.track_forward_angles[cur_speed]
        #     curveAngle = self.gapForwardAngle[dist]
        #     dist = (dist+1)*10-2
        #     dy = (np.sign(curveAngle)*(self.half_road_limit-1)-sensing_info.to_middle)
        #     dx = dist
        
        angle = sensing_info.moving_angle*-1
        angle = angle+(atan(dy/dx)*(180/pi))
        angle = angle+sensing_info.track_forward_angles[3]
        #angle = angle-sensing_info.moving_angle+plusAngle
        angle = min(abs(angle),50)*np.sign(angle)
        # 도로 범위 안에 있는 경우
        #if self.half_road_limit-abs(sensing_info.to_middle) > 0
        #    angle = angle*(max(abs(np.sign(sensing_info.to_middle) + np.sign(angle)),1)/2
        print("angle : ",angle)

        self.err = angle-self.steer
        self.err = 1/max(self.err,1)

        cal_steering = angle*self.err*0.02
        self.steer = angle

        if sensing_info.speed>100:
            cal_steering = cal_steering/2
        elif sensing_info.speed>150:
            cal_steering = cal_steering/3
        elif sensing_info.speed>200:
            cal_steering = cal_steering/4

        car_controls.steering = cal_steering


        
        if self.is_debug:
            print("[MyCar] steering:{}, throttle:{}, brake:{}".format(car_controls.steering, car_controls.throttle, car_controls.brake))
            print("limited_speed : ",self.limited_speed)
            print("cal_steering : ",cal_steering)
            print("index : ",self.index)
            #self.is_debug = False

        self.ed = time.time()

        # print('middle : ' , sensing_info.to_middle)
        # print('angle : ' , angle)
        # print('sensing_info.moving_angle : ', sensing_info.moving_angle)
        # print('steering : ' , cal_steering)

        #
        # Editing area ends
        # ==========================================================#
        #return car_controls
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

    def get_max_speed(self, sensing_info):
        self.gapForwardAngle = np.array(sensing_info.moving_angle)
        self.gapForwardAngle = np.hstack((self.gapForwardAngle, sensing_info.track_forward_angles[:9]))
        self.gapForwardAngle = sensing_info.track_forward_angles - np.array(self.gapForwardAngle)
        self.index=1
        temp = max(sensing_info.speed-70,0)

        if temp >=100:
            self.index = 10
        else:
            self.index = int(temp/10)+1
        
        max_angle = np.max(np.abs(self.gapForwardAngle[:self.index]))

        #max_angle = np.max(np.abs(sensing_info.track_forward_angles[:self.index]))

        if max_angle <= 6:
            max_speed=250
        else:
            max_speed=max(180-(max_angle*2),65)

        return max_speed

    


if __name__ == '__main__':
    print("[MyCar] Start Bot!")
    client = DrivingClient()
    return_code = client.run()
    print("[MyCar] End Bot!")

    exit(return_code)
