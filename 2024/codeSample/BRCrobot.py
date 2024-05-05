from controller import Robot, Motor, DistanceSensor, Camera, Emitter, GPS, PositionSensor, Gyro, InertialUnit, Lidar, Accelerometer
import cv2 as cv
import numpy as np 
import logging
import json
import time
import matplotlib.pyplot as plt

DEFAULT_SPEED = 2
OBSTACLE_THRESH = 0.15
TILE_SIZE = 0.12

class Colors:
    HOLE = (67, 67, 67)
    SWAMP = (255, 255, 0)
    A1 = (255, 0, 0)
    A2 = (0, 255, 0)
    A3 = (0, 0, 255)

class RobotPosition:
    x = TILE_SIZE/2*10**2
    y = TILE_SIZE/2*10**2
    theta = 0
    def __str__(self):
        return f"x: {self.x}, y: {self.y}, theta: {self.theta}"
    
    def __repr__(self):
        return f"x: {self.x}, y: {self.y}, theta: {self.theta}"
    
class BRC:
    def __init__(self, robotConfig:str="default"):
        self.logger = logging.getLogger(__name__)
        self.logger.setLevel(logging.DEBUG)
        self.logger.info("Initializing BRC")        
        self._robot = Robot()
        self.position = RobotPosition()
        self.colors = Colors()
        self.timeStep = 32
        self.sensorValues = {}
        self.victim = False
        self.map = []
        self.available_sensors = {
                "Distance Sensor": False,
                "Camera": False,
                "Color Sensor": False,
                "Lidar": False,
                "Gyro": False,
                "IMU": False,
                "GPS": False,
                "Accelerometer": False
            }
        if robotConfig != "default":
            self._parseConfig(robotConfig)

    ##################### MOTION MODEL ##################### 

    def _updatePosition(self):
        wheel_radius = 20.5 * 10**-3 #m
        wheel_distance = 58 * 10**-3 #m

        #movement is the enconders in radians
        wheel_left = robot.sensorValues['wheel2_desloc'] * wheel_radius
        wheel_right = robot.sensorValues['wheel1_desloc']*wheel_radius
        theta = (wheel_right-wheel_left) / wheel_distance

        self.position.theta += theta
        distance = (wheel_right + wheel_left) / 2
        self.position.x += distance * np.cos(self.position.theta)*10**2
        self.position.y += distance * np.sin(self.position.theta)*10**2

        # plot position
        plt.plot(self.position.x, self.position.y, 'ro')
        plt.pause(0.001)

        

    ##################### SENSORS ##################### 

    def _parseConfig(self, robotConfig:str)->None:
        self.logger.info("Parsing robot configuration")
        try:
            with open(robotConfig, 'r') as f:
                data = json.load(f)
                self.logger.info("Confi guration file loaded")
        except FileNotFoundError:
            self.logger.error("Configuration file not found")
            return
        # print keys
        distanceSensors = []
        wheels = []
        cameras = []
        colorSensor = ""
        lidar = ""
        for key in data.keys():
            if "Distance Sensor" in key:
                distanceSensors.append(key)
                self.available_sensors["Distance Sensor"] = True
                self.logger.info("Distance sensors: " + str(distanceSensors))
            if "Wheel" in key:
                wheels.append(key)
                self.logger.info("Wheel: " + key)
            if "camera" in key:
                cameras.append(key)
                self.available_sensors["Camera"] = True
                self.logger.info("Camera: " + key)
            if "colsensor" in key:
                colorSensor = key
                self.available_sensors["Color Sensor"] = True
                self.logger.info("Color sensor: " + key)
            if "lidar" in key:
                lidar = key
                self.available_sensors["Lidar"] = True
                self.logger.info("Lidar: " + key)
            if "gyro" in key:
                self._initializeGyro(key)
                self.available_sensors["Gyro"] = True
                self.logger.info("Gyro: " + key)
            if "imu" in key:
                self._initializeIMU(key)
                self.available_sensors["IMU"] = True
                self.logger.info("IMU: " + key)
            if "gps" in key:
                self._initializeGPS(key)
                self.available_sensors["GPS"] = True
                self.logger.info("GPS: " + key)
        self.logger.info("Configuration parsed")

        self._initializeDistanceSensors(len(distanceSensors))
        self._initializeWheels(len(wheels))
        if cameras:
            self._initializeCamera(nCameras=len(cameras))
        if colorSensor:
            self._initializeColorSensor()
        if lidar:
            self._initializeLidar(lidar) 
        self.logger.info("Robot initialized")

    def _initializeWheels(self, enableEncoders:bool=True, nWheels:int=2)->None:
        """Function to initialize the wheels of the robot

        Args:
            enableEncoders (bool, optional): Should encoders ber initialized, default is True
        """
        self.wheels = []
        self.encoders = []
        for i in range(nWheels):
            wheel = self._robot.getDevice("wheel" + str(i+1) + " motor")
            wheel.setPosition(float('inf'))
            self.wheels.append(wheel)
            if enableEncoders:
                encoder = self.wheels[i].getPositionSensor()
                encoder.enable(self.timeStep)
                self.encoders.append(encoder)

        self.logger.info("Wheels initialized")

    def _initializeDistanceSensors(self, nSensors:int=4, customPrefix:str="distance sensor")->None:
        """Function to initialize the distance sensors of the robot

        Args:
            nSensors (int, optional): Numer of distance sensors. Defaults to 4.
            customPrefix (str, optional): Prefix for the distance sensor. Defaults to "distance sensor".
        """        
        self.distance_sensors = []
        for i in range(nSensors):
            sensor = self._robot.getDevice(customPrefix + str(i+1))
            sensor.enable(self.timeStep)
            self.distance_sensors.append(sensor)
        self.logger.info("Distance sensors initialized")

    def _initializeCamera(self, nCameras:int=1, customPrefix:str="camera")->None:
        """Function to initialize the camera of the robot

        Args:
            cameraName (str, optional): Name of the camera. Defaults to "camera".
        """        
        self.cameras = []
        for i in range(nCameras):
            camera = self._robot.getDevice(customPrefix + str(i+1))
            camera.enable(self.timeStep)
            self.cameras.append(camera)
        self.logger.info("Camera initialized")

    def _initializeColorSensor(self, sensorName:str="colour_sensor")->None:
        """Function to initialize the color sensor of the robot

        Args:
            sensorName (str, optional): Name of the color sensor. Defaults to "color sensor".
        """        
        self.color_sensor = self._robot.getDevice(sensorName)
        self.color_sensor.enable(self.timeStep)
        self.logger.info("Color sensor initialized")

    def _initializeLidar(self, lidarName:str="lidar")->None:
        """Function to initialize the lidar of the robot

        Args:
            lidarName (str, optional): Name of the lidar. Defaults to "lidar".
        """        
        self.lidar = self._robot.getDevice(lidarName)
        self.lidar.enable(self.timeStep)
        self.lidar.enablePointCloud(); # Enable the point cloud computation
        self.logger.info("Lidar initialized")

    def _initializeGyro(self, gyroName:str="gyro")->None:
        """Function to initialize the gyro of the robot

        Args:
            gyroName (str, optional): Name of the gyro. Defaults to "gyro".
        """        
        self.gyro = self._robot.getDevice(gyroName)
        self.gyro.enable(self.timeStep)
        self.logger.info("Gyro initialized")

    def _initializeIMU(self, imuName:str="inertial_unit")->None:
        """Function to initialize the IMU of the robot

        Args:
            imuName (str, optional): Name of the IMU. Defaults to "inertial unit".
        """        
        self.imu = self._robot.getDevice(imuName)
        self.imu.enable(self.timeStep)
        self.logger.info("IMU initialized")

    def _initializeGPS(self, gpsName:str="gps")->None:
        """Function to initialize the GPS of the robot

        Args:
            gpsName (str, optional): Name of the GPS. Defaults to "gps".
        """        
        self.gps = self._robot.getDevice(gpsName)
        self.gps.enable(self.timeStep)
        self.logger.info("GPS initialized")

    def _initializeAccelerometer(self, accelerometerName:str="accelerometer")->None:
        """Function to initialize the accelerometer of the robot

        Args:
            accelerometerName (str, optional): Name of the accelerometer. Defaults to "accelerometer".
        """        
        self.accelerometer = self._robot.getDevice(accelerometerName)
        self.accelerometer.enable(self.timeStep)
        self.logger.info("Accelerometer initialized")

    def updateSensors(self)->None:
        """Function to update the sensors of the robot
        """        
        self.available_sensors['lidar'] = False
        if self.available_sensors["Distance Sensor"]:
            for i in range(len(self.distance_sensors)):
                self.sensorValues["distance_sensor" + str(i+1)] = self.distance_sensors[i].getValue()
            self.logger.info("Distance sensors updated")

        for i in range(len(self.wheels)):
            current = self.encoders[i].getValue()
            if "wheel" + str(i+1) in self.sensorValues:
                desloc = (current - self.sensorValues["wheel" + str(i+1)])
            else:
                desloc = current
            self.sensorValues["wheel" + str(i+1)+"_desloc"] = desloc
            self.sensorValues["wheel" + str(i+1)] = current
        self.logger.info("Wheels updated")

        if self.available_sensors["Camera"]:
            for i in range(len(self.cameras)):
                image = self.cameras[i].getImage()
                image = np.frombuffer(image, np.uint8).reshape((self.cameras[i].getHeight(), self.cameras[i].getWidth(), 4))
                self.sensorValues["camera" + str(i+1)] = image
                # frame = cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
                # frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) # Grayscale
                # cv2.threshold(frame, 80, 255, cv2.THRESH_BINARY) # Threshold
                
            self.logger.info("Camera updated")
        if self.available_sensors["Color Sensor"]:
            image = self.color_sensor.getImage()
            r = self.color_sensor.imageGetRed(image, 1, 0, 0)
            g = self.color_sensor.imageGetGreen(image, 1, 0, 0)
            b = self.color_sensor.imageGetBlue(image, 1, 0, 0)
            self.sensorValues["color_sensor"] = (r, g, b)
            if (r, g, b) == self.colors.HOLE:
                self.sensorValues["ground"] = "hole"
            elif (r, g, b) == self.colors.SWAMP:
                self.sensorValues["ground"] = "swamp"
            elif (r, g, b) == self.colors.A1:
                self.sensorValues["ground"] = "a1"
            elif (r, g, b) == self.colors.A2:
                self.sensorValues["ground"] = "a2"
            elif (r, g, b) == self.colors.A3:
                self.sensorValues["ground"] = "a3"
            else:
                self.sensorValues["ground"] = "ground"
            
            self.logger.info("Color sensor updated")
            
        if self.available_sensors["Lidar"]:
            self.sensorValues["lidar"] = self._pointCloudProcessing(self.lidar.getPointCloud())
            self.logger.info("Lidar updated")
            pass
        if self.available_sensors["Gyro"]:
            self.sensorValues["gyro"] = self.gyro.getValues()
            self.logger.info("Gyro updated")
        if self.available_sensors["IMU"]:
            self.sensorValues["imu"] = self.imu.getRollPitchYaw()
            self.logger.info("IMU updated")
        if self.available_sensors["GPS"]:
            self.sensorValues["gps"] = self.gps.getValues()
            self.logger.info("GPS updated")
        if self.available_sensors["Accelerometer"]:
            self.sensorValues["accelerometer"] = self.accelerometer.getValues()
            self.logger.info("Accelerometer updated")
        self._updatePosition()

    ##################### ACTUATORS ##################### 

    def updateMotorSpeed(self, speeds:tuple):
        if len(speeds) != len(self.wheels):
            raise ValueError("Number of speeds does not match number of wheels")
        # for i in range(len(self.wheels)):
        self.wheels[1].setVelocity(speeds[0])
        self.wheels[0].setVelocity(speeds[1])
        
    ##################### LOGIC ##################### 

    def _checkVic(self, img, cam):
        img_og = img
        img = np.frombuffer(img, np.uint8).reshape((cam.getHeight(), cam.getWidth(), 4))  # Convert img to RGBA format (for OpenCV)
        img = cv.cvtColor(img, cv.COLOR_BGR2GRAY) # Grayscale image
        _, thresh = cv.threshold(img, 205, 255, cv.THRESH_BINARY) # Threshold image
        kernel = np.ones((5, 5), np.uint8)
        thresh = cv.dilate(thresh, kernel, iterations=1) # Dilate image
        # contours, hierarchy = cv.findContours(thresh, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE) # Find all shapes within thresholded image
        # remove small contours
        # contours = [cnt for cnt in contours if cv.contourArea(cnt) >15]
        # show img wirh contours
        # cv.drawContours(img_og, contours, -1, (0, 255, 0), 1)
        cv.namedWindow("Image", cv.WINDOW_NORMAL)
        cv.imshow("Image", img_og)
        cv.resizeWindow("Image", 400, 300)
        cv.waitKey(1)

    def _pointCloudProcessing(self, pointCloud):
        # remove inf and -inf values
        
        for point in pointCloud:
            if point.x != float('inf') and point.x != float('-inf'):
                rotated_x = point.x * np.cos(self.position.theta) - point.y * np.sin(self.position.theta)
                rotated_y = point.x * np.sin(self.position.theta) + point.y * np.cos(self.position.theta)
                self.map.append((rotated_x*10 + self.position.x, rotated_y*10 + self.position.y))
                # print(self.position.x, rotated_x*10)
        # plt.scatter(*zip(*self.map), s=1)
        # plt.pause(0.001)
        # plt.clf()

    def checkObstacle(self):
        if self.sensorValues["distance_sensor1"] < OBSTACLE_THRESH:
            return "L"
        elif self.sensorValues["distance_sensor2"] < OBSTACLE_THRESH:
            return "F"
        elif self.sensorValues["distance_sensor3"] < OBSTACLE_THRESH:
            return "R"
        else:
            return None

robot = BRC("/Users/roberto/Projects/Rescue-Simulation-ORG/2024/codeSample/MyAwesomeRobot.json")

while robot._robot.step(robot.timeStep) != -1:
    SPEED_LEFT = DEFAULT_SPEED
    SPEED_RIGHT = DEFAULT_SPEED
    robot.updateSensors()

    if robot.checkObstacle():
        if robot.checkObstacle() == "L":
            print("Obstacle on the left")
            SPEED_LEFT = DEFAULT_SPEED
            SPEED_RIGHT = -DEFAULT_SPEED
        elif robot.checkObstacle() == "R":
            print("Obstacle on the right")
            SPEED_LEFT = -DEFAULT_SPEED
            SPEED_RIGHT = DEFAULT_SPEED
        elif robot.checkObstacle() == "F":
            print("Obstacle in front")
            if robot.sensorValues["distance_sensor1"] < robot.sensorValues["distance_sensor3"]:
                print("Obstacle on the left")
                SPEED_LEFT = 0
                SPEED_RIGHT = -DEFAULT_SPEED
            else:
                print("Obstacle on the right")
                SPEED_LEFT = -DEFAULT_SPEED
                SPEED_RIGHT = 0
            
    if robot.sensorValues["ground"] == "hole":
        print("Hole detected")
        SPEED_LEFT = -DEFAULT_SPEED/2
        SPEED_RIGHT = -DEFAULT_SPEED
    print("nothing")
    print(robot.position)

    robot.updateMotorSpeed((SPEED_LEFT, SPEED_RIGHT))
    
    