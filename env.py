import math
import pygame

class buildEnviroment:
    def __init__(self,MapDimentions):
        pygame.init()
        self.pointCloud = []
        
        self.externalMap= pygame.image.load('map/1.png')
        self.maph, self.mapw = MapDimentions
        self.MapWindowsName = "КУРСОВА"
        pygame.display.set_caption(self.MapWindowsName)
        self.map = pygame.display.set_mode((self.mapw,self.maph))
        self.map.blit(self.externalMap,(0,0))

        self.black = (0,0,0)
        self.grey = (70, 70, 70)
        self.blue = (0, 0, 255)
        self.Green = (0, 255, 0)
        self.Red = (255, 0, 0)
        self.white = (255, 255, 255)
        
    
    def AD2pos(self, distance, angle, robot_position):
        x = distance*math.cos(angle) + robot_position[0]
        y = -distance*  math.sin(angle) + robot_position[1]
        return int(x),int(y)

    def dataStorage(self,data):

        for element in data:
            point = self.AD2pos(element[0],element[1],element[2])
            if point not in self.pointCloud:
                self.pointCloud.append(point)

    def show_sensorData(self):

        self.infomap = self.map.copy()
        for point in self.pointCloud:
            self.infomap.set_at((int(point[0]), int(point[1])),(255, 0, 0))

        
        
        
        
        
        
        
        
        
        
        