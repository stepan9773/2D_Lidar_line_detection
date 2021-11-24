import env, sensors , feature
import random
import pygame
import math



def random_color():
    levels = range(32,256,32)
    return tuple(random.choice(levels) for _ in range(3))

FeatureMap = feature.featuresDetection()
enviroment = env.buildEnviroment((900,1000))
enviroment.originalMap = enviroment.map.copy()
laser = sensors.LaserSensor(200,enviroment.originalMap,uncertenty=(0.5,0.01))
enviroment.map.fill((255,255,255))
enviroment.infomap = enviroment.map.copy()
originalMap = enviroment.map.copy()

running = True
FEATURE_DETECTION = True
BREAKE_POINT_INO = 0

while running:
    enviroment.infomap = originalMap.copy()
    FEATURE_DETECTION = True
    BREAKE_POINT_INO = 0
    END_POINT = [0,0]
    sensorON = False
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
    if pygame.mouse.get_focused():
        sensorON = True
    elif not pygame.mouse.get_focused():
        sensorON = False
    if sensorON:
        position = pygame.mouse.get_pos()
        laser.position = position
        sensor_data = laser.sense_obstacle()
        FeatureMap.laser_points_set(sensor_data)
        print("epoch")
        while BREAKE_POINT_INO < abs(FeatureMap.NP - FeatureMap.PMIN):
            seedSeg = FeatureMap.seed_segment_detection(laser.position, BREAKE_POINT_INO)
            if seedSeg == False:
                break
            else:
                seedSegment = seedSeg[0]
                PREDICTED_POINTS_TODRAW = seedSeg[1]
                INDICES = seedSeg[2]
                results = FeatureMap.seed_segment_growing(INDICES,BREAKE_POINT_INO)
                if results == False:
                    BREAKE_POINT_INO = INDICES[1]
                    continue
                else:
                    line_eq = results[1]
                    m,c = results[5]
                    line_seg = results[0]
                    OUTERMOST = results[2]
                    BREAKE_POINT_INO = results[3]
                    END_POINT[0] = FeatureMap.projection_point2line(OUTERMOST[0],m,c)
                    END_POINT[1] = FeatureMap.projection_point2line(OUTERMOST[1], m, c)



                    COLOR = random_color()
                    for point in line_seg:
                        enviroment.infomap.set_at((int(point[0][0]),int(point[0][1])), (0,255,0))
                        pygame.draw.circle(enviroment.infomap,COLOR,(int(point[0][0]),int(point[0][1])),2,0)
                    pygame.draw.line(enviroment.infomap,  (0,0,255) ,END_POINT[0], END_POINT[1],2)
                    print(f"Alfa {m}  beta {c}")
                    enviroment.dataStorage(sensor_data)
                    #enviroment.show_sensorData()



    enviroment.map.blit(enviroment.infomap,(0,0))
    pygame.display.update()
