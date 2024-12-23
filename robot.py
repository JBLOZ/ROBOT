


import numpy as np
import math

RADIOROBOT = 0.15
BGCOLOR = "black"
VMAX = 3 #m/s
WMAX = 1 #rad/s
VACC = 1 #m/s2
WACC = 0.5 #rad/s2


class Robot:
    def __init__(self):

        self.coordX = 0.0
        self.coordY = 0.0
        self.heading = 0.0 #vector (1,0) in degrees

        self.linearVel = 0
        self.angularVel = 0 
        self.actualLinearVel = 0
        self.actualAngularVel = 0
    
    def setPose(self, pose):
        self.coordX = pose[0]
        self.coordY = pose[1]
        self.heading = pose[2]
    
    def getPose(self):
        return (self.coordX, self.coordY, self.heading, self.actualLinearVel, self.actualAngularVel)

    def setVel(self, velocidades):

        self.linearVel = velocidades[0]
        if self.linearVel > VMAX:
            self.linearVel = VMAX
        if self.linearVel < -VMAX:
            self.linearVel = -VMAX

        self.angularVel = velocidades[1]
        if self.angularVel > WMAX:
            self.angularVel = WMAX
        if self.angularVel < -WMAX:
            self.angularVel = -WMAX

    #
    def updateDynamics(self, timelapse):
        # Partimos de V y W
        # Actualizamos las velocidades si todavía no se ha llegado a la velocidad deseada!!!!!!
        timeSeconds = timelapse / 1000.0


        #Actualizamos velocidades
        changeV = VACC * timeSeconds
        changeW = WACC * timeSeconds
        if self.actualLinearVel < self.linearVel:
            self.actualLinearVel = self.actualLinearVel + changeV
            if self.actualLinearVel > VMAX:
                self.actualLinearVel = VMAX
        if self.actualLinearVel > self.linearVel:
            self.actualLinearVel = self.actualLinearVel - changeV
            if self.actualLinearVel < -VMAX:
                self.actualLinearVel = -VMAX
        if self.linearVel == 0 and abs(self.actualLinearVel)<changeV:
            self.actualLinearVel = 0

        if self.actualAngularVel < self.angularVel:
            self.actualAngularVel = self.actualAngularVel + changeW
            if self.actualAngularVel > WMAX:
                self.actualAngularVel = WMAX
        if self.actualAngularVel > self.angularVel:
            self.actualAngularVel = self.actualAngularVel - changeW
            if self.actualAngularVel < -WMAX:
                self.actualAngularVel = -WMAX
        if self.angularVel == 0 and abs(self.actualAngularVel)<changeW:
            self.actualAngularVel = 0

        # calculamos cuántos grados hay que girar por el tiempo que ha pasado (si W>0)
        #   obtenemos el ICC y aplicamos la rotación
        #   o calculamos la traslación en linea recta también ponderada por el tiempo que ha pasado
        if abs(self.actualAngularVel) > 0.000001:
            if self.actualLinearVel != 0:
                ICCRad = self.actualLinearVel/self.actualAngularVel
                
                headingRad = (90-self.heading) * math.pi / 180
                ICCgx = self.coordX - ICCRad * math.cos(headingRad)
                ICCgy = self.coordY + ICCRad * math.sin(headingRad)

                ICCtgx = self.coordX - ICCgx
                ICCtgy = self.coordY - ICCgy

                rotatedICCx = ICCtgx * math.cos(self.actualAngularVel*timeSeconds) - ICCtgy * math.sin(self.actualAngularVel*timeSeconds)
                rotatedICCy = ICCtgx * math.sin(self.actualAngularVel*timeSeconds) + ICCtgy * math.cos(self.actualAngularVel*timeSeconds)


                self.coordX = rotatedICCx + ICCgx
                self.coordY = rotatedICCy + ICCgy
            acAngVelDeg = self.actualAngularVel*180/math.pi
            self.heading = self.heading + acAngVelDeg * timeSeconds 
        else:
            #linea recta
            dist = self.actualLinearVel * timeSeconds
            angleRad = self.heading * math.pi / 180
            self.coordX = self.coordX + dist*math.cos(angleRad)
            self.coordY = self.coordY + dist*math.sin(angleRad)

