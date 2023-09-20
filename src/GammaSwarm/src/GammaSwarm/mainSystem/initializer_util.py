#!/usr/bin/env python
#sa
#as
#as

from utils import *
import numpy as np
from State import *


class Formation:
    def __init__(self):
        self.center = None
        self.meanAngle = 0
        self.formationPoints = None
        self.uav_count = None
        self.uavPositionPoses = None
        self.horizontalDistance = 0.4
        self.verticalDistance = 2.
        self.closestToCenterAngleIndex = 0
        self.firstPositions = []

    def saveFirstPoses(self, uxvPoses):
        for pose in uxvPoses:
            self.firstPositions.append(pose)

    def poseListToArray(self):
        for i in range(self.uav_count):
            self.poseList[i][0] = self.uavPositionPoses[i].x
            self.poseList[i][1] = self.uavPositionPoses[i].y
            self.poseList[i][2] = self.uavPositionPoses[i].z

    def initialize(self):
        self.uav_count = len(self.uavPositionPoses)
        self.poseList = np.zeros((self.uav_count, 3), dtype=int).tolist()
        self.formationPoints = []
        self.matchedFormationPoints = []
        self.poseListToArray()
        if self.center == -1:
            self.center = self.defineCenter()

    def defineCenter(self):
        x, y, z = 0, 0, 0
        for pose in self.uavPositionPoses:
            x += pose.x / self.uav_count
            y += pose.y / self.uav_count
            z += pose.z / self.uav_count
        return Position(x, y, z)

    def commonFormationPoints(self, uav_poseList,  centerPoint=-1, formationNum=-1):
        # print("poseList Len: ", len(uav_poseList))
        self.uavPositionPoses = uav_poseList
        self.center = centerPoint
        self.initialize()
        uavCnt = len(uav_poseList)
        nonClosestIndexes = -1
        removeLasts = False
        if formationNum == -1:
            pass
        elif len(self.uavPositionPoses) > formationNum:
            uavPoses, nonClosestIndexes = self.filterPoses(formationNum, self.uavPositionPoses)
            uavCnt = len(uavPoses)
        else:
            uavCnt = formationNum
            removeLasts = True
        self.formationPoints = []
        center_point = [self.center.x, self.center.y]
        # print(center_point)
        angle = 360 / uavCnt
        distanceFromCenter = self.horizontalDistance / (2 * np.sin(np.pi / self.uav_count))

        for i in range(uavCnt):
            x1, y1 = pol12cart(distanceFromCenter, i * angle)
            x = x1 + center_point[0]
            y = y1 + center_point[1]
            self.formationPoints.append(Position(x, y, float(self.center.z)))
        if nonClosestIndexes != -1:
            for index in nonClosestIndexes:
                self.formationPoints.insert(index, uav_poseList[index])
        if removeLasts:
            self.formationPoints = self.formationPoints[:len(uav_poseList)]
        self.simpleMatchThePoints()
        self.simpleMatchPoints()
        self.uavPositionPoses = self.formationPoints
        self.center = self.defineCenter()
        return self.formationPoints


    def filterPosesToSetTrue(self, count):
        potentialPoses = [[2, 2], [2, -2], [-2, -2], [-2, 2], [2, 0], [0, -2], [-2, 0], [0, 2]]
        center = [self.center.x, self.center.y]
        for j in range(len(potentialPoses)):
            potentialPoses[j].append(dist_of_two_pt(potentialPoses[j], center))
        potentialPoses.sort(key=lambda x: x[2])

        if count == len(self.formationPoints):
            pass
        else:
            difNum = count - len(self.formationPoints)
            if difNum > 0:
                for i in range(difNum):
                    i = i % len(potentialPoses)
                    self.formationPoints.append(Position(potentialPoses[i][0],
                                                         potentialPoses[i][1],
                                                         self.formationPoints[i].z))
            elif difNum < 0:
                self.formationPoints = self.formationPoints[:count]

        return self.formationPoints

    def simpleMatchThePoints(self):
        listLen = self.uav_count
        poseToPointDistances = np.zeros((listLen, listLen), dtype=int).tolist()

        for i in range(listLen):
            for j in range(listLen):
                poseToPointDistances[i][j] = dist3d(self.poseList[i][0], self.poseList[i][1], self.poseList[i][2],
                                                    self.formationPoints[j].x, self.formationPoints[j].y, self.formationPoints[j].z)

        for i in range(listLen):
            minIndex = poseToPointDistances[i].index(min(poseToPointDistances[i]))
            self.matchedFormationPoints.append(self.formationPoints[minIndex])
            for row in poseToPointDistances:
                row[minIndex] = np.inf

        self.formationPoints = self.matchedFormationPoints

    def simpleMatchPoints(self):
        # print("formation Poses : ", len(self.formationPoints))
        # print("uav poses : ", len(self.uavPositionPoses))
        listLen = len(self.formationPoints)
        self.matchedFormationPoints = self.formationPoints
        intersectionFlags = np.zeros((listLen, listLen), dtype=bool).tolist()
        intersectionFlags2 = np.zeros((listLen, listLen), dtype=bool).tolist()

        for a in range(listLen):
            terminateCntr = 0
            while terminateCntr != listLen:
                j = (a + 1 + terminateCntr) % listLen
                intersectionFlag = line_intersect(self.uavPositionPoses[a],
                                                  self.formationPoints[a],
                                                  self.uavPositionPoses[j],
                                                  self.formationPoints[j])
                intersectionFlags[a][j] = intersectionFlag
                if intersectionFlag and abs(self.formationPoints[a].z - self.formationPoints[j].z) < 0.3:
                    self.matchedFormationPoints[a], self.matchedFormationPoints[j] = \
                        self.formationPoints[j], self.formationPoints[a]
                    self.formationPoints = self.matchedFormationPoints
                    intersectionFlag2 = line_intersect(self.uavPositionPoses[a],
                                                       self.formationPoints[a],
                                                       self.uavPositionPoses[j],
                                                       self.formationPoints[j])
                    intersectionFlags2[a][j] = intersectionFlag2
                else:
                    pass
                terminateCntr += 1

    def filterPoses(self, formationNum, uavPoseList):
        lengthList = len(uavPoseList)
        distancesNPoses = []
        for i in range(lengthList):
            distancesNPoses.append((distP(uavPoseList[i], self.center), i))
        distancesNPoses.sort(key=lambda x: x[0])
        closestIndexes = [j[1] for j in distancesNPoses[:formationNum]]
        notClosestIndexes = []
        for ele in range(lengthList):
            if ele not in closestIndexes:
                notClosestIndexes.append(ele)
        targetPoses = [uavPoseList[j] for j in closestIndexes]
        return targetPoses, notClosestIndexes


    