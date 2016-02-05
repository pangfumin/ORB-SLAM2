
from __future__ import division
import numpy as np
import datetime
import rosbag
import rospy
from copy import copy, deepcopy
from exceptions import KeyError, ValueError
from segway_rmp.msg import SegwayStatusStamped
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from math import cos, sin, tan, exp
from filter import ParticleFilter, nrand


ndtFName = '/home/sujiwo/ORB_SLAM/Data/20151106-1/ndt.csv'
orbFName = '/home/sujiwo/ORB_SLAM/Data/20151106-1/orb-slam.csv'



class Pose :
    def __init__ (self, t=0, _x=0, _y=0, _z=0, _qx=0, _qy=0, _qz=0, _qw=1):
        try:
            self.timestamp = t[0]
            self.x = t[1]
            self.y = t[2]
            self.z = t[3]
            self.qx = t[4]
            self.qy = t[5]
            self.qz = t[6]
            self.qw = t[7]
        except (TypeError, IndexError):
            self.timestamp = t
            self.x = _x
            self.y = _y
            self.z = _z
            self.qx = _qx
            self.qy = _qy
            self.qz = _qz
            self.qw = _qw
            
    def __str__ (self):
        return "X={}, Y={}, Z={}, Qx={}, Qy={}, Qz={}, Qw={}".format(self.x, self.y, self.z, self.qx, self.qy, self.qz, self.qw)
    
    def time (self):
        return datetime.datetime.fromtimestamp(self.timestamp)
    
    def offsetTime (self, t):
        self.timestamp += t
        
    def coord (self):
        return np.array([self.x, self.y, self.z])
            
    def rot (self):
        return np.array([self.qx, self.qy, self.qz, self.qw])
        
    def __sub__ (self, p1):
        return np.array([self.x-p1.x, self.y-p1.y, self.z-p1.z])

    # Only calculate movement, but does not overwrite the values
    # currentTimestamp must be in second
    def segwayMove (self, currentTimestamp, leftWheelVelocity, rightWheelVelocity, yawRate):
        # this is minimum speed to consider yaw changes (ie. yaw damping)
        minSpeed = 5e-3        
        
        v = (leftWheelVelocity + rightWheelVelocity) / 2
        dt = currentTimestamp - self.timestamp
        # XXX: May need to change this line 
        if v > minSpeed:
            w = (yawRate+0.011)*0.98
        else:
            w = 0.0
        
        x = self.x + v*cos(self.theta) * dt
        y = self.y + v*sin(self.theta) * dt
        theta = self.theta + w * dt
        return x, y, theta
        
    @staticmethod
    def interpolate (pose1, pose2, tm):
        if (pose1.timestamp > pose2.timestamp) :
            raise ValueError ("pose1 timestamp must be > pose2")
        if (tm < pose1.timestamp or tm > pose2.timestamp) :
            raise ValueError ("Requested timestamp is outside range")
        v = (tm - pose1.timestamp) / (pose2.timestamp - pose1.timestamp)
        intpose = Pose(tm, 
            pose1.x + v*(pose2.x-pose1.x),
            pose1.y + v*(pose2.y-pose1.y),
            pose1.z + v*(pose2.z-pose1.z))
        return intpose
        
    @staticmethod
    def average (*poses):
        avgpose = Pose()
        xs = [p.x for p in poses]
        ys = [p.y for p in poses]
        zs = [p.z for p in poses]
        avgpose.x = sum(xs) / len(poses)
        avgpose.y = sum(ys) / len(poses)
        avgpose.z = sum(zs) / len(poses)
        avgpose.timestamp = np.average([p.timestamp for p in poses])
        return avgpose


class PoseTable :
    def __init__ (self):
        self.table = []
        self.idList = {}
        self.c = 0
    
    def __setitem__ (self, key, value):
        self.table.append (value)
        self.idList[key] = self.c
        self.c += 1
        
    def __getitem__ (self, key):
        p = self.idList[key]
        return self.table[p]
    
    def __len__ (self):
        return len(self.table)
    
    def __iadd__ (self, offset):
        for pose in self.table :
            pose.offsetTime (offset)
        return self
    
    def __isub__ (self, offset):
        return self.__iadd__(-offset)
        
    def append (self, pose):
        self.table.append (pose)
        ckeys = self.idList.keys()
        if (len(ckeys)==0):
            ckey = -1
        else :
            ckey = max (ckeys)
        self.idList[ckey+1] = self.c
        self.c += 1
        
    def length (self, tolerance=0):
        """
        Compute distance spanned by this pose. If miliSecTolerance is not specified, \
        we assume that there is no gap
        """
        totaldist = 0
        for p in range(1, len(self.table)):
            cpose = self.table[p]
            ppose = self.table[p-1]
            if (tolerance>0) :
                if abs(ppose.timestamp - cpose.timestamp) > tolerance:
                    print('far')
                    continue
            dist = np.linalg.norm([cpose.x-ppose.x, cpose.y-ppose.y, cpose.z-ppose.z])
            totaldist += dist
        return totaldist
        
    def lengths (self):
        dists = []
        for p in range(1, len(self.table)):
            cpose = self.table[p]
            ppose = self.table[p-1]
            dists.append (np.linalg.norm([cpose.x-ppose.x, cpose.y-ppose.y, cpose.z-ppose.z]))
        return dists
        
    def timeLengths (self):
        timeDists = []
        for p in range(1, len(self.table)):
            cpose = self.table[p]
            ppose = self.table[p-1]
            timeDists.append (abs(cpose.timestamp - ppose.timestamp))
        return timeDists        
        
    
    def toArray (self, includeTimestamp=False):
        if (includeTimestamp==True) :
            mlt = [[p.timestamp, p.x, p.y, p.z, p.qx, p.qy, p.qz, p.qw] for p in self.table]
        else :
            mlt = [[p.x, p.y, p.z, p.qx, p.qy, p.qz, p.qw] for p in self.table]
        return np.array(mlt)
    
    def findNearestByTime (self, pose, tolerance=0):
        if (pose.timestamp < self.table[0].timestamp) :
            raise KeyError ("Timestamp less than table")
        if (pose.timestamp > self.table[self.c-1].timestamp) :
            raise KeyError ("Timestamp is outside table")
        candidates = set()
        i = 0
        for p in range(len(self.table)) :
            i = p
            cpose = self.table[i]
            if (cpose.timestamp > pose.timestamp) :
                candidates.add(cpose)
                i-=1
                break
        while i!=0 :
            cpose = self.table[i]
            i -= 1
            candidates.add (cpose)
            if (cpose.timestamp < pose.timestamp) :
                candidates.add (cpose)
                break
        if (tolerance>0) :
            tcandidates=[]
            for c in candidates:
                c.tdif = abs(c.timestamp-pose.timestamp)
                if c.tdif > tolerance:
                    pass
                else:
                    tcandidates.append(c)
            return sorted (tcandidates, key=lambda pose: pose.tdif)
        return sorted (candidates, key=lambda pose: pose.timestamp)
        
    def findNearestInTime (self, timestamp, tolerance):
        candidates = set()
        for p in self.table:
            tdiff = abs(p.timestamp - timestamp)
            if (tdiff < tolerance):
                candidates.add(p)
            if p.timestamp > timestamp:
                break
        if (len(candidates)==0):
            return None
        return min(candidates, key=lambda p: abs(p.timestamp-timestamp))
            
    def findNearestByDistance (self, pose, returnIdx=False):
        if (returnIdx==False):
            return min(self.table, 
                       key=lambda p: 
                           np.linalg.norm(pose.coord()-p.coord()))
        else:
            dist = np.array([np.linalg.norm(pose.coord()-p.coord()) for p in self.table])
            return np.argmin(dist)
            
        
    def last(self):
        return self.table[-1]
    
    @staticmethod
    def plotMulti (*tables):
        pass
    
    def plot (self, *columnList):
        array = self.toArray()
        return plt.plot(array[:,columnList[0]], array[:,columnList[1]])
        
    
    @staticmethod 
    def loadCsv (filename):
        mat = np.loadtxt(filename)
        records = PoseTable ()
        for r in mat :
            p = Pose (r[0], r[1], r[2], r[3], r[4], r[5], r[6], r[7])
            records.append (p)
        return records
    
    @staticmethod
    def loadFromBagFile (filename, sourceFrameName=None, targetFrameName=None) :
        bagsrc = rosbag.Bag(filename, mode='r')
        #topicInfo = bagsrc.get_type_and_topic_info('/tf')
        i = 0
        bagRecord = PoseTable ()
        for topic, msg, timestamp in bagsrc.read_messages('/tf') :
            transform = msg.transforms[0].transform
            header = msg.transforms[0].header
            child_frame_id = msg.transforms[0].child_frame_id
            if (sourceFrameName!=None and targetFrameName!=None) :
                if header.frame_id!=sourceFrameName or child_frame_id!=targetFrameName :
                    continue
            pose = Pose (timestamp.to_sec(), 
                transform.translation.x, transform.translation.y, transform.translation.z,
                transform.rotation.x, transform.rotation.y, transform.rotation.z, transform.rotation.w)
            bagRecord[i] = pose
            i += 1
        return bagRecord
        
    @staticmethod
    def loadFromArray (msrc):
        if (msrc.shape[1] != 8):
            raise ValueError ('Source has no timestamp')
        # try to sort
        msrc = sorted(msrc, key=lambda r: r[0])
        table = PoseTable ()
        for row in msrc:
            pose = Pose(row)
            table.append(pose)       
        return table

    @staticmethod
    def getFrameList (filename):
        bagsrc = rosbag.Bag(filename, mode='r')
        frames = set()
        for topic, msg, timestamp in bagsrc.read_messages('/tf'):
            transform = msg.transforms[0].transform
            header = msg.transforms[0].header
            child_frame_id = msg.transforms[0].child_frame_id
            frames.add((header.frame_id, child_frame_id))
        return frames
        
    # Find all poses in current table that are in range of targetPoses timeframe
    def getAllInTimeRanges (self, targetPoses):
        matchInTime = PoseTable()
        
        p1 = targetPoses[0]
        p2 = targetPoses.last()
        
        for p in self.table:
            if p.timestamp >= p1.timestamp and p.timestamp<=p2.timestamp:
                matchInTime.append (copy (p))
        
        return matchInTime
        
    @staticmethod
    def compareErrors (poseTbl1, poseTbl2):
        """
        poseTbl1 -> for source table
        poseTbl2 -> for ground truth
        """
        errorVect = []
        i=0
        for pose in poseTbl1.table:
            nearp = poseTbl2.findNearestByDistance(pose)
            errv = np.linalg.norm([pose.x-nearp.x, pose.y-nearp.y, pose.z-nearp.z], 2)
            errorVect.append(errv)
            i+=1
            #if i>=10000:
            #    break
            print ("{} out of {}".format(i, len(poseTbl1)))
        return errorVect
        
    @staticmethod
    # XXX: Unfinished
    def removeSpuriousPoints (poseTbl1):
        newposetbl = PoseTable()
        for pose in poseTbl1.table:
            pass
        return newposetbl
        
    def findBlankTime(self, timeTolerance=0.5):
        blanks = []
        for p in range(1, len(self.table)):
            cpose = self.table[p]
            ppose = self.table[p-1]
            if abs(cpose.timestamp - ppose.timestamp) > timeTolerance:
                blanks.append([ppose, cpose])
        return blanks
        
    def lengthFrom2Pose (self, poseIndex1, poseIndex2):
        if (type(poseIndex1)==int):
            dist = 0.0
            for i in range(poseIndex1+1, poseIndex2+1):
                cpose = self.table[i]
                ppose = self.table[i-1]
                cdist = np.linalg.norm([cpose.x-ppose.x, cpose.y-ppose.y, cpose.z-ppose.z], 2)
                dist += cdist
            return dist
    
    def lengthFrom2Times(self, time1, time2):
        pose1 = self.findNearestInTime(time1, 0.25)
        idx1 = self.table.index(pose1)
        pose2 = self.findNearestInTime(time2, 0.25)
        idx2 = self.table.index(pose2)
        return self.lengthFrom2Pose (idx1, idx2)
        
    def subset (self, startIdx, stopIdx):
        poseTblSubset = PoseTable()
        for i in range(startIdx, stopIdx+1):
            p = self.table[i]
            poseTblSubset.append(p)
        return poseTblSubset
        
    def findBlankLengthFromGroundTruth (self, groundTruthTbl):
        tolerance = 0.25
        blankDistFront = 0
        # Find blank distance in front
        if groundTruthTbl[0].timestamp < self.table[0].timestamp:
            pgrnd = groundTruthTbl.findNearestInTime (self.table[0].timestamp, tolerance)
            idx = groundTruthTbl.table.index(pgrnd)
            blankDistFront = groundTruthTbl.lengthFrom2Pose (0, idx)
        else:
            blankDistFront = 0
        # Find blank distance in rear
        blankDistRear = 0
        if (groundTruthTbl.last().timestamp > self.table[-1].timestamp):
            pgrnd = groundTruthTbl.findNearestInTime (self.table[-1].timestamp, tolerance)
            idx = groundTruthTbl.table.index (pgrnd)
            blankDistRear = groundTruthTbl.lengthFrom2Pose (idx, len(groundTruthTbl)-1)
        else:
            blankDistRear = 0
        # Find blank distances in middle
        blankPoses = self.findBlankTime(tolerance)
        blankDistMid = 0
        for bPose in blankPoses:
            d = groundTruthTbl.lengthFrom2Times (bPose[0].timestamp, bPose[1].timestamp)
            blankDistMid += d
        return blankDistFront + blankDistMid + blankDistRear
            
            
    @staticmethod
    def loadSegwayStatusFromBag (bagFilename, limitMsg=0) :
        segwayPose = PoseTable()
        
        bagsrc = rosbag.Bag(bagFilename, mode='r')
        
        cPose = Pose()
        cPose.theta = 0.0
        i = 0
        
        for topic, msg, timestamp in bagsrc.read_messages():
            try:
                if cPose.timestamp == 0:
                    cPose.timestamp = timestamp.to_sec()
                    continue
                x, y, theta = cPose.segwayMove(timestamp.to_sec(), 
                    msg.segway.left_wheel_velocity, 
                    msg.segway.right_wheel_velocity, 
                    msg.segway.yaw_rate)
                cPose.x = x
                cPose.y = y
                cPose.theta = theta
                cPose.timestamp = timestamp.to_sec()
                
                segwayPose.append (copy(cPose))
                i += 1
                if (limitMsg!=0 and i>=limitMsg):
                    break
                print (i)
                
            except KeyError:
                continue
        
        return segwayPose
        
        
def joinOrbOdometry (orb1, orb2, odomTable):
    orbError = 0.5
    numOfParticles = 250
    timeTolerance = 0.2
    WheelError = 0.3
    
    # Here, "Particle State" is an estimated Pose
    def odoMotionModel(particleState, move):
        if particleState.timestamp==0:
            particleState.timestamp = move['time']
            return particleState
        
        # XXX: Add randomized component to left & right wheel velocity
        vl = move['left'] + nrand(WheelError)
        vr = move['right'] + nrand(WheelError)
        # vr = move['right] + random_vr
            
        x, y, theta = particleState.segwayMove (move['time'], vl, vr, move['yawRate'])
        particleState.x = x
        particleState.y = y
        particleState.theta = theta
        particleState.timestamp = move['time']
        return particleState

    def odoMeasurementModel(particleOdomState, orbPose):
        x = particleOdomState.x
        y = particleOdomState.y
        w = exp(-(((x-orbPose.x)**2)/(2*orbError*orbError) + 
            ((y-orbPose.y)**2)/(2*orbError*orbError)))        
        return w

    def stateInitFunc ():
        p = Pose(0)
        p.theta = 0.0
        return p
        
    def buildStateTables (stateList):
        tbl = PoseTable()
        for p in stateList:
            tbl.append(p)
        return tbl
        
    PF = ParticleFilter(numOfParticles, stateInitFunc, odoMotionModel, odoMeasurementModel)
    
    # We assume odometry is always available from start to finish
    for odomMove in odomTable:
        # determine final ORB pose from two maps
        orbp1 = orb1.findNearestInTime (odomMove[0], timeTolerance)
        orbp2 = orb2.findNearestInTime (odomMove[0], timeTolerance)
        if (orbp1 != None and orbp2 == None):
            orbj = orbp1
        elif (orbp1 == None and orbp2 != None):
            orbj = orbp2
        elif (orbp1 == None and orbp2 == None):
            orbj = None
        else:
            orbj = Pose()
            orbj.timestamp = (orbp1.timestamp+orbp2.timestamp)/2
            orbj.x = (orbp1.x+orbp2.x)/2
            orbj.y = (orbp1.y+orbp2.y)/2
            orbj.z = (orbp1.z+orbp2.z)/2
        if (orbj.timestamp > odomMove[0]):
            orbj = None
            
        movement = {'time':odomMove[0], 'left':odomMove[1], 'right':odomMove[2], 'yawRate':odomMove[3]}
        print str(datetime.datetime.fromtimestamp(movement['time']))
        PF.update (movement, orbj)
        states = PF.getStates()
        curStates = buildStateTables(states)
        continue
        
    
    
def joinPoseTables (*poseTbls):
    #Find maximum & minimum time
    mintimes = [ptb[0].timestamp for ptb in poseTbls]
    startTime = min(mintimes)
    maxtimes = [ptb.last().timestamp for ptb in poseTbls]
    stopTime = max(maxtimes)
    
    # Find optimal time resolution
    def timeDiffs (poseTbl) :
        diff = []
        for p in range(1, len(poseTbl.table)):
            cpose = poseTbl.table[p]
            ppose = poseTbl.table[p-1]
            diff.append(cpose.timestamp - ppose.timestamp)
        return diff
    poseTblDiffs = [timeDiffs(ptbl) for ptbl in poseTbls]
    minDiffs = [min(td) for td in poseTblDiffs]
    timeRez = min(minDiffs)

    poseList = set()
    for ptbl in poseTbls:
        for pose in ptbl.table:
            pose.parent = ptbl
            poseList.add(pose)
    allPosesList = sorted(poseList, key=lambda p: p.timestamp)

    jointPoses = PoseTable()    
    for pose in allPosesList:
        if pose not in poseList:
            continue
        cPoses = []
        cPoses.append(pose)
        for ptbl in poseTbls:
            if (pose.parent==ptbl):
                continue
            friend = ptbl.findNearestInTime(pose.timestamp, 2*timeRez)
            if friend != None and friend in poseList:
                cPoses.append (friend)
        poseAvg = Pose.average(*cPoses)
        jointPoses.append(poseAvg)
        for p in cPoses:
            poseList.discard(p)
        # For debugging progress
        print ("Length: {} / {}".format(len(jointPoses), len(allPosesList)))
    return jointPoses        


def OrbFixOffline (orbLocalisationBagFilename, mapCsv):
    offset = 1
    orbLoc = PoseTable.loadFromBagFile(orbLocalisationBagFilename, 'ORB_SLAM/World', 'ORB_SLAM/Camera')
    mapArray = np.loadtxt(mapCsv)
    orbMapTbl = np.array([[r[0],r[1],r[2],r[3],r[4],r[5],r[6],r[7]] for r in mapArray])
    orbMap = PoseTable.loadFromArray(orbMapTbl)
    ndtMapTbl = np.array([[r[0],r[8],r[9],r[10],r[11],r[12],r[13],r[14]] for r in mapArray])
    ndtMap = PoseTable.loadFromArray(ndtMapTbl)
    
    for loc in orbLoc.table:
        loc.kfId = orbMap.findNearestByDistance(loc, True)
        loc.kf = orbMap[loc.kfId]

    # Fix axes
    for pose in orbLoc.table:
        x=pose.x
        y=pose.y
        z=pose.z
        pose.x=z
        pose.y=-x
        pose.z=-y
        x=pose.kf.x
        y=pose.kf.y
        z=pose.kf.z
        pose.kf.x=z
        pose.kf.y=-x
        pose.kf.z=-y
        
        ndtPose = ndtMap[pose.kfId]
        ndtPoseOffset = None
        try:
            ndtPoseOffset = ndtMap[pose.kfId-offset]
            kfOffset = orbMap[pose.kfId-offset]
        except KeyError:
            continue
        
        scale = np.linalg.norm(ndtPose.coord()-ndtPoseOffset.coord()) / \
            np.linalg.norm(pose.kf.coord()-kfOffset.coord())
        poseRel = Pose(0, ndtPose.x-pose.x, ndtPose.y-pose.y, ndtPose.z-pose.z)
            
        pose.cx = ndtPose.x + scale*poseRel.x
        pose.cy = ndtPose.y + scale*poseRel.y
        pose.cz = ndtPose.z + scale.poseRel.z

    return orbLoc, orbMap, ndtMap
    


def formatResultAsRecords (resultMat):
    records = PoseTable()
    for r in range(len(resultMat)) :
        id = int (resultMat[r][0])
        pose = Pose(resultMat[r][1:])
        records[id] = pose
    return records


if __name__ == '__main__' :
    
    segwayPoseMat = np.loadtxt('/media/sujiwo/TsukubaChallenge/TsukubaChallenge/20151103/run0-segway-bak.csv')
    orb1poses = PoseTable.loadFromBagFile('/media/sujiwo/TsukubaChallenge/TsukubaChallenge/20151103/localizerResults/run0-map1.bag', 'ORB_SLAM/World', 'ORB_SLAM/ExtCamera')
    orb2poses = PoseTable.loadFromBagFile('/media/sujiwo/TsukubaChallenge/TsukubaChallenge/20151103/localizerResults/run0-map3.bag', 'ORB_SLAM/World', 'ORB_SLAM/ExtCamera')
    joinOrbOdometry (orb1poses, orb2poses, segwayPoseMat)