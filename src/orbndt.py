
from __future__ import division
import numpy as np
import datetime
import rosbag
import rospy
from copy import copy, deepcopy
from exceptions import KeyError, ValueError
import matplotlib.pyplot as plt


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
                if ppose.timestamp - cpose.tolerance > tolerance:
                    continue
            dist = np.linalg.norm([cpose.x-ppose.x, cpose.y-ppose.y, cpose.z-ppose.z])
            totaldist += dist
        return totaldist
    
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
        i = 0        
        for p in self.table:
            tdiff = abs(p.timestamp - timestamp)
            if (tdiff < tolerance):
                candidates.add(p)
                if p.timestamp > timestamp:
                    break
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
    
    t = startTime
    joinPoses = []
    while t < stopTime:
        cPoses = [ptbl.findNearestInTime(t, timeRez) for ptbl in poseTbls]
        if (len(cPoses) != 0):
            joinPoses.append(cPoses)
        t += timeRez

    return joinPoses        


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
    orb1 = PoseTable.loadFromBagFile('/home/sujiwo/Data/TsukubaChallenge/run2-map1.bag', 'ORB_SLAM/World', 'ORB_SLAM/ExtCamera')
    orb2 = PoseTable.loadFromBagFile('/home/sujiwo/Data/TsukubaChallenge/run2-map3.bag', 'ORB_SLAM/World', 'ORB_SLAM/ExtCamera')
    orbj = joinPoseTables (orb1, orb2)