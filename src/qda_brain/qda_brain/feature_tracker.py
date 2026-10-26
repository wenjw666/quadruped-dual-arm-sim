"""ArUco identity with at most two KLT bridge frames and bidirectional checks."""
import cv2
import numpy as np


class MarkerTracker:
    def __init__(self, marker_id, max_bridge_frames=2, max_bridge_seconds=.45):
        self.marker_id=marker_id;self.max_bridge_frames=max_bridge_frames;self.max_bridge_seconds=max_bridge_seconds
        self.dictionary=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.gray=None;self.points=None;self.last_detection=None;self.last_stamp=None;self.misses=0;self.cached=None

    def detect(self, gray):
        corners,ids,_=cv2.aruco.detectMarkers(gray,self.dictionary)
        if ids is None or self.marker_id not in ids.flatten():return None
        return corners[list(ids.flatten()).index(self.marker_id)].reshape(4,2).astype(np.float32)

    def update(self,gray,stamp):
        if stamp==self.last_stamp and self.cached is not None:return self.cached
        if self.last_stamp is not None and stamp<=self.last_stamp:raise ValueError('IMAGE_TIME_REGRESSION')
        points=self.detect(gray);source='aruco'
        if points is None:
            if self.gray is None or self.misses>=self.max_bridge_frames or stamp-self.last_detection>self.max_bridge_seconds:
                raise ValueError('MARKER_LOST')
            previous=self.points.reshape(-1,1,2)
            points,status,error=cv2.calcOpticalFlowPyrLK(self.gray,gray,previous,None,winSize=(21,21),maxLevel=3)
            if points is None:raise ValueError('KLT_LOST')
            back,back_status,_=cv2.calcOpticalFlowPyrLK(gray,self.gray,points,None,winSize=(21,21),maxLevel=3)
            if (back is None or not np.all(status) or not np.all(back_status) or not np.all(np.isfinite(points))
                or np.max(np.linalg.norm(back-previous,axis=2))>.8 or np.max(error)>25
                or np.max(np.linalg.norm(points-previous,axis=2))>30):raise ValueError('KLT_QUALITY_REJECTED')
            points=points.reshape(4,2);source='klt';self.misses+=1
        else:self.misses=0;self.last_detection=stamp
        contour=points.astype(np.float32).reshape(-1,1,2)
        h,w=gray.shape
        if (not cv2.isContourConvex(contour) or abs(cv2.contourArea(contour))<64
            or np.any(points<2) or np.any(points[:,0]>w-3) or np.any(points[:,1]>h-3)):
            raise ValueError('INVALID_FEATURE_QUADRILATERAL')
        self.gray=gray.copy();self.points=points;self.last_stamp=stamp
        self.cached=(points,source,self.misses);return self.cached
