"""Robust scan-to-wall localisation for the fixed official 10 m arena.

Only static outer walls are used. Internal shelf/table/robot returns are trimmed.
The initial position is the prescribed Start zone; heading is searched globally.
"""
import numpy as np
from scipy.optimize import least_squares


def world_points(points, pose):
    x,y,a = pose
    c,s = np.cos(a),np.sin(a)
    return points @ np.array([[c,s],[-s,c]]) + [x,y]


def distances(points, pose):
    p = world_points(points,pose)
    return np.c_[p[:,0]+3.975,p[:,0]-5.975,p[:,1]+4.975,p[:,1]-4.975]


class WallLocalizer:
    def __init__(self):
        self.pose = None
        self.error = float('inf')
        self.points = None

    def update(self, points):
        """Fit laser returns to the outer walls, ignoring most interior objects."""
        self.points = points
        if len(points)<80:
            return None
        if self.pose is None:
            candidates = []
            for a in np.linspace(-np.pi,np.pi,145)[:-1]:
                guess = np.array([0.,0.,a])
                d = np.min(abs(distances(points,guess)),axis=1)
                candidates.append((np.sum(np.minimum(d,.3)**2),guess))
            guess = min(candidates,key=lambda pair:pair[0])[1]
        else:
            guess = self.pose.copy()
        for _ in range(4):
            d = distances(points,guess)
            closest = np.argmin(abs(d),axis=1)
            good = np.min(abs(d),axis=1)<.35
            if good.sum()<50 or not (np.any(closest[good]<2) and np.any(closest[good]>=2)):
                return None
            chosen,wall = points[good],closest[good]
            def residual(p):
                return distances(chosen,p)[np.arange(len(chosen)),wall]
            fit = least_squares(residual,guess,loss='soft_l1',f_scale=.025,max_nfev=25)
            guess = fit.x
        error = float(np.median(abs(residual(guess))))
        if error>.06 or not (-3.5<guess[0]<5.5 and -4.5<guess[1]<4.5):
            return None
        self.pose,self.error = guess,error
        return guess

    def initialize_shelf(self, point, normal):
        """Use the visible shelf to resolve the square room's heading ambiguity."""
        yaw = -np.arctan2(normal[1],normal[0])
        x = 2.755-float(point @ normal)
        candidates = []
        if self.points is None:
            return False
        for y in np.linspace(-3.,3.,61):
            guess = np.array([x,y,yaw])
            d = np.min(abs(distances(self.points,guess)),axis=1)
            candidates.append((np.sum(np.minimum(d,.3)**2),guess))
        self.pose = min(candidates,key=lambda pair:pair[0])[1]
        return self.update(self.points) is not None
