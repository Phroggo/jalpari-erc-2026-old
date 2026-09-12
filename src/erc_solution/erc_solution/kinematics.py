"""Read the arm geometry from the URDF and solve its joint poses."""
import xml.etree.ElementTree as ET
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation


def transform(xyz=(0, 0, 0), rpy=(0, 0, 0)):
    """Build a 4x4 pose matrix from metres and roll/pitch/yaw in radians."""
    result = np.eye(4)
    result[:3, :3] = Rotation.from_euler('xyz', rpy).as_matrix()
    result[:3, 3] = xyz
    return result


def transport_path(position):
    """Upright carrying poses, used only after retreating clear of the shelf.

    The lowest row must first move outward to the right: a straight inward/up
    line crosses a joint-limit hole in this arm's upright-tool workspace.
    """
    position = np.asarray(position,dtype=float).copy()
    endpoints = []
    if position[2] < 1.10:
        outward = position.copy()
        outward[1] = -.50
        endpoints.append(outward)
    endpoints.append(np.array([.50,-.24,1.28]))
    goals = []
    for endpoint in endpoints:
        goals.extend(transform(position+a*(endpoint-position))
                     for a in np.linspace(0.,1.,9)[1:])
        position = endpoint
    return goals


class Kinematics:
    def __init__(self, path, base='torso_lift_link', tip='gripper_right_grasping_link'):
        root = ET.parse(path).getroot()
        parents = {j.find('child').get('link'): j for j in root.findall('joint')}
        chain = []
        while tip != base:
            j = parents[tip]
            chain.append(j)
            tip = j.find('parent').get('link')
        self.chain, self.names, self.lower, self.upper = [], [], [], []
        for j in reversed(chain):
            origin = j.find('origin')
            xyz = [float(v) for v in origin.get('xyz', '0 0 0').split()] if origin is not None else [0]*3
            rpy = [float(v) for v in origin.get('rpy', '0 0 0').split()] if origin is not None else [0]*3
            axis = j.find('axis')
            axis = np.array([float(v) for v in axis.get('xyz').split()]) if axis is not None else np.array([1., 0., 0.])
            movable = j.get('type') == 'revolute'
            if movable:
                limit = j.find('limit')
                self.names.append(j.get('name'))
                self.lower.append(float(limit.get('lower')) + .005)
                self.upper.append(float(limit.get('upper')) - .005)
            elif j.get('type') != 'fixed':
                raise ValueError('Arm chain must contain only revolute/fixed joints')
            self.chain.append((transform(xyz, rpy), axis, movable))
        self.lower, self.upper = np.array(self.lower), np.array(self.upper)

    def fk(self, q):
        """Get the tool pose from joint angles, relative to the chain's base."""
        result, i = np.eye(4), 0
        for fixed, axis, movable in self.chain:
            result = result @ fixed
            if movable:
                turn = np.eye(4)
                turn[:3, :3] = Rotation.from_rotvec(axis*q[i]).as_matrix()
                result = result @ turn
                i += 1
        return result

    def points(self, q):
        """Return joint/link origins for rough arm-clearance checks."""
        result, i, points = np.eye(4), 0, []
        for fixed,axis,movable in self.chain:
            result = result @ fixed
            if movable:
                turn = np.eye(4)
                turn[:3,:3] = Rotation.from_rotvec(axis*q[i]).as_matrix()
                result = result @ turn
                i += 1
            points.append(result[:3,3].copy())
        return np.array(points)

    def solve(self, goal, current):
        """Find joints that match both tool position and angle.

        The grasp frame's X axis points out of the fingers. Rotation vectors
        keep the angle error usable even when the tool is 180 degrees off.
        """
        current = np.clip(current, self.lower, self.upper)
        def residual(q):
            actual = self.fk(q)
            rotation = Rotation.from_matrix(goal[:3, :3] @ actual[:3, :3].T).as_rotvec()
            return np.r_[4*(actual[:3, 3]-goal[:3, 3]), rotation, .005*(q-current)]
        best = None
        seeds = [current, np.clip([-.5, -.6, 0, 1.4, 0, -.8, 0], self.lower, self.upper)]
        rng = np.random.default_rng(2026)
        # Repeatable IK guesses only; this does not seed the simulation layout.
        seeds.extend(rng.uniform(self.lower,self.upper) for _ in range(10))
        for seed in seeds:
            sol = least_squares(residual, seed, bounds=(self.lower, self.upper), max_nfev=160)
            actual = self.fk(sol.x)
            pe = np.linalg.norm(actual[:3, 3]-goal[:3, 3])
            re = Rotation.from_matrix(goal[:3, :3] @ actual[:3, :3].T).magnitude()
            if best is None or pe+re < best[1]+best[2]:
                best = (sol.x, pe, re)
            if pe < .008 and re < .045:
                return sol.x
        raise RuntimeError(f'Unreachable grasp pose: position error {best[1]:.3f} m, orientation {best[2]:.3f} rad')

    def startup_waypoint(self,current,side=-1):
        goal = np.array([.25,side*.32,.10])
        seed = np.clip(current,self.lower,self.upper)
        fit = least_squares(lambda q:np.r_[10*(self.fk(q)[:3,3]-goal),.02*(q-seed)],
                            seed,bounds=(self.lower,self.upper),max_nfev=200)
        if np.linalg.norm(self.fk(fit.x)[:3,3]-goal)>.01:
            raise RuntimeError('Cannot reach raised startup waypoint')
        return fit.x

    def compact(self, current, side=-1):
        """Find a tucked pose with limited forward and sideways arm reach."""
        goal = np.array([.25,side*.32,.10])
        seed = np.clip(current,self.lower,self.upper)
        def residual(q):
            points = self.points(q)
            return np.r_[2*(points[-1]-goal),.02*(q-seed),
                         30*np.maximum(points[:,0]-.38,0),
                         10*np.maximum(abs(points[:,1])-.50,0)]
        rng = np.random.default_rng(19)
        for initial in [seed]+[rng.uniform(self.lower,self.upper) for _ in range(8)]:
            fit = least_squares(residual,initial,bounds=(self.lower,self.upper),max_nfev=200)
            points = self.points(fit.x)
            if np.linalg.norm(points[-1]-goal)<.18 and points[:,0].max()<.40 and abs(points[:,1]).max()<.53:
                return fit.x
        raise RuntimeError('Cannot find whole-arm compact startup posture')
