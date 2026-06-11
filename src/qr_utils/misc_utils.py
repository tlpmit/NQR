import datetime
import math
import os
import pickle

# from qr_utils.traceFile import tr


def time_string():
    return (
        str(datetime.datetime.now())
        .replace(" ", "_")
        .replace(":", "_")
        .replace(".", "_")
    )


_log_dir_root = {
    "HPN": os.path.expanduser("~/.HPN_logs/"),
    "QR": os.path.expanduser("~/.QR_logs/"),
}
_current_log_dir = {"HPN": None, "QR": None}
_current_log_file_num = {"HPN": -1, "QR": -1}


def current_log_dir(hpn_or_qr="HPN"):
    if _current_log_dir[hpn_or_qr] is None:
        new_log_dir("log", hpn_or_qr)
    return _current_log_dir[hpn_or_qr]


def current_log_file_num(hpn_or_qr="HPN"):
    return _current_log_file_num[hpn_or_qr]


def increment_log_file_num(hpn_or_qr="HPN"):
    _current_log_file_num[hpn_or_qr] += 1


def reset_log_file(hpn_or_qr="HPN"):
    _current_log_file_num[hpn_or_qr] = -1


def new_log_dir(file_tag, hpn_or_qr="HPN"):
    """Make a new directory in log_dir_root with name and timestamp"""
    _current_log_dir[hpn_or_qr] = (
        _log_dir_root[hpn_or_qr] + file_tag + "_" + time_string() + "/"
    )
    if not os.path.exists(_current_log_dir[hpn_or_qr]):
        os.makedirs(_current_log_dir[hpn_or_qr], exist_ok=True)
    reset_log_file(hpn_or_qr)


def pickle_to_QR_log_dir(obj, file_tag, hpn_or_qr="QR", increment_num=True):
    """Pickle obj to a file in the current log directory"""
    if increment_num:
        increment_log_file_num(hpn_or_qr)

    file_name = (
        current_log_dir(hpn_or_qr) + f"{file_tag}_{current_log_file_num(hpn_or_qr)}.pkl"
    )
    # tr('log', 'Saving pickles:', file_name)
    with open(file_name, "wb") as f:
        pickle.dump(obj, f)


def load_from_pkl_dir(pkl_path, file_tag, increment_num=True):
    if increment_num:
        increment_log_file_num(hpn_or_qr="QR")
    file_name = (
        pkl_path + "/" + f"{file_tag}_{current_log_file_num(hpn_or_qr='QR')}.pkl"
    )
    if os.path.exists(file_name):
        with open(file_name, "rb") as f:
            print("Loading pickles", file_name)
            return pickle.load(f)


class SymbolGenerator:
    """
    Generate new symbols guaranteed to be different from one another
    Optionally, supply a prefix for mnemonic purposes
    Call gensym("foo") to get a symbol like 'foo37'
    """

    def __init__(self):
        self.count = 0

    def gensym(self, prefix="i", zeroPadded=False):
        self.count += 1
        if zeroPadded:
            return (prefix + "_%08i") % self.count
        else:
            return prefix + "_" + str(self.count)


gensym = SymbolGenerator().gensym
"""Call this function to get a new symbol"""


class Pose2D:
    """
    Represent the x, y, theta pose of an object in 2D space
    """

    x = 0.0
    y = 0.0
    theta = 0.0

    def __init__(self, x, y, theta):
        self.x = x
        """x coordinate"""
        self.y = y
        """y coordinate"""
        self.theta = theta
        """rotation in radians"""

    def point(self):
        """
        Return just the x, y parts represented as a C{util.Point}
        """
        return Point2D(self.x, self.y)

    def isNear(self, pose, distEps, angleEps):
        """
        @returns: True if pose is within distEps and angleEps of self
        """
        return self.point().isNear(pose.point(), distEps) and nearAngle(
            self.theta, pose.theta, angleEps
        )

    def distance(self, pose):
        """
        @param pose: an instance of C{util.Pose}
        @returns: the distance between the x,y part of self and the x,y
        part of pose.
        """
        return self.point().distance(pose.point())

    def xytTuple(self):
        """
        @returns: a representation of this pose as a tuple of x, y,
        theta values
        """
        return (self.x, self.y, self.theta)

    def __repr__(self):
        return f"pose:({self.x:.4f}, {self.y:.4f}, {self.theta:.4f})"


class Point2D:
    """
    Represent a point with its x, y values
    """

    x = 0.0
    y = 0.0

    def __init__(self, x, y):
        self.x = float(x)
        """x coordinate"""
        self.y = float(y)
        """y coordinate"""

    def near(self, point, distEps):
        """
        @param point: instance of C{util.Point}
        @param distEps: positive real number
        @returns: true if the distance between C{self} and C{util.Point} is less
        than distEps
        """
        return self.distance(point) < distEps

    # This is hear for backward compatibility
    isNear = near

    def distance(self, point):
        """
        @param point: instance of C{util.Point}
        @returns: Euclidean distance between C{self} and C{util.Point}
        """
        return math.sqrt((self.x - point.x) ** 2 + (self.y - point.y) ** 2)

    def magnitude(self):
        """
        @returns: Magnitude of this point, interpreted as a vector in
        2-space
        """
        return math.sqrt(self.x**2 + self.y**2)

    def xyTuple(self):
        """
        @returns: pair of x, y values
        """
        return (self.x, self.y)

    def __repr__(self):
        return "point:" + str(self.xyTuple())

    def angleTo(self, p):
        """
        @param p: instance of C{util.Point} or C{util.Pose}
        @returns: angle in radians of vector from self to p
        """
        dx = p.x - self.x
        dy = p.y - self.y
        return math.atan2(dy, dx)

    def add(self, point):
        """
        Vector addition
        """
        return Point2D(self.x + point.x, self.y + point.y)

    def __add__(self, point):
        return self.add(point)

    def sub(self, point):
        """
        Vector subtraction
        """
        return Point2D(self.x - point.x, self.y - point.y)

    def __sub__(self, point):
        return self.sub(point)

    def scale(self, s):
        """
        Vector scaling
        """
        return Point2D(self.x * s, self.y * s)

    def __rmul__(self, s):
        return self.scale(s)

    def dot(self, p):
        """
        Dot product
        """
        return self.x * p.x + self.y * p.y


def fixAnglePlusMinusPi(a):
    """
    A is an angle in radians;  return an equivalent angle between plus
    and minus pi
    """
    return ((a + math.pi) % (2 * math.pi)) - math.pi


def angle_diff(x, y):
    twoPi = 2 * math.pi
    z = (x - y) % twoPi
    if z > math.pi:
        return z - twoPi
    else:
        return z


def nearAngle(a1, a2, eps):
    """
    @param a1: number representing angle; no restriction on range
    @param a2: number representing angle; no restriction on range
    @param eps: positive number
    @returns: C{True} if C{a1} is within C{eps} of C{a2}.  Don't use
    within for this, because angles wrap around!
    """
    return abs(fixAnglePlusMinusPi(a1 - a2)) < eps


def clip(v, vMin, vMax):
    """
    @param v: number
    @param vMin: number (may be None, if no limit)
    @param vMax: number greater than C{vMin} (may be None, if no limit)
    @returns: If C{vMin <= v <= vMax}, then return C{v}; if C{v <
    vMin} return C{vMin}; else return C{vMax}
    """
    if vMin is None:
        if vMax is None:
            return v
        else:
            return min(v, vMax)
    else:
        if vMax is None:
            return max(v, vMin)
        else:
            return max(min(v, vMax), vMin)


