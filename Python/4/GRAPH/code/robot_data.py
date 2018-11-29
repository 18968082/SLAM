import numpy as _np
import coordinate_transforms as _cord_trans

class Lidar_Scan:
    def __init__(self, angles, ranges):
        self.ranges = ranges
        self.angles = angles
        self.cartesian = _cord_trans.ploar_to_cartesian_2D(self.angles,self.ranges)
    def set_pose(self, xy_coordinates):
        self.pose = xy_coordinates

class Load_Lidar_Scans:    
    def load_angles(self,angles_location):
        self.LidarScan_angles = _np.loadtxt(angles_location, dtype='f', delimiter=' ',usecols = _np.linspace(1,240,240,dtype='int'))
        self.num_of_LidarScans_angles = int(self.LidarScan_angles.shape[0])
        
    def load_ranges(self,ranges_location):
        LidarScan_ranges = _np.loadtxt(ranges_location, dtype='f', delimiter=' ',usecols = _np.linspace(1,241,241,dtype='int'))
        self.LidarScan_ranges = _np.delete(LidarScan_ranges, 0, 1)
        self.num_of_LidarScans = int(LidarScan_ranges[0,0])
        
class robot_trajectory:
    def load_trajectory(self,trajectory_location):
        _np.set_printoptions(formatter={'float_kind':lambda x: "%2.5f" % x})
        self.trajectory = _np.loadtxt(trajectory_location, dtype='f', delimiter=' ',usecols = (1,2,3))
        self.trajectory_dimension = self.trajectory.shape[1]-1
        self.trajectory_length = self.trajectory.shape[0]