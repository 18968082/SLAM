import numpy as _np

def ploar_to_cartesian_2D(angle,distance):
    cartesian_coords = _np.zeros([2])
    for rho, phi in zip(distance,angle):
        x = rho * _np.cos(phi)
        y = rho * _np.sin(phi)
        cartesian_coords = _np.append(cartesian_coords,[x,y])
    cartesian_coords = _np.reshape(cartesian_coords,(-1,2))
    return _np.delete(cartesian_coords,0,0)

def cartesian_to_ploar_2D(cartesian_coords):
    polar_coords = _np.zeros([2])
    for x, y in cartesian_coords:
        rho = _np.sqrt(x**2 + y**2)
        phi = _np.arctan2(y, x)
        polar_coords = _np.vstack((polar_coords,[phi,rho]))
    return _np.delete(polar_coords,0,0)