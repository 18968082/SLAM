from matplotlib import pyplot as _plt

def plot_scan_surfaces(S_cartesian,highlighted_section_cartesian):

    _plt.figure()

    _plt.plot(S_cartesian[:,0],S_cartesian[:,1],'.')
    _plt.plot(highlighted_section_cartesian[:,0],highlighted_section_cartesian[:,1],'.')
    _plt.plot(0,0,'x')
    _plt.grid(color='k', linestyle='--', linewidth=0.5)
    
    return
    
def plot_custom_axes(axis,origin_coords):#(axis_list,origin_coords):
#    for axis in axis_list:
#        axis /= axis.std()
#        x_axis, y_axis = axis
#        
#        _plt.plot(0.1 * x_axis, 0.1 * y_axis, linewidth=0.01)
#        _plt.quiver(origin_coords[0], origin_coords[1], x_axis, y_axis, zorder=11, width=0.005, scale=15)

    x_axis, y_axis = axis

    _plt.plot(0.1 * x_axis, 0.1 * y_axis, linewidth=0.01)
    _plt.quiver(origin_coords[0], origin_coords[1], x_axis, y_axis, zorder=11, width=0.005, scale=15)

    return

def plot_points(S_cartesian,count):

    if(count):
        _plt.figure()

        _plt.plot(S_cartesian[:,0],S_cartesian[:,1],'.')
        
        count = 0
        for x, y in zip(S_cartesian[:,0], S_cartesian[:,1]):
            _plt.text(x, y, str(count), color="black", fontsize=12)
            count = count + 1
            
        _plt.grid(color='k', linestyle='--', linewidth=0.5)
    else:
        _plt.figure()

        _plt.plot(S_cartesian[:,0],S_cartesian[:,1],'.')
        _plt.grid(color='k', linestyle='--', linewidth=0.5)
    
    return    