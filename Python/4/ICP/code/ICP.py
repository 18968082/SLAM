import numpy as _np
from sklearn.neighbors import NearestNeighbors as _NearestNeighbors

def matching(reference, read, number_of_neighbors):
    
    nbrs = _NearestNeighbors(n_neighbors=number_of_neighbors, algorithm='ball_tree').fit(reference)
    distances, indices = nbrs.kneighbors(read)
    
    matched_points = []
    
    for i,ref_point in enumerate(indices):
        
        single_point_matches = []
        
        for j,k in enumerate(ref_point):
            x,y = reference[k]
            single_point_matches.append([x,y])
            
        #single_matched_point = np.reshape(single_matched_point,(-1,2))
        #single_matched_point = np.delete(single_matched_point,0,0)
        matched_points.append(single_point_matches)
        
        #for j,save_point in enumerate(single_matched_point):
            #matched_points[j,:,i] = save_point 
            
    return matched_points,distances,indices

def transformation_2D(parameters,x):
    
    theta,tx,ty = parameters
    
    rotation_matrix = _np.array([[_np.cos(theta),_np.sin(theta)],[-1*_np.sin(theta),_np.cos(theta)]])
    
    translation_matrix = _np.array([tx,ty])
    
    new_cloud = _np.zeros(2)
    
    for point in x:
        point = _np.transpose(point)
        new_point = _np.transpose(_np.matmul(rotation_matrix,point) + translation_matrix)
        new_cloud = _np.vstack((new_cloud,new_point))
    
    return _np.delete(new_cloud,0,0)

def observation_normal_angle_minimization(normal_vectors,obse_sens_vectors):
    
    def unit_vector(vector):
        if not (vector.all() == 0):
            return vector / _np.linalg.norm(vector)
        else:
            return vector
    
    angle = []
    for i,norm in enumerate(normal_vectors):
        norm_angle = _np.arctan2(norm[1],norm[0])
        rotation_matrix = _np.array([[_np.cos(norm_angle),_np.sin(norm_angle)],[-1*_np.sin(norm_angle),_np.cos(norm_angle)]])
        angle_single_point = []
        for obse in obse_sens_vectors[i]:
            transformed_obse_vector = _np.matmul(rotation_matrix,_np.hstack(obse))
            transformed_obse_vector = unit_vector(transformed_obse_vector)

            angle_between = _np.arctan2(transformed_obse_vector[1],transformed_obse_vector[0])
            
            if(_np.absolute(angle_between) > _np.pi/2 and _np.absolute(angle_between) <= _np.pi):
                if(angle_between > 0):
                    angle_between = angle_between - _np.pi
                else:
                    angle_between = _np.pi + angle_between

            angle_single_point.append(angle_between)
        angle.append(angle_single_point)
    return angle

def outlier_rejection(distances_read_refnorm,angles_read_refnorm):
    rejection_weights_per_point = []
    rejection_weights = _np.zeros([distances_read_refnorm.shape[0],distances_read_refnorm.shape[0]])
    #for angles_to_multi_plane,distances_to_multi_plane in zip(angles_read_refnorm,distances_read_refnorm):
    #for angle_to_plane,distance_to_plane in zip(angles_to_multi_plane,distances_to_multi_plane):
    for angles_to_plane,distances_to_plane in zip(angles_read_refnorm,distances_read_refnorm):
        for single_angle,single_distance in zip(angles_to_plane,distances_to_plane):
            if(single_distance > 1) or (single_angle > _np.pi/3) or (single_angle < -1*_np.pi/3):
                rejection_weights_per_point = _np.hstack((rejection_weights_per_point,0))
            else:
                rejection_weights_per_point = _np.hstack((rejection_weights_per_point,1))
                
    #for j,weight in enumerate(rejection_weights_per_point):
        #rejection_weights[j,-1,i] = weight
        
    rejection_weights_per_point = _np.vstack(rejection_weights_per_point)
    rejection_weights = _np.delete(rejection_weights,-1,1)
    rejection_weights = _np.hstack((rejection_weights, rejection_weights_per_point))
    return rejection_weights