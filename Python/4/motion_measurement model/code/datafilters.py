import numpy as _np
from sklearn.decomposition import PCA as _PCA
from sklearn.neighbors import NearestNeighbors as _NearestNeighbors

class filters:
    def subsample(S, n):
        
        S_filtered = S[0:S.size:n]
        
        return S_filtered

    def random_sample_uniform(S,prob):
        
        S_filtered = _np.zeros(2)
        
        for i in range(S.shape[0]):
            
            sampled_prob = _np.random.uniform(0,1,1)
            
            if(prob < sampled_prob):
                S_filtered = _np.append(S_filtered,S[i])
                
            S_filtered = _np.reshape(S_filtered,(-1,2))
            
        return _np.delete(S_filtered,0,0)
    
    def bounded_box(S,x_boundries,y_boundries):
        
        x_min, x_max = x_boundries
        y_min, y_max = y_boundries

        S_filtered = _np.zeros(2)

        for i in range(S.shape[0]):
            
            x, y = S[i]
            
            if ((x <= x_max)&(x >= x_min)&(y <= y_max)&(y >= y_min)):
                S_filtered = _np.append(S_filtered,[x,y])
                
            S_filtered = _np.reshape(S_filtered,(-1,2))
            
        return _np.delete(S_filtered,0,0)
    
    def surface_normals(S,number_of_neighbors):

        nbrs = _NearestNeighbors(n_neighbors=number_of_neighbors, algorithm='kd_tree').fit(S)
        distances, indices = nbrs.kneighbors(S)

        pca = _PCA()
        normals_S = []#np.zeros(2)
        principle_axes = []#np.zeros([2,2])
        surface_region = _np.zeros([1,S.shape[1]])

        for i in range(indices.shape[0]):

            for j in indices[i]:
                surface_region = _np.append(surface_region,S[j])

            surface_region = _np.reshape(surface_region,(-1,S.shape[1]))
            surface_region = _np.delete(surface_region,0,0)

            pca_region = pca.fit(surface_region)

            principle_axes.append(pca_region.components_)
            #principle_axes = _np.append(principle_axes,pca_region.components_)
            #normals_S = np.append(normals_S,pca_region.components_[1])
            normals_S.append(pca_region.components_[1])

        #normals_S = np.reshape(normals_S,(-1,2))
        #principle_axes = np.reshape(principle_axes,(-1,2,2))

        #return [np.delete(normals_S,0,0),principle_axes,distances]
        return normals_S,principle_axes,distances
    
    def orientate_normals(normal_vectors,observation_directions):
        
        i = 0
        for norm,obse in zip(normal_vectors,observation_directions):
            normal_observation_angle_orientation = _np.dot(norm,obse)
            if(normal_observation_angle_orientation > 0):
                normal_vectors[i] = _np.multiply(norm,-1)
            i = i + 1
        return normal_vectors
    
    def NaN_removal(S):
        S_nan_filtered = _np.zeros(2)

        for i in range(S.shape[0]):
            if not ((_np.isnan(S[i,0]))|(_np.isnan(S[i,1]))):
                S_nan_filtered = _np.append(S_nan_filtered,S[i])

        S_nan_filtered = _np.reshape(S_nan_filtered,(-1,2))

        return _np.delete(S_nan_filtered,0,0)    
    
    def observation_to_sensor_vectors_polar(angles,ranges):
        
        observation_sensor_vectors = _np.zeros(2)

        for i in range(ranges.shape[0]):
            observation_sensor_vectors = _np.append(observation_sensor_vectors,(angles[i],1))

        observation_sensor_vectors = _np.reshape(observation_sensor_vectors,(-1,2))

        return _np.delete(observation_sensor_vectors,0,0)

    def sensor_to_observation_vectors_cartesian(S):

        def unit_vector(vector):
                return vector / _np.linalg.norm(vector)

        observation_sensor_vectors = _np.zeros(2)

        for x,y in S:
            observation_sensor_vectors = _np.append(observation_sensor_vectors,[x,y])

        observation_sensor_vectors = _np.reshape(observation_sensor_vectors,(-1,2))
        observation_sensor_vectors = _np.delete(observation_sensor_vectors,0,0)

        for i,vector in enumerate(observation_sensor_vectors):
            observation_sensor_vectors[i] = unit_vector(vector)

        return observation_sensor_vectors
    
    def read_reference_normal_vectors(read,reference):

        def unit_vector(vector):
            if not (vector.all() == 0):
                return vector / _np.linalg.norm(vector)
            else:
                return vector

        normalized_read_reference_vectors = []
        for i in range(read.shape[0]):
            point_read = read[i]
            single_reference_matched = []
            for reference_matched in reference[i]:
                vector = unit_vector(reference_matched - point_read)
                single_reference_matched.append(vector)
            normalized_read_reference_vectors.append(single_reference_matched)

        return normalized_read_reference_vectors