
% Solves a graph-based slam problem via least squares
% vmeans: 3xn matrix containing the column vectors of the n poses of the vertices.
%	  The vertices are odrered such that vmeans(:,i)
%	  corresponds to the vertex with the i-th id
% eids:	  2xm matrix describing the edges. eids contains the column vectors [idFrom, idTo]' of the ids of the vertices
%	  eids(:,k) corresponds to emeans(:,k) and einfs(:,:,k).
% emeans: 3xm matrix containing the column vectors of the relative
%         poses represented by the edges. emeans(:,k) is the vector
%         correspdoning to the k-th edge.
% einfs:  3D matrix of dimension 3x3xm containing the information
%         matrices of the m edges. einfs(:,:,k) refers to the
%         information 
%         matrix of the k-th edge.
% n:	  number of iterations
% newmeans: 3xn matrix containing the column vectors of the updated vertices positions
function newmeans=ls_slam(vmeans, eids, emeans, einfs, n)
    for i=1:n,
        
        %% THIS NEEDS TO BE IMPLEMENTED
        
    end;
end;

