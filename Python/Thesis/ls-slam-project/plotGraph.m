%   This source code is part of the graph optimization package
%   deveoped for the robotics lectures of the AIS lab at the 
%   University of Freiburg.
%  
%   Copyright (c) by Giorgio Grisetti, Gian Diego Tipaldi, Cyrill Stachniss
%  
%   It is licences under the Common Creative License,
%   Attribution-NonCommercial-ShareAlike 3.0
%  
%   You are free:
%     - to Share - to copy, distribute and transmit the work
%     - to Remix - to adapt the work
%  
%   Under the following conditions:
%  
%     - Attribution. You must attribute the work in the manner specified
%       by the author or licensor (but not in any way that suggests that
%       they endorse you or your use of the work).
%    
%     - Noncommercial. You may not use this work for commercial purposes.
%    
%     - Share Alike. If you alter, transform, or build upon this work,
%       you may distribute the resulting work only under the same or
%       similar license to this one.
%  
%   Any of the above conditions can be waived if you get permission
%   from the copyright holder.  Nothing in this license impairs or
%   restricts the author's moral rights.
%  
%   This software is distributed in the hope that it will be useful,
%   but WITHOUT ANY WARRANTY; without even the implied 
%   warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
%   PURPOSE.  

% plots a pose-graph consisting of vertices and edges
% v:    3xn matrix storing the x,y,theta coordinate of the n vertices
% eids: 2xm matrix where m is the number of edges. For each edge
%       the index of the vertices connected by the edge is provided.
function plotGraph(v, eids)
    clf;
    axis('square');
    hold on;
    plot (v(1,:),v(2,:),'+');
    plot (v(1,:),v(2,:));
    
    va = zeros(1,2*size(eids,2));
    vb = zeros(1,2*size(eids,2));
    
    for i=1:size(eids,2)
        va(2*i-1)=v(1,eids(1,i));
        va(2*i  )=v(1,eids(2,i));
        vb(2*i-1)=v(2,eids(1,i));
        vb(2*i  )=v(2,eids(2,i));
    end
    line(va,vb);
    hold off;
    pause(1);
end