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


more off;

% this loads the means of the vertices, the means of the edges, the edges ids and the edges information
% from the dat files
disp('Loading the graph file');

%% Select file to read in
[vmeans, eids, emeans,  einfs]=readGraph('intel-v.dat', 'intel-e.dat');
%[vmeans, eids, emeans,  einfs]=readGraph('killian-v.dat', 'killian-e.dat');
%[vmeans, eids, emeans,  einfs]=readGraph('test_quadrat-v.dat', 'test_quadrat-e.dat');

v = vmeans;

% this plots the input trajectory
disp('Plotting graph');
figure(1);
clf;
axis('square');
hold on;
plotGraph(v,eids);
pause(1)
disp('Graph plotted, press ENTER to continue');
pause();


vold=zeros(size(v));
diff = reshape(v,size(v,1)*size(v,2),1) - reshape(vold,size(vold,1)*size(vold,2),1);

it=1;
while (it<10 && diff'*diff > 0.01)  
    disp(' '),disp('Iteration:'),disp(it);
    % compute next iteration
    v=ls_slam(v, eids, emeans,  einfs, 1);

    % this plots result
    diff =reshape(v,size(v,1)*size(v,2),1) - reshape(vold,size(vold,1)*size(vold,2),1);
    disp('Squared difference in the node positions since last iteration:'),disp(diff'*diff);
    plotGraph(v,eids);
    it=it+1;
    vold=v;
end

disp('Finished.');