function [leftXs_w, rightXs_w,   ns_w, vs_w, ls_O] = GetRelVar(spaceLSs_w, imgLSs)

[row, col] = size(spaceLSs_w);

% 3d noise
noise3d = [0.0, 0.0, 0.0];
for i = 1:row
    spaceLSs_w(i, :) = spaceLSs_w(i, :) + noise3d;
end


numLSs = row / 2;

%% Line' variable
% 1. midXs_w
midXs_w = [];
leftXs_w = [];
rightXs_w = [];
for i = 1:numLSs
    midX_w = ((spaceLSs_w(2*i-1, :) + spaceLSs_w(2*i, :))') / 2;
    midXs_w = [midXs_w, midX_w];
    
    leftXs_w = [leftXs_w, spaceLSs_w(2*i-1, :)'];
    rightXs_w = [rightXs_w, spaceLSs_w(2*i, :)'];
end

% 2. ns_c
ns_c = [];
for i = 1:numLSs
    % imgLS's third and forth col stores end points' normalized coordinate
    p_N = [imgLSs(2*i-1, 3); imgLSs(2*i-1, 4); 1];
    q_N = [imgLSs(2*i, 3); imgLSs(2*i, 4); 1];
    n_c = cross(p_N, q_N);
    n_c = n_c / norm(n_c);
    
    ns_c = [ns_c, n_c];
end

% ns_w and vs_w construct Plucker Matrix(As Guoxuan Zhang said in Stereo SLAM p1366, the vector needn't to normalized)
% 3. ns_w
ns_w = []; % stores in col
for i = 1:numLSs
    n_w = -(cross(spaceLSs_w(2*i-1, :), spaceLSs_w(2*i, :)))'; % back ¡Á front
% 	n_w = n_w / norm(n_w);
    ns_w = [ns_w, n_w];
end

% 4. vs_w
vs_w = []; % stores in col
for i = 1:numLSs
    v_w = (spaceLSs_w(2*i-1, :) - spaceLSs_w(2*i, :))'; % front - back
    
% 	v_w = v_w / norm(v_w);
    vs_w = [vs_w, v_w];
end

% 5. ls_O

% 2d noise
% we can add noise at here
noise2d = 0.0;



ls_O = [];
for i = 1:numLSs
    % imgLS's first and second col stores end points' ordinary coordinate
    p_O = [imgLSs(2*i-1, 1) + noise2d; imgLSs(2*i-1, 2) + noise2d; 1];
    q_O = [imgLSs(2*i, 1) + noise2d; imgLSs(2*i, 2) + noise2d; 1];
    l_O = cross(p_O, q_O);
    l_O = l_O / norm(l_O);
    
    ls_O = [ls_O, l_O];
end




end