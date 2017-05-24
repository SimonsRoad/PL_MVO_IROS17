function coeffMat = expressTransByRotMinCase(n_c_Ws, v_c_Ws, l_ch_os, K, c_Ws, c_ns, minTypeFlag) % First 4 for Ls, Last 2 for Pt
% num of feature is corresponding to flag 

if minTypeFlag == 1 % 3Ls
    coeffMat = expressTransByRot_3Ls(n_c_Ws, v_c_Ws, l_ch_os, K);
end

if minTypeFlag == 2 % 3Pt
    coeffMat = expressTransByRot_3Pt(c_Ws, c_ns);
end

if minTypeFlag == 3 % 2Ls_1Pt
    coeffMat = expressTransByRot_2Ls_1Pt(n_c_Ws, v_c_Ws, l_ch_os, K, c_Ws, c_ns);
end

if minTypeFlag == 4 % 1Ls_2Pt
    coeffMat = expressTransByRot_1Ls_2Pt(n_c_Ws, v_c_Ws, l_ch_os, K, c_Ws, c_ns);
end

end


%% 3Ls
function coeffMat = expressTransByRot_3Ls(n_c_Ws, v_c_Ws, l_ch_os, K)

% Constraint 1:
%                                             0
%  (v_cX_W * l' * K) * t = -l' * K * R * [  -n_cZ_W  ]
%                                            n_cY_W

% Constraint 2:
%                                            n_cZ_W
%  (v_cY_W * l' * K) * t = -l' * K * R * [    0      ]
%                                           -n_cX_W

syms b c d real
R = [1 + b*b - c*c - d*d, 2 * (b*c - d), 2 * (b*d + c);
    2 * (c*b + d), 1 - b*b + c*c - d*d, 2 * (c*d - b);
    2 * (d*b - c), 2 * (d*c + b), 1 - b*b - c*c + d*d];

A = [];
B = [];

for i = 1:3 % ***Use 3 lines***
    n_cX_W = n_c_Ws(1, i);
    n_cY_W = n_c_Ws(2, i);
    n_cZ_W = n_c_Ws(3, i);
    
    v_cX_W = v_c_Ws(1, i);
    v_cY_W = v_c_Ws(2, i);
    v_cZ_W = v_c_Ws(3, i);
    
    l = l_ch_os(:, i);
    
    % Constraint 1:
    oneElemOfA = v_cX_W * l' * K; 
    A = [A; oneElemOfA];
    oneElemOfB = -l' * K * R * [0; -n_cZ_W; n_cY_W];
    B = [B; oneElemOfB];
    
    % Constraint 2:
    oneElemOfA = v_cY_W * l' * K; 
    A = [A; oneElemOfA];
    oneElemOfB = -l' * K * R * [n_cZ_W; 0; -n_cX_W];
    B = [B; oneElemOfB];
end

t = (A' * A)^-1 * (A' * B);
t = vpa(t, 4);

t_elem1 = t(1);
expCoeffMat1 = coeffs(t_elem1);
expCoeffMat1 = vpa(fliplr(expCoeffMat1), 4);

t_elem2 = t(2);
expCoeffMat2 = coeffs(t_elem2);
expCoeffMat2 = vpa(fliplr(expCoeffMat2), 4);

t_elem3 = t(3);
expCoeffMat3 = coeffs(t_elem3);
expCoeffMat3 = vpa(fliplr(expCoeffMat3), 4);

coeffMat = [];

coeffMat = [coeffMat; expCoeffMat1];
coeffMat = [coeffMat; expCoeffMat2];
coeffMat = [coeffMat; expCoeffMat3];

end


%% 3Pt
function coeffMat = expressTransByRot_3Pt(c_Ws, c_ns)

% Constraint 1:
% [ 0, -1, cv_n ] * t = - [ 0, -1, cv_n ] * R * c_W 

% Constraint 2:
% [ 1, 0, -cu_n ] * t = - [ 1, 0, -cu_n ] * R * c_W 

A = [];
B = [];

syms b c d real
R = [1 + b*b - c*c - d*d, 2 * (b*c - d), 2 * (b*d + c);
    2 * (c*b + d), 1 - b*b + c*c - d*d, 2 * (c*d - b);
    2 * (d*b - c), 2 * (d*c + b), 1 - b*b - c*c + d*d];

for i = 1:3 % ***Use 3 points***
    oneElemOfA = [0, -1, c_ns(2, i)];
    A = [A; oneElemOfA];
    oneElemOfB = - [0, -1, c_ns(2, i)] * R * c_Ws(:, i);
    B = [B; oneElemOfB];
    
    oneElemOfA = [1, 0, -c_ns(1, i)];
    A = [A; oneElemOfA];
    oneElemOfB = - [1, 0, -c_ns(1, i)] * R * c_Ws(:, i);
    B = [B; oneElemOfB];
end

t = (A' * A)^-1 * (A' * B);
t = vpa(t, 4);

t_elem1 = t(1);
expCoeffMat1 = coeffs(t_elem1);
expCoeffMat1 = vpa(fliplr(expCoeffMat1), 4);

t_elem2 = t(2);
expCoeffMat2 = coeffs(t_elem2);
expCoeffMat2 = vpa(fliplr(expCoeffMat2), 4);

t_elem3 = t(3);
expCoeffMat3 = coeffs(t_elem3);
expCoeffMat3 = vpa(fliplr(expCoeffMat3), 4);

coeffMat = [];

coeffMat = [coeffMat; expCoeffMat1];
coeffMat = [coeffMat; expCoeffMat2];
coeffMat = [coeffMat; expCoeffMat3];

end


%% 2Ls_1Pt
function coeffMat = expressTransByRot_2Ls_1Pt(n_c_Ws, v_c_Ws, l_ch_os, K, c_Ws, c_ns)

A = [];
B = [];

syms b c d real
R = [1 + b*b - c*c - d*d, 2 * (b*c - d), 2 * (b*d + c);
    2 * (c*b + d), 1 - b*b + c*c - d*d, 2 * (c*d - b);
    2 * (d*b - c), 2 * (d*c + b), 1 - b*b - c*c + d*d];

for i = 1:2 % ***Use 2 lines***
    n_cX_W = n_c_Ws(1, i);
    n_cY_W = n_c_Ws(2, i);
    n_cZ_W = n_c_Ws(3, i);
    
    v_cX_W = v_c_Ws(1, i);
    v_cY_W = v_c_Ws(2, i);
    v_cZ_W = v_c_Ws(3, i);
    
    l = l_ch_os(:, i);
    
    % Constraint 1:
    oneElemOfA = v_cX_W * l' * K; 
    A = [A; oneElemOfA];
    oneElemOfB = -l' * K * R * [0; -n_cZ_W; n_cY_W];
    B = [B; oneElemOfB];
    
    % Constraint 2:
    oneElemOfA = v_cY_W * l' * K; 
    A = [A; oneElemOfA];
    oneElemOfB = -l' * K * R * [n_cZ_W; 0; -n_cX_W];
    B = [B; oneElemOfB];
end

for i = 1:1 % ***Use 1 points***
    % Constraint 1:
    oneElemOfA = [0, -1, c_ns(2, i)];
    A = [A; oneElemOfA];
    oneElemOfB = - [0, -1, c_ns(2, i)] * R * c_Ws(:, i);
    B = [B; oneElemOfB];
    
    % Constraint 2:
    oneElemOfA = [1, 0, -c_ns(1, i)];
    A = [A; oneElemOfA];
    oneElemOfB = - [1, 0, -c_ns(1, i)] * R * c_Ws(:, i);
    B = [B; oneElemOfB];
end

t = (A' * A)^-1 * (A' * B);
t = vpa(t, 4);

t_elem1 = t(1);
expCoeffMat1 = coeffs(t_elem1);
expCoeffMat1 = vpa(fliplr(expCoeffMat1), 4);

t_elem2 = t(2);
expCoeffMat2 = coeffs(t_elem2);
expCoeffMat2 = vpa(fliplr(expCoeffMat2), 4);

t_elem3 = t(3);
expCoeffMat3 = coeffs(t_elem3);
expCoeffMat3 = vpa(fliplr(expCoeffMat3), 4);

coeffMat = [];

coeffMat = [coeffMat; expCoeffMat1];
coeffMat = [coeffMat; expCoeffMat2];
coeffMat = [coeffMat; expCoeffMat3];


end


%% 2Pt_1Ls
function coeffMat = expressTransByRot_1Ls_2Pt(n_c_Ws, v_c_Ws, l_ch_os, K, c_Ws, c_ns)

A = [];
B = [];

syms b c d real
R = [1 + b*b - c*c - d*d, 2 * (b*c - d), 2 * (b*d + c);
    2 * (c*b + d), 1 - b*b + c*c - d*d, 2 * (c*d - b);
    2 * (d*b - c), 2 * (d*c + b), 1 - b*b - c*c + d*d];

for i = 1:1 % ***Use 1 lines***
    n_cX_W = n_c_Ws(1, i);
    n_cY_W = n_c_Ws(2, i);
    n_cZ_W = n_c_Ws(3, i);
    
    v_cX_W = v_c_Ws(1, i);
    v_cY_W = v_c_Ws(2, i);
    v_cZ_W = v_c_Ws(3, i);
    
    l = l_ch_os(:, i);
    
    % Constraint 1:
    oneElemOfA = v_cX_W * l' * K; 
    A = [A; oneElemOfA];
    oneElemOfB = -l' * K * R * [0; -n_cZ_W; n_cY_W];
    B = [B; oneElemOfB];
    
    % Constraint 2:
    oneElemOfA = v_cY_W * l' * K; 
    A = [A; oneElemOfA];
    oneElemOfB = -l' * K * R * [n_cZ_W; 0; -n_cX_W];
    B = [B; oneElemOfB];
end

for i = 1:2 % ***Use 2 points***
    % Constraint 1:
    oneElemOfA = [0, -1, c_ns(2, i)];
    A = [A; oneElemOfA];
    oneElemOfB = - [0, -1, c_ns(2, i)] * R * c_Ws(:, i);
    B = [B; oneElemOfB];
    
    % Constraint 2:
    oneElemOfA = [1, 0, -c_ns(1, i)];
    A = [A; oneElemOfA];
    oneElemOfB = - [1, 0, -c_ns(1, i)] * R * c_Ws(:, i);
    B = [B; oneElemOfB];
end

t = (A' * A)^-1 * (A' * B);
t = vpa(t, 4);

t_elem1 = t(1);
expCoeffMat1 = coeffs(t_elem1);
expCoeffMat1 = vpa(fliplr(expCoeffMat1), 4);

t_elem2 = t(2);
expCoeffMat2 = coeffs(t_elem2);
expCoeffMat2 = vpa(fliplr(expCoeffMat2), 4);

t_elem3 = t(3);
expCoeffMat3 = coeffs(t_elem3);
expCoeffMat3 = vpa(fliplr(expCoeffMat3), 4);

coeffMat = [];

coeffMat = [coeffMat; expCoeffMat1];
coeffMat = [coeffMat; expCoeffMat2];
coeffMat = [coeffMat; expCoeffMat3];


end