%% Prposed Unified Model
% PGCi in paper:
% [ 0, -1, xv_N ] * t = - [ 0, -1, xv_N ] * R * X_w 
% 
% [ 1, 0, -xu_N ] * [ R * X_w +  t] = 0

% LGCj in paper:
%                                           0
%  (vx_w * l' * K) * t = -l' * K * R * [  -nz_w  ]
%                                          ny_w
% 
% 
%   [  nz_w    0     -nx_w   vy_w   ] * [R | t]' * K' * l  = 0    
%     

%% Construct synthetic data

[K, Rw2c_true, Qw2c_true, tw2c_true,   spaceLSs_w, imgLSs,   spacePts_w, imgPts] = ConstrSynData();

% LS's variable:
[leftXs_w, rightXs_w,   ns_w, vs_w, ls_O] = GetRelVar(spaceLSs_w, imgLSs); % for return values, the first two variable is used to select optimal t and R by minimal re-projection error

% Pt's variable:
xs_O = imgPts(:, 1:2)';
xs_N = imgPts(:, 3:4)';
Xs_w = spacePts_w';

%% Express t by R [ 4 minimal case ]

% using groud true bcd, we can verify the coeff Mat which is used to express t
% by R, etc., to compare tw2c_der and tw2c_true
b_true = Qw2c_true(1);
c_true = Qw2c_true(2);
d_true = Qw2c_true(3);

s = 1 + b_true*b_true + c_true*c_true + d_true*d_true;
q = [b_true^2, b_true*c_true, b_true*d_true, b_true, c_true^2, c_true*d_true, c_true, d_true^2, d_true, 1]';

% Following 4 case, we can choose 1 randomly
% tw2c_der can be used to check if R express t correctly

% % 3Ls
% coeffMat = expressTransByRotMinCase(ns_w, vs_w, ls_O, K, 0, 0, 1);
% 
% tw2c_der = 1/s * coeffMat * q;

% 3Pt
coeffMat = expressTransByRotMinCase(0, 0, 0, 0, Xs_w, xs_N, 2);

tw2c_der = 1/s * coeffMat * q;
% 
% % 2Ls 1Pt
% coeffMat = expressTransByRotMinCase(ns_w(:, 1:2), vs_w(:, 1:2), ls_O(:, 1:2), K, Xs_w(:, 1:1), xs_N(:, 1:1), 3);
% 
% tw2c_der = 1/s * coeffMat * q;
% 
% % 1Ls 2Pt
% coeffMat = expressTransByRotMinCase(ns_w(:, 1:1), vs_w(:, 1:1), ls_O(:, 1:1), K, Xs_w(:, 1:2), xs_N(:, 1:2), 4);
% 
% tw2c_der = 1/s * coeffMat * q;

%% Cost Function
[diff_bCoeff, diff_cCoeff, diff_dCoeff] = constrCostFunc(ns_w, vs_w, ls_O, K,   Xs_w, xs_N,   coeffMat); 

%% Call the solver

bM11 = diff_bCoeff(1); bM21 = diff_bCoeff(2); bM31 = diff_bCoeff(3); bM41 = diff_bCoeff(4); bM51 = diff_bCoeff(5); bM61 = diff_bCoeff(6); bM71 = diff_bCoeff(7); bM81 = diff_bCoeff(8); bM91 = diff_bCoeff(9); bM101 = diff_bCoeff(10); bM111 = diff_bCoeff(11); bM121 = diff_bCoeff(12); bM131 = diff_bCoeff(13); bM141 = diff_bCoeff(14); bM151 = diff_bCoeff(15); bM161 = diff_bCoeff(16); bM171 = diff_bCoeff(17); bM181 = diff_bCoeff(18); bM191 = diff_bCoeff(19); bM201 = diff_bCoeff(20);
cM11 = diff_cCoeff(1); cM21 = diff_cCoeff(2); cM31 = diff_cCoeff(3); cM41 = diff_cCoeff(4); cM51 = diff_cCoeff(5); cM61 = diff_cCoeff(6); cM71 = diff_cCoeff(7); cM81 = diff_cCoeff(8); cM91 = diff_cCoeff(9); cM101 = diff_cCoeff(10); cM111 = diff_cCoeff(11); cM121 = diff_cCoeff(12); cM131 = diff_cCoeff(13); cM141 = diff_cCoeff(14); cM151 = diff_cCoeff(15); cM161 = diff_cCoeff(16); cM171 = diff_cCoeff(17); cM181 = diff_cCoeff(18); cM191 = diff_cCoeff(19); cM201 = diff_cCoeff(20);
dM11 = diff_dCoeff(1); dM21 = diff_dCoeff(2); dM31 = diff_dCoeff(3); dM41 = diff_dCoeff(4); dM51 = diff_dCoeff(5); dM61 = diff_dCoeff(6); dM71 = diff_dCoeff(7); dM81 = diff_dCoeff(8); dM91 = diff_dCoeff(9); dM101 = diff_dCoeff(10); dM111 = diff_dCoeff(11); dM121 = diff_dCoeff(12); dM131 = diff_dCoeff(13); dM141 = diff_dCoeff(14); dM151 = diff_dCoeff(15); dM161 = diff_dCoeff(16); dM171 = diff_dCoeff(17); dM181 = diff_dCoeff(18); dM191 = diff_dCoeff(19); dM201 = diff_dCoeff(20);

[cayley_b, cayley_c, cayley_d] = solver_PnL_minCase(bM11, bM21, bM31, bM41, bM51, bM61, bM71, bM81, bM91, bM101, bM111, bM121, bM131, bM141, bM151, bM161, bM171, bM181, bM191, bM201, cM11, cM21, cM31, cM41, cM51, cM61, cM71, cM81, cM91, cM101, cM111, cM121, cM131, cM141, cM151, cM161, cM171, cM181, cM191, cM201, dM11, dM21, dM31, dM41, dM51, dM61, dM71, dM81, dM91, dM101, dM111, dM121, dM131, dM141, dM151, dM161, dM171, dM181, dM191, dM201);


% output candidate candRw2c

candRs = {};
candts = {};

for i = 1:length(cayley_b)

    b = cayley_b(i);
    c = cayley_c(i);
    d = cayley_d(i);
    s = 1 + b*b + c*c + d*d;
    candRw2c = (1/s) * [1 + b*b - c*c - d*d, 2 * (b*c - d), 2 * (b*d + c);
                 2 * (c*b + d), 1 - b*b + c*c - d*d, 2 * (c*d - b);
                 2 * (d*b - c), 2 * (d*c + b), 1 - b*b - c*c + d*d];

    
    q = [b^2, b*c, b*d, b, c^2, c*d, c, d^2, d, 1]';
    
    candtw2c = double((1/s) * coeffMat * q);

    candRs = [candRs, candRw2c];
    candts = [candts, candtw2c];
end

%% Select the best by min re-projection error

mean_delta_us = [];
mean_delta_vs = [];

for i = 1:length(cayley_b)
    candR = cell2mat(candRs(i));
    candt = cell2mat(candts(i));
    
    accum_delta_u = 0;
    accum_delta_v = 0;
    
    % Pt
    for j = 1:size(Xs_w, 2)
        spacePt_c = candR*Xs_w(:, j) + candt;

        imgPtHomog = K*spacePt_c;
        ordImgPt = [imgPtHomog(1)/imgPtHomog(3), imgPtHomog(2)/imgPtHomog(3)]; % (xOu, xOv), stores in row 
        
        delta_u = ordImgPt(1) - xs_O(1, j);
        delta_v = ordImgPt(2) - xs_O(2, j);
        
        accum_delta_u = accum_delta_u + delta_u;
        accum_delta_v = accum_delta_v + delta_v;
    end
    
    % Ls
    for j = 1:size(ls_O, 2)
        S_c_W = candR * leftXs_w(:, j) + candt;
        s_ch_o = K*S_c_W;
        s_ch_o = s_ch_o/s_ch_o(3);
        
        % pt to ls distance
        numer_s = abs( dot(ls_O(:, j), s_ch_o) );
        dem_s = sqrt( ls_O(1, j)^2 + ls_O(2, j)^2 );
        
        dis_s = numer_s / dem_s;
            
        %%%
        
        E_c_W = candR * rightXs_w(:, j) + candt;
        e_ch_o = K*E_c_W;
        e_ch_o = e_ch_o/e_ch_o(3);
        
        % pt to ls distance
        numer_e = abs( dot(ls_O(:, j), e_ch_o) );
        dem_e = sqrt( ls_O(1, j)^2 + ls_O(2, j)^2 );
        
        dis_e = numer_e / dem_e;
        
        accum_delta_u = accum_delta_u + dis_s;
        accum_delta_v = accum_delta_v + dis_e;
    end
    
    
    mean_delta_u = accum_delta_u / size(Xs_w, 2);
    mean_delta_v = accum_delta_v / size(Xs_w, 2);
    
    mean_delta_us = [mean_delta_us, abs(mean_delta_u)];
    mean_delta_vs = [mean_delta_vs, abs(mean_delta_v)];
    
end

mean_delta_total = mean_delta_us + mean_delta_vs;

[minNum, idx]=min(mean_delta_total);

estimatedR = cell2mat(candRs(idx))
estimatedt = cell2mat(candts(idx))

Rw2c_true
tw2c_true

disp('Complete!')



