function [K, Rw2c, Qw2c, tw2c, spaceLSs_w, imgLSs, spacePts_w, imgPts] = ConstrSynData()
% In the following, we provide some test data: 
% 1) one intrinsic mat
% 2) four pose
% 3) three lines
% 4) three points
% 
% Readers can also use other data to test
% 


%% 1. Intrinsic Mat
K = [919.826, 0, 506.897;
    0, 921.387, 335.603;
    0, 0, 1];

%% 2. Extrinsic Mat(provide 3 test R&t)
% % test data 1
% Rw2c = [0.901, -0.054, -0.429;
%     0.046, 0.998, -0.028;
%     0.430, 0.005, 0.902];
% Qw2c = [0.0090, -0.2259, 0.0262];
% 
% tw2c = [2.867;
%     0.102;
%     0.171];

% % test data 2
% Rw2c = [0.988, -0.021, -0.149;
%     0.024, 0.999, 0.014;
%     0.149, -0.018, 0.988];
% Qw2c = [-0.0084, -0.0749, 0.0113];
% 
% tw2c = [0.997;
% 	0.031;
% 	-0.063];

% test data 3
Rw2c = [0.365, -0.102, -0.925;
	0.0218, 0.994, -0.101;
	0.930, 0.016, 0.365];
Qw2c = [0.0431, -0.6810, 0.0454];

tw2c = [6.142;
	0.211;
	3.231];

% % test data 4
% Rw2c = [0.540, -0.090, -0.836;
% 	0.029, 0.995, -0.088;
% 	0.840, 0.0233, 0.540];
% Qw2c = [0.0741, -0.5304, 0.0346];
% 
% tw2c = [5.578;
%  0.186;
%  2.208];

%% 3. Line Segment (3)
% 3.1 space LS
% one space LS:
% [beginPtX_w, beginPtY_w, beginPtZ_w;
%  endPtX_w, endPtY_w, endPtZ_w]
% 
% Space LS1  expressed by two end pts, each end pt stores in row
P1_w = [1.803, 1.175, 4.521];
Q1_w = [0.851, -1.383, 5.093];
spaceLS1_w = [P1_w; Q1_w];

% Space LS2
P2_w = [-1.090, 0.935, 4.743];
Q2_w = [0.724, 0.735, 5.075];
spaceLS2_w = [P2_w; Q2_w];

% Space LS3
P3_w = [0.608, -0.919, 5.289];
Q3_w = [1.015, 0.610, 4.667];
spaceLS3_w = [P3_w; Q3_w];


spaceLSs_w = [spaceLS1_w; spaceLS2_w; spaceLS3_w];

[rowLS, colLS] = size(spaceLSs_w);

% 3.2 img LS
% one img LS is stored in following form:
% [beginPt_uO, beginPt_vO, beginPt_uN, beginPt_vN;
%  endPt_uO, endPt_vO, endPt_uN, endPt_vN]

imgLSs = [];
for i = 1:rowLS
    oneImgLSEndPt = [];
    % for every row, trans X_w into xO & xN
    x_O = getOrdImgPt(K, spaceLSs_w(i, :), Rw2c, tw2c);
    oneImgLSEndPt = [oneImgLSEndPt, x_O];  
    x_N = getNormalImgPt(K, spaceLSs_w(i, :), Rw2c, tw2c);
    oneImgLSEndPt = [oneImgLSEndPt, x_N];
    
    imgLSs = [imgLSs; oneImgLSEndPt];
end


%% 4. Point (3)
% 3.1 space Pt
% Space Pt1(Xx, Xy, Xz)
X1_w = [-0.031, 0.217, 5.588]; % each pt stores in row
% Space Pt2
X2_w = [-0.933, -0.947, 6.186];
% Space Pt3
X3_w = [1.113, 0.153, 4.641];

spacePts_w = [X1_w; X2_w; X3_w];

% 3.2 img Pt
% img Pt1(xOu, xOv, xNu, xNv) % each pt stores in row, first two is ordinary coordinate, last two is normalized coordinate
[rowPt, colPt] = size(spacePts_w);

imgPts = [];
for i = 1:rowPt
    oneImgPt = [];
    x_O = getOrdImgPt(K, spacePts_w(i, :), Rw2c, tw2c);
    oneImgPt = [oneImgPt, x_O];
    x_N = getNormalImgPt(K, spacePts_w(i, :), Rw2c, tw2c);
    oneImgPt = [oneImgPt, x_N];
    
    imgPts = [imgPts; oneImgPt];
end


%% aux Functions:
function [ordImgPt] = getOrdImgPt(K, spacePt_w, Rw2c, tw2c)
% space Pt in Cam coordinate
spacePt_c = Rw2c*spacePt_w' + tw2c;

imgPtHomog = K*spacePt_c;
noise = 0.0;
ordImgPt = [imgPtHomog(1)/imgPtHomog(3) + noise, imgPtHomog(2)/imgPtHomog(3) + noise]; % (xOu, xOv), stores in row
end 

function [normalImgPt] = getNormalImgPt(K, spacePt_w, Rw2c, tw2c)
% space Pt in Cam coordinate
spacePt_c = Rw2c*spacePt_w' + tw2c;


imgPtHomog = K*spacePt_c;
imgPt = [imgPtHomog(1)/imgPtHomog(3), imgPtHomog(2)/imgPtHomog(3)];

normalImgPt = [(imgPt(1) - K(1, 3))/K(1, 1), (imgPt(2) - K(2, 3))/K(2, 2)]; % (xNu, xNv), stores in row
end


end