% Constraint 1:
% [ 0, -1, cv_n ] * t = - [ 0, -1, cv_n ] * R * c_W 

% Constraint 2:
% [ 1, 0, -cu_n ] * t = - [ 1, 0, -cu_n ] * R * c_W 

syms cX_W cY_W cZ_W...
    cu_n cv_n real

c_W = [cX_W cY_W cZ_W]';
c_n = [cu_n cv_n]';


A = [];
B = [];

syms b c d real
R = [1 + b*b - c*c - d*d, 2 * (b*c - d), 2 * (b*d + c);
    2 * (c*b + d), 1 - b*b + c*c - d*d, 2 * (c*d - b);
    2 * (d*b - c), 2 * (d*c + b), 1 - b*b - c*c + d*d];

% Constraint 1:
oneElemOfA = [0, -1, cv_n];
A = [A; oneElemOfA];
oneElemOfB = - [0, -1, cv_n] * R * c_W;
B = [B; oneElemOfB];

% Constraint 2:
oneElemOfA = [1, 0, -cu_n];
A = [A; oneElemOfA];
oneElemOfB = - [1, 0, -cu_n] * R * c_W;
B = [B; oneElemOfB];


A

%% 一次循环得到一个3*1 (for one loop, we can get one 3*1 matrix)
%                         2*1
% ( 3*2 | 3*2 | 3*2 )  (  2*1  )   = 3*1 + 3*1 + 3*1
%                         2*1

syms l00 l01...
    l10 l11...
    l20 l21 real
 
left_32 = [l00 l01;
    l10 l11;
    l20 l21];

one31 = left_32 * B;
expand1 = expand(one31(1));
collect1 = collect(expand1, [b^2, b*c, b*d, b, c^2, c*d, c, d^2, d, 1])
expand2 = expand(one31(2));
collect2 = collect(expand2, [b^2, b*c, b*d, b, c^2, c*d, c, d^2, d, 1])
expand3 = expand(one31(3));
collect3 = collect(expand3, [b^2, b*c, b*d, b, c^2, c*d, c, d^2, d, 1])