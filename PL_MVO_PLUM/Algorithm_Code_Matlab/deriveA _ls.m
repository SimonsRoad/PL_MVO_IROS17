syms b c d real
R = [1 + b*b - c*c - d*d, 2 * (b*c - d), 2 * (b*d + c);
    2 * (c*b + d), 1 - b*b + c*c - d*d, 2 * (c*d - b);
    2 * (d*b - c), 2 * (d*c + b), 1 - b*b - c*c + d*d];

A = [];
B = [];

syms n_cX_W n_cY_W n_cZ_W...
    v_cX_W v_cY_W v_cZ_W...
    l_cha_o l_chb_o l_chc_o real

l = [l_cha_o l_chb_o l_chc_o]';

syms cx cy fx fy real
K = [fx 0 cx;
    0 fy cy;
    0 0 1];

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

% A


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

% one 3*1
one31 = left_32 * B;
expand1 = expand(one31(1));
collect1 = collect(expand1, [b^2, b*c, b*d, b, c^2, c*d, c, d^2, d, 1])
expand2 = expand(one31(2));
collect2 = collect(expand2, [b^2, b*c, b*d, b, c^2, c*d, c, d^2, d, 1])
expand3 = expand(one31(3));
collect3 = collect(expand3, [b^2, b*c, b*d, b, c^2, c*d, c, d^2, d, 1])

% manually -> 3*10 乘 10*1

