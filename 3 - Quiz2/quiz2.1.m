clc
clear all
close all
%%
%Q1
a = [0.3835 0.5710 0.9287;
0.5730 0.5919 -0.4119;
-1.3954 0.0217 1.1105];
a_check = a * a'

b = [sqrt(2)/2 0 sqrt(2)/2;
0 1 0;
-sqrt(2)/2 0 sqrt(2)/2];
b_check = b * b'

c = [0.2120 0.7743 0.5963;0.2120 -0.6321 0.7454;0.9540 -0.0316 -0.2981];
c_check = c * c'

syms theta real
d = [cos(theta) -sin(theta);
    sin(theta) cos(theta)];
d_check = simplify(d * d')

%%
% Q2
syms spi phi theta real

Rz_spi = [cos(spi) -sin(spi) 0;
    sin(spi) cos(spi) 0;
    0 0 1];

Ry_theta = [cos(theta) 0 sin(theta);
    0 1 0;
    -sin(theta) 0 cos(theta)];

Rz_phi = [cos(phi) -sin(phi) 0;
    sin(phi) cos(phi) 0;
    0 0 1];

Rot = Rz_spi * Ry_theta * Rz_phi;

Rot_num = [0.6927 -0.7146 0.0978;
    0.7165 0.6973 0.0198;
    -0.0824 0.0564 0.995];

theta = acos(0.995)

phi = asin(0.0564/sin(theta))

spi = asin(0.0198/sin(theta))
%%
%Q3
R = [0.675 -0.1724 0.7174;
    0.2474 0.9689 0;
    -0.6951 0.1775 0.6967];
w_b = [0 -1 0.9689;
    1 0 -0.2474;
    -0.9689 0.2474 0];

R_dot = w_b * inv(R');

w_s = R_dot * R' % this is wrong !


% this is true ! why ?! 
% w_s2 = [0 -0.6967 1;
%     0.6967 0 -0.7174;
%     -1 0.7174 0];
%%
%Q4
Rotm1 = [0.2919 0.643 -0.7081;
    -0.643 -0.4161 -0.643;
    -0.7081 0.643 0.2919]; 
axang1 = rotm2axang(Rotm1)
[V1,D1] = eig(Rotm1)

%%
%Q5
Rotm2 = [-1/3 2/3 -2/3;
    2/3 -1/3 -2/3;
    -2/3 -2/3 -1/3];
axang2 = rotm2axang(Rotm2)
[V2,D2] = eig(Rotm2)
% Not enough information is given to uniquely determine the axis-angle representation
%%
%Q6
% Assuming \mathbf{p}p and \mathbf{q}q represent the vectors 
% from the origin to the points P and Q respectively, 
% which of the following are correct expressions for the 
% cross-product of the rotated vectors p′×q′?

% g∗(p)×g∗(q) this is true
% g∗(p×q) % and this is true too
% (p×q)






