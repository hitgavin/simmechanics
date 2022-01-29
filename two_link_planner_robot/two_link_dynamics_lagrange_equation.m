% TWO_LINK_DYNAMICS_LAGRANGE_EQUATION is used for dynamic symbolic equation derivation
% for two link planner robot based on Lagrange method.
% 
% Notes::
% - Based on my CSDN/guyuehome blog
%   If you are interested in robotics, welcome to my blog
%   https://blog.csdn.net/hitgavin.
%   https://www.guyuehome.com(nick name: hitgavin)
% blog title: 机器人动力学---二连杆平面臂牛顿欧拉方程
syms l1 l2 m1 m2 g real;
% declare symbolic function of time, so we can compute the derivative of 
% the symbolic function(such as q1(t)) with respect to t.
syms q1(t) q2(t) dq1(t) dq2(t);

syms L_1xx L_1xy L_1xz L_1yy L_1yz L_1zz real;
syms L_2xx L_2xy L_2xz L_2yy L_2yz L_2zz real;
syms l_1x l_1y l_1z l_2x l_2y l_2z real;
l_1x = l1/2;
l_1y = 0;
l_1z = 0;
l_2x = l2/2;
l_2y = 0;
l_2z = 0;
% declare that the symbolic function is a real(not imaginary) function 
% see also:
% https://www.mathworks.com/matlabcentral/answers/457466-issue-with-reality-assumption-of-an-implicit-function
% https://www.mathworks.com/matlabcentral/answers/78614-how-do-you-declare-a-symbolic-function-of-time-as-a-real-variable
assumeAlso(q1(t), 'real');
assumeAlso(q2(t), 'real');
assumeAlso(dq1(t), 'real');
assumeAlso(dq2(t), 'real');

G = [0; 0; g];
% formula is a function to get the body of the symbolic function using
% which you can do slicing operation just like the matrix(such as T1(1:3, 1:3))
% see also:
% https://www.mathworks.com/help/symbolic/sym.formula.html
% https://www.mathworks.com/matlabcentral/answers/260444-use-a-symfun-as-index-of-a-matrix
T1 = formula([cos(q1) -sin(q1) 0 0; 0 0 -1 0; sin(q1) cos(q1) 0 0;0 0 0 1]);
T2 = formula([cos(q2) -sin(q2) 0 l1; sin(q2) cos(q2) 0 0; 0 0 1 0; 0 0 0 1]);
T20 = T1*T2;
pc1 = formula(simplify(T1 * [l_1x; 0; 0; 1]));
pc1 = pc1(1:3);
pc2 = formula(simplify(T1 * T2 * [l_2x; 0; 0; 1]));
pc2 = pc2(1:3);
U1 = -m1 * G' * pc1;
U2 = -m2 * G' * pc2;

z1 = T1(1:3,3);
o1 = [0; 0; 0];
oc1 = pc1;
o1oc1 = pc1 - o1;
J1c = [cross(z1, o1oc1) zeros(3,1); z1 zeros(3,1)];
vw1 = formula(J1c * [dq1; dq2]);
v1 = formula(vw1(1:3));
w1 = formula(vw1(4:6));
R1 = T1(1:3, 1:3);
I1 = [L_1xx L_1xy L_1xz; L_1xy L_1yy L_1yz; L_1xz L_1yz L_1zz];
E1 = 1/2 * m1 * v1' * v1 + 1/2 * w1' * R1 * I1 * R1' * w1;

o2 = T20(:,4);
o2 = o2(1:3);
oc2 = pc2;
o1oc2 = oc2 - o1;
o2oc2 = oc2 - o2;
z2 = T20(1:3, 3);
J2c = [cross(z1, o1oc2) cross(z2, o2oc2); z1 z2];
vw2 = formula(J2c * [dq1; dq2]);
v2 = vw2(1:3);
w2 = vw2(4:6);
R2 = T20(1:3, 1:3);
I2 = [L_2xx L_2xy L_2xz; L_2xy L_2yy L_2yz; L_2xz L_2yz L_2zz];
E2 = 1/2 * m2 * v2' * v2 + 1/2 * w2' * R2 * I2 * R2' * w2;

L = simplify(E1 + E2 - U1 - U2);

% lagrange equation
tau1 = formula(diff(diff(L, dq1), t) - diff(L, q1));
tau2 = formula(diff(diff(L, dq2), t) - diff(L, q2));

syms q1(t) q2(t) dq1(t) dq2(t);
assumeAlso(q1(t), 'real');
assumeAlso(q2(t), 'real');
assumeAlso(dq1(t), 'real');
assumeAlso(dq2(t), 'real');
syms ddq1 ddq2;
tau1 = subs(tau1, diff(dq1(t), t), ddq1);
tau1 = subs(tau1, diff(dq2(t), t), ddq2);
tau1 = subs(tau1, diff(q1(t), t), dq1);
tau1 = subs(tau1, diff(q2(t), t), dq2);
tau1 = simplify(tau1);


tau2 = subs(tau2, diff(dq1(t), t), ddq1);
tau2 = subs(tau2, diff(dq2(t), t), ddq2);
tau2 = subs(tau2, diff(q1(t), t), dq1);
tau2 = subs(tau2, diff(q2(t), t), dq2);
tau2 = simplify(tau2);
