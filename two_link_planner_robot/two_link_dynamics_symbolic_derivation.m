% TWO_LINK_DYNAMICS_SYMBOLIC_DERIVATION is used for dynamics symbolic derivation
% for two link planner robot based on Newton-Euler method
%
% Notes::
% - Based on my CSDN blog
%   If you are interested in robotics, welcome to my blog
%   https://blog.csdn.net/hitgavin.

% Copyright (C) 2020-2021, by gavin
syms L_1xx L_1xy L_1xz L_1yy L_1yz L_1zz real;
syms l_1x l_1y l_1z real;
syms m_1 real;
syms fv_1 fc_1 real;

syms L_2xx L_2xy L_2xz L_2yy L_2yz L_2zz real;
syms l_2x l_2y l_2z real;
syms m_2 real;
syms fv_2 fc_2 real;

syms q1 dq1 ddq1 q2 dq2 ddq2 l1 l2 real;

l_1x = l1 / 2;
l_1y = 0;
l_1z = 0;
l_2x = l2 / 2;
l_2y = 0;
l_2z = 0;

% angular velocity of base link
w_0_0 = [0;0;0];
% linear velocity of base link
v_0_0 = [0;0;0];
% angular acceleration of base link
alpha_0_0 = [0; 0; 0];
% linear acceleration of base link
a_0_0 = [0; 9.81; 0];

dh = [0, 0, 0, q1;
      0, l1, 0, q2];
% acceleration of CoM p_ci_i
w = cell(1, size(dh, 1));
v = cell(1, size(dh, 1));
alpha = cell(1, size(dh, 1));
a = cell(1, size(dh, 1));
ac = cell(1, size(dh, 1));
l = cell(1, size(dh, 1));
T = cell(1, size(dh, 1) + 1);
R = cell(1, size(dh, 1) + 1);
r = cell(1, size(dh, 1) + 1);
I = cell(1, size(dh, 1));
T_e = cell(1, size(dh, 1) + 1);
t_e_pre = eye(4);
for i = 1:size(dh, 1)
    T{i} = trotx(dh(i, 1))*transl([dh(i, 2), 0, dh(i, 3)])*trotz(dh(i, 4));
    R{i} = T{i}(1:3, 1:3);
    r{i} = T{i}(1:3, 4);
    T_e{i} = t_e_pre * T{i};
    t_e_pre = T{i};
end
T{i+1} = eye(3);
R{i+1} = eye(3);
r{i+1} = [l2, 0, 0];
T_e{i+1} = T_e{i} * transl([l2, 0, 0]);
q = [q1 q2];
q_dot = [dq1 dq2];
q_ddot = [ddq1 ddq2];
l{1} = [l_1x; l_1y; l_1z];
l{2} = [l_2x; l_2y; l_2z];

pre_w = w_0_0;
pre_v = v_0_0;
pre_alpha = alpha_0_0;
pre_a = a_0_0;
% forward iteration
for i = 1:2
    t = R{i}';
    w_i_i = t * pre_w + q_dot(i) * [0;0;1];
    v_i_i = t * (pre_v + cross(pre_w, r{i}));
    alpha_i_i = t * pre_alpha + cross((t * pre_w), q_dot(i) * [0;0;1]) + q_ddot(i) * [0;0;1];
    a_i_i = t * (pre_a + cross(pre_alpha, r{i}) + cross(pre_w, cross(pre_w, r{i})));
    ac_i_i = a_i_i + cross(alpha_i_i, l{i}) + cross(w_i_i, cross(w_i_i, l{i}));
    w{i} = w_i_i;
    v{i} = v_i_i;
    alpha{i} = alpha_i_i;
    a{i} = a_i_i;
    ac{i} = ac_i_i;
    
    pre_w = w_i_i;
    pre_v = v_i_i;
    pre_alpha = alpha_i_i;
    pre_a = a_i_i;
end
m = [m_1; m_2];
% backward iteration
f_3_3 = [0;0;0];
n_3_3 = [0;0;0];
next_f_i_i = f_3_3;
next_n_i_i = n_3_3;

I{1} = [L_1xx L_1xy L_1xz; L_1xy L_1yy L_1yz; L_1xz L_1yz L_1zz];
I{2} = [L_2xx L_2xy L_2xz; L_2xy L_2yy L_2yz; L_2xz L_2yz L_2zz];
tau = cell(1, size(dh, 1));

for i = 2:-1:1
    r = R{i+1};
    f_i_i = m(i)*ac{i} + r * next_f_i_i;
    z_i = [0;0;1];
    
    TT = T_e{i};
    TT(1:3,1:3) = (T_e{i}(1:3,1:3))';
    TT(1:3, 4) = -(T_e{i}(1:3,1:3))' * T_e{i}(1:3,4);
    xx = TT * T_e{i+1}(1:4, 4);
    r_ip1_ci = l{i} - xx(1:3);
    torque_i = I{i}*alpha{i} + cross(w{i}, I{i}*w{i}) + r * next_n_i_i + cross(l{i}, f_i_i) - cross(r_ip1_ci, r * next_f_i_i);
    tau{i} = simplify(torque_i(3));
    next_f_i_i = f_i_i;
    next_n_i_i = torque_i;
end

