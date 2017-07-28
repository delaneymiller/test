%% MAE 433 Final Exam, Question 8
% Delaney Miller
% 22 January 2016

%% part (c) 

A = [0 1; 0 -5];
B = [0; 50];
C = [1 0];
sys = ss(A,B,C,0);
P = tf(sys);
s = tf('s');

% Proportional-derivative control
K = 2;
z = 5;
C = K * (s/z + 1);

L = P*C;
figure()
margin(L)
title('part c: proportional-derivative control')

%% part (d)

% Test if (A,B) is controllable
Control = ctrb(A,B);
n = rank(Control);

%% part (e)

A = [0 1; 0 -5];
B = [0; 50];
C = [1 0];
sys = ss(A,B,C,0);

% Using LQR
q = 20;
Q = q*C'*C;
R = [1];
K = lqr(A,B,Q,R);

% Loop transfer function
loop_lqr = ss(A,B,K,0);
figure()
margin(loop_lqr)
title('part e: LQR controller design, loop gain')

%% part (g) Observer-based compensator

poles_obs = [-75+75i, -75-75i];
L_obs = place(A', C', poles_obs)';
loop_comp = ss(A-B*K-L_obs*C, L_obs, K, 0);
loop_obs = sys*loop_comp;

% stability margins of observer-based compensator
figure()
margin(loop_obs)

% comparison of three controllers
figure()
bode(L, loop_lqr, loop_obs)
legend('Classical','LQR','Observer')