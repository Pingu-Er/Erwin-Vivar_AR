# Homework 3: Forward Kinematics
For the following problems, we used Matlab to solve all the matrices. The script calculates the matrices both individually (between each frame) and calculates the homogeneous transformation matrix between frame 0 and the frame associated with the final element of the robot.

The Matlab script was the following:

```python
%% Denavit–Hartenberg (table Evaluator)
% Computes individual link transforms (A_i) and cumulative transforms T_0_i.

% Notes:
% - Angles are in radians.
% - This script can be configured for different types of joints.


% -------------------- Local Function: Standard DH homogeneous transform --------------------
function H = dhStandard(a, alpha, d, theta)

ct = cos(theta);  st = sin(theta);
ca = cos(alpha);  sa = sin(alpha);

H = [ ct, -st*ca,  st*sa, a*ct;
      st,  ct*ca, -ct*sa, a*st;
      0,      sa,     ca,    d;
      0,       0,      0,    1 ];
end
% -------------------- Local Function --------------------

clear; clc;

% -------------------- Parameters --------------------
% Notes:
% - In the case of joints with negligible distance between them, L = 0.

l1 = 0;
l2 = 1;
l3 = 0;
l4 = 1;
l5 = 0;
l6 = 1;

% It is assumed that q = 0 for the initial pose.
q = zeros(6,1);

% -------------------- DH Table (Standard) --------------------
% Columns: a [m], alpha [rad], d [m], theta [rad]
DH = table( ...
    [0;   l2;  0;   0;   0;   0], ...
    [pi/2;pi/2;-pi/2;pi/2;-pi/2;0], ...
    [0;   0;   0;   l4;  0;   l6], ...
    [pi/2+q(1); pi/2+q(2); q(3); q(4); q(5); -pi/2+q(6)], ...
    'VariableNames', {'a','alpha','d','theta'} );

N = height(DH);

% -------------------- Compute A_i and T_0_i --------------------
A = cell(N,1); % A{i} = A_i
T = cell(N,1); % T{i} = T_0_i

Tcum = eye(4);
for i = 1:N
    A{i} = dhStandard(DH.a(i), DH.alpha(i), DH.d(i), DH.theta(i));
    Tcum = Tcum * A{i};
    T{i} = Tcum;
end

% -------------------- Display Results --------------------
disp("=== Evaluated DH Table (Standard DH) ===");
disp(DH);

for i = 1:N
    fprintf("\nA_%d =\n", i);
    disp(A{i});
end

T_0_N = T{end};
disp("=== T_0_N (end-effector pose) ===");
disp(T_0_N);
```

It is important to note that a link length of 1 was assumed for every link whenever its length could not be considered negligible. Additionally, the script was evaluated using the initial configuration, $q=0$. These assumptions were adopted to keep the focus strictly on the kinematic analysis of the robots and on the systematic generation of Denavit–Hartenberg transformation matrices.

## Excersice 1

<div style="text-align:center;">
  <img src="../recursos/imgs/HW3/E1.png" alt="RRP" style="max-height:320px; width:auto;">
</div>

*Result (image of planes, origins, axis, links):*

<div style="text-align:center;">
  <img src="../recursos/imgs/HW3/S1.png" alt="RRP" style="max-height:320px; width:auto;">
</div>

**DH parameters**

| L | a         | α         | θ              | d                  |
|---:|:----------|:----------|:---------------|:-------------------|
| 1 | $l_{1.2}$  | $-\pi/2$  | $-\pi/2 + q_1$ | $0$                |
| 2 | $0$        | $0$       | $0$            | $l_{1.1}+l_2+q_2$   |

**DH after sustitution (script):** 

$$
\begin{array}{c c c c c}
a & \alpha & d & \theta\\ \hline
1 & -1.5708 & 0 & -1.5708\\
0 & 0 & 2 & 0
\end{array}
$$

$$
T_1=
\begin{bmatrix}
0  & 0  & 1 & 0\\
-1 & 0  & 0 & -1\\
0 & -1 & 0 & 0\\
0 & 0 & 0 & 1
\end{bmatrix}
\qquad
T_2=
\begin{bmatrix}
1 & 0 & 0 & 0\\
0 & 1 & 0 & 0\\
0 & 0 & 1 & 2\\
0 & 0 & 0 & 1
\end{bmatrix}
$$

$$
^{0}_{2}T=
\begin{bmatrix}
0  & 0 & 1 & 2\\
-1 & 0 & 0 & -1\\
0 & -1 & 0 & 0 \\
0 & 0 & 0 & 1
\end{bmatrix}
$$


---

## Excersice 2

<div style="text-align:center;">
  <img src="../recursos/imgs/HW3/E2.png" alt="RRP" style="max-height:320px; width:auto;">
</div>

*Result (image of planes, origins, axis, links):*

<div style="text-align:center;">
  <img src="../recursos/imgs/HW3/S2.png" alt="RRP" style="max-height:320px; width:auto;">
</div>

**DH parameters**

| L | a   | α       | θ       | d            |
|---:|:----|:--------|:--------|:-------------|
| 1 | $0$ | $\pi/2$ | $\pi/2$ | $l_1 + q_1$  |
| 2 | $0$ | $\pi/2$ | $\pi/2$ | $l_2 + q_2$  |
| 3 | $0$ | $\pi$   | $0$     | $l_3 + q_3$  |

**DH after sustitution (script):** 
 
$$
\begin{array}{c c c c c}
a & \alpha & d & \theta &\\ \hline
0 & 1.5708 & 1 & 1.5708 &\\646
0 & 3.1416 & 1 & 0 &
\end{array}
$$

$$
T_1=
\begin{bmatrix}
0 & 0 & 1 & 0\\
1 & 0 & 0 & 0\\
0 & 1 & 0 & 1\\
0 & 0 & 0 & 1
\end{bmatrix}
\qquad
T_2=
\begin{bmatrix}
0 & 0 & 1 & 0\\
1 & 0 & 0 & 0\\
0 & 1 & 0 & 1\\
0 & 0 & 0 & 1
\end{bmatrix}
$$

$$
T_3=
\begin{bmatrix}
1 & 0 & 0 & 0\\
0 & -1 & 0 & 0\\
0 & 0 & -1 & 1\\
0 & 0 & 0 & 1
\end{bmatrix}
$$

$$
^{0}_{3}T=
\begin{bmatrix}
0 & -1 & 0 & 1\\
0 & 0 & -1 & 1\\
1 & -0 & 0 & 1\\
0 & 0 & 0 & 1
\end{bmatrix}
$$

---

## Excersice 3

<div style="text-align:center;">
  <img src="../recursos/imgs/HW3/E3.png" alt="RRP" style="max-height:320px; width:auto;">
</div>

*Result (image of planes, origins, axis, links):*

<div style="text-align:center;">
  <img src="../recursos/imgs/HW3/S3.png" alt="RRP" style="max-height:320px; width:auto;">
</div>

**DH parameters**

| L | a     | α        | θ            | d            |
|---:|:------|:---------|:-------------|:-------------|
| 1 | $0$   | $-\pi/2$ | $q_1$        | $l_1$        |
| 2 | $l_2$ | $0$      | $q_2$        | $0$          |
| 3 | $0$   | $\pi/2$  | $\pi/2+q_3$  | $0$          |
| 4 | $0$   | $-\pi/2$ | $\pi/2+q_4$  | $l_3+l_4$    |
| 5 | $0$   | $\pi/2$  | $q_5$        | $0$          |
| 6 | $0$   | $0$      | $q_6$        | $l_5+l_6$    |

**DH after sustitution (script):**

$$
\begin{array}{c c c c c}
a & \alpha & d & \theta\\ \hline
0 & -1.5708 & 1 & 0\\
1 & 0       & 0 & 0\\
0 & 1.5708  & 0 & 1.5708\\
0 & -1.5708 & 2 & 1.5708\\
0 & 1.5708  & 0 & 0\\
0 & 0       & 2 & 0
\end{array}
$$

$$
T_1=
\begin{bmatrix}
1 & 0 & 0 & 0\\
0 & 0 & 1 & 0\\
0 & -1 & 0 & 1\\
0 & 0 & 0 & 1
\end{bmatrix}
\qquad
T_2=
\begin{bmatrix}
1 & 0 & 0 & 1\\
0 & 1 & 0 & 0\\
0 & 0 & 1 & 0\\
0 & 0 & 0 & 1
\end{bmatrix}
$$

$$
T_3=
\begin{bmatrix}
0 & 0 & 1 & 0\\
1 & 0 & 0 & 0\\
0 & 1 & 0 & 0\\
0 & 0 & 0 & 1
\end{bmatrix}
\qquad
T_4=
\begin{bmatrix}
0 & 0 & -1 & 0\\
1 & 0 & 0  & 0\\
0 & -1 & 0 & 2\\
0 & 0  & 0 & 1
\end{bmatrix}
$$

$$
T_5=
\begin{bmatrix}
1 & 0 & 0 & 0\\
0 & 0 & -1 & 0\\
0 & 1 & 0 & 0\\
0 & 0 & 0 & 1
\end{bmatrix}
\qquad
T_6=
\begin{bmatrix}
1 & 0 & 0 & 0\\
0 & 1 & 0 & 0\\
0 & 0 & 1 & 2\\
0 & 0 & 0 & 1
\end{bmatrix}
$$

$$
^{0}_{6}T=
\begin{bmatrix}
0 & 0 & 1 & 5\\
1 & 0 & 0 & 0\\
0 & 1 & 0 & 1\\
0 & 0 & 0 & 1
\end{bmatrix}
$$

---

## Excersice 4

<div style="text-align:center;">
  <img src="../recursos/imgs/HW3/E4.png" alt="RRP" style="max-height:320px; width:auto;">
</div>

*Result (image of planes, origins, axis, links):*

<div style="text-align:center;">
  <img src="../recursos/imgs/HW3/S4.png" alt="RRP" style="max-height:320px; width:auto;">
</div>

**DH parameters**

| L | $d_z$  | a   | α        | Θ              |
|---:|:-------|:----|:---------|:---------------|
| 1 | $l_1$  | $0$ | $-\pi/2$ | $q_1$          |
| 2 | $0$    | $l_2$ | $0$    | $q_2$          |
| 3 | $-l_3$ | $0$ | $\pi/2$  | $\pi/2+q_3$    |
| 4 | $l_4$  | $0$ | $-\pi/2$ | $q_4$          |
| 5 | $0$    | $0$ | $\pi/2$  | $-\pi/2+q_5$   |
| 6 | $l_6$  | $0$ | $0$      | $q_6$          |

**DH after sustitution (script):**  

$$
\begin{array}{c c c c c}
a & \alpha & d & \theta\\ \hline
0 & -1.5708 & 1  & 0\\
1 & 0       & 0  & 0\\
0 & 1.5708  & -1 & 1.5708\\
0 & -1.5708 & 1  & 0\\
0 & 1.5708  & 0  & -1.5708\\
0 & 0       & 1  & 0  
\end{array}
$$

$$
T_1=
\begin{bmatrix}
1 & 0 & 0 & 0\\
0 & 0 & 1 & 0\\
0 & -1 & 0 & 1\\
0 & 0 & 0 & 1
\end{bmatrix}
\qquad
T_2=
\begin{bmatrix}
1 & 0 & 0 & 1\\
0 & 1 & 0 & 0\\
0 & 0 & 1 & 0\\
0 & 0 & 0 & 1
\end{bmatrix}
$$

$$
T_3=
\begin{bmatrix}
0 & 0 & 1 & 0\\
1 & 0 & 0 & 0\\
0 & 1 & 0 & -1\\
0 & 0 & 0 & 1
\end{bmatrix}
\qquad
T_4=
\begin{bmatrix}
1 & 0 & 0 & 0\\
0 & 0 & 1 & 0\\
0 & -1 & 0 & 1\\
0 & 0 & 0 & 1
\end{bmatrix}
$$

$$
T_5=
\begin{bmatrix}
0 & 0 & -1 & 0\\
-1 & 0 & 0 & 0\\
0 & 1 & 0 & 0\\
0 & 0 & 0 & 1
\end{bmatrix}
\qquad
T_6=
\begin{bmatrix}
1 & 0 & 0 & 0\\
0 & 1 & 0 & 0\\
0 & 0 & 1 & 1\\
0 & 0 & 0 & 1
\end{bmatrix}
$$

$$
^{0}_{6}T=
\begin{bmatrix}
1 & 0 & 0 & 2\\
0 & 1 & 0 & -1\\
0 & 0 & 1 & 2\\
0 & 0 & 0 & 1
\end{bmatrix}
$$

---

## Excersice 5

<div style="text-align:center;">
  <img src="../recursos/imgs/HW3/E5.png" alt="RRP" style="max-height:320px; width:auto;">
</div>

*Result (image of planes, origins, axis, links):*

<div style="text-align:center;">
  <img src="../recursos/imgs/HW3/S5.png" alt="RRP" style="max-height:320px; width:auto;">
</div>

**DH parameters**

| L | $d_z$ | a     | α        | θ              |
|---:|:------|:------|:---------|:---------------|
| 1 | $0$   | $0$   | $\pi/2$  | $\pi/2+q_1$    |
| 2 | $0$   | $l_2$ | $-\pi/2$ | $\pi/2+q_2$    |
| 3 | $0$   | $0$   | $\pi/2$  | $q_3$          |
| 4 | $l_4$ | $0$   | $\pi/2$  | $q_4$          |
| 5 | $0$   | $0$   | $-\pi/2$ | $q_5$          |
| 6 | $l_6$ | $0$   | $0$      | $-\pi/2+q_6$   |

**DH after sustitution (script):**

$$
\begin{array}{c c c c}
a & \alpha & d & \theta\\ \hline
0 & 1.5708  & 0 & 1.5708\\
1 & 1.5708  & 0 & 1.5708\\
0 & -1.5708 & 0 & 0\\
0 & 1.5708  & 1 & 0\\
0 & -1.5708 & 0 & 0\\
0 & 0       & 1 & -1.5708
\end{array}
$$

$$
T_1=
\begin{bmatrix}
0 & 0 & 1 & 0\\
1 & 0 & 0 & 0\\
0 & 1 & 0 & 0\\
0 & 0 & 0 & 1
\end{bmatrix}
\qquad
T_2=
\begin{bmatrix}
0 & 0 & 1 & 0\\
1 & 0 & 0 & 1\\
0 & 1 & 0 & 0\\
0 & 0 & 0 & 1
\end{bmatrix}
$$

$$
T_3=
\begin{bmatrix}
1 & 0 & 0 & 0\\
0 & 0 & 1 & 0\\
0 & -1 & 0 & 0\\
0 & 0 & 0 & 1
\end{bmatrix}
\qquad
T_4=
\begin{bmatrix}
1 & 0 & 0 & 0\\
0 & 0 & -1 & 0\\
0 & 1 & 0 & 1\\
0 & 0 & 0 & 1
\end{bmatrix}
$$

$$
T_5=
\begin{bmatrix}
1 & 0 & 0 & 0\\
0 & 0 & 1 & 0\\
0 & -1 & 0 & 0\\
0 & 0 & 0 & 1
\end{bmatrix}
\qquad
T_6=
\begin{bmatrix}
0 & 1 & 0 & 0\\
-1 & 0 & 0 & 0\\
0 & 0 & 1 & 1\\
0 & 0 & 0 & 1
\end{bmatrix}
$$

$$
^{0}_{6}T=
\begin{bmatrix}
0 & 0 & 1 & 2\\
1 & 0 & 0 & 0\\
0 & 1 & 0 & 1\\
0 & 0 & 0 & 1
\end{bmatrix}
$$
