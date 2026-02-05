# Homework 3: Forward Kinematics

## Excersice 1

$$
\textbf{DH after sustitution:  }
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

$$
\textbf{DH after sustitution:  }
\begin{array}{c c c c c}
a & \alpha & d & \theta &\\ \hline
0 & 1.5708 & 1 & 1.5708 &\\
0 & 1.5708 & 1 & 1.5708 &\\
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

$$
\textbf{DH after sustitution:  }
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

$$
\textbf{DH after sustitution:  }
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

$$
\textbf{DH after substitution:  }
\begin{array}{c c c c c}
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
