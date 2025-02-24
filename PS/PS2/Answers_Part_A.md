## Q1
Write the value for the covariance Q of the noise added to the observation function, knowing that the parameter `bearing_std` is its standard deviation.

## A1
Value of `bearing_std` is $0.35$, thus $Q = 0.35^2 = 0.1225$

## Q2
Write the equation for the covariance $R_t$ of the noise added to the transition function, and their corresponding numeric values for the initial robot command $u = [ \delta_{rot1}, \delta_{trans}, \delta_{rot2} ]^\top  = [0, 10, 0]^\top$.

## A2
According to lecture 3, odometry model, $R$ calculated as $R = VMV^\top$, where $V$ is the Jacobian matrix of the transition function with derriviative according to control input $u$ and $M$ is covariance value:

$$
M = \begin{bmatrix} 
\alpha_1\delta^2_{rot1} + \alpha_2\delta^2_{trans} & 0 & 0 \\
0 & \alpha_3\delta^2_{trans} + \alpha_4(\delta^2_{rot1} + \delta^2_{rot2}) & 0 \\
0 & 0 & \alpha_1\delta^2_{rot2} + \alpha_2\delta^2_{trans}
\end{bmatrix}
$$

Nummerically:

$$
M = \begin{bmatrix} 
0.05^2 * 0^2 + 0.001^2 * 10^2 & 0 & 0 \\
0 & 0.05^2 * 10^2 + 0.01^2(0^2 + 0^2) & 0 \\
0 & 0 & 0.05^2 * 0^2 + 0.001^2 * 10^2
\end{bmatrix}
$$

$$
M = \begin{bmatrix} 
0.05^2 & 0 & 0 \\
0 & 0.05^2 & 0 \\
0 & 0 & 0.05^2
\end{bmatrix} = 0.05^2 \times \mathrm{I}
$$

Now, we need $V$

## Q3
Derive the equations for the Jacobians $G_t$, $V_t$ and $H_t$, and evaluate them at the initial mean state $\mu_1 = [x, y, \theta]^\top = [180, 50, 0]^\top$.

## A3
From same lecture we can take the Jacobian $G_t$:
$$
G_t = \left.\frac{\partial g(x_{t-1}, u_t, \varepsilon_t)}{\partial x_{t-1}}\right|_{\mu_{t-1},\varepsilon_t=0} = 
\underset{\partial/\partial x \quad  \partial/\partial y \quad \quad \quad \quad \quad \quad \quad
 \partial/\partial \theta}{\begin{bmatrix}
1 \quad & 0 \quad & -\delta_{trans}\sin(\theta + \delta_{rot1}) \\
0 \quad & 1 \quad & \delta_{trans}\cos(\theta + \delta_{rot1}) \\
0 \quad & 0 \quad & 1
\end{bmatrix}}
$$

$$
G_1 = \begin{bmatrix}
1 & 0 & 0 \\
0 & 1 & 10 \\
0 & 0 & 1
\end{bmatrix}
$$

Jacobian $V_t$ is:

$$
V_t = \left.\frac{\partial g(x_{t-1}, u_t, \varepsilon_t)}{\partial u_t}\right|_{\mu_{t-1},\varepsilon_t=0} = 
\begin{bmatrix}
-\delta_{trans} \cdot \sin(\theta + \delta_{rot1}) & \cos(\theta + \delta_{rot1}) & 0 \\
\delta_{trans} \cdot \cos(\theta + \delta_{rot1}) & \sin(\theta + \delta_{rot1}) & 0 \\
1 & 0 & 1
\end{bmatrix}
$$

$$
V_1 = \begin{bmatrix}
0 & 1 & 0 \\
10 & 0 & 0 \\
1 & 0 & 1
\end{bmatrix}
$$

Returning to previous question, let's calculate $R$:

$$
R = VMV^\top = 
\begin{bmatrix}
0 & 1 & 0 \\
10 & 0 & 0 \\
1 & 0 & 1
\end{bmatrix} \times 
\begin{bmatrix} 
0.05^2 & 0 & 0 \\
0 & 0.05^2 & 0 \\
0 & 0 & 0.05^2
\end{bmatrix} \times 
\begin{bmatrix} 
0 & 10 & 1 \\
1 & 0 & 0 \\
0 & 0 & 1
\end{bmatrix}
$$

$$
R = 
\begin{bmatrix}
0.05^2 & 0 & 0 \\
0 & 0.5^2 & 0.1 \times 0.5^2 \\
0.05^2 & 0.1 \times 0.5^2 & 0.05^2
\end{bmatrix}
$$

And finally Jacobian $H_t$ is:

$$
H = \begin{bmatrix}
0 & 0 & 0 \\
\frac{\partial}{\partial x}(\text{atan2}(m_{i,y} - y, m_{i,x} - x) - \theta) & \frac{\partial}{\partial y}(\text{atan2}(m_{i,y} - y, m_{i,x} - x) - \theta) & -1 \\
0 & 0 & 0
\end{bmatrix}
$$

first row is $0$ because in our case range is zero. Thus: 

$$
H = \begin{bmatrix}
0 & 0 & 0 \\
\frac{m_{i,y} - y}{(m_{i,x} - x)^2 + (m_{i,y} - y)^2} & \frac{-(m_{i,x} - x)}{(m_{i,x} - x)^2 + (m_{i,y} - y)^2} & -1 \\
0 & 0 & 0
\end{bmatrix}
$$

