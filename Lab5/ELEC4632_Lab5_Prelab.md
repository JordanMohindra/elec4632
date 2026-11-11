# ELEC4632 Lab 5 – Pre-lab Exercise

Notation: $K_p$ proportional gain, $K_i$ integral gain, $h$ sampling time (0.75 s),
$e(k) = y_{ref}(k) - y(k)$, $u(k)$ the controller output.

## (a) ZOH equivalent of the PI controller

$$G_{CT}(s) = \frac{U(s)}{E(s)} = K_p + \frac{K_i}{s}$$

The zero-order-hold equivalent of a continuous transfer function is

$$G_{DT}(z) = (1 - z^{-1})\,\mathcal{Z}\left\{\frac{G_{CT}(s)}{s}\right\}
= \frac{z-1}{z}\,\mathcal{Z}\left\{\frac{K_p}{s} + \frac{K_i}{s^2}\right\}$$

Taking the inverse Laplace transform and sampling at $t = kh$:

$$\mathcal{L}^{-1}\left\{\frac{K_p}{s} + \frac{K_i}{s^2}\right\} = K_p\,\mathbb{1}(t) + K_i\,t
\quad\Rightarrow\quad K_p + K_i\,kh$$

Using $\mathcal{Z}\{1\} = \dfrac{z}{z-1}$ and $\mathcal{Z}\{kh\} = \dfrac{hz}{(z-1)^2}$:

$$\mathcal{Z}\left\{\frac{G_{CT}(s)}{s}\right\} = \frac{K_p z}{z-1} + \frac{K_i h z}{(z-1)^2}$$

Multiplying by $(z-1)/z$:

$$\boxed{G_{DT}(z) = \frac{U(z)}{E(z)} = K_p + \frac{K_i h}{z-1}}$$

which is Eq. (1). The ZOH equivalent of the integrator $1/s$ is $h/(z-1)$, which is the same
thing as the Forward Euler integrator used by the Simulink Discrete PID Controller block in
the real-time model.

## (b) Difference equation of the PI controller

Over a common denominator:

$$G_{DT}(z) = \frac{K_p(z-1) + K_i h}{z-1} = \frac{K_p z + (K_i h - K_p)}{z-1}$$

So

$$(z - 1)\,U(z) = \big[K_p z + (K_i h - K_p)\big]E(z)$$

Divide by $z$ to get delay operators, then take the inverse Z-transform:

$$(1 - z^{-1})\,U(z) = \big[K_p + (K_i h - K_p)z^{-1}\big]E(z)$$

$$\boxed{u(k) = u(k-1) + K_p\,e(k) + (K_i h - K_p)\,e(k-1)}$$

The same controller written in parallel form (how the Simulink block computes it):

$$u(k) = K_p\,e(k) + u_I(k), \qquad u_I(k) = u_I(k-1) + K_i h\,e(k-1)$$

## (c) Closed-loop transfer functions

From Fig. 1: $E(z) = Y_{ref}(z) - Y(z)$, $U(z) = G_c(z)E(z)$, $Y(z) = G(z)U(z)$.

$$Y = G G_c (Y_{ref} - Y) \;\Rightarrow\;
\boxed{G_{cl}(z) = \frac{Y(z)}{Y_{ref}(z)} = \frac{G_c(z)G(z)}{1 + G_c(z)G(z)}}$$

$$U = G_c (Y_{ref} - G U) \;\Rightarrow\;
\boxed{\frac{U(z)}{Y_{ref}(z)} = \frac{G_c(z)}{1 + G_c(z)G(z)}}$$

With the PI controller $G_c(z) = \dfrac{K_p z + (K_i h - K_p)}{z-1}$ and the identified
second-order process $G(z) = \dfrac{b_1 z + b_2}{z^2 + a_1 z + a_2}$:

$$G_{cl}(z) = \frac{\big(K_p z + K_i h - K_p\big)\big(b_1 z + b_2\big)}{D(z)}, \qquad
\frac{U(z)}{Y_{ref}(z)} = \frac{\big(K_p z + K_i h - K_p\big)\big(z^2 + a_1 z + a_2\big)}{D(z)}$$

$$D(z) = (z-1)(z^2 + a_1 z + a_2) + (K_p z + K_i h - K_p)(b_1 z + b_2)$$

Expanded, with $c = K_i h - K_p$:

$$D(z) = z^3 + (a_1 - 1 + K_p b_1)\,z^2 + (a_2 - a_1 + K_p b_2 + c\,b_1)\,z + (c\,b_2 - a_2)$$

At $z = 1$ the $(z-1)$ term vanishes, so $D(1) = K_i h\,(b_1 + b_2)$ and $G_{cl}(1) = 1$:
the integrator gives zero steady-state error to a step, and $U/Y_{ref}$ at $z = 1$ equals
$1/G(1)$, the input needed to hold the new level.

### Numbers for this lab

Identified model (Part 1): $a_1 = -1.377233$, $a_2 = 0.391909$, $b_1 = -0.006530$,
$b_2 = 0.050939$. Chosen gains (Part 2): $K_p = 1.30$, $K_i = 0.045$, $h = 0.75$ s.

$$G_{cl}(z) = \frac{-0.008489 z^2 + 0.07449 z - 0.06450}{z^3 - 2.3857 z^2 + 1.8436 z - 0.4564}$$

$$\frac{U(z)}{Y_{ref}(z)} = \frac{1.3 z^3 - 3.0567 z^2 + 2.2534 z - 0.4963}{z^3 - 2.3857 z^2 + 1.8436 z - 0.4564}$$

Closed-loop poles: 0.9735, 0.8786, 0.5336 (all inside the unit circle, so the loop is
stable). $G_{cl}(1) = 1$.
