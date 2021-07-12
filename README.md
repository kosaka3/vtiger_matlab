# V-Tiger<sup>[1,2]</sup>

is similar to [Simulink Design Optimization](https://www.mathworks.com/help/sldo/ref/responseoptimizationtool-app.html)[3] that can automatically tune the controller gain to optimize settling time, overshoot, and stability margins. `Simulink Design Optimization` has a problem that requires an accurate model of the plant, but V-Tiger has no such problem. V-Tiger does not require the model but requires only step responses of the plant in an open- or closed-loop.

`Control System Toolbox` is required.

V-Tiger is ported to [Python](https://github.com/takoyagi77/vtiger4python)[4]. Thank you very much to takoyagi77.

### Examples

vtigerPID.m is a m-file to design PID controller using just only plant input/output step responses, and used in vtiger_demo.m. Please type:
```Matlab
vtiger_demo
```
, and PID controller `K = kp + ki/s + kd*s` is designed for the following plant G(s):
```Matlab
                        5
exp(-0.1*s) * ---------------------
              0.01 s^2 + 0.2 s + 10
```
using step responses data y00 instead of the plant model G(s). The overshoot and settling time is better than conventional Ziegler-Nichols rule method as follows:

![](https://fc5d4f24-a-62cb3a1a-s-sites.googlegroups.com/site/kosaka3lab/fairu--kyabinetto/vtiger_fig1.png)

### Overview

Since Artificial Intelligence (AI) has won over human pros such as Chess, Shogi and Go, expectations for AI have been increasing dramatically. One of the reasons why AI has developed so much is the tremendous increase in the processing speed of computers, which makes it possible to virtually repeat simulated competitions such as Othello, Shogi, Go and so on in the computer very fast. Finally, AI has gained strength over human pros. Also in control engineering, if gain tuning experiments of controllers can be virtually performed in a computer, it can be expected to dramatically improve control performance with an AI-like approach. `V-Tiger` is the abbreviation for "Virtual Time-response based Iterative Gain Evaluation and Redesign" which iterates: 1) to calculate virtual time responses of the closed-loop system when a certain controller is inserted based on one-shot experimental data, 2) to measure the overshoot and settling time from the virtual time responses, and 3) to evaluate and redesign the controller gain considering the stability margin.

### Keywords
Data-driven control, Time response, Gain tuning.

### References
[1] M. Kosaka et al., Virtual Time-response based Iterative Gain Evaluation and Redesign, In the proc. of the IFAC World Congress 2020 at Berlin (IFAC-PapersOnLine in 2020), July 2020.

[2] M. Kosaka et al., Virtual Time-response based Iterative Gain Evaluation and Redesign, T. of SICE, Vol. 56, No. 4, 2020  (in Japanese).

[3] https://www.mathworks.com/help/sldo/ref/responseoptimizationtool-app.html

[4] https://github.com/takoyagi77/vtiger4python
