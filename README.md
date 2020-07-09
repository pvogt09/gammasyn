# gammasyn


## Purpose

Pole placement or pole assignment is a major control design method for linear time-invariant systems.
*gammasyn* is a toolbox for Matlab which enables an easy (robust) pole region assignment by
* offering an easy description of the (robust) pole region assignment and
* providing a unified interface for a large number of optimizers.

As this control design method relies on "heavy" optimization, this toolbox offers functionality to create and use compiled and paralleled versions of crucial functions, provided the necessary Matlab toolboxes are available.

It works with continuous time and discrete time systems.

This documentation assumes a basic knowledge of control systems and the concept of pole placement.



### Pole region assignment

With static and/or structured output feedback it is in general not possible to assign the poles of the closed loop system freely.
In this case the control objective may be softened in the sense that no specific values for the closed loop poles are demanded, but a region is specified, in which the poles of the closed loop have to lie.

Another application is robust control.
Here the aim is to design a single controller which has to work for a whole set of plants with different parameters.
In this case it is not possible to achieve the exact same pole locations for all admissible plants, even with full state feedback, and one has to soften the objectives to a region as well.

Especially in the robust controller design with rather large parameter variation it often proves to be a hard task to place all poles in the desired region.
With this in mind, this toolbox allows to define two fundamental regions:
* A "hard" region which translates to an optimization constraint. This region should assure stability and a minimal performance.
* A "soft" region which translates to an optimization objective. The optimization tries to place all poles within or as near as possible to this region, which may be motivated by some more eager performance goals.

In addition, individual pole regions can be defined for the different plants within a robust design.
This allows to define a good control performance for plants near the rated plant and only some basic behavior for plants under "extreme" working conditions.




## Installation and setup


### Prerequisities

* Matlab R2015B (might also work with older versions but is not tested) (Octave is not supported because it does not support packages)
* Toolboxes:
  * Matlab Coder (optional, if objective and constraint functions should be compiled. If the eigenvalue derivative calculation method of van der Aa is to be used, at least Matlab R2016A is needed to support runtime recursion in generated code)
  * Control Systems Toolbox (optional, if `tf` and `ss` system descriptions should be used)
  * Robust Control Toolbox (optional, if uncertain or parametric system descriptions should be used)
  * Symbolic Toolbox (optional, if symbolic constraints should be used)

If optimizers of the Optimization Toolbox or Global Optimization Toolbox of Matlab are used, the corresponding toolbox is necessary.


### Installation
This repository has to be cloned or copied to a location accessible for the local Matlab installation.
For the ease of installation and usage, it contains all necessary code including the open source optimizers.


### Setup
To set up *gammasyn* within Matlab the script *startup.m* has to be executed.
The effects of this script are only temporary, therefore it has to be executed anew after a restart of Matlab.

This script works directly in the base workspace.
It cleans up behind itself, i.e. it clears all variables used by it, but it does not check beforehand if the variables were in use.
Therefore, it is recommended to execute this script with a cleared base workspace to avoid an inconsistent state.


## Minimal examples

### Test system

For the minimal examples the test system is a simple one-mass-oscillator described by

<p align="center"><img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/cc6824dc09cabeaff44dffe078fa4051.svg?invert_in_darkmode" align=middle width=272.43645pt height=95.16181784999999pt/></p>

The nominal parameters are <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/a08aa3f720eb59983ccb69372a8b620d.svg?invert_in_darkmode" align=middle width=44.56994024999999pt height=21.18721440000001pt/>, <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/7474a88176e3b7318768ff82da2d4a2c.svg?invert_in_darkmode" align=middle width=46.91201294999998pt height=22.831056599999986pt/> and <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/982ad20eaa8b08dbe1bd927db94f4959.svg?invert_in_darkmode" align=middle width=61.90827224999998pt height=21.18721440000001pt/>.
For the robust design the parameters <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/0e51a2dede42189d77627c4d742822c3.svg?invert_in_darkmode" align=middle width=14.433101099999991pt height=14.15524440000002pt/> and <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/2103f85b8b1477f430fc407cad462224.svg?invert_in_darkmode" align=middle width=8.55596444999999pt height=22.831056599999986pt/> are assumed to be uncertain, given by

<p align="center"><img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/72a72220711931d569f1f838a3132f6b.svg?invert_in_darkmode" align=middle width=98.4512991pt height=41.09589pt/></p>

The following function is used in the examples to construct the system for given parameter values <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/0e51a2dede42189d77627c4d742822c3.svg?invert_in_darkmode" align=middle width=14.433101099999991pt height=14.15524440000002pt/>, <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/2103f85b8b1477f430fc407cad462224.svg?invert_in_darkmode" align=middle width=8.55596444999999pt height=22.831056599999986pt/> and <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/3e18a4a28fdee1744e5e3f79d13b9ff6.svg?invert_in_darkmode" align=middle width=7.11380504999999pt height=14.15524440000002pt/>:
```matlab
function sys = omo_sys(m, d, c)

    A = [0, 1; -c/m, -d/m];
    B = [0; 1/m];
    C = [1, 0];

    sys = struct('A', A, 'B', B, 'C', C);

end
```


### Pole region assignment

As simplest example static proportional output feedback is applied to the rated system

```matlab
sys = omo_sys(1, 10, 1000);
```

As only the position is measured, the poles cannot be placed arbitrarily.

#### Region

The target region is the sector shown in the following image.

<img src="docs/images/tex/tikz_ext/ex-omo-area.png" width=25%/>

It can be defined by
```matlab
polearea = [
    control.design.gamma.area.Line(1, 0),...
    control.design.gamma.area.Imag(1, 1)
];
```

For this example the solution can be determined analytically:

<p align="center"><img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/2567c6558fe0b341767a7e13c90ac5a4.svg?invert_in_darkmode" align=middle width=129.50350545pt height=16.438356pt/></p>

(<img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/1e438235ef9ec72fc51ac5025516017c.svg?invert_in_darkmode" align=middle width=12.60847334999999pt height=22.465723500000017pt/> is negative, which means that it is actually positive feedback.
This is correct, as with this feedback structure the only possibility to dampen the system is to partly "compensate" the spring <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/3e18a4a28fdee1744e5e3f79d13b9ff6.svg?invert_in_darkmode" align=middle width=7.11380504999999pt height=14.15524440000002pt/>.)

#### Pole region assignment

```matlab
weight = 1;
R0 = 1;
Rfixed = [];
optimizer = optimization.solver.Optimizer.IPOPT;

gammaopts = control.design.gamma.GammasynOptions();

[R_opt, J_opt, info] = control.design.gamma.gammasyn(...
	sys,...
	polearea, weight,...
	Rfixed, R0, optimizer, gammaopts...
);
```


#### Additional objective term

With the code above, all solutions for <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/1e438235ef9ec72fc51ac5025516017c.svg?invert_in_darkmode" align=middle width=12.60847334999999pt height=22.465723500000017pt/> within the interval <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/13233b491e745213e4f4d891faadb905.svg?invert_in_darkmode" align=middle width=96.80389784999998pt height=24.65753399999998pt/> are "equally good" solution of the feasibility problem.
The exact value found depends on the initial value and the optimizer used.
(You could change the start value to <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/cd9981ea0a7b60c42e599e40874e4798.svg?invert_in_darkmode" align=middle width=45.66227159999998pt height=21.18721440000001pt/> to observe a difference.)

This means there exists a certain degree of freedom which can be used for other purposes.
The following code finds the controller among the feasible ones with the smallest norm of the feedback matrix.
In this case, it is just the <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/1e438235ef9ec72fc51ac5025516017c.svg?invert_in_darkmode" align=middle width=12.60847334999999pt height=22.465723500000017pt/> with the smallest absolute value, i.e. the unique solution now is <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/5e6cdf4bc23893eaed8ff5077bdd34de.svg?invert_in_darkmode" align=middle width=91.61394659999998pt height=22.465723500000017pt/>.

```matlab
weight = 1;
R0 = 1;
Rfixed = [];
optimizer = optimization.solver.Optimizer.IPOPT;

gammaopts = control.design.gamma.GammasynOptions(...
	'type', GammaJType.NORMGAIN,...
	'objective.normgain.R', (1)...
);

[R_opt, J_opt, info] = control.design.gamma.gammasyn(...
	sys,...
	polearea, weight,...
	Rfixed, R0, optimizer, gammaopts...
);
```


### Robust pole region assignment

#### Systems

Now it is assumed that the mass and the damping coefficient are not exactly known but only intervals are given.
To apply the multiple model approach in a first step the "corner" models as well as the rated model are build and stored in one vector.

```matlab
m = [0.9, 1.1];
d = [8, 12];
c = 1000;

sys = [
	omo_sys(m(1), d(1), c),...
	omo_sys(m(2), d(1), c),...
	omo_sys(m(1), d(2), c),...
	omo_sys(m(2), d(2), c),...
	omo_sys(mean(m), mean(d), c)
];
```

#### Region

The target pole region is the same as in the example above.

```matlab
polearea = [
	control.design.gamma.area.Line(1, 0),...
	control.design.gamma.area.Imag(1, 1)
];
```

#### Robust pole region assignment

Also the call to *gammasyn* is the same as above, the only difference is that `sys` now contains five instead of one system.

```matlab
weight = 1;
R0 = 1;
Rfixed = [];
optimizer = optimization.solver.Optimizer.IPOPT;

gammaopts = control.design.gamma.GammasynOptions();

[R_opt, J_opt, info] = control.design.gamma.gammasyn(...
	sys,...
	polearea, weight,...
	Rfixed, R0, optimizer, gammaopts...
);
```

This task proves to be feasible, i.e. there exists a controller gain for which the poles of all five closed loops lie within the specified region.



The following image shows the poles of all five closed loop systems 

<img src="docs/images/tex/tikz_ext/ex-omo-area-probust-sol.png" width=25%/>

This multiple model approach is heuristic and makes no guarantees about the <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/b2af456716f3117a91da7afe70758041.svg?invert_in_darkmode" align=middle width=10.274003849999989pt height=22.465723500000017pt/>-stability of the other systems described by the given parameter ranges.
It is advisable to check the properties for a larger subset of the admissible systems.
This is made in the following image, where the poles of 100 additional closed loop systems are shown in gray.

<img src="docs/images/tex/tikz_ext/ex-omo-area-probust-sol2.png" width=25%/>

## Method/Theory

### System and Controller

This framework considers static, possibly structured output feedback only.
As will be discussed later, this is actually not a restriction, as any dynamic feedback can be cast into this form.

In the simplest form for this framework a system is given by the three matrices <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/53d147e7f3fe6e47ee05b88b166bd3f6.svg?invert_in_darkmode" align=middle width=12.32879834999999pt height=22.465723500000017pt/>, <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/61e84f854bc6258d4108d08d4c4a0852.svg?invert_in_darkmode" align=middle width=13.29340979999999pt height=22.465723500000017pt/> and <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/9b325b9e31e85137d1de765f43c0f8bc.svg?invert_in_darkmode" align=middle width=12.92464304999999pt height=22.465723500000017pt/> of the state space representation

<p align="center"><img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/79b8a0e65d55f29f0ebc79692907e8c3.svg?invert_in_darkmode" align=middle width=95.8312509pt height=39.086746049999995pt/></p>

and the control loop is to be closed with the controller

<p align="center"><img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/431a93c229be016721b862bcab9a46a4.svg?invert_in_darkmode" align=middle width=106.18908134999998pt height=14.42921205pt/></p>

where <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/89f2e0d2d24bcf44db73aab8fc03252c.svg?invert_in_darkmode" align=middle width=7.87295519999999pt height=14.15524440000002pt/> is the reference value.
This leads to a closed loop 

<p align="center"><img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/6441c187a5f19bdb2b066bcd90e6801e.svg?invert_in_darkmode" align=middle width=190.72297365pt height=16.438356pt/></p>

whose poles <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/962558fefb867b121af066dabdd42400.svg?invert_in_darkmode" align=middle width=16.94645864999999pt height=22.831056599999986pt/> are the solutions of the eigenvalue problem

<p align="center"><img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/5ce3ada98f33f1a871ae8d97eb96a53a.svg?invert_in_darkmode" align=middle width=196.16081265pt height=16.438356pt/></p>


#### Mass matrix

As a small notational convenience in some cases, the model can be described as

<p align="center"><img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/dd772d5cdd3a074845f5a783f28d6c0c.svg?invert_in_darkmode" align=middle width=108.91340955pt height=39.086746049999995pt/></p>

with the *invertible* mass matrix <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/84df98c65d88c6adf15d4645ffa25e47.svg?invert_in_darkmode" align=middle width=13.08219659999999pt height=22.465723500000017pt/>.

The feedback has the same form as above which leads to the associated eigenvalue problem

<p align="center"><img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/5f8aa2ed9dd8397b11b636aec205b4c9.svg?invert_in_darkmode" align=middle width=200.72702429999998pt height=16.438356pt/></p>

to determine the eigenvalues or poles of the closed loop system.


#### Differential feedback

To allow modeling true differential feedback, the model can be extended to

<p align="center"><img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/3f35481c00d321dd83e66a5c15e9186e.svg?invert_in_darkmode" align=middle width=99.6973593pt height=63.744281699999995pt/></p>

for which the controller has the structure

<p align="center"><img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/b6d180e9ddba9cd08b6889365ecc49e3.svg?invert_in_darkmode" align=middle width=154.67835405pt height=16.3763325pt/></p>

* The prime-notation <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/15f93b25ba881e5829e8fc647b680fb2.svg?invert_in_darkmode" align=middle width=12.43916849999999pt height=24.7161288pt/> is not the same as <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/5ab82db224334014ef8fba6bda1943d8.svg?invert_in_darkmode" align=middle width=8.649225749999989pt height=21.95701200000001pt/> but allows that not all or others outputs are used for the differential feedback than for the "normal" feedback. If all outputs should be used for the differential feedback, i.e. <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/bd5a17cad5a2a508fe4fbd64a14a2d1b.svg?invert_in_darkmode" align=middle width=43.82793689999998pt height=24.7161288pt/>, then <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/5a9e46cf5d21302df360f9c1c5161e52.svg?invert_in_darkmode" align=middle width=52.37878634999999pt height=24.7161288pt/> can be chosen.
* The differential feedback is defined as positive feedback whereas the normal feedback is defined as negative feedback. This is a deliberate choice which leads to a more symmetric generalized eigenvalue problem

<p align="center"><img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/da5d99d4cc5e0f1916d255bcef7c10f1.svg?invert_in_darkmode" align=middle width=286.87633589999996pt height=17.2895712pt/></p>



#### Process variables

As the model used here is an augmented system, as discussed below, the output <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/deceeaf6940a8c7a5a02373728002b0f.svg?invert_in_darkmode" align=middle width=8.649225749999989pt height=14.15524440000002pt/> doesn't generally reflect the actual process variables.
Therefore, the process variables for which sensible reference values (or set points) exist are described by an additional output equation:

<p align="center"><img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/02978dc3af4338072b52fa50af24d230.svg?invert_in_darkmode" align=middle width=144.62609039999998pt height=63.744281699999995pt/></p>



#### Full continuous time model

Combining all extensions, the most general system description used by this toolbox is

<p align="center"><img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/61320ea8d976382c9602dff3b7714f21.svg?invert_in_darkmode" align=middle width=144.62609039999998pt height=88.4018157pt/></p>

where <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/84df98c65d88c6adf15d4645ffa25e47.svg?invert_in_darkmode" align=middle width=13.08219659999999pt height=22.465723500000017pt/> must be an invertible matrix and the controller is given by

<p align="center"><img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/b6d180e9ddba9cd08b6889365ecc49e3.svg?invert_in_darkmode" align=middle width=154.67835405pt height=16.3763325pt/></p>

The eigenvalues or poles of the closed loop are the solution of the generalized eigenvalue problem

<p align="center"><img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/50289367060766165207a4ef95fe5c08.svg?invert_in_darkmode" align=middle width=291.44254755pt height=17.2895712pt/></p>

The structure is depicted here:

<img src="docs/images/tex/tikz_ext/augsys-cl-full.png" width=75%/>




#### Discrete time model
The discrete time model is defined analogously to the continuous time case as

<p align="center"><img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/4192c7ca84506298908e0e3ce34dd463.svg?invert_in_darkmode" align=middle width=182.33157195pt height=89.9086386pt/></p>

where <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/84df98c65d88c6adf15d4645ffa25e47.svg?invert_in_darkmode" align=middle width=13.08219659999999pt height=22.465723500000017pt/> must be an invertible matrix and the controller is given by

<p align="center"><img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/bd2468c2da6b85be45a6e9f316ee9748.svg?invert_in_darkmode" align=middle width=179.96015565pt height=17.24382pt/></p>

The discrete time analogous "derivative" output <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/23eee364c1d8932bb54dc8fa9e0709e2.svg?invert_in_darkmode" align=middle width=15.325460699999988pt height=24.7161288pt/> is only defined for accordance with the continuous time system matrices and serves no engineering purpose because it results in a non causal system.

The structure is depicted here:

<img src="docs/images/tex/tikz_ext/augsys-cl-full-discrete.png" width=75%/>



### Augmented System

The system given by the structure described above is an augmented system in the sense that it may contain parts of the controller.

The approach applied in this toolbox relies on static structured output feedback.
However, this is a very general approach, as all the controller dynamics can be added to the system, resulting in the "augmented system".

If for example the system

<p align="center"><img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/79b8a0e65d55f29f0ebc79692907e8c3.svg?invert_in_darkmode" align=middle width=95.8312509pt height=39.086746049999995pt/></p>

is to be controlled by a PI-controller

<p align="center"><img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/5a250d6cd9eda5dc5de503d4f8dd076d.svg?invert_in_darkmode" align=middle width=149.97517425pt height=36.53007435pt/></p>

with <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/b0b4a9c53fe95b7982c4d835492ac549.svg?invert_in_darkmode" align=middle width=66.18513989999998pt height=19.1781018pt/>, <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/89f2e0d2d24bcf44db73aab8fc03252c.svg?invert_in_darkmode" align=middle width=7.87295519999999pt height=14.15524440000002pt/> being the reference value, which can be written in the state space representation

<p align="center"><img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/02dc8a3d6e5f3b6d4092a4879105eab5.svg?invert_in_darkmode" align=middle width=175.32334545pt height=39.086746049999995pt/></p>

the resulting augmented system is

<p align="center"><img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/e2f626007b3991335f65b78aca654f5f.svg?invert_in_darkmode" align=middle width=238.0232877pt height=85.48022999999999pt/></p>

to which the static output feedback

<p align="center"><img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/bc4bdaff1419aef520ae65cbf999528a.svg?invert_in_darkmode" align=middle width=233.8014558pt height=61.890745949999996pt/></p>

is applied.
This is a *structured* feedback, as the second row of the feedback matrix <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/1e438235ef9ec72fc51ac5025516017c.svg?invert_in_darkmode" align=middle width=12.60847334999999pt height=22.465723500000017pt/> doesn't contain any free parameter but values which must not be altered by the optimizer.

More generally, if the given system is controlled with a general dynamic controller

<p align="center"><img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/3b2a09cf0713942791bea9eaa603d080.svg?invert_in_darkmode" align=middle width=183.59271149999998pt height=39.086746049999995pt/></p>

(where <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/9e33e6bbd7546c272dc91ad2a39ca6c4.svg?invert_in_darkmode" align=middle width=22.22612204999999pt height=22.465723500000017pt/> to <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/607e0a21936280851772fd807af6b597.svg?invert_in_darkmode" align=middle width=23.50691969999999pt height=22.465723500000017pt/> may be structured) the augmented system is

<p align="center"><img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/310a5a5c838c098bcca85b1f6e6dc81c.svg?invert_in_darkmode" align=middle width=248.32008464999996pt height=85.48022999999999pt/></p>

which is closed by

<p align="center"><img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/571c124d01eddb3091a294a161c7c5ae.svg?invert_in_darkmode" align=middle width=245.13942254999998pt height=61.890745949999996pt/></p>

where <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/1e438235ef9ec72fc51ac5025516017c.svg?invert_in_darkmode" align=middle width=12.60847334999999pt height=22.465723500000017pt/> (and <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/b8bc815b5e9d5177af01fd4d3d3c2f10.svg?invert_in_darkmode" align=middle width=12.85392569999999pt height=22.465723500000017pt/>) are generally structured corresponding to the structure of <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/9e33e6bbd7546c272dc91ad2a39ca6c4.svg?invert_in_darkmode" align=middle width=22.22612204999999pt height=22.465723500000017pt/> to <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/607e0a21936280851772fd807af6b597.svg?invert_in_darkmode" align=middle width=23.50691969999999pt height=22.465723500000017pt/> (and <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/2144c3c4436ef35026e895fed8fd671f.svg?invert_in_darkmode" align=middle width=17.12334524999999pt height=22.465723500000017pt/> and <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/3956a0723425282b441b7a17d9cc4be7.svg?invert_in_darkmode" align=middle width=17.12334524999999pt height=22.465723500000017pt/>).

As such structure is mandatory to achieve given controller structures as for example PI or PID controllers, this toolbox provides the possibility to define such structures.




### Pole region

The basic aim of this control design procedure is to determine <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/1e438235ef9ec72fc51ac5025516017c.svg?invert_in_darkmode" align=middle width=12.60847334999999pt height=22.465723500000017pt/> and <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/d6328eaebbcd5c358f426dbea4bdbf70.svg?invert_in_darkmode" align=middle width=15.13700594999999pt height=22.465723500000017pt/> such that all poles <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/962558fefb867b121af066dabdd42400.svg?invert_in_darkmode" align=middle width=16.94645864999999pt height=22.831056599999986pt/>, <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/ef66b7d203ea6148b4134cf2b8a8c9c5.svg?invert_in_darkmode" align=middle width=96.65870279999999pt height=21.18721440000001pt/>, of the closed loop system lie within a certain region <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/b2af456716f3117a91da7afe70758041.svg?invert_in_darkmode" align=middle width=10.274003849999989pt height=22.465723500000017pt/> of the complex plane.

This toolbox distinguishes between two regions:
* <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/de5a52553d8cf2cc14565b900b829269.svg?invert_in_darkmode" align=middle width=36.484191149999994pt height=22.465723500000017pt/>: All poles <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/962558fefb867b121af066dabdd42400.svg?invert_in_darkmode" align=middle width=16.94645864999999pt height=22.831056599999986pt/> must lie within this region to consider the problem solved.
* <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/1936e598115038507ce0370e92814601.svg?invert_in_darkmode" align=middle width=31.24674299999999pt height=22.465723500000017pt/>: All poles should lie within or as near as possible to this region.



For a compact notation, the real part of a complex value is written as <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/8cda31ed38c6d59d14ebefa440099572.svg?invert_in_darkmode" align=middle width=9.98290094999999pt height=14.15524440000002pt/> and the imaginary part as <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/ae4fb5973f393577570881fc24fc2054.svg?invert_in_darkmode" align=middle width=10.82192594999999pt height=14.15524440000002pt/>, i.e. for example

<p align="center"><img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/9967ec9c5bc32f0503537757369d6fa6.svg?invert_in_darkmode" align=middle width=99.96191864999999pt height=14.611878599999999pt/></p>


#### Mathematical description of pole regions

A region is defined by one or the intersection of more areas.
Here, "area" refers to the "left side" of a curve in the complex plane.

<p align="center"><img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/d6e7c93d3dc694dba8b20e4b1fe1dddb.svg?invert_in_darkmode" align=middle width=325.115769pt height=69.0417981pt/></p>

Depending on the optimizer, a function <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/7d9cb65db969d7e8addac3679a5e6916.svg?invert_in_darkmode" align=middle width=60.74912744999998pt height=24.65753399999998pt/> should be differentiable twice after each argument.

A region is defined as a set of areas, <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/bc3dd8032ae136048582aa1b14dbbb0e.svg?invert_in_darkmode" align=middle width=126.06144375pt height=24.65753399999998pt/>.
The condition that all poles lie within this area translates to

<p align="center"><img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/59b293ae4c0329f390ee07395e5bc4d0.svg?invert_in_darkmode" align=middle width=322.29142275pt height=17.031940199999998pt/></p>

For the robust case, where <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/0e51a2dede42189d77627c4d742822c3.svg?invert_in_darkmode" align=middle width=14.433101099999991pt height=14.15524440000002pt/> models are to be considered, the condition is

<p align="center"><img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/ca50c663f676c922c62da6fbd9d7915b.svg?invert_in_darkmode" align=middle width=488.23708394999994pt height=17.031940199999998pt/></p>

* The region may depend on the model <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/07617f9d8fe48b4a7b3f523d6730eef0.svg?invert_in_darkmode" align=middle width=9.90492359999999pt height=14.15524440000002pt/>. This can be important from a practical point of view. If the uncertainty is rather large one may have to loosen the performance goals, described by the region, for corner case models.
* The system order <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/b8bec55ac391e9fa94465fbf510f20cd.svg?invert_in_darkmode" align=middle width=17.85973364999999pt height=14.15524440000002pt/> may depend on the model <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/07617f9d8fe48b4a7b3f523d6730eef0.svg?invert_in_darkmode" align=middle width=9.90492359999999pt height=14.15524440000002pt/> as well.

As there are two pole regions, <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/de5a52553d8cf2cc14565b900b829269.svg?invert_in_darkmode" align=middle width=36.484191149999994pt height=22.465723500000017pt/> and <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/1936e598115038507ce0370e92814601.svg?invert_in_darkmode" align=middle width=31.24674299999999pt height=22.465723500000017pt/>, there are also two sets of functions <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/52d8d23328f6321bd363d9d31b42b4cc.svg?invert_in_darkmode" align=middle width=22.457762249999988pt height=14.15524440000002pt/>: <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/dbbfd6a0d7f6f830f7d362fc841281b7.svg?invert_in_darkmode" align=middle width=129.05609474999997pt height=24.65753399999998pt/> and <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/151a8a36f5efa9a4b3a7696282e72e3d.svg?invert_in_darkmode" align=middle width=118.58120834999998pt height=24.65753399999998pt/>.




### Problem formulation

The aim is to determine the matrices of the controller

<p align="center"><img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/b6d180e9ddba9cd08b6889365ecc49e3.svg?invert_in_darkmode" align=middle width=154.67835405pt height=16.3763325pt/></p>


#### Controller structure

Generally it is structured feedback, that is, the matrices cannot be chosen freely but certain entries are fixed and there may be additional conditions to be respected.
Mathematically fixed entries and linear dependencies between different entries can be expressed in the form

<p align="center"><img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/77d8641ea4edf0196c7175849625519c.svg?invert_in_darkmode" align=middle width=149.0489649pt height=66.34700985pt/></p>

which allows dependecies of entries of the same matrix only or the more general form

<p align="center"><img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/c786656a197398007fe3b59173f467c1.svg?invert_in_darkmode" align=middle width=207.01724174999998pt height=59.1786591pt/></p>

where <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/13f9c330184054cf60379a7375c5472a.svg?invert_in_darkmode" align=middle width=22.831119299999987pt height=14.15524440000002pt/> is the vectorization operator.
Mathematically the latter form comprises the precedent three equations, but this framework allows the specification in either form or both forms simultanously.

The notation used here is versatile.
Of course equality conditions of the form <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/a77f5c7d34114e3f704ff10b3466e703.svg?invert_in_darkmode" align=middle width=73.36185449999999pt height=24.65753399999998pt/> actually simply reduce the effective number of optimization variables.
The same is valid for linear equation constraints between two and more optimization variables.
This is used by the toolbox when it constructs the problem, but for the sake of readability it is not denoted explicitly here.

The possibility to formulate linear equality conditions is necessary for the design of a structured controller.
Not necessary but possible are linear inequality conditions (aside from the ones resulting from the pole region constraints which are introduced below), which can be specified in the form

<p align="center"><img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/4d37086b1cebd6410e45bfe7b3196716.svg?invert_in_darkmode" align=middle width=149.96215245pt height=66.34700985pt/></p>

and

<p align="center"><img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/bb2b5d8d419645d7b031743fdf55c08f.svg?invert_in_darkmode" align=middle width=207.9304293pt height=59.1786591pt/></p>

To provide more flexibility, this toolbox allows also for nonlinear equality and inequality conditions,

<p align="center"><img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/5a9e42d9048c1a4457ca02917a250d5a.svg?invert_in_darkmode" align=middle width=88.86044475pt height=140.31961184999997pt/></p>

Instead of referring to these seven equations and seven inequalities in the feasibility and optimization problems that follow, it is used the shorter notation

<p align="center"><img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/8f7731e6ac7c8ec2aef246b280d6edbc.svg?invert_in_darkmode" align=middle width=109.32055199999999pt height=16.438356pt/></p>

For example

<p align="center"><img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/87fb7b10f72e22eaa08922919676841b.svg?invert_in_darkmode" align=middle width=91.45655144999999pt height=25.47942045pt/></p>






#### Pole regions

These are translated into constraints or into an objective function, depending on the type of region (hard or soft) and the capabilities of the optimizer.


##### Hard pole region - Constrained optimization

If the optimizer supports inequality constraints directly,

<p align="center"><img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/1a2d2b88eca03a70fe09a98ce5c9c0d2.svg?invert_in_darkmode" align=middle width=598.3804348499999pt height=56.5021809pt/></p>

If no additional objective function is given, i.e. <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/570884d514512e4e25a77aaf2bae5688.svg?invert_in_darkmode" align=middle width=40.83319019999999pt height=22.465723500000017pt/>, this is a feasibility problem.

The weights <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/28457d95bce65adb96ba5eb9eebd9dec.svg?invert_in_darkmode" align=middle width=26.58161714999999pt height=14.15524440000002pt/> are not necessary from a theoretical - and mostly practical - point of view.
Generally they should be set to 1.

* The weights can be used to reverse the left and right side of an area.
* They could be used to "help" the optimizer to find a solution.


##### Hard pole region - Unconstrained optimization using a loss function

If the optimizer doesn't support inequality constraints (or for the soft pole region <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/1936e598115038507ce0370e92814601.svg?invert_in_darkmode" align=middle width=31.24674299999999pt height=22.465723500000017pt/>) the inequality constraints have to be transformed into an objective function using loss functions.

In most cases the resulting objective function has the form

<p align="center"><img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/ba2cab457c819429dc87191f93737284.svg?invert_in_darkmode" align=middle width=241.89984719999998pt height=48.717066749999994pt/></p>

i.e. for each combination of model, pole and area the value of <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/2f7e7a6f478ee8461ca31a848d05f5dd.svg?invert_in_darkmode" align=middle width=100.81992854999999pt height=24.65753399999998pt/> is assessed by some loss function <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/36b5afebdba34564d884d347484ac0c7.svg?invert_in_darkmode" align=middle width=7.710416999999989pt height=21.68300969999999pt/> and the sum is used as objective function.
The following table lists the most common choices for <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/36b5afebdba34564d884d347484ac0c7.svg?invert_in_darkmode" align=middle width=7.710416999999989pt height=21.68300969999999pt/>:

| loss function | <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/63de8bf13006e75fc90561e40af887e0.svg?invert_in_darkmode" align=middle width=72.99929339999998pt height=24.65753399999998pt/> |
| --- | --- |
| Quadratic loss function | <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/8fdf71042a11097ae98df42e2cdae3b9.svg?invert_in_darkmode" align=middle width=163.62863879999998pt height=26.76175259999998pt/>
| <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/469f525d671e1e96713a0a17a13f2468.svg?invert_in_darkmode" align=middle width=11.45742179999999pt height=22.831056599999986pt/> loss function | <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/4729d237a36fe7960fd0883a2d691f84.svg?invert_in_darkmode" align=middle width=144.29065859999997pt height=24.65753399999998pt/> |
| Exponential loss function | <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/2ad3f337209d9b064b8670da975ad833.svg?invert_in_darkmode" align=middle width=117.80662904999998pt height=24.65753399999998pt/>
| Logarithmic loss function | <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/550d940341e7aadd9a9f436e27409b64.svg?invert_in_darkmode" align=middle width=142.23584595pt height=24.65753399999998pt/>

* The downside of the <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/469f525d671e1e96713a0a17a13f2468.svg?invert_in_darkmode" align=middle width=11.45742179999999pt height=22.831056599999986pt/> loss function is that it is not differentiable on the border curves.
* The logarithmic loss function is an inner penalty function which is not defined for any pole not lying within the defined region. Therefore, it can only be used if the initial value for the optimization variables is feasible.
* The exponential loss function may lead to very high values if the poles are far out of the regions. This may results in problems if the initial value for the optimization variables are not chosen carefully. In this case, the quadratic loss function may be a better choice.

An alternative objective function is based on the Kreisselmeier-Steinhauser function,

<p align="center"><img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/56d1aed16f96fee239e759e203753632.svg?invert_in_darkmode" align=middle width=509.728032pt height=50.2012401pt/></p>

which is an (rough) approximation of <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/a715e8d5c642395eb64658e9490f8b33.svg?invert_in_darkmode" align=middle width=123.2861223pt height=24.65753399999998pt/>.

The resulting optimization problem is

<p align="center"><img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/10732488f4f40866f7cafc7193f15253.svg?invert_in_darkmode" align=middle width=248.922432pt height=29.771669399999997pt/></p>

#### Soft pole region

If a constrained optimizer is used, a second pole region can be defined.
This soft region is treated in the same way as unconstrained optimizers treat the hard pole region, i.e. 
* The soft pole region <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/1936e598115038507ce0370e92814601.svg?invert_in_darkmode" align=middle width=31.24674299999999pt height=22.465723500000017pt/>
* It makes only sense if the optimizer supports inequality constraints


<p align="center"><img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/64a0d9ecf09e518157ebe78f4c4ad7ed.svg?invert_in_darkmode" align=middle width=565.4974578pt height=56.5021809pt/></p>


#### Additional objective terms

Additional objective functions can be selected.


##### Controller norm
In order to get a small control action, the controller matrices can be minimized by the choice of `GammaJType.NORMGAIN` as objective type with the objective function

<p align="center"><img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/d3dae745cd59a32004ac4d8626881702.svg?invert_in_darkmode" align=middle width=509.23495095pt height=18.312383099999998pt/></p>

where the matrices <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/84c95f91a742c9ceb460a83f9b5090bf.svg?invert_in_darkmode" align=middle width=17.80826024999999pt height=22.465723500000017pt/> of appropriate dimension are chosen for weighting.


##### Condition of the eigenvector matrix
For greater robustness of the closed loop, the condition number of the eigenvector matrix can be minimized by the choice of `GammaJType.EIGENVALUECONDITION` with the objective function

<p align="center"><img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/12c1a5cdba8992099e82efaadae23dca.svg?invert_in_darkmode" align=middle width=110.38815479999998pt height=16.438356pt/></p>

##### Norm of the Lyapunov matrix
Another possibility for achieving greater robustness against time varying unstructured uncertainty in the system matrix of the closed loop, is the minimization of the norm of the Lyapunov matrix of the closed loop system, which can be achieved by the choice of `GammaJType.LYAPUNOV`.
The objective function in this case has the form

<p align="center"><img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/b632ad29ffa7e203661a19adc1d3168b.svg?invert_in_darkmode" align=middle width=202.83691394999997pt height=39.8706165pt/></p>

where the matrices <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/1afcdb0f704394b16fe85fb40c45ca7a.svg?invert_in_darkmode" align=middle width=12.99542474999999pt height=22.465723500000017pt/> in the Lyapunov equation can be chosen independently for every multiple model.
The matrices <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/dc865b14fe6def23743a60b71824ed4a.svg?invert_in_darkmode" align=middle width=23.65874114999999pt height=30.267491100000004pt/> and <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/77d779e1d0f8d94889a60f70335de686.svg?invert_in_darkmode" align=middle width=23.65874114999999pt height=30.267491100000004pt/> which correspond to the unstable and stable part of the system respectively stem from a Schur decomposition of the closed loop system matrix where the unstable system matrix is replaced by <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/604210d982922a6467b88270ff97a28d.svg?invert_in_darkmode" align=middle width=25.114232549999993pt height=22.465723500000017pt/> in the continuous time case and <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/471d65ea6d03a4f1ea1dd8be931d26c9.svg?invert_in_darkmode" align=middle width=29.155366349999987pt height=26.76175259999998pt/> in the discrete time case.


#### Complete optimization problem

##### Constrained optimizers

For constrained optimizers the "full" optimization problem is

<p align="center"><img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/202479fd72442495770ab76aad1026fe.svg?invert_in_darkmode" align=middle width=623.0152697999999pt height=56.5021809pt/></p>

For unconstrained optimizers the "full" optimization problem is

<p align="center"><img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/8777bdceb61c64dbe65e75ea87c9b087.svg?invert_in_darkmode" align=middle width=439.06821749999995pt height=29.771669399999997pt/></p>

In this case only "simple" linear equality conditions can be imposed for the entries of <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/1e438235ef9ec72fc51ac5025516017c.svg?invert_in_darkmode" align=middle width=12.60847334999999pt height=22.465723500000017pt/>, <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/d6328eaebbcd5c358f426dbea4bdbf70.svg?invert_in_darkmode" align=middle width=15.13700594999999pt height=22.465723500000017pt/> and <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/f7fe419d8681ca1be2f53fbd90acf017.svg?invert_in_darkmode" align=middle width=16.64388494999999pt height=24.7161288pt/> which can be incorporated directly by reducing the number of optimization variables.


## Usage


```matlab
[R_opt, J_opt, info] = control.design.gamma.gammasyn(...
	sys,...
	polearea, weight,...
	Rfixed, R0, optimizer, gammaopts,...
	[Rbounds [, Rnonlin]]...
);
```

### Return values

* `R_opt`: Found solution, the format depends on relevant 
  * If `sys` defines neither <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/f0e9592dd37df872e4eaae9c6e9b44e5.svg?invert_in_darkmode" align=middle width=16.71459899999999pt height=24.7161288pt/> nor <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/fc8611f3dc01d5ede1c5fd180b2e52f2.svg?invert_in_darkmode" align=middle width=27.72499289999999pt height=22.465723500000017pt/> and <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/0ef8acdb75fa4d1cadb0bd690498414a.svg?invert_in_darkmode" align=middle width=29.585739149999988pt height=22.465723500000017pt/>, then `R_opt`  is simply a numerical matrix corresponding to <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/9882033dc7a3868ecfaf27408a6ea3e3.svg?invert_in_darkmode" align=middle width=19.343664449999988pt height=22.63846199999998pt/>
  * If `sys` defines the matrix <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/f0e9592dd37df872e4eaae9c6e9b44e5.svg?invert_in_darkmode" align=middle width=16.71459899999999pt height=24.7161288pt/> but not <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/fc8611f3dc01d5ede1c5fd180b2e52f2.svg?invert_in_darkmode" align=middle width=27.72499289999999pt height=22.465723500000017pt/> and <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/0ef8acdb75fa4d1cadb0bd690498414a.svg?invert_in_darkmode" align=middle width=29.585739149999988pt height=22.465723500000017pt/>, then `R_opt` is a cell array with two numerical entries corresponding to the solution <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/9d2fcc540b6ff3a29b99b18e5fb670eb.svg?invert_in_darkmode" align=middle width=68.43040709999998pt height=24.65753399999998pt/>
  * If `sys` defines <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/fc8611f3dc01d5ede1c5fd180b2e52f2.svg?invert_in_darkmode" align=middle width=27.72499289999999pt height=22.465723500000017pt/> and <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/0ef8acdb75fa4d1cadb0bd690498414a.svg?invert_in_darkmode" align=middle width=29.585739149999988pt height=22.465723500000017pt/>, but not <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/f0e9592dd37df872e4eaae9c6e9b44e5.svg?invert_in_darkmode" align=middle width=16.71459899999999pt height=24.7161288pt/>, then `R_opt` is a cell array with two numerical entries corresponding to the solution <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/f585e0be9cadad385dec311b294743d2.svg?invert_in_darkmode" align=middle width=66.14733014999999pt height=24.65753399999998pt/>
  * If `sys` defines all matrixes, then `R_opt` is a cell array with three numerical entries corresponding to the solution <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/cfe96b7369b7d046a2114a06db8e1932.svg?invert_in_darkmode" align=middle width=101.62674555pt height=24.65753399999998pt/>
* `J_opt`: value of the objective function at `R_opt`
* `info`: structure with additional information about the result


PLEASE NOTE: The current version of the toolbox "ignores" the prefilter <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/b8bc815b5e9d5177af01fd4d3d3c2f10.svg?invert_in_darkmode" align=middle width=12.85392569999999pt height=22.465723500000017pt/>.
Current work aims to extend the toolbox for the design of coupling and decoupling controllers.
In theses cases the manipulation of <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/b8bc815b5e9d5177af01fd4d3d3c2f10.svg?invert_in_darkmode" align=middle width=12.85392569999999pt height=22.465723500000017pt/> is necessary.
Therefore, it is included in the API.
But in the current release version, <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/b8bc815b5e9d5177af01fd4d3d3c2f10.svg?invert_in_darkmode" align=middle width=12.85392569999999pt height=22.465723500000017pt/> will always be returned as the initial value or a zero matrix.


### System `sys`

The argument `sys` describes one or more systems.
A system is described by a `struct` whose fields correspond to the matrices of a state space realization.
If more than one system is given, `sys` is a vector of structs.

These systems are always the augmented systems which may include (parts of) the controller.

In the simplest form a system is given by the three matrices <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/53d147e7f3fe6e47ee05b88b166bd3f6.svg?invert_in_darkmode" align=middle width=12.32879834999999pt height=22.465723500000017pt/>, <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/61e84f854bc6258d4108d08d4c4a0852.svg?invert_in_darkmode" align=middle width=13.29340979999999pt height=22.465723500000017pt/> and <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/9b325b9e31e85137d1de765f43c0f8bc.svg?invert_in_darkmode" align=middle width=12.92464304999999pt height=22.465723500000017pt/> of the state space representation

<p align="center"><img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/79b8a0e65d55f29f0ebc79692907e8c3.svg?invert_in_darkmode" align=middle width=95.8312509pt height=39.086746049999995pt/></p>

Optionally, this toolbox allows to specify a mass matrix and to design ideal differential feedback as well as it can design a prefilter.
The "full" system form is given by

<p align="center"><img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/61320ea8d976382c9602dff3b7714f21.svg?invert_in_darkmode" align=middle width=144.62609039999998pt height=88.4018157pt/></p>

where <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/84df98c65d88c6adf15d4645ffa25e47.svg?invert_in_darkmode" align=middle width=13.08219659999999pt height=22.465723500000017pt/> must be an invertible matrix.

| Fields | Remark |
| --- | --- |
| A, B, C | Minimum form |
| C_dot | Optional |
| E  | Optional |
| C_ref, D_ref | Optional | 


PLEASE NOTE: A field D may be given (and is also returned by functions of this toolbox), but it must be a zero matrix of compatible size! (Future versions may allow systems with feed through but in the current version a non-zero D results in undefined behavior.)

If the additional flexibility of differential feedback or reference outputs is not needed, it is also possible to supply system descriptions that are included in Matlab's Control System toolbox as `ss`, `tf`, `uss` or `dss`.


### Controller structure `Rfixed`

The controller structure is given by the mandatory argument `Rfixed` as well as the optional arguments `Rbounds` and `Rnonlin`.

In the following, the structure of one of the matrices <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/1e438235ef9ec72fc51ac5025516017c.svg?invert_in_darkmode" align=middle width=12.60847334999999pt height=22.465723500000017pt/>, <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/d6328eaebbcd5c358f426dbea4bdbf70.svg?invert_in_darkmode" align=middle width=15.13700594999999pt height=22.465723500000017pt/> and <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/b8bc815b5e9d5177af01fd4d3d3c2f10.svg?invert_in_darkmode" align=middle width=12.85392569999999pt height=22.465723500000017pt/> are explained.
(It is the same for each one.)
They are combined by forming a cell array, i.e. if all three matrices are used:
```matlab
Rfixed = {Ra_fixed, Ka_fixed, Fa_fixed, RKFa_fixed}
```
If no dependencies between different gain matrices are needed, this can be reduced to
```matlab
Rfixed = {Ra_fixed, Ka_fixed, Fa_fixed}
```
If only <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/1e438235ef9ec72fc51ac5025516017c.svg?invert_in_darkmode" align=middle width=12.60847334999999pt height=22.465723500000017pt/> and <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/d6328eaebbcd5c358f426dbea4bdbf70.svg?invert_in_darkmode" align=middle width=15.13700594999999pt height=22.465723500000017pt/> is used, 
```matlab
Rfixed = {Ra_fixed, Ka_fixed}
```
and if only <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/1e438235ef9ec72fc51ac5025516017c.svg?invert_in_darkmode" align=middle width=12.60847334999999pt height=22.465723500000017pt/> is used, 
```matlab
Rfixed = {Ra_fixed}
```

* Fixed values

  `Ra_fixed = {Rfix, Rval}`

  `Rfix` is a logical matrix with `true`-entries marking the fixed entries.
  `Rval` is a numerical matrix where the fixed values are given.
  The non-fixed values are marked as `NaN` in this matrix.
  
  (This is redundant, as `Rfix = ~isnan(Rval)` but is needed to distinguish the format.)

  For example, if <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/710691f91bcc303044ab837f0292077e.svg?invert_in_darkmode" align=middle width=111.16254269999997pt height=47.6716218pt/> with the parameters <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/11b758cdd4af7852d80a4a6bfd04319c.svg?invert_in_darkmode" align=middle width=16.25198519999999pt height=14.15524440000002pt/> and <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/7ea284f8038dab36b863285a40d1c92f.svg?invert_in_darkmode" align=middle width=12.165227249999989pt height=14.15524440000002pt/> being free, the definition of the structure would be
  ```matlab
    {[false, false; true, true], [NaN, NaN; 1, 0]}
  ```

* Linear dependencies between controller parameters

  `Ra_fixed = {Zlhs, Zrhs}`

  Linear dependencies are given by linear equations of the form

  <p align="center"><img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/f59970cca51c074aaaff912788aedd8c.svg?invert_in_darkmode" align=middle width=125.36717324999998pt height=38.89287435pt/></p>

  where <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/9b808701e2b68072679bcc95e3891b8e.svg?invert_in_darkmode" align=middle width=12.785434199999989pt height=19.1781018pt/> means element-wise multiplication (Hadamard product).
  If there is more than one equation <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/63bb9849783d01d91403bc9a5fea12a2.svg?invert_in_darkmode" align=middle width=9.075367949999992pt height=22.831056599999986pt/>, the matrices <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/3173677423a86e28caa6d954dbc490ff.svg?invert_in_darkmode" align=middle width=18.487510799999992pt height=22.465723500000017pt/> are stacked along the third dimension in `Zlhs`.
  I.e, if `Nz` linear dependencies are specified, the dimensions of `Zlhs` and `zrhs` are `size(Zlhs): [size(R, 1), size(R, 2), Nz]` (for the combined constraints `size(Zlhs): [size(R, 1), size(R, 2) + size(K, 2) + size(F, 2), Nz]`) and `size(Zrhs): [Nz, 1]`, resp.

  For example, if <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/fe99331370ca6afffac4ff559965a264.svg?invert_in_darkmode" align=middle width=129.12676919999998pt height=47.6716218pt/> with <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/3cf87ea38a615ed99e0232f8ed9431fe.svg?invert_in_darkmode" align=middle width=12.067218899999991pt height=14.15524440000002pt/> being free but subject to the constraints <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/28c88fd24ac31015c0584a4e470e21ed.svg?invert_in_darkmode" align=middle width=87.38568794999999pt height=14.15524440000002pt/> and <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/41c2a69ed2fcbc9e857470e0439203e2.svg?invert_in_darkmode" align=middle width=50.67727829999998pt height=14.15524440000002pt/>, the definition of the structure would be
  ```matlab
    {cat(3,...
         [1, -1, 0; 0, 0, 0],... % lhs of r_1 - r_2 = 0
         [1, 0, -1; 0, 0, 0],... % lhs of r_1 - r_3 = 0
         [0, 0, 0; 1, -1, 0],... % lhs of r_4 - r_5 = 0
         [0, 0, 0; 0, 0, 1]...   % lhs of r_6 = 1
        ),...
     [
      0;    % rhs of r_1 - r_2 = 0
      0;    % rhs of r_1 - r_3 = 0
      0;    % rhs of r_4 - r_5 = 0
      1     % rhs of r_6 = 1
     ]
    }
  ```
  * As said above, linear dependencies for `K` and `F` can be specified in the same way in `Ka_fixed` and `Fa_fixed`. For dependecies involving all matrices `RKFa_fixed` can be used, specifying `{Zlhs, Zrhs}` corresponding to

  <p align="center"><img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/3ac3562814a0de990a99cf23cdfb6d53.svg?invert_in_darkmode" align=middle width=199.9334601pt height=38.89287435pt/></p>

* Symbolic dependencies between controller parameters
If the Symbolic Math Toolbox is available, it is also possible, to formulate the controller coefficient constraints as symbolic expressions.
This can be achieved by specifying the symbolic gain matrix in the first element of a cell array and the symbolic equation system in the second element of the cell array for every gain matrix like in
```matlab
	R = sym('R', [2, 2]);
	K_I = sym('K_I');
	K_P = sym('K_P');
	R(1, 1) = K_P;
	R(1, 2) = K_I;
	Ka_fixed = {R, [
		K_I == K_P;
		R(2, 1) == 1;
		R(2, 2) == 0
	]};
```
for the proportional part of a PID controller where integral and proportional action should have the same coefficient value.


As it may be cumbersome to construct the augmented system as well as to specify the structure of the controller matrices manually for the desired controller structure, this toolbox provides helper functions for common controller structures.

Alternatively the capabilities of the parameterized and uncertain system objects of the Control System Toolbox and the Robust Control Toolbox can be used.


#### Predefined controller setups

| Class | Parameters | Description |
| --- | --- | --- |
| ConstantOutputFeedback | |
| DynamicOutputFeedback | nD | nD: order of the dynamic controller |
| ObserverDirectOutputFeedback | |
| ObserverOutputFeedback | |
| ObserverStateFeedback | |
| PDOutputFeedback | |
| PIDOutputFeedback | |
| StateFeedback | |


```matlab
sys = omo_sys(1, 10, 1000);

controller = control.design.outputfeedback.PIDOutputFeedback();

% construct augmented system
sys_augmented = controller.amend(sys);

% get structure description
[Ra_fixed, Ka_fixed] = controller.gainpattern(sys);
Rfixed = {Ra_fixed, Ka_fixed};
```
This works as well if `sys` contains several systems:
```matlab
m = [0.9, 1.1];
d = [8, 12];
c = 1000;

sys = [
	omo_sys(m(1), d(1), c),...
	omo_sys(m(2), d(1), c),...
	omo_sys(m(1), d(2), c),...
	omo_sys(m(2), d(2), c),...
	omo_sys(mean(m), mean(d), c)
];

controller = control.design.outputfeedback.PIDOutputFeedback();

sys_augmented = controller.amend(sys);

[Ra_fixed, Ka_fixed] = controller.gainpattern(sys);
Rfixed = {Ra_fixed, Ka_fixed};
```


#### Using Matlab system definitions
Instead of supplying the system descripton as a structure with the neccessary fields, it is also possible to use a `tf` or `ss` object (or array in case of multiple models) as system.
```matlab
sys = omo_sys(1, 10, 1000);
sys = ss(sys.A, sys.B, sys.C);

controller = control.design.outputfeedback.PIDOutputFeedback();

% construct augmented system
sys_augmented = controller.amend(sys);

% get structure description
[Ra_fixed, Ka_fixed] = controller.gainpattern(sys);
Rfixed = {Ra_fixed, Ka_fixed};
```



### Initial value `R0`

An initial value for the feedback matrix <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/1e438235ef9ec72fc51ac5025516017c.svg?invert_in_darkmode" align=middle width=12.60847334999999pt height=22.465723500000017pt/> must be given.

#### Numerical value
It must have the same dimension as <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/1e438235ef9ec72fc51ac5025516017c.svg?invert_in_darkmode" align=middle width=12.60847334999999pt height=22.465723500000017pt/>, i.e. `size(R0) : [size(sys.B, 1), size(sys.C, 2)]`.
If <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/1e438235ef9ec72fc51ac5025516017c.svg?invert_in_darkmode" align=middle width=12.60847334999999pt height=22.465723500000017pt/> contains fixed values, the corresponding entries of `R0` are ignored.

An inital value for the differential feedback matrix <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/d6328eaebbcd5c358f426dbea4bdbf70.svg?invert_in_darkmode" align=middle width=15.13700594999999pt height=22.465723500000017pt/> may be given.
In this case, `R0` is a cell array containing two matrices,
```matlab
R0 = {Ra0, Ka0}
```
If the structure employs differential feedback but no initial value for <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/d6328eaebbcd5c358f426dbea4bdbf70.svg?invert_in_darkmode" align=middle width=15.13700594999999pt height=22.465723500000017pt/> is given, it is set to <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/29632a9bf827ce0200454dd32fc3be82.svg?invert_in_darkmode" align=middle width=8.219209349999991pt height=21.18721440000001pt/>.
Supplying an initial value for the prefilter gain <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/b8bc815b5e9d5177af01fd4d3d3c2f10.svg?invert_in_darkmode" align=middle width=12.85392569999999pt height=22.465723500000017pt/> is possbile with three matrices
```matlab
R0 = {Ra0, Ka0, Fa0}
```


#### Multiple numerical values

It is possible to provide more than one initial value.
In this case the optimization problem is solved for each (set of) initial value(s) and the best solution is returned.

To provide multiple initial values, instead a two-dimensional matrix `R0`, a cell array of two two-dimensional matrices `{Ra0, Ka0}` or a cell array of three two-dimensional matrices `{Ra0, Ka0, Fa0}` three-dimensional values have to be provided.


#### InitialValue
If the user supplied initial value for optimization is to be changed by a further processing step to make it more suitable for the pole region assignment problem by taking the multiple models into account, this is possible by the use of the `InitialValue` class.
This class is a container for different `InitialValueElement` objects that may implement this processing, e.g. using the `oplace` toolbox for design of structurally constrained output feedback controllers with pole assignment developed at the institute of automatic control at the TU Darmstadt.
Currently this task is left to the user and there are only two simple `InitialValueElement` classes defined, `GammasynInitialValueElement` which is basically only a wrapper for numeric initial values described above and `RandomInitialValue` which returns random initial values of appropriate size.
An initial value consisting of a random value and a numeric value can be constructed by
```matlab
R0 = control.design.gamma.InitialValue(...
	control.design.gamma.RandomInitialValue(2),...
	control.design.gamma.GammasynInitialValueElement(Ra0)...
)
```
which creates two random initial values and the numeric initial value `Ra0`.



### Region definition

A region is defined by one or the intersection of more areas.
Here, "area" means a function which maps any point of the complex plane, <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/f571789b2d5d5d518339628e412509bd.svg?invert_in_darkmode" align=middle width=45.91885649999999pt height=21.95701200000001pt/>, to a real number, <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/7d9cb65db969d7e8addac3679a5e6916.svg?invert_in_darkmode" align=middle width=60.74912744999998pt height=24.65753399999998pt/>.
It is zero on the border of the area, has negative values on the side at which poles must or should lie and positive values on the other side.
It should be differentiable twice.

These are translated into constraints or into an objective function, depending on the type of region (hard or soft) and the capabilities of the optimizer.
If it is translated to an objective function, the area functions are applied to a loss function defined with `GammaJType`.

To allow for an efficient optimization, an area should not only provide its value when called for a certain point of the complex plane, but also returns its first (and second) derivatives.

For the common areas these functions are predefined, such that there should be seldom the need to define custom area functions.


Generally, a region (hard or soft) is defined by a row vector of `control.design.gamma.area`-Objects, for example
```matlab
region = [
	control.design.gamma.area.Line(1, 0),...
	control.design.gamma.area.Imag(1, 1)
]
```
If a robust design is performed, i.e. there is more than one model defined, *and* different regions should be given for different models, then the regions are defined by a matrix, where every row describes the region for the corresponding model in the `sys`-vector.
(If the same region is used for all models, only one has to be defined.)
For three systems the regions may be defined for example by
```matlab
region = [
	control.design.gamma.area.Line(1, 0), control.design.gamma.area.Imag(1, 1);
	control.design.gamma.area.Line(0.7, 0), control.design.gamma.area.Imag(1, 1);
	control.design.gamma.area.Line(0.5, 0), control.design.gamma.area.None()
]
```
If the number of areas differs for the different regions, the `None`-area can be used to fill the rows of matrices.

If both a hard and a soft region are refined, `polearea` is a cell array with two elements, each containing a vector of areas:
```matlab
polearea = {hardregion, softregion}
```
If only a hard region is defined, the soft region can be set to an empty matrix
```matlab
polearea = {hardregion, []}
```
or equivalently, `polearea` can be set directly as the hard region vector or matrix
```matlab
polearea = hardregion
```

NOTE: If an unconstrained optimizer is used, only the hard region is used.
I.e. if a soft region is defined, it is simply ignored.


If a constrained optimizer is used, and the user wants to specify a soft region only, that could be achieved with
```matlab
polearea = {control.design.gamma.area.None(), softregion}
```
explicitly or by specifying `[]` implicitly.
However, it is advisable to specify at least the "minimal" hard region, i.e. the left half plane for continuous time systems or the unit circle for discrete time systems.






#### Predefined areas

| Area | Parameters | Description | |
| ----------- | --- | --- | --- |
| Circle <br/> Circlesquare | R | R: Radius | <img src="docs/images/tex/tikz_ext/area-circle.png" width=50%/>
| Ellipse <br/> Ellipsesquare | a, b | a: semi axis along real axis <br/> b: semi axis along imaginary axis |  <img src="docs/images/tex/tikz_ext/area-ellipse.png" width=50%/>
| Hyperbola <br/> Hyperbolasquare | a, b | a: semi axis along real axis <br/> b: semi axis along imaginary axis | <img src="docs/images/tex/tikz_ext/area-hyperbola.png" width=50%/>
| Imag | a, b | Vertical line <br/> For left part: a: 1, b: neg. position of line on real axis <br/> For right part: a: -1, b: position of line on real axis | <img src="docs/images/tex/tikz_ext/area-imag-left.png" width=50%/> <img src="docs/images/tex/tikz_ext/area-imag-right.png" width=50%/>
| Line | a, b | a: slope <br/> b: intercept <br/> The "left" area is always the part above the line! | <img src="docs/images/tex/tikz_ext/area-line.png" width=50%/>
| LogSpiral | r, k | r: start point on real axis <br/> k: polar slope | <img src="docs/images/tex/tikz_ext/area-logspiral.png" width=50%/>
| None |
| PolyEllipse <br/> PolyEllipsesquare |

All areas (except None, where it is ignored) accept an additional optional parameter *shift*.
This parameter is a complex number which allows to shift the origin of the "base" coordinate system defining the area.
I.e. for circles, *shift* defines the center point.
If *shift* is not specified, it is set to zero.


##### "square" versions


Some of the areas are more naturally defined by a squared expression.
Taking the square root results in a function that may not be differentiable, if the argument of the root can attain zero.
Therefore, it may be a good choice to work with the square form directly.

However, depending on the usage this may lead to other problems if squared and non-squared forms are mixed, as in the first case the "distance" is measured squared and in the latter it is measured "normally".
Therefore, the non-squared versions are provided as well.

And an additional definition has to be added, if the shape doesn't distinguish between the left and right half plane.


#### Custom areas

```matlab
z = areafun(re, im)
```

```matlab
[z, dzdre, dzdim] = areafun(re, im)
```

```matlab
[f, dfdre, dfdim, d2fdredre, d2fdimdre, d2fdredim, d2fdimdim] = areafun(re, im)
```


##### Example

In this simple example, the _exterior_ of a circle with radius <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/1e438235ef9ec72fc51ac5025516017c.svg?invert_in_darkmode" align=middle width=12.60847334999999pt height=22.465723500000017pt/> is chosen as desired area.
In square form, the area is defined by

<p align="center"><img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/086fcab481666dc45a17cc33b5c5c369.svg?invert_in_darkmode" align=middle width=184.25200695pt height=42.1113528pt/></p>

which gives the derivatives

<p align="center"><img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/63cc4d758e244ec8b8566571f9849984.svg?invert_in_darkmode" align=middle width=184.51355009999997pt height=33.81208709999999pt/></p>

Accordingly, the function could be implemented as
```matlab
function [z, dzdre, dzdim] = mindistance_area(re, im, R)
	z = R^2 - re^2 - im^2;
	dzdre = -2 * re;
	dzdim = -2 * im;
end
```
and used as
```matlab
polearea = @(re, im) mindistance_area(re, im, 5)
```
Multiple regions can be defined by simply letting the function return row vectors of function and gradient values instead of scalars.




### `weight`: Weighting of areas

As described above, each area and model is associated with a weight <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/28457d95bce65adb96ba5eb9eebd9dec.svg?invert_in_darkmode" align=middle width=26.58161714999999pt height=14.15524440000002pt/> which appears within the inequality constraints or the derived loss functions.

`weight` is structured like `polearea`.
If both a hard and a soft region is defined, `weight` is a cell array with two numeric elements (scalar, vector or matrix)
```matlab
weight = {hardweight, softweight}
```
If only a hard region is given, `weight` might be set directly to corresponding numeric element 
```matlab
weight = hardweight
```

`hardweight` and `softweight` can be defined in one of the following forms:

 * Scalar value: All weights (for all areas and all models) are set to this value. This is the standard choice for the hard region, where all weights are normally set to one.
 * Row vector: Each entry is the weight of the corresponding area. If more than one model is used, the weights may differ for the different areas, but they are the same for all models.
 * Column vector: Each entry is the weight for all areas for the corresponding model.
 * Matrix: Each entry is the weight for the corresponding model (row) and area (column).
 

**Remarks:**

* If the hard region is translated to inequality constraints, the weights for the hard region are more academic (a weight might be set to a negative value to inverse the area). With some experience the weights may be used to "help" the optimizer to find a feasible solution.
* If negative weights are used, the option `allownegativeweight` must be set to true.



### `gammaopts`: Objective function and associated options

The options, which are described below, can be set using the function
```matlab
gammaopts = control.design.gamma.GammasynOptions(parameter, value, ...)
```
The parameter name may contain dots, for example `'objective.normgain.K`.


| Option | Remark |
| --- | --- |
| `type` | objective functions, element of `GammaJType` |
| `objective.normgain.{R, K, F}`| Weights for minimization of controller norm | 
| `objective.normgain.{R_shift, K_shift, F_shift}`| Shift for minimization of controller norm | 
| `objective.kreisselmeier.{rho, max}`| Parameters for Kreisselmeier objective function | 
| `objective.lyapunov.Q`| right hand side matrix for Lyapunov equation | 
| `weight` | weight for the used objective functions
| `allowvarorder` | true, {false}: indicator if systems with different numbers of states are allowed |
| `allownegativeweight` | true, {false}: indicator if negative area weights are allowed |
| `usecompiled` | true, {false}: indicator if generated code should be used |
| `numthreads` | number of parallel threads to use
| `eigenvaluederivative` | method for calculation of eigenvalue derivatives to use
| `eigenvaluefilter` | filter for eigenvalues to use
| `strategy` | strategy for solution to use
| `errorhandler` | type of error handler to use
| `errorhandler_function` | error handler function to use in case of `GammaErrorHandler.USER`
| `system.usereferences` | indicator if output matrix `C` should be used as <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/fc8611f3dc01d5ede1c5fd180b2e52f2.svg?invert_in_darkmode" align=middle width=27.72499289999999pt height=22.465723500000017pt/> for matlab system descriptions
| `system.usemeasurements_xdot` | indicator if output matrix `C_dot` should be used as <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/f0e9592dd37df872e4eaae9c6e9b44e5.svg?invert_in_darkmode" align=middle width=16.71459899999999pt height=24.7161288pt/> for matlab system descriptions
| `system.samples` | structure with fields equal to the names of uncertain blocks in the `uss` system description to indicate the number of multiple models to create from the corresponding uncertain parameter
| `system.Blocks` | structure with fields equal to the names of uncertain blocks in the `uss` system description to indicate the number of multiple models to create from the corresponding uncertain parameter


#### Objective functions `type`

With this option the type of loss function used for the soft region (constrained optimizers) or the hard region (unconstrained optimizers) is selected.
Also, additional objective function terms can be selected.
`type` is a scalar or a vector of elements of the enumeration `GammaJType`.
The elements are listed in the following table and some examples are given below.

| GammaJType | Remark | Loss function
| --- | --- | --- |
| ZERO | no objective function (pure feasibility problem) | <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/570884d514512e4e25a77aaf2bae5688.svg?invert_in_darkmode" align=middle width=40.83319019999999pt height=22.465723500000017pt/>
| MAX | <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/469f525d671e1e96713a0a17a13f2468.svg?invert_in_darkmode" align=middle width=11.45742179999999pt height=22.831056599999986pt/> loss function | <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/758292e218d7e32a67393291d1feab28.svg?invert_in_darkmode" align=middle width=173.91870705pt height=24.65753399999998pt/>
| SQUAREPENALTY | Quadratic loss function | <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/b3bab96dfe2f245a69181fb7e194407f.svg?invert_in_darkmode" align=middle width=193.25668725pt height=26.76175259999998pt/>
| EXP | Exponential loss function | <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/475a959b7f59ffef29889f79ed5e3360.svg?invert_in_darkmode" align=middle width=147.4346775pt height=24.65753399999998pt/>
| LINEAR | (*) linear weighting of pole areas | <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/b8489ef8b7dc196e8a1473c5df8af8d7.svg?invert_in_darkmode" align=middle width=109.53502394999998pt height=24.65753399999998pt/>
| SQUARE | (*) *signed* quadratic weighting of pole areas | <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/a48511cf7a575025c4146ef0dab3aceb.svg?invert_in_darkmode" align=middle width=261.8392854pt height=26.76175259999998pt/> |
| CUBIC | (*) cubic weighting of pole areas | <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/b52ea3cee45e3c3b494047ef6faab37e.svg?invert_in_darkmode" align=middle width=128.87300414999999pt height=26.76175259999998pt/> |
| LOG | (*) Logarithmic loss function | <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/ef63c03d9de6976c06ff53ff6616b74f.svg?invert_in_darkmode" align=middle width=171.86389274999996pt height=24.65753399999998pt/>
| KREISSELMEIER | vector performance index weighting according to Kreisselmeier | <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/62d6f6e3b9e3509a437667e83e4ca157.svg?invert_in_darkmode" align=middle width=460.54834485pt height=37.80850590000001pt/> 
| EIGENVALUECONDITION | (**) eigenvector matrix condition objective function |
| NORMGAIN | (**) norm of gain matrices | <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/c51c6e0c57b00ee6828bfddbddc8132d.svg?invert_in_darkmode" align=middle width=327.19724399999996pt height=26.76175259999998pt/>
| LYAPUNOV | (**) norm of Lyapunov matrix of closed loop |

(*) These loss functions are unbounded below.
This may lead to unexpected results if the closed loop possesses conjugate complex poles.
They are provided mainly for experimental and academic reasons.

(**) These loss functions don't assess the poles at all but are motivated by the aim to get a robust or economic controller.


For example, if `type` is set to
```matlab
    GammaJType.SQUAREPENALTY
```
the quadratic loss function is used and none of the additional objective functions <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/0fbada9664742235c13fd5f06143b572.svg?invert_in_darkmode" align=middle width=32.49442349999999pt height=22.465723500000017pt/> and <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/4d699c8c7d348b88a53015712da1e79b.svg?invert_in_darkmode" align=middle width=27.831157199999993pt height=22.465723500000017pt/> is added.

If `type` is set to 
```matlab
    [GammaJType.SQUAREPENALTY; GammaJType.NORMGAIN]
```
the quadratic loss function is used and the controller norm is added.
To weight these two terms, the option `weight`can be set appropriately.

If `type` is set to
```matlab
    [GammaJType.ZERO; GammaJType.NORMGAIN]
```
only the norm of the controller is used in the objective function.
This can be a sensible choice when constrained optimizers are used, as the hard region is still respected.
When an unconstrained optimizer is used, this is not a sensible choice, as the pole region is ignored completely and therefore the optimum is that all controller matrices are zero.


The function accepts multiple types of loss functions, but there should be no use for.
However, it is important to notice that
```matlab
    [GammaJType.SQUAREPENALTY; GammaJType.SQUAREPENALTY]
```
does *not* mean that the hard and the soft region are both translated to an objective function term using the quadratic loss function but the following!

* In the case of a constrained optimizer, the soft region appears twice in the objective function.
* In the case of an unconstrained optimizer, the hard region appears twice in the objective function. The soft region is still ignored.



##### NORMGAIN

If `NORMGAIN` is used as objective function, the weighting matrices <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/4a2945d96c9cdea43eb20c863ebd23dd.svg?invert_in_darkmode" align=middle width=25.06856384999999pt height=22.465723500000017pt/>, <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/d8191adeee56915fd4b7583ffe1d6b9a.svg?invert_in_darkmode" align=middle width=25.58229134999999pt height=22.465723500000017pt/> and <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/90cf59c9816b6b1d764ff69d1b443366.svg?invert_in_darkmode" align=middle width=24.006927449999992pt height=22.465723500000017pt/> have to be specified using the following parameters of `objoptions`:

| Parameter | Description |
| --- | --- |
| `objective.normgain.R` | <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/4a2945d96c9cdea43eb20c863ebd23dd.svg?invert_in_darkmode" align=middle width=25.06856384999999pt height=22.465723500000017pt/> |
| `objective.normgain.K` | <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/d8191adeee56915fd4b7583ffe1d6b9a.svg?invert_in_darkmode" align=middle width=25.58229134999999pt height=22.465723500000017pt/> (only necessary if <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/d6328eaebbcd5c358f426dbea4bdbf70.svg?invert_in_darkmode" align=middle width=15.13700594999999pt height=22.465723500000017pt/> is used in the structure) |
| `objective.normgain.F` | <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/90cf59c9816b6b1d764ff69d1b443366.svg?invert_in_darkmode" align=middle width=24.006927449999992pt height=22.465723500000017pt/> (only necessary if <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/b8bc815b5e9d5177af01fd4d3d3c2f10.svg?invert_in_darkmode" align=middle width=12.85392569999999pt height=22.465723500000017pt/> is used in the structure) |
| `objective.normgain.R_shift` | <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/5f0dccb5552bd232b7985454add6e7b4.svg?invert_in_darkmode" align=middle width=19.62335594999999pt height=22.465723500000017pt/> |
| `objective.normgain.K_shift` | <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/c373456ab0d7851561301c6b2678a4ab.svg?invert_in_darkmode" align=middle width=20.13708179999999pt height=22.465723500000017pt/> |
| `objective.normgain.F_shift` | <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/c4873c061eefcc8ebd4339ace473b4a1.svg?invert_in_darkmode" align=middle width=18.56171789999999pt height=22.465723500000017pt/> |

The weighting matrices have to be of the same dimension as the corresponding controller matrix.
It is not sufficient to use a scalar value, even if the weight should be the same for all entries.
The shifting matrices <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/e257acd1ccbe7fcb654708f1a866bfe9.svg?invert_in_darkmode" align=middle width=11.027402099999989pt height=22.465723500000017pt/> are optional.


##### KREISSELMEIER

| Parameter | Description |
| --- | --- |
| `objective.kreisselmeier.rho` |  <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/0a2b32d0756b6f073b21139d7bc38e1d.svg?invert_in_darkmode" align=middle width=30.382601699999988pt height=14.15524440000002pt/> |
| `objective.kreisselmeier.max` |  <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/ab2af6a289bafde852951069cbfc771c.svg?invert_in_darkmode" align=middle width=58.093962299999994pt height=22.831056599999986pt/> |

##### LYAPUNOV
If `LYAPUNOV` is used as objective function, the matrices <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/1afcdb0f704394b16fe85fb40c45ca7a.svg?invert_in_darkmode" align=middle width=12.99542474999999pt height=22.465723500000017pt/> have to be specified using the following parameters of `objoptions`:

| Parameter | Description |
| --- | --- |
| `objective.lyapunov.Q` |  <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/1afcdb0f704394b16fe85fb40c45ca7a.svg?invert_in_darkmode" align=middle width=12.99542474999999pt height=22.465723500000017pt/> |

If the same matrix is to be used for all multiple model, it is sufficient to supply a single matrix.
In case a specific matrix <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/1afcdb0f704394b16fe85fb40c45ca7a.svg?invert_in_darkmode" align=middle width=12.99542474999999pt height=22.465723500000017pt/> for every multiple model should be used, the matrices have to be concatenated in the third dimension.
When nothing is specified, the identity matrix is used.
If the discrete time Lyapunov equation is to be solved in case of discrete time systems, it is vital to add a field `T` with the sampling time to the system description in order to signal this.
When the option `allowvarorder` is set to `true` and therefore systems with different state dimension are allowed, the remaining elements of <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/1afcdb0f704394b16fe85fb40c45ca7a.svg?invert_in_darkmode" align=middle width=12.99542474999999pt height=22.465723500000017pt/> must be filled with `NaN` to match the dimension of the largest system in use.

#### Weighting of the objective function terms

If more than one objective function term is selected by `type`, their weighting can be specified by `weight` which is a numeric vector of the same dimension as `type` with the corresponding non-negative weights.

If for example the objective function

<p align="center"><img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/4cf5522341a4729a8051b4639859b423.svg?invert_in_darkmode" align=middle width=194.19157065pt height=18.905967299999997pt/></p>

is to be used, the following options are to be set:
```matlab
    'type'   : [GammaJType.SQUAREPENALTY; GammaJType.NORMGAIN]
    'weight' : [1; 1e-5]
```


#### Algorithm for eigenvalue derivation


| GammaEigenvalueDerivativeType | Remark |
| --- | --- |
| DEFAULT | calculation of eigenvalue derivatives by Rayleigh coefficient
| VANDERAA | calculation of eigenvalue and eigenvector derivatives with method of van der Aa
| RUDISILLCHU | calculation of eigenvalue and eigenvector derivatives with Rayleigh coefficient and pseudo inverse


#### Filtering

| GammaEigenvalueFilterType | Remark |
| --- | --- |
| NONE | no filtering of eigenvalues
| NEGATIVEIMAG | remove eigenvalues with negative imaginary part from calculation by taking negative imaginary part instead
| POSITIVEIMAG | remove eigenvalues with positive imaginary part from calculation by taking negative imaginary part instead



#### Strategy


| GammaSolutionStrategy | Remark |
| --- | --- |
| SINGLESHOT | solve pole region assignment problem "as is"
| FEASIBILITYITERATION | solve a feasibility problem before solving the actual problem and use the solution of the feasibility problem as initial value


#### Error handling

| GammaErrorHandler | Remark |
| --- | --- |
| WARNING | convert errors to warnings
| ERROR | leave errors as is
| USER | use user defined error handler

If `USER` is chosen, a handler to an error handler function has to be provided in the option `errorhandler_function`.



### Optimizer

The optimizers marked with (*) are included in this repository.


| Optimizer | Constr.  | Obj. | Origin           | Licence | Remark  |
| ----------- | --- | --- |-------------| ----- | ----| 
| FMINCON        | * |   | Matlab, Optimization Toolbox                        | prop.      |  |
| FMINCONGLOBAL  | * |   | Matlab, Global Optimization Toolbox                 | prop.      |  |
| IPOPT          | * |   | https://github.com/coin-or/Ipopt                    | Eclipse Public License 1.0 |  |
| FMINIMAX       | * | * | Matlab, Optimization Toolbox                        | prop.      |  |
| FMINUNC        |   |   | Matlab, Optimization Toolbox                        | prop.      |  |
| FMINUNCGLOBAL  |   |   | Matlab, Global Optimization Toolbox                 | prop.      |  |
| FMINSEARCH     |   |   | Matlab                                              | prop.      |  |
| GA             | * | * | Matlab, Global Optimization Toolbox                 | prop.      |  |
| KSOPT          | * | * | https://github.com/madebr/pyOpt                     | LGPL       |  |
| NLOPTUNC       |   |   | https://nlopt.readthedocs.io/en/latest/             | MIT Lizenz |  |
| NLOPTCON       | * |   | https://nlopt.readthedocs.io/en/latest/             | MIT Lizenz |  |
| NLOPTUNCGLOBAL |   |   | https://nlopt.readthedocs.io/en/latest/             | MIT Lizenz |  |
| NLOPTCONGLOBAL | * |   | https://nlopt.readthedocs.io/en/latest/             | MIT Lizenz |  |
| PARTICLESWARM  |   |   | Matlab, Global Optimization Toolbox                 | prop.      |  |
| PATTERNSEARCH  | * |   | Matlab, Global Optimization Toolbox                 | prop.      |  |
| SIMULANNEAL    |   |   | Matlab, Global Optimization Toolbox                 | prop.      |  |
| PPPBOX         |   | * | ?                                                   |            |  |
| SCBFGS         |   |   | https://coral.ise.lehigh.edu/frankecurtis/software/ | ?          |  |
| SLPGS          |   |   | https://coral.ise.lehigh.edu/frankecurtis/software/ | ?          |  |
| SNOPT          | * |   | http://ccom.ucsd.edu/~optimizers/solvers/snopt/     | prop.      |  |
| SQPGS          |   |   | https://coral.ise.lehigh.edu/frankecurtis/software/ | ?          |  |

Optimizers which are marked with an asterisk in the column "Constr." support constraint optimization directly.
For these optimizers, the hard pole region is by default expressed as a constraint.
Optimizers which are marked with an asterisk in the column "Obj." support multi objective optimization directly.

For the other optimizers the hard pole region is transformed into a soft pole region by an outer penalty function specified with `GammaJType`.
These optimiziers are only applicable to pole region assignment problems with one hard or one soft region and no other objective terms (e.g. norm of controller).


To specify one of the available optimizers with their default options, the argument `optimizer` can be set to
```matlab
optimizer = optimization.solver.Optimizer.IPOPT;
```
or any other optimizer than IPOPT.

To change the options of the optimizer, the following syntax can be used.
This is an example for IPOPT, other optimizers may provide different options:
```matlab
options = optimization.options.OptionFactory.instance.options(...
    optimization.solver.Optimizer.IPOPT, ...
	'ProblemType',                  optimization.options.ProblemType.CONSTRAINED,...
	'Retries',                      1,...
	'Algorithm',                    solver.defaultalgorithm,...
	'FunctionTolerance',            1E-8,...
	'StepTolerance',                1E-10,...
	'ConstraintTolerance',          1E-7,...
	'MaxFunctionEvaluations',       25E3,...
	'MaxIterations',                25E3,...
	'MaxSQPIter',                   25E3,...
	'SpecifyObjectiveGradient',     true,...
	'SpecifyObjectiveHessian',      false,...
	'SpecifyConstraintGradient',    true,...
	'SpecifyConstraintHessian',     false,...
	'CheckGradients',               false,...
	'FunValCheck',                  false,...
	'FiniteDifferenceType',         'forward',...
	'Diagnostics',                  false,...
	'Display',                      'iter-detailed'...
);
```


The most important options are

| Option | Description |
| --- | --- |
| `ProblemType` | type of problem formulation (constrained, unconstrained, multiobjective)
| `Algorithm` | algorithm to use for optimization (if the solver supports different algorithms)
| `SpecifyObjectiveGradient` | indicator if gradient information for the objective function should be used
| `SpecifyObjectiveHessian` | indicator if hessian information for the objective function should be used
| `SpecifyConstraintGradient` | indicator if gradient information for the constraint functions should be used
| `SpecifyConstraintHessian` | indicator if hessian information for the constraint functions should be used
| `Display` | level of verbosity of the displayed information

In general the class `Options` has most the fields that the `optimoptions` class of the Optimization Toolbox of Matlab R2016B and newer has with the same meaning and additionally the fields `ProblemType` and `Retries`.
Which of these settings are taken into account by a certain solver depends on the solvers interface.
Generally speaking, the solvers from the Opimization Toolbox support the same settings as can be set by `optimoptions` while the external solvers with MEX interfaces only support a limited subset of all options.

### Bounds and nonlinear constraints

#### `Rbounds`: Bounds (linear inequality constraints)

Bounds can be imposed for single entries of the controller matrices as well as bounds on linear combinations of parameters of the same matrix <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/1e438235ef9ec72fc51ac5025516017c.svg?invert_in_darkmode" align=middle width=12.60847334999999pt height=22.465723500000017pt/>, <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/d6328eaebbcd5c358f426dbea4bdbf70.svg?invert_in_darkmode" align=middle width=15.13700594999999pt height=22.465723500000017pt/> or <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/b8bc815b5e9d5177af01fd4d3d3c2f10.svg?invert_in_darkmode" align=middle width=12.85392569999999pt height=22.465723500000017pt/> (i.e. linear inequality constraints) can be imposed.

They are defined similarly to the equality constraints in `Rfixed`.

Combined constraints on all gain coefficients can be formed as a cell array containing matrices for constraints of the single matrices and another one for the combined constraints as follows:
```matlab
Rbounds = {Ra_bounds, Ka_bounds, Fa_bounds, RKFa_bounds}
```
The definitions of the bounds for the single matrices (which are explained below) are combined by forming a cell array, i.e. if all three matrices are used:
```matlab
Rbounds = {Ra_bounds, Ka_bounds, Fa_bounds}
```
If only <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/1e438235ef9ec72fc51ac5025516017c.svg?invert_in_darkmode" align=middle width=12.60847334999999pt height=22.465723500000017pt/> and <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/d6328eaebbcd5c358f426dbea4bdbf70.svg?invert_in_darkmode" align=middle width=15.13700594999999pt height=22.465723500000017pt/> is used, 
```matlab
Rbounds = {Ra_bounds, Ka_bounds}
```
and if only <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/1e438235ef9ec72fc51ac5025516017c.svg?invert_in_darkmode" align=middle width=12.60847334999999pt height=22.465723500000017pt/> is used, 
```matlab
Rbounds = {Ra_bounds}
```

The bounds of <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/1e438235ef9ec72fc51ac5025516017c.svg?invert_in_darkmode" align=middle width=12.60847334999999pt height=22.465723500000017pt/> (<img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/d6328eaebbcd5c358f426dbea4bdbf70.svg?invert_in_darkmode" align=middle width=15.13700594999999pt height=22.465723500000017pt/> and <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/b8bc815b5e9d5177af01fd4d3d3c2f10.svg?invert_in_darkmode" align=middle width=12.85392569999999pt height=22.465723500000017pt/> analogously) are defined by
```matlab
Ka_bounds = {Zlhs, Zrhs}
```
where `Zlhs` and `Zrhs` correspond to <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/9c3c09544033898d9b2ca67679838af6.svg?invert_in_darkmode" align=middle width=38.84148674999999pt height=22.465723500000017pt/> and <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/73b5748e1a609785e1fca7bc7a8e83f0.svg?invert_in_darkmode" align=middle width=35.26465964999999pt height=14.15524440000002pt/>, resp., in

<p align="center"><img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/cadca05a188d2a2bb153636935485a0e.svg?invert_in_darkmode" align=middle width=166.0751037pt height=38.89287435pt/></p>

where <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/9b808701e2b68072679bcc95e3891b8e.svg?invert_in_darkmode" align=middle width=12.785434199999989pt height=19.1781018pt/> means element-wise multiplication (Hadamard product).
If there is more than one inequality <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/63bb9849783d01d91403bc9a5fea12a2.svg?invert_in_darkmode" align=middle width=9.075367949999992pt height=22.831056599999986pt/>, the matrices <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/3173677423a86e28caa6d954dbc490ff.svg?invert_in_darkmode" align=middle width=18.487510799999992pt height=22.465723500000017pt/> are stacked along the third dimension in `Zlhs`.
I.e, if `Nz` linear inequalities are specified, the dimensions of `Zlhs` and `zrhs` are `size(Zlhs): [size(R, 1), size(R, 2), Nz]` (`size(Zlhs): [size(R, 1), size(R, 2) + size(K, 2) + size(F, 2), Nz]` for combined constraints) and `size(Zrhs): [Nz, 1]`, resp.

For an example refer to the section about the parameter `Rfixed`.



#### `Rnonlin`: Nonlinear inequality and equality constraints

It is possible to impose nonlinear equality and inequality constraints on the parameters of the matrices <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/1e438235ef9ec72fc51ac5025516017c.svg?invert_in_darkmode" align=middle width=12.60847334999999pt height=22.465723500000017pt/>, <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/d6328eaebbcd5c358f426dbea4bdbf70.svg?invert_in_darkmode" align=middle width=15.13700594999999pt height=22.465723500000017pt/> or <img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/b8bc815b5e9d5177af01fd4d3d3c2f10.svg?invert_in_darkmode" align=middle width=12.85392569999999pt height=22.465723500000017pt/>.
In contrast to the linear constraints, a single constraint can only be imposed on one or more parameter of the same matrix, i.e.

<p align="center"><img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/5a9e42d9048c1a4457ca02917a250d5a.svg?invert_in_darkmode" align=middle width=88.86044475pt height=140.31961184999997pt/></p>

These functions are provided by a single function `Rnonlin_wrapper` which is to be passed as the argument `Rnonlin` and has the signature
```matlab
[cineq_R, ceq_R, cineq_K, ceq_K, cineq_F, ceq_F] = Rnonlin_wrapper(R, K, F)
```
or
```matlab
[cineq_R, ceq_R, cineq_K, ceq_K, cineq_F, geq_F, gineq_R, geq_R, gineq_K, geq_K, gineq_F, geq_F] = Rnonlin_wrapper(R, K, F)
```
The second variation returns the gradients as well.

* The return values `c...` are vectors of possibly different length corresponding to the functions above.
* If the gradients `g...` are returned, they are returned in three-dimensional matrices. For each entry in the corresponding `c...`vector `g...`contains a matrix with the derivations after each entry of the concerned controller matrix. These are stacked in the third dimension. I.e. 
```matlab
size(gineq_R) : [size(R, 1), size(R, 2), length(cineq_R) ]
```

##### Example
If the controller matrix is

<p align="center"><img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/eda266d1bfb293a074b2176ca2ff3c51.svg?invert_in_darkmode" align=middle width=129.12676919999998pt height=39.452455349999994pt/></p>

and the constraints

<p align="center"><img src="https://raw.githubusercontent.com/pvogt09/gammasyn/master/docs/svgs/6c128ed5eb19340a3e0d02e3bf190e45.svg?invert_in_darkmode" align=middle width=80.26617555pt height=44.96263035pt/></p>

are given (and ignoring that the first constraint can be expressed as two simpler linear constraints), the `Rnonlin_wrapper` would be
```matlab
function [cineq_R, ceq_R, cineq_K, ceq_K, cineq_F, ceq_F] = Rnonlin_wrapper(R, K, F)

	ceq_R = []; cineq_K = []; ceq_K = []; cineq_F = []; ceq_F = [];

	cineq_R = [
		R(1, 2)^2 - 9;
		R(1, 2) + R(2, 3)^2 - 4
	];

end
```
or
```matlab
function [cineq_R, ceq_R, cineq_K, ceq_K, cineq_F, geq_F, gineq_R, geq_R, gineq_K, geq_K, gineq_F, geq_F] = Rnonlin_wrapper(R, K, F)

	ceq_R = []; cineq_K = []; ceq_K = []; cineq_F = []; ceq_F = [];
	geq_R = []; gineq_K = []; geq_K = []; gineq_F = []; geq_F = [];

	cineq_R = [
		R(1, 2)^2 - 9;
		R(1, 2) + R(2, 3)^2 - 4
	];

	gineq_R = cat(3,...
		[0, 2 * R(1, 2), 0; 0, 0, 0],...
		[0, 1, 0; 0, 0, 2 * R(2, 3)]...
	);
end
```



### Saving results
For comparing and examining different calculated solutions the `SolutionSet` class can be used.
It is instantiated like
```matlab
controllerdata = control.design.gamma.SolutionSet(controller, system, polearea, weight, R_fixed, R_bounds, true);
controllerdata.add_solution(R_opt, J_opt, information, R0, options, gammaopts);
controllerdata.save();
```
after a call to `gammasyn` and expects the controller type used as `OutputFeedback` and the arguments passed to and returned by `gammasyn`.
It has the ability to plot the closed loop eigenvalues and pole regions with the `plot` method, plot step responses with the `step` method and solve the problem again with possibly different initial values or different optimizers with the `rerun` method.



## Examples

### Robust PID control

```matlab
m = [0.9, 1.1];
d = [8, 12];
c = 1000;

sys = [
	omo_sys(m(1), d(1), c),...
	omo_sys(m(2), d(1), c),...
	omo_sys(m(1), d(2), c),...
	omo_sys(m(2), d(2), c),...
	omo_sys(mean(m), mean(d), c)...
];

controller = control.design.outputfeedback.PIDOutputFeedback();

sys_augmented = controller.amend(sys);

[Ra_fixed, Ka_fixed] = controller.gainpattern(sys);

Rfixed = {Ra_fixed, Ka_fixed};


polearea = [
	control.design.gamma.area.Line(1, 0),...
	control.design.gamma.area.Imag(1, 1)
];

weight = 1;
R0 = ones(2, 2);


gammaopts = control.design.gamma.GammasynOptions();

optimizer = optimization.solver.Optimizer.IPOPT;

[R_opt, J_opt, info] = control.design.gamma.gammasyn(...
	sys_augmented,...
	polearea, weight,...
	Rfixed, R0, optimizer, gammaopts...
);
```

### Hard and soft regions

If a more ambitious region is targeted, as the one with the darker shade in the following image, and only a P-controller is to be used, there is no feasible solution.

<img src="docs/images/tex/tikz_ext/ex-omo-area-soft.png" width=25%/>

Therefore, this region can be defined as a soft region.

```matlab
m = [0.9, 1.1];
d = [8, 12];
c = 1000;

sys = [
	omo_sys(m(1), d(1), c),...
	omo_sys(m(2), d(1), c),...
	omo_sys(m(1), d(2), c),...
	omo_sys(m(2), d(2), c),...
	omo_sys(mean(m), mean(d), c)...
];

hardregion = [
	control.design.gamma.area.Line(1, 0),...
	control.design.gamma.area.Imag(1, 1)
];

softregion = [
	control.design.gamma.area.Line(0.5, 0),...
	control.design.gamma.area.Imag(1, 2)
];

polearea = {hardregion, softregion};
weight = {1, 1};
R0 = 1;
Rfixed = [];

gammaopts = control.design.gamma.GammasynOptions('type', GammaJType.SQUAREPENALTY);

optimizer = optimization.solver.Optimizer.IPOPT;

[R_opt, J_opt, info] = control.design.gamma.gammasyn(...
	sys,...
	polearea, weight,...
	Rfixed, R0, optimizer, gammaopts...
);
```

This is a feasible problem.
The following image shows the poles of the closed loop systems.
(Not all poles are shown.
The poles which aren't shown lie to the left of the shown section and are less critical concerning the defined region.)
Clearly, some poles lie outside of the soft region, but all poles lie inside of the lighter shaded hard region.

<img src="docs/images/tex/tikz_ext/ex-omo-area-soft-sol1.png" width=25%/>

With the weights the designer can influence the result further.
If for the problem at hand the velocity of the closed loop system is more important than its oscillation, the weight of the vertical border at -2 could be increased by
```matlab
weight = {1, [1, 10]};
```
As a result, the poles respect the right border of the soft region more, while the distances to the upper and lower borders increase.

<img src="docs/images/tex/tikz_ext/ex-omo-area-soft-sol2.png" width=25%/>



<!--- ### Specifying parameter bounds --->


### Using compiled functions
In order to make the objective and constraint functions run a couple of orders faster it is possible to generate mex files for these functions with the help of the Matlab Coder toolbox.
The functions for compiling the functions are located in the `compile` package and the `all` function therein can be used to generate code and compile all the supported functions.
If it is intended to use the eigenvector derivative supplied by van der Aa's method it is necessary to use at least Matlab R2016B for code generation as this method relies on run time recursion which is not supported in earlier versions.
When the compiled functions should be used the option `usecompiled` in the `gammaopts` argument has to be set to true.
Since the generated code only support a limited set of Matlab's functionality, only builtin pole areas can be used and no custom defined areas as function handles.


### Using Matlab system definitions
Besides the structure description of the systems it is also possible to use the system description provided by the Control toolbox and Robust Control toolbox.
The system descriptions without tunable or uncertain parameters (i.e. `tf`, `ss` and `dss`) are internally converted to structure arrays of systems with the respective fields and whether the fields `C_dot` and `C_ref` should be filled by the output matrix of the system can be controlled by the options `usemeasurements_xdot` and `usereferences` in the `gammaopts` structure.
For systems with uncertain parameters (i.e. `uss`) it is possible to create multiple models from a single uncertain system.
This behavior can be controlled by the structures `Blocks` or `systems` which should contain the number of sampling points for the respective uncertain parameter.
For example an uncertain system with uncertain parameter `u` can be converted into 10 multiple models by setting
```matlab
gammaopts.Blocks.u = 10;
```
Currently it is only possible to handle real uncertainty this way while complex uncertainty results in an error.

If tunable coefficients should be contained in the system description (i.e. `genss`) a specialized wrapper function named `gammasyn_loopune` has to be used which converts the internal system description to the output feedback formulation used by this toolbox and splits the system in an uncertain part and the controller matrix.
Bounds on the coefficients and fixed elements are respected as well in the conversion steps.
For the creation of multiple models from the uncertain part of the resulting system the same rules as above apply.
The signature of the function is
```matlab
[system_cl, Jopt, information] = control.design.gamma.gammasyn_looptune(systems, areafun, weights, solveroptions, objectiveoptions)
```
and it expects the system to be of type `genss` while the rest of the arguments equals the arguments used for the normal call to `gammasyn`.
The output contains the optimal objective value as well as the structure with information about the optimization run while the controller is returned as part of the closed loop into which the found solution is inserted.


### Iterative calls to gammasyn

Especially if the constrained optimizers cannot be used, it is quite probable that a desired pole region cannot be reached directly if the initial value of the controller matrices aren't good as the objective functions generally possess many bad local minima.
Therefore, it is often necessary to begin with rather wide pole regions and a quadratic loss function and interactively narrowing the region and perhaps switching to an exponential loss function.
For the exponential loss function the weights might be increased over multiple steps as well.




## 3rd party components

* Optimizers -> see table above
* [ROLMIP](https://github.com/agulhari/ROLMIP)
* oplace
* [DataHash](https://de.mathworks.com/matlabcentral/fileexchange/31272-datahash)


## Licence

GNU Lesser General Public License Version 3.0

Patrick Vogt

