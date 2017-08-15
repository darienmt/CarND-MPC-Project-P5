# CarND-MPC-Project-P5
Udacity Self-Driving Car Nanodegree - Model Predictive Control (MPC) Project

# Overview

TBW

# Prerequisites

The project has the following dependencies (from Udacity's seed project):

- cmake >= 3.5
- make >= 4.1
- gcc/g++ >= 5.4
- Udacity's simulator.

For instructions on how to install these components on different operating systems, please, visit [Udacity's seed project](https://github.com/udacity/CarND-MPC-Project). As this particular implementation was done on Mac OS, the rest of this documentation will be focused on Mac OS. I am sorry to be that restrictive.

In order to install the necessary libraries, use the [install-mac.sh](./install-mac.sh).

After that installation, there are two other required libraries:

- [Ipopt](https://projects.coin-or.org/Ipopt): In order to install this library we need to tap homebrew/science first:

```
> brew tap homebrew/science
```

Then install the package:

```
> brew install ipopt
```

- [CppAD](https://www.coin-or.org/CppAD/): `brew install cppad`

Note: It is possible you get this error message: `/usr/local/bin is not writable.` In order to fix that problem, follow the instruction suggested in this [link](https://stackoverflow.com/questions/26647412/homebrew-could-not-symlink-usr-local-bin-is-not-writable):

```
> sudo chown -R `whoami`:admin /usr/local/bin
> sudo chown -R `whoami`:admin /usr/local/share
> sudo chown -R `whoami`:admin /usr/local/lib
```
