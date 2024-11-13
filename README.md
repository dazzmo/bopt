<a name="readme-top"></a>

<!-- PROJECT SHIELDS -->

<!-- PROJECT LOGO -->
<br>
<div align="center">

  <h1 align="center">Basic OPTimisation (BOPT) Library</h1>

  <p align="left">
    A work-in-progress optimisation library to express optimisation problems and provide a common interface to solvers, as well as offering a framework to create and develop custom optimisation programs. This library is designed to be as generic as possible to allow any type of BLAS package to be interfaced to it.
  </p>
</div>

<!-- ABOUT THE PROJECT -->
## About The Project
<p align="left">
This is a basic implementation of the necessary components required to represent optimisation problems within a programatic context. This library offers a means to easily express cost and constraints in a unified manner, to which we can interface to solvers (existing or customly made) to solve.

In addition, we provide add-ons for automatic differentiation tools to provide the necessary derivatives.
</p>
<p align="right">(<a href="#readme-top">back to top</a>)
</p>

## Getting Started
<a name="getting-started"></a>

### Prerequisites

bopt requires the following third-party libraries in order to be built and installed.
* [Boost](https://www.boost.org/) (Version 1.70 or higher)

For testing purposes we have (this is later be a toggle-able option):
* [googletest](https://github.com/google/googletest)
* [glog](https://github.com/google/glog)

We also include interfaces to open-source solvers for numerical optimisation, we currently include:
* [Ipopt](https://github.com/coin-or/Ipopt)
* [qpOASES](https://github.com/coin-or/qpOASES) (-DWITH_QPOASES=ON in <a href="#installation">Installation</a>). Be sure to install qpOASES as a shared library, as this is what is expected by bopt.

<p align="right">(<a href="#readme-top">back to top</a>)</p>

### Installation
<a name="installation"></a>

1. Clone the repo
   ```sh
   git clone https://github.com/dazzmo/bopt
   ```
2. Build the library
    ```sh
    cd bopt
    mkdir build && cd build
    cmake ..
    make
   ```
3. Installation of the library can then be performed by
    ```sh
    make install
    ```
<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!-- LICENSE -->
## License

Distributed under the GNU LESSER GENERAL PUBLIC LICENSE License. See `LICENSE.txt` for more information.

<p align="right">(<a href="#readme-top">back to top</a>)</p>

<!-- CONTACT -->
## Contact

Damian Abood - damian.abood@sydney.edu.au

<p align="right">(<a href="#readme-top">back to top</a>)</p>

## Acknowledgements
Thank you to Jesse Morris for his assistance with build-related concerns and improving the layout of the library.

<p align="right">(<a href="#readme-top">back to top</a>)</p>
