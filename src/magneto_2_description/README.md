# Magneto 2 Description

---

Robot description and configuration for the Magneto 2 plaform.

## Table of Contents

- [About](#about)
- [Getting Started](#getting_started)
- [Usage](#usage)

## About <a name = "about"></a>

This is a collection of descriptions, meshes and configuration files specific to the Magneto 2 platform. This platform was developed in 2016. It has 4 legs and 3 active degrees of freedom in each leg. A passive gimbal provides compliance in 3 degrees of freedom at the foot.

## Getting Started <a name = "getting_started"></a>

These instructions will get you a copy of the project up and running on your local machine for development and testing purposes.

#### Prerequisites

The description files should be useable without any other dependancies.

For full Gazebo functionality (includes plugins for magnetic attachement etc) then you will need the following:

- [magneto_simulation](https://bitbucket.csiro.au/projects/CSR/repos/magneto_simulation/) - Gazebo plugins and worlds for all Magneto platforms.

#### Installing

Clone files to your workspace:

```
git clone https://bitbucket.csiro.au/scm/csr/magneto_2_description.git
```


## Usage <a name="usage"></a>

The Magneto URDF is generated using a xacro which reads parameters from yaml files in the config folder.

To regenerate the URDF in the robots folder:
```
xacro magneto_2.xacro > magneto_2.urdf
```