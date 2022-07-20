***********************
eagerx_pybullet package
***********************

.. image:: https://img.shields.io/badge/License-Apache_2.0-blue.svg
   :target: https://opensource.org/licenses/Apache-2.0
   :alt: license

.. image:: https://img.shields.io/badge/code%20style-black-000000.svg
   :target: https://github.com/psf/black
   :alt: codestyle

.. image:: https://github.com/eager-dev/eagerx_pybullet/actions/workflows/ci.yml/badge.svg?branch=master
  :target: https://github.com/eager-dev/eagerx_pybullet/actions/workflows/ci.yml
  :alt: Continuous Integration

.. contents:: Table of Contents
    :depth: 2

What is the *eagerx_pybullet* package?
=================================
This repository/package contains the *PybulletEngine* for simulating robots with EAGERx in pybullet.
EAGERx (Engine Agnostic Graph Environments for Robotics) enables users to easily define new tasks, switch from one sensor to another, and switch from simulation to reality with a single line of code by being invariant to the physics engine.

`The core repository is available here <https://github.com/eager-dev/eagerx>`_.

`Full documentation and tutorials (including package creation and contributing) are available here <https://eagerx.readthedocs.io>`_.

Installation
============

You can install the package using pip:

.. code:: shell

    pip3 install eagerx-pybullet

Examples
========
For examples on how to use this package to add a pybullet implementation for your objects, please see:

- `Example environment <example/example.py>`_ containing the objects:
   - `Example manipulator (vx300s) <example/objects/vx300s/objects.py>`_
   - `Example camera <example/objects/camera/objects.py>`_
   - `Example solid <example/objects/solid/objects.py>`_

Documentation
=============

- `PybulletEngine <eagerx_pybullet/engine.py>`_

- `EngineNodes <eagerx_pybullet/enginenodes.py>`_:
   - `LinkSensors <eagerx_pybullet/enginenodes.py>`_
   - `JointSensors <eagerx_pybullet/enginenodes.py>`_
   - `JointController <eagerx_pybullet/enginenodes.py>`_
   - `CameraSensor <eagerx_pybullet/enginenodes.py>`_
   - Or implement a custom engine node (see `here <https://eagerx.readthedocs.io/en/master/guide/developer_guide/index.html>`_).

- `EngineStates <eagerx_pybullet/enginestates.py>`_:
   - `JointState <eagerx_pybullet/enginestates.py>`_
   - `LinkState <eagerx_pybullet/enginestates.py>`_
   - `PbDynamics <eagerx_pybullet/enginestates.py>`_
   - Or implement a custom engine state (see `here <https://eagerx.readthedocs.io/en/master/guide/developer_guide/index.html>`_).

Cite EAGERx
===========
If you are using EAGERx for your scientific publications, please cite:

.. code:: bibtex

    @article{eagerx,
        author  = {van der Heijden, Bas and Luijkx, Jelle, and Ferranti, Laura and Kober, Jens and Babuska, Robert},
        title = {EAGERx: Engine Agnostic Graph Environments for Robotics},
        year = {2022},
        publisher = {GitHub},
        journal = {GitHub repository},
        howpublished = {\url{https://github.com/eager-dev/eagerx}}
    }

Acknowledgements
=================
EAGERx is funded by the `OpenDR <https://opendr.eu/>`_ Horizon 2020 project.
