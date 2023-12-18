![logo](doc/img/logo-banner.png)

# raros
[![CI](https://github.com/francWhite/raros/actions/workflows/ci.yaml/badge.svg)](https://github.com/francWhite/raros/actions/workflows/ci.yaml)
[![licence](https://img.shields.io/github/license/francWhite/raros)](https://github.com/francWhite/raros/blob/main/LICENSE)
[![maven](https://img.shields.io/maven-central/v/io.github.francwhite/raros-client?label=maven)](https://central.sonatype.com/artifact/io.github.francwhite/raros-client)
[![docker](https://ghcr-badge.egpl.dev/francwhite/raros/latest_tag?color=%23107acb&ignore=latest&label=docker&trim=)](https://github.com/francWhite/raros/pkgs/container/raros)

raros is an application consisting of different subsystems that can be used to control a physical robot.
This application was developed as part of a project at the Lucerne University of Applied Sciences and Arts with the goal
of evaluating how such a control system can be implemented with ROS2. 

Ultimately, the project should be able to be used to teach students the basic concepts of programming in a playful way, 
as they can see their work directly resulting in actions executed by the robot instead of just viewing some text in a terminal.

## Table of contents
- [Hardware](#hardware)
- [Architecture](#architecture)
- [Installation](#installation)
  - [Client-Library](#client_library_install)
  - [Controller](#controller_install)
  - [Microcontroller](#microcontroller_install)
- [Usage](#usage)
  - [Client-Library](#client_library_usage) 
  - [Controller](#microcontroller_usage)
  - [Microcontroller](#microcontroller_usage)
- [Development](#development)
  - [Prerequisites](#prerequisites)
  - [Build](#build)
  - [Run](#run)
- [License](#license)

## Hardware
TODO
## Architecture

TODO

## Installation
<a name="client_library_install"></a>
### Client-Library
The client library is available on [Maven Central](https://search.maven.org/artifact/io.github.francwhite/raros-client) and can be used in any Java project.

Maven:
```xml
<dependency>
    <groupId>io.github.francwhite</groupId>
    <artifactId>raros-client</artifactId>
    <version>1.0.1</version>
</dependency>
```

Gradle:
```groovy
implementation 'io.github.francwhite:raros-client:1.0.1'
```

<a name="controller_install"></a>
### Controller
All required subsystems for the master controller (RaspberryPi) are available as docker images on [GitHub Container Registry](https://github.com/francWhite?tab=packages&repo_name=raros).
The easiest way to get started is to use the [docker-compose.yml](https://github.com/francWhite/raros/blob/main/docker-compose.yaml) 
file in the root directory of this repository.

**Prerequisites**:
- [docker](https://docs.docker.com/engine/install/) is installed

<a name="microcontroller_install"></a>
### Microcontroller
The code for the microcontroller (arduino) is available in the [micro_ros](https://github.com/francWhite/raros/tree/main/apps/micro_ros)
directory of this repository. To build and flash the software onto the microcontroller, clone the repository and use the `flash.sh` script located in root of said directory.

**Prerequisites**:
- [PlatformIO CLI](https://docs.platformio.org/en/stable/core/installation/index.html) is installed


## Usage

<a name="client_library_usage"></a>
### Client-Library
TODO

<a name="controller_usage"></a>
### Controller
TODO

<a name="microcontroller_usage"></a>
### Microcontroller
TODO

## Development

### Prerequisites
TODO

### Build
TODO

### Run
TODO

## License
This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.