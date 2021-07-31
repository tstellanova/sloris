# Sloris

A Particle project named sloris.

This project demonstrates how to do battery-conserving remote condition monitoring. 
The application sleeps periodically, waking up via the real time clock (RTC) 
or via remote (cloud) polling of one of its sensor values. 


### To Build & Flash with Particle Workbench (vscode)

This application may be built with Device OS version 2.1.0.

1. Clone this repository 
2. Init & Update Submodules `git submodule update --init --recursive`
3. Open Particle Workbench
4. Run the `Particle: Import Project` command, follow the prompts, to select this project's `project.properties` file and wait for the project to load
5. Run the `Particle: Configure Workspace for Device` command and select a compatible Device OS version and the `boron` platform when prompted ([docs](https://docs.particle.io/tutorials/developer-tools/workbench/#cloud-build-and-flash))
6. Connect your boron to your computer with a usb cable
7. Compile & Flash using Workbench


### To Build & Flash with Particle CLI

This application may be built with Device OS version 2.1.0.

1. Clone this repository 
2. Init & Update Submodules `git submodule update --init --recursive`
3. Cloud build (for Tracker) with CLI :
`particle compile --target 2.1.0 boron --saveTo sloris_boron.bin`

4. Connect your Tracker to your computer with a usb cable
5. Use the CLI to flash the device using dfu:

```
particle usb dfu
particle flash --usb sloris_boron.bin
```


## Project structure

Every new Particle project is composed of 3 important elements .

#### ```/src``` folder:  
This is the source folder that contains the firmware files for your project. It should *not* be renamed. 
Anything that is in this folder when you compile your project will be sent to our compile service and compiled into a firmware binary for the Particle device that you have targeted.

If your application contains multiple files, they should all be included in the `src` folder. If your firmware depends on Particle libraries, those dependencies are specified in the `project.properties` file referenced below.

#### ```.cpp``` file:
This file is the firmware that will run as the primary application on your Particle device. It contains a `setup()` and `loop()` function, and can be written in Wiring or C/C++. For more information about using the Particle firmware API to create firmware for your Particle device, refer to the [Firmware Reference](https://docs.particle.io/reference/firmware/) section of the Particle documentation.

#### ```project.properties``` file:  
This is the file that specifies the name and version number of the libraries that your project depends on. Dependencies are added automatically to your `project.properties` file when you add a library to a project using the `particle library add` command in the CLI or add a library in the Desktop IDE.

## Adding additional files to your project

#### Projects with multiple sources
If you would like add additional files to your application, they should be added to the `/src` folder. All files in the `/src` folder will be sent to the Particle Cloud to produce a compiled binary.

#### Projects with external libraries
If your project includes a library that has not been registered in the Particle libraries system, you should create a new folder named `/lib/<libraryname>/src` under `/<project dir>` and add the `.h`, `.cpp` & `library.properties` files for your library there. Read the [Firmware Libraries guide](https://docs.particle.io/guide/tools-and-features/libraries/) for more details on how to develop libraries. Note that all contents of the `/lib` folder and subfolders will also be sent to the Cloud for compilation.

