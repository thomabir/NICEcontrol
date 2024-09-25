# NICEcontrol

NICEcontrol is a program to monitor and control the [Nulling Interferometry Cryogenic Experiment (NICE)](https://quanz-group.ethz.ch/research/instrumentation/nice.html).

This nulling testbed, built at ETH Zürich by the [Exoplanets & Habitability group](https://quanz-group.ethz.ch/), will demonstrate the feasibility of the [LIFE space mission](https://life-space-mission.com/).

![User interface of NICEcontrol](./img/ui.png)

## Install

### Prerequisites

* Install libraries

   ```bash
   sudo add-apt-repository ppa:berndporr/dsp # for iir1 (https://github.com/berndporr/iir1)
   sudo apt-get install libglfw3-dev libfftw3-dev libboost-all-dev iir1-dev python3-venv
   ```

* (Obsolte) Install the piezo controller drivers from [MCL](http://www.madcitylabs.com/) (ask their support for the files)
* Install the [Physikinstrumente](https://www.physikinstrumente.de/de/) piezo controller drivers.
  * Obtain the software bundle from PI, referred to as `PI_C-990_CD1/` in the following.
    The files can be downloaded from the PI website or found on the installation CD that came with the stage.
  * Install the PI GCS2 C++ library

    ```sh
    cd ~/Downloads/PI_C-990_CD1/Linux/
    tar -xvf PI_Application_Software-1.18.1.0-INSTALL.tar.bz2
    cd pi-drivers/Linux/PI_Application_Software
    sudo ./INSTALL
    ```

    This should install, among other things, the `lipi_pi_gcs2.so` library
  * Copy some header files into `NICEcontrol`:

    ```sh
    cd ~/code/NICEcontrol
    mkdir lib/pi
    cp ~/Downloads/PI_C-990_CD1/Development/C++/Samples/E-727/AutoZeroSample/AutoZeroSample.h lib/pi/
    cp ~/Downloads/PI_C-990_CD1/Linux/PI_Application_Software/libpi_pi_gcs2_x86_64/include/PI_GCS2_DLL.h lib/pi/
    ```

  * Comment out the line `#include <windows.h>` in `lib/pi/AutoZeroSample.h`
  * Restart the computer.
* Install the piezo controller drivers from [nanoFaktur](https://www.nanofaktur.com/support).
  * The files can be found on their support website, and the password is on the calibration certificate of the controller. You can also ask their support for the password. The installation instructions they provide are not best practice, so instead you may want to follow the instructions here:
  * Locate `libnF_interface.so` and install it as a library. By default:

    ```bash
    sudo cp ~/Downloads/EBx-120\ Support/software/lib/linux/libnF_interface_x64.so /usr/local/lib/
    sudo ldconfig
     ```

  * Copy the header files (`nF_common.h`, `nF_error.h`, `nF_interface.h`) to the lib directory:

    ```bash
    cd ~/code/NICEcontrol/
    mkdir lib/nF
    cp ~/Downloads/EBx-120\ Support/software/programming_examples/c-testLinuxLib/include/* lib/nF/
    ```

    In `nF_interface.h`, add the line `#define LINUX` at the beginning

### NICEcontrol

Clone the `NICEcontrol` repository and its submodules

   ```bash
   git clone --recurse-submodules https://github.com/thomabir/NICEcontrol
   ```

## Compile

   ```bash
   cd ~/code/NICEcontrol
   make
   ```

## Use

```bash
./bin/NICEcontrol
```

## Debugging with Analog Discovery 2

### Install

Download the [Adept 2 Runtime](https://digilent.com/reference/software/adept/runtime-previous-versions) (64 bit `.deb` file).
Download [Digilent WaveForms](https://digilent.com/reference/software/waveforms/waveforms-3/previous-versions) (64 bit `.deb` file).

Install their dependencies and the `.deb` files:

```bash
# sudo apt-get install libqt5multimedia5-plugins libqt5scripttools5 libqt5network5 libqt5serialport5
sudo dpkg -i ~/Downloads/digilent.adept.runtime_2.16.6-amd64.deb
sudo dpkg -i ~/Downloads/digilent.waveforms_3.23.4_amd64.deb 
```

### Use

* Open WaveForms, and open the `teensy-spi.dwf3work` file.
* The waveforms shown by default are the data output of the ADCs, which are fed into the FPGA.

## Teensy to activate ADCs

### Install

Using PlatformIO in VSCode.

Download and install the rules for the Teensy:

```bash
cd ~/Downloads
wget https://www.pjrc.com/teensy/00-teensy.rules
sudo cp 00-teensy.rules /etc/udev/rules.d/
```

### Use

* Open the PlatformIO project in VSCode.
* Compile and upload the code to the Teensy.
* If the upload fails, press the program button on the Teensy and try again.

## License

NICEcontrol is licensed under the MIT License, see [LICENSE](LICENSE) for more information.
