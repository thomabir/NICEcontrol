# NICEcontrol

NICEcontrol is a program to monitor and control the [Nulling Interferometry Cryogenic Experiment (NICE)](https://quanz-group.ethz.ch/research/instrumentation/nice.html).

This nulling testbed, built at ETH Zürich by the [Exoplanets & Habitability group](https://quanz-group.ethz.ch/), will demonstrate the feasibility of the [LIFE space mission](https://life-space-mission.com/).

![User interface of NICEcontrol](./img/ui.png)

## Install

### Prerequisites

* Install libraries

   ```bash
   sudo apt-get install libglfw3-dev fftw-dev libboost-all-dev
   ```

* (Obsolte) Install the piezo controller drivers from [MCL](http://www.madcitylabs.com/) (ask their support for the files)
* Install the piezo controller drivers from Physikinstrumente. (Download from their website or copy from installation medium). Then, copy `AutoZeroSample.h` and `PI_GCS2_DLL.h` from the installation files into `NICEcontrol/lib/pi`. See [PIcontrol](https://github.com/thomabir/PIcontrol) Readme for more information.
* Install the piezo controller drivers from [nanoFaktur](https://www.nanofaktur.com/support). The files can be found on their support website, and the password is on the calibration certificate of the controller. You can also ask their support for the password. The installation instructions they provide are not best practice, so instead you may want to follow the instructions here:
  * Locate `libnF_interface.so` and install it as a library. By default:

    ```bash
    sudo cp ~/Downloads/EBx-120\ Support/software/lib/linux/libnF_interface.so /usr/local/lib/
    sudo ldconfig
     ```

  * Copy the header files (`nF_common.h`, `nF_error.h`, `nF_interface.h`) to the lib directory:

    ```bash
    cp ~/Downloads/EBx-120\ Support/software/programming_examples/C++/demoCpp/nF_Interface/include/* ~/code/NICEcontrol/lib/nF/
    ```

* Install [iir1](https://github.com/berndporr/iir1)

  ```bash
  sudo add-apt-repository ppa:berndporr/dsp
  sudo apt install iir1-dev
  ```

### NICEcontrol

Clone the `NICEcontrol` repository and its submodules

   ```bash
   git clone --recurse-submodules https://github.com/thomabir/NICEcontrol
   ```

## Compile

   ```bash
   cd NICEcontrol
   make
   ```

## Use

```bash
./bin/NICEcontrol
```

## License

NICEcontrol is licensed under the MIT License, see [LICENSE](LICENSE) for more information.
