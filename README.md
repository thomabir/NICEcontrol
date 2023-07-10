# NICEcontrol

NICEcontrol is a program to monitor and control the [Nulling Interferometry Cryogenic Experiment (NICE)](https://quanz-group.ethz.ch/research/instrumentation/nice.html).

This nulling testbed, built at ETH Zürich by the [Exoplanets & Habitability group](https://quanz-group.ethz.ch/), will demonstrate the feasibility of the [LIFE space mission](https://life-space-mission.com/).


## Install

1. Install prerequisites
   * macOS
     ```bash
     brew install glfw
     ```
   * Linux (untested)
     ```bash
     apt-get install libglfw-dev
     ```
3. Clone the `NICEcontrol` repository and its submodules
   ```bash
   git clone --recurse-submodules https://github.com/thomabir/NICEcontrol
   ```
4. Compile
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
