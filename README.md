
```
├── apps
│   ├── can                 -- MCP2515 CAN controller driver. Depends on SPI.
│   ├── comm                -- Communications system, with ringbuffer and RPCCall.
│   ├── comm-mavlink-relay  -- Relay plaintext messages, with SMP support.
│   ├── comm-smaccm         -- SMACCM structure, periodic dispatcher and monitors.
│   ├── comm-uart           -- Communications system, with no queues and no ringbuffer.
│   ├── dencypt-test
│   ├── empty
│   ├── hello
│   ├── hello-camkes-1      -- "hello world" example with CAmkES.
│   ├── hello-camkes-2      -- CAmkES dataport/event tutorial.
│   ├── spi                 -- SPI driver, yet to be tested.
│   ├── uart                -- Two serial devices in a single component.
│   ├── uart2               -- One serial device in each component.
│   └── uart-timer          -- Use TimeServer's periodic callback.
├── backup
│   ├── firmware            -- Firmware for seL4 running on Raspberry Pi 4B.
│   ├── kernel
│   ├── projects
│   └── tools
├── build
├── diffs
│   ├── projects
│   └── tools
├── docs
├── kernel
│   ├── configs
│   ├── include
│   ├── libsel4
│   ├── LICENSES
│   ├── manual
│   ├── src
│   └── tools
├── projects
│   ├── capdl
│   ├── global-components
│   ├── musllibc
│   ├── projects_libs
│   ├── seL4_libs
│   ├── seL4_projects_libs
│   ├── sel4runtime
│   ├── sel4-tutorials
│   └── util_libs
├── scripts
└── tools
    ├── camkes
    └── seL4
```
