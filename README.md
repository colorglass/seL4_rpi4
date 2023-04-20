
```
├── apps
│   ├── can                 -- MCP2515 CAN controller driver. Depends on SPI.
│   ├── comm                -- Communications system, with ringbuffer and RPCCall.
│   ├── comm-crypto         -- [Obsolete] Basically the same as comm-crypto-block, with 280-byte plaintext block for each message.
│   ├── comm-crypto-block   -- Encrypted communication between Raspi and GCS, with 64-byte plaintext blocks. Manually craft heartbeat messages.
│   ├── comm-crypto-relay   -- [Crypto example] Encrypted communication between Pixhawk and GCS, relayed by Raspi. With 64-byte plaintext blocks.
│   ├── comm-packet         -- Manually craft heartbeat messages.
│   ├── comm-mavlink-relay  -- [Plaintext example] Relay plaintext messages, with SMP support.
│   ├── comm-smaccm         -- SMACCM structure, periodic dispatcher and monitors.
│   ├── comm-uart           -- Communications system, with no queues and no ringbuffer.
│   ├── dencypt-test
│   ├── empty
│   ├── gcs-crypto          -- [Obsolete] Ground Control Station on Linux. Use 280-byte plaintext block for each message.
│   ├── gcs-crypto-block    -- Ground Control Station on Linux. Use 64-byte plaintext blocks.
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
├── GEC                     -- GEC library. Use 280-byte plaintext blocks.
├── GEC-block               -- GEC library. Use 64-byte plaintext blocks.
├── GEC-block-linux         -- GEC library for Linux apps. Symbolic link to `GEC-block`.
├── GEC-linux               -- GEC library for Linux apps. Symbolic link to `GEC`.
├── kernel
│   ├── configs
│   ├── include
│   ├── libsel4
│   ├── LICENSES
│   ├── manual
│   ├── src
│   └── tools
├── mavlink                 -- MAVLink library.
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
