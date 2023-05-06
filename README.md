# Secure communications system on seL4

## Up-to-date examples
(From newer to older):

1. `key-exchange`: Based on `comm-crypto-rpc-relay`, but first establishes two pairs of keys with GCS for each session.

2. `comm-crypto-rpc-relay`: Based on `comm-crypto-relay`, but use RPC calls for driver output.

3. `comm-crypto-relay`: Integrate driver output inside Encrypt/Decrypt.


## How to ... ?

1. Build an app:

Run the script, followed by a subdirectory name in the `apps/` directory.

```sh
./scripts/build-app.sh comm-crypto-relay
```

The compiled image is in `build/apps/<app_name>/images`.


## Directory structure

```
├── apps
│   ├── can                             -- MCP2515 CAN controller driver. Depends on SPI.
│   ├── comm                            -- [Obsolete] Communications system, with ringbuffer and RPCCall.
│   ├── comm-crypto                     -- [Obsolete] Basically the same as comm-crypto-block, with 280-byte plaintext block for each message.
│   ├── comm-crypto-block               -- Encrypted communication between Raspi and GCS, with 64-byte plaintext blocks. Manually craft heartbeat messages.
│   ├── comm-crypto-relay               -- [Crypto example] Encrypted communication between Pixhawk and GCS, relayed by Raspi. With 64-byte plaintext blocks.
│   ├── comm-crypto-rpc-relay           -- [Crypto example with RPC calls] Same as `comm-crypto-relay`, but use RPC calls for output.
│   ├── comm-mavlink-relay              -- [Plaintext example] Relay plaintext messages.
│   ├── comm-mavlink-rpc-relay          -- [Plaintext example with RPC calls] Same as `comm-mavlink-relay `, but use RPC calls for output.
│   ├── comm-packet                     -- Manually craft heartbeat messages.
│   ├── comm-smaccm                     -- [Obsolete] SMACCM structure, periodic dispatcher and monitors.
│   ├── comm-uart                       -- Communications system, with no queues and no ringbuffer.
│   ├── dencypt-test
│   ├── empty
│   ├── gcs-crypto                      -- [Obsolete] Ground Control Station on Linux. Use 280-byte plaintext block for each message.
│   ├── gcs-crypto-block                -- Ground Control Station on Linux. Use 64-byte plaintext blocks.
│   ├── hello
│   ├── hello-camkes-1                  -- "hello world" example with CAmkES.
│   ├── hello-camkes-2                  -- CAmkES dataport/event tutorial.
│   ├── key-exchange                    -- [Crypto example with RPC calls & key exchange] Latest version.
│   ├── spi                             -- SPI driver, yet to be tested.
│   ├── telemetry
│   ├── uart                            -- Two serial devices in a single component.
│   ├── uart2                           -- One serial device in each component.
│   └── uart-timer                      -- Use TimeServer's periodic callback.
├── backup
│   ├── firmware                        -- Firmware on SD card, for seL4 running on Raspberry Pi 4B.
│   ├── kernel
│   ├── projects
│   ├── tools
│   └── USB
├── build
│   └── apps
├── diffs
│   ├── projects
│   └── tools
├── docs
├── GEC                                 -- GEC library. Use 280-byte plaintext blocks.
│   ├── include
│   └── src
├── GEC-block                           -- GEC library. Use 64-byte plaintext blocks.
│   ├── include
│   └── src
├── GEC-block-linux                     -- GEC library for Linux apps. Symbolic link to `GEC-block`.
│   ├── include -> ../GEC-block/include/
│   └── src -> ../GEC-block/src
├── GEC-linux                           -- GEC library for Linux apps. Symbolic link to `GEC`.
│   ├── include -> ../GEC/include/
│   └── src -> ../GEC/src
├── kernel
│   ├── configs
│   ├── include
│   ├── libsel4
│   ├── LICENSES
│   ├── manual
│   ├── src
│   └── tools
├── mavlink                             -- MAVLink library
│   └── include
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
