Raspberry Pi 4B sends to and receives from GCS.

Utilizing SMP provided by Elf-Loader.

Enforce key exchange, every time the Raspberry Pi boots.

Relay packets from/to Pixhawk to/from GCS. Use GEC encryption from/to Raspberry Pi to/from GCS.

Separate MAVLink message frames into blocks.

```

                                      MAVLink message view

+-------------------------+----------+-----------------------------------------------------------+
|     MAVLINK Frame 1     | Frame 2  |                         Frame 3                           | ...
+-------------------------+----------+-----------------------------------------------------------+

=>

                                        GEC plaintext view
+------------------------------------------------------------------------------------------------+
|         20         |          20        |          20        |          20        |     12     | ...
+------------------------------------------------------------------------------------------------+

```

Raspberry Pi [UART 3, 3DR Radio Telemetry]    <===[915 MHz wireless]===>    [3DR Radio Telemetry] Laptop.
