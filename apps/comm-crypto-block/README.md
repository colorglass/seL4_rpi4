Raspberry Pi 4B sends to and receives from GCS.

Utilizing SMP.

Manually craft heartbeat packets. Use encryption.

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
