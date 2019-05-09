# Brake Throttle Pedal Emulator Automated Tester
Automated tester for the Brake Throttle Pedal Emulator for production end-of-line testing.

# Acronyms
* BTPE - Brake Throttle Pedal Emulator
* BPEC - Brake Pedal Emulator Combo
* BPET - Brake Pedal Emulator Tester
* TPEC - Throttle Pedal Emulator Combo
* TPET - Throttle Pedal Emulator Tester
* BOO  - Brake On Off unit

# Turn all hardware on for debugging
```
roslaunch eol_tester_btpe debug.launch
```

# Run the Tester, specifying the platform to test (currently ```ford_c1```, ```ford_cd4```, ```ford_cd4_boo```, ```ford_p5```, ```fca_wk2``` and ```fca_ru```  are supported)
```
roslaunch eol_tester_btpe live.launch platform:=fca_ru
```
```
roslaunch eol_tester_btpe live.launch platform:=ford_cd4

```
To skip reflashing the firmware, add the ```reflash:=false``` option:
```
roslaunch eol_tester_btpe live.launch platform:=ford_cd4 reflash:=false
```

# ROS Topics
|   Topic           |   Type                                                        |
|-------------------|---------------------------------------------------------------|
| /bpec/can/cmd     | [BrakeCmd](hil_msgs/msg/BrakeCmd.msg)                 |
| /bpec/can/boo     | [BrakeCmd](std_msgs/msg/Bool.msg                              |
| /bpec/can/report  | [BrakeReport](hil_msgs/msg/BrakeReport.msg)           |
| /bpec/can/version | [Version](hil_msgs/msg/Version.msg)                   |
| /bpet/can/cmd     | [BrakeCmd](hil_msgs/msg/BrakeCmd.msg)                 |
| /bpet/can/report  | [BrakeTestData](hil_msgs/msg/BrakeTestData.msg)       |
| /bpet/can/version | [Version](hil_msgs/msg/Version.msg)                   |
| /bpet/set_uut     | [UUT](hil_msgs/msg/UUT.msg)                           |
| /tpec/can/cmd     | [ThrottleCmd](hil_msgs/msg/ThrottleCmd.msg)           |
| /tpec/can/report  | [ThrottleReport](hil_msgs/msg/ThrottleReport.msg)     |
| /tpec/can/version | [Version](hil_msgs/msg/Version.msg)                   |
| /tpet/can/cmd     | [ThrottleCmd](hil_msgs/msg/ThrottleCmd.msg)           |
| /tpet/can/report  | [ThrottleTestData](hil_msgs/msg/ThrottleTestData.msg) |
| /tpet/can/version | [Version](hil_msgs/msg/Version.msg)                   |
| /relay1/ready     | [Bool](std_msgs/msg/Bool.msg)                                 |
| /relay1/cmd       | [Byte](std_msgs/msg/Byte.msg)                                 |
| /relay2/ready     | [Bool](std_msgs/msg/Bool.msg)                                 |
| /relay2/cmd       | [Byte](std_msgs/msg/Byte.msg)                                 |

# ROS Services
|   Topic                 |   Type                                                         |
|-------------------------|----------------------------------------------------------------|
| /bpec/get_inputs        | [GetInputs](dbw_dev_usb_msgs/srv/GetInputs.srv)                   |
| /bpec/reset_params      | [Trigger](std_msgs/srv/Trigger.srv)                            |
| /bpec/set_vcc_latch     | [SetVccLatch](dbw_dev_usb_msgs/srv/SetVccLatch.srv)               |
| /tpec/get_inputs        | [GetInputs](dbw_dev_usb_msgs/srv/GetInputs.srv)                   |
| /tpec/reset_params      | [Trigger](std_msgs/srv/Trigger.srv)                            |
| /tpec/set_vcc_latch     | [SetVccLatch](dbw_dev_usb_msgs/srv/SetVccLatch.srv)               |
