## PCAN FD firmware for STM32G431 based boards

目标硬件:
* CANable2.0硬件，或跟它相似的、非标版本（带8MHz外部晶振或LED管脚有差异）的USB CAN模块。
* MCU型号为：STM32G431CBT6/STM32G431CBU6

管脚:
|PIN/PINS|DESCRIPTION|
| ------ | ------ |
|PA15/PA6|STATUS LED|
|PA0/PA5|WORK LED|
|PB9/PB8|TXD/RXD CAN|
|PA11/PA12 |USB FS DM/DP|

特色:
- 支持CAN FD
- 适配PCAN-View的大部分功能（CAN FD、设置时钟频率、波特率/采样点、ISO/Non-ISO模式、仅侦听模式等）

限制:
- Some protocol specific messages not implemented yet
- 1ms周期发送有丢包现象
- 未支持硬件过滤

编译方法:
- 1，ubuntu等linux主机  
a). 安装arm-none-eabi-gcc交叉编译环境，譬如ARM官网上的gcc-arm-none-eabi-7-2018-q2-update等  
b). 使用 make 、 make pcanfd 或者 make canable2进行编译  
c). 使用 make clean清理工程  

- 2，windows主机  
推荐使用官方“STM32CubeIDE”的最新版本进行编译和仿真。  
以“Create a new Makefile project in a directory containing existing code”方式创建工程。  
在Makefile中修改“DEBUG=1”（增加“-g”编译选项）后，搭配ST-Link仿真器还能下断点和单步调试。  

须知:
- 本工程从https://github.com/moonglow/pcan_pro_x 移植过来。
- CANFD驱动源自https://github.com/Elmue/CANable-2.5-firmware-Slcan-and-Candlelight.git
关于CANable-2.5请参考https://netcult.ch/elmue/CANable%20Firmware%20Update/
- ISO/Non-ISO模式设置的协议解析和STM32运行时切换系统时钟思路参考https://bbs.21ic.com/icview-3491240-1-1.html

License
----
WTFPL
