embedded_project

stm32-based multi-function gimbal

简易双轴云台控制系统

基于 STM32F103C8T6 的双轴云台，支持串口通信、MPU6050 姿态从动、四路光敏追光，OLED 显示。

硬件

MCU：STM32F103C8T6  
传感器：MPU6050、HS-S06B  
执行：MG90S 舵机 ×2  
调试：ST-Link  
舵机独立 5V 供电并共地

软件

IDE:Keil 5 + STM32CubeMX  
链:ARM-MDK  
库:HAL

cnt      模式                功能                          LED1  
 0      模式 1       串口指令控制                    灭  
 1      模式 2     MPU6050 姿态随动           常亮  
 2      模式 3      四路光敏追光 1Hz            闪烁

核心算法

互补滤波：Pitch = 0.98*(Pitch+gx*dt) + 0.02*acc_pitch，dt=20ms  
陀螺仪零偏：静止采样 200 次均值去偏，极差/重力模长校验  
ADC：8 点环形滑动平均  
追光：差值 → 死区 150 → 一阶低通 β=0.3 → 限幅 ±80 → 增量累加  
角度映射：Pitch=1380-(θ-90)\*11，Yaw=1450+(θ-90)\*10.5  
全程 HAL_GetTick() 时间戳，无阻塞 Delay

编译烧录

Keil 5 打开工程 → Build → ST-Link Download → RESET，自检

License  
仅供参考
