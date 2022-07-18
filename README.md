# electromagnetic_gun

电磁炮

# ball and plate double

板球系统

# inverted pendulum - double

旋转倒立摆系统

# wind control

风力摆系统



# 目录说明

这四个运动控制系统的程序结构基本相同；四个运动控制系统的不同在于task_control.c中采用的控制方法不同。

## 1.core文件

stm32的启动汇编文件，主要是中断函数的描述和main的启动流程

startup_stm32f427xx.s

```assembly
Reset_Handler    PROC
                 EXPORT  Reset_Handler             [WEAK]
        IMPORT  SystemInit
        IMPORT  __main
```

## 2.USER

main函数在这里

main函数里面建立多个固定大小的栈的任务。

然后开启任务调度。

```c
vTaskStartScheduler()
```

## 3.TASK

创建完任务之后，开始执行任务，在task文件中的任务分别开始执行

一个任务可以理解为一个进程，开始执行任务

## 4.Driver

驱动层的东西，就是task执行过程中需要调用的

## 其他文件

BSP：初始化文件，底层串口、can等外设接口

DMP：IMU的运动处理库，有用到IMU的才调用

FreeRTOS：操作系统的kernel，移植的文件

FWLIB：stm32的库，移植的文件

MATH：数学库，自己瞎写的

OBJ：就是输出文件库，里面的hex文件是要烧录到芯片中

SYSTEM：自己需要printf的时候配置的一个文件，以及加上FreeRTOS后修改的delay文件