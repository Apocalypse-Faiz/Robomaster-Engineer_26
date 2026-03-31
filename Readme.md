[TOC]

# 1️⃣模板工程简介
---
由于实验室硬件环境默认stm32f405rgt6，且同一赛季的keil工程需求都相对固定，一份通用的keil工程就足以支撑绝大部分车组和其它相关的软件开发工作。

同时，为了统一和规范实验室嵌入式软件架构，适配SRML，形成一套统一的代码编写标准，也顺便简化每次重新配置新工程的繁琐操作，加快开发效率，所以就有了这份[模板工程](https://git.scutbot.cn/Embedded/Simple_Template.git)。

[SRML与模板工程](https://scutrobotlab.feishu.cn/wiki/BW21wuJQxidV7rkr67gcKIpknje?from=from_copylink)

模板工程原生适配stm32f405rgt6的HAL环境，也可支持以下芯片（GD32同型号芯片也适用）：
* STM32F103xx
* STM32F405xx
* STM32F407xx
* STM32H723xx
* STM32H750xx

**需要注意的是，因为使用模板工程直接省去了大部分stm32工程配置操作，导致甚至没有stm32基础也能直接上手机器人的代码编写、驱动和控制，即降低了stm32能力需求，这就间接弱化了模板工程使用者的stm32甚至嵌入式开发能力，这是模板工程对电控组不利的方面。所以要求电控组每一个使用模板工程的成员都会自己配置一份一样的工程，保证自己脱离模板工程后也依旧能编写嵌入式代码，保持stm32基础。**



# 2️⃣如何用Template配置一个属于自己的新项目？
---
**在SRML ver3.0的模板工程中，编译器请选择版本为v6！！！**
基于模板工程Template配置自己的工程，只需以下步骤：
- 文件夹名的更改：
  - `Template` -> `xxx`
  - `Template.ioc` -> `xxx.ioc`
  - `MDK-ARM\Template.uvprojx` -> `MDK-ARM\xxx.uvprojx`
  - 打开一次`MDK-ARM\xxx.uvprojx`，让其自动生成`xxx.uvoptx`文件（不执行这一步，CubeMX将无法生成代码！！！）
  - 用VsCode打开`xxx.uvprojx`将所有的Template替换为xxx  


# 3️⃣Clone模板工程并拉取SRML的代码
---
SRML，即实验室机器人软件开发栈的中间件库，是有自己的独立仓库的，仓库链接➡️https://git.scutbot.cn/Embedded/SRML.git
，在模板工程中是作为模板工程仓库的子仓库，所以clone模板工程到本地时，SRML代码不会被拉取下来，是个空文件夹，需要手动拉取子仓库。方法如下：

- 方式一：

    ```bash
    git clone https://git.scutbot.cn/Embedded/Simple_Template.git
    cd .\Simple_Template\   #切换到克隆下来的仓库文件夹路径中
    git submodule update --init --recursive
    ```

- 方式二：

    ```bash
    git clone https://git.scutbot.cn/Embedded/Simple_Template.git
    cd .\Simple_Template\   #切换到克隆下来的仓库文件夹路径中
    git submodule init
    git submodule update
    ```

- 方式三（自动克隆子模块）：

    ```bash
     git clone --recurse-submodules https://git.scutbot.cn/Embedded/Simple_Template.git
    ```

    `--recurse-submodules`代表递归初始化并克隆子模块。


拉取子模块后请务必检查子模块是否处于目标分支和目标commit上。
>23赛季模板工程（分支：master）适配的SRML版本为v2（分支：master），编译环境ARM Compiler V5.06

>24赛季模板工程（分支：SRML_v3.0）适配的SRML版本为v3（分支：23-new-dev），编译环境ARM Compiler V6.16


# 4️⃣模板工程使用规范
---
模板工程的USP目录即用户代码区，用来存放模板工程使用者自己写的代码，同时，按照代码分层规范将用户从上到下代码分为Application、Middlewares、Drivers三个层，具体分层思想，请移步[代码分层与模块化](https://scutrobotlab.feishu.cn/wiki/wikcnoVXmC5f3HhpeFyz2Bs6jFb)。

现在具体来看看模板工程的USP目录：

```
USP
│  app.h            //没什么作用，大概率为了方便程序从c环境跳到C++环境而设置的过渡文件
│  internal.h       //主要 extern 全局变量和函数等，用于声明或引用用户代码全局需要经常使用的变量等，大部分自己写的代码文件都会包含该头文件
│  srml_config.h    //srml配置文件，选择启用哪些srml模块，不被启用的模块不会参与到编译中
│
├─Application
|      System_Config.cpp          //机器人系统初始化（配置）文件，决定用户代码是否能正常启动
│      System_DataPool.cpp        //数据池，用于存放各种数据和全局变量，强烈建议全局变量、数组、各种数据参数（比如PID参数）都放这里，实现参数与代码分离
│      Service_Device.cpp         //跑各种设备的FreeRTOS任务，如电机任务，DR16任务，IMU任务等  
│      Service_Communication.cpp  //跑通信相关的FreeRTOS任务，如串口收发任务、can收发任务等
│      Service_Debug.cpp          //跑调试相关任务，调试机器人时用，如使用上位机，打印日志等
│      
├─Drivers
│      drv_example.md             //空文件，由于空路径在git中是被忽略的，所以为了远程仓库也能显示出Drivers路径，放了这个文件，可删
│
└─Middlewares
    └─Systemview                  //为使用Systemvies软件服务，用于可视化监测FreeRTOS任务调度情况
```

上面都是Application层的代码，主要跑机器人的上层逻辑，调用的是Middlewares层的接口，也可直接调用Drivers层的。注意，这里的Middlewares不仅仅是USP里的，是包括SRML/Middlewares、RTOS等所在的中间层，Drivers同理。

USP/Middlewares存放自己写的或移植的中间件、各种模块等，如gimbal模块、chassis模块、robotic_arm(机械臂)模块等，对接的是Drivers层。

USP/Drivers存放自己写的底层驱动代码，主要面向直接使用HAL接口甚至更底层的代码，一些兵种特有的外设，不通用的或srml没有的底层驱动模块，都可以放这里。

自己封装的模块或驱动，如果测试完备，运行稳定，通用性强，将鼓励并入到SRML库中，为后辈留下自己的财富



# 5️⃣模板工程解读
---
以下是模板工程的使用注意事项

## 1. 通过`srml_config.h`来启用/关闭SRML中的功能模块

自己新建工程移植srml时，需要将.\SRML\​路径下的srml配置模板文件`srml_config_template.h`复制到自己的工程.\USP\路径中，并改名为`srml_config.h`。
在模板工程中，`srml_config.h`已经放在了对应路径内。

在`srml_config.h`中为每个功能模块配置了`#define USE_SRML_XXX `，通过配置其为0或1选择是否启用

```c++
#define USE_SRML_CAN                      1
#define USE_SRML_TIMER                    1
#define USE_SRML_UART                     1
......
```

在`SRML.h`中包含了`srml_config.h`，通过宏定义来决定是否包含某个模块的头文件，如：

```c++
#if USE_SRML_CAN
  #include "Drivers/Components/drv_can.h"
#endif
......
```

而在SRML的c和cpp文件中，均有以下同一格式，使该模块只有在启用了才参与到编译

```
#include "srml_config.h"
#if USE_SRML_XXX
......
#endif /* USE_SRML_XXX */
```

## 2. SRML定时器配置

>**SRML的PID、微分计算等等与时间相关的功能均要用`drv_timer`模块，在23赛季中出现过因为SRML定时器配置不正确导致PID计算异常的情况出现，因此请后人多多留意自己的SRML定时器是否有配置错误的情况。**

SRML的TIMER模块使用需要配置TIM的分频数，使其CNT的自增周期为1us（即自增频率为1MHz，若TIM总线频率为84MHz，则需要配置分频数为84-1，将定时器频率降为1Mhz），而ARR设置为65535，并启用更新中断**（中断优先级为0，不受RTOS管辖）**，还要在CubeMX的`Project Manager`中的`Advanced Settings`右侧，有一个`Register CallBack`，将`TIM`设置为`ENABLE`。

​	关于`Register CallBack`功能的详解请前往飞书文档[‬⁢‬‍﻿⁡‌‍﻿‬⁣⁤﻿⁤‌‬‍‍‬‌⁢‌⁤‌⁡⁤﻿﻿⁣⁡‬‬⁣﻿‬‍‬‬⁡﻿中断回调函数重定向功能启用 - 飞书云文档 (feishu.cn)](https://scutrobotlab.feishu.cn/wiki/YbvfwSQJdiaejvkVmxicuOY2nmd?table=tblLO19c5hJMKmts&view=vewsxFI2yp)

​	正确配置后，在`System_Config.cpp`中的函数`System_Device_Init`使用以下代码，则SRML定时器功能就会被正确启用。

```c++
void System_Device_Init(void)
{
    ......
    Timer_Init(&htim4, USE_MODULE_DELAY);
    SRML_Timer::getMicroTick_regist(Get_SystemTimer);
    ......
}
```
当然，以上过程模板工程也已配置好。


## 3. 串口初始化与收发逻辑

在`srml_config.h`中有以下宏定义，用于配置串口接收的缓冲器是用heap_4内存管理的malloc申请，还是直接使用静态数组。

```c++
 /**选择Uart的缓冲区是否使用malloc申请，若选择0则会根据UARTx_RX_BUFFER_SIZE在drv_uart.cpp内创建静态数组
 * 使用FreeRTOS的heap4替换了malloc后才能使用该功能，否则可能会出错
 * 考虑可能在裸机上跑，因此添加创建静态数组的选项
 * 出于空间节省考虑，当SRML_UARTBUFF_MALLOC为0时，请把没有使用的串口的UARTx_RX_BUFFER_SIZE定义为0
 * SRML_UARTBUFF_MALLOC为1时，缓冲区大小只跟Uart_Init的传参相关
 * 若没有定义SRML_UARTBUFF_MALLOC，效果等同于SRML_UARTBUFF_MALLOC为1
*/
#define SRML_UARTBUFF_MALLOC 1  

#define UART1_RX_BUFFER_SIZE 128
#define UART2_RX_BUFFER_SIZE 128
#define UART3_RX_BUFFER_SIZE 128
#define UART4_RX_BUFFER_SIZE 128
#define UART5_RX_BUFFER_SIZE 128
#define UART6_RX_BUFFER_SIZE 256
......
```

该宏为0和1时的串口初始化函数是不同的，如果选用静态数组作为缓冲区，则`Uart_Init`不用传入缓冲区大小，缓冲区大小将由`srml_config.h`中的`UARTx_RX_BUFFER_SIZE`决定，且无论是否使用`Uart_Init`，该空间将会被一直占用。而如果使用malloc创建缓冲区，则需要给`Uart_Init`传入缓冲区大小，缓冲区会在使用`Uart_Init`时才会开始占用空间。

而`Uart_Init`具有`Uart_Init_DMA`和`Uart_Init_Normal_IT`两种使用形式，使用的场景就像函数名一般，为带DMA串口的初始化、使用普通中断的串口初始化。

并且除了`Uart_Init`函数外，串口库剩下所有函数均只用传入串口号，提高用户代码与HAL库的隔离性。

```c++
#if SRML_UARTBUFF_MALLOC
  void Uart_Init(UART_HandleTypeDef *huart, uint32_t length, User_Uart_Callback fun, UART_Receive_Type receive_type);
  #define Uart_Init_DMA(huart, length, fun) Uart_Init(huart, length, fun, DMA_IT);
  #define Uart_Init_Normal_IT(huart, length, fun) Uart_Init(huart, length, fun, Normal_IT);
#else
  void Uart_Init(UART_HandleTypeDef *huart, User_Uart_Callback fun, UART_Receive_Type receive_type);
  #define Uart_Init_DMA(huart, fun) Uart_Init(huart, fun, DMA_IT);
  #define Uart_Init_Normal_IT(huart, fun) Uart_Init(huart, fun, Normal_IT);
#endif
```

以`SRML_UARTBUFF_MALLOC`为1时为例，在`System_Device_Init`中传入串口句柄、缓冲区大小、用户回调函数的函数指针，即可成功启用SRML的串口相关功能。

```c++
void System_Device_Init(void)
{
	......
  	Uart_Init_DMA(&huart1, UART1_RX_BUFFER_SIZE, UART1_RxCpltCallback);
  	Uart_Init_DMA(&huart3, UART3_RX_BUFFER_SIZE, UART3_RxCpltCallback);
  	Uart_Init_DMA(&huart4, UART4_RX_BUFFER_SIZE, UART4_RxCpltCallback);
  	Uart_Init_DMA(&huart5, UART5_RX_BUFFER_SIZE, UART5_RxCpltCallback);
	......
}

```

**如果没有开启串口回调函数的重定向功能**，需要在`stm32f4xx_it.c`中的每个串口中断服务函数`USARTx_IRQHandler()`中使用函数`Uart_Receive_Handler(x)`，启用的x表示触发中断的串口外设号，比如`USART1_IRQHandler`是串口1的中断，则x为1。在该函数内部会进行是否是IDLE中断的判断、DMA接收数据的长度计算、用户回调函数的调用等等。  
**注意！！！`Uart_Receive_Handler`必须在`HAL_UART_IRQHandler`前面使用，库使用了`HAL_UARTEx_ReceiveToIdle_DMA`，会导致在`HAL_UART_IRQHandler`内会把IDLE标志位清除，导致在`HAL_UART_IRQHandler`后的代码无法判断此次中断是IDLE中断。**
```c++
void USART6_IRQHandler(void)
{
  /* USER CODE BEGIN USART6_IRQn 0 */
  Uart_Receive_Handler(6);
  /* USER CODE END USART6_IRQn 0 */
  HAL_UART_IRQHandler(&huart6);
  /* USER CODE BEGIN USART6_IRQn 1 */

  /* USER CODE END USART6_IRQn 1 */
}
```

若在CubeMX内开启了串口回调函数的重定向功能，则不需要手动添加`Uart_Receive_Handler`，就像Timer库一样，在`Uart_Init`内部已经完成了串口回调函数的替换。同时，`Uart_Receive_Handler`将会被定义为空实现。

对于串口的用户回调函数，使用了模板的形式减少重复代码的编写，在该函数内，触发中断的的串口号通过模板参数来决定，接收数据的地址与长度在调用函数时传入。将串口号、数据地址、数据长度存进`USART_COB`中通过队列`USART_RxPort`发送到负责串口接收数据处理的任务中。

```c++
template <uint8_t uart_id>
uint32_t User_UART_RxCpltCallback(uint8_t *Recv_Data, uint16_t ReceiveLen);

User_Uart_Callback UART1_RxCpltCallback = User_UART_RxCpltCallback<1>;
User_Uart_Callback UART2_RxCpltCallback = User_UART_RxCpltCallback<2>;
User_Uart_Callback UART3_RxCpltCallback = User_UART_RxCpltCallback<3>;
User_Uart_Callback UART4_RxCpltCallback = User_UART_RxCpltCallback<4>;
User_Uart_Callback UART5_RxCpltCallback = User_UART_RxCpltCallback<5>;
User_Uart_Callback UART6_RxCpltCallback = User_UART_RxCpltCallback<6>;
/**
 * @brief  Callback function in USART Interrupt
 * @param  Recv_Data		接收数组
 *	@param	ReceiveLen	接收数据长度
 * @return None.
 */
template <uint8_t uart_id>
uint32_t User_UART_RxCpltCallback(uint8_t *Recv_Data, uint16_t ReceiveLen)
{
  static USART_COB Usart_RxCOB;
  static BaseType_t xHigherPriorityTaskWoken;
  // Send To UART Receive Queue
  if (USART_RxPort != NULL)
  {
    Usart_RxCOB.port_num = uart_id;
    Usart_RxCOB.len = ReceiveLen;
    Usart_RxCOB.address = Recv_Data;
    xQueueSendFromISR(USART_RxPort, &Usart_RxCOB, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }
  return 0;
}
```

​	任务`Task_UsartReceive`在队列`USART_RxPort`有数据之后马上开始运行，用户在`switch`内添加对于某个串口发来的数据的处理代码即可。

```c++
void Task_UsartReceive(void *arg)
{
  /* Cache for Task */
  USART_COB Usart_RxCOB;
  /* Pre-Load for task */
  /* Infinite loop */
  for (;;)
  {
    /* Usart Receive Port*/
    if (xQueueReceive(USART_RxPort, &Usart_RxCOB, portMAX_DELAY) == pdPASS)
    {
      /* User Code Begin Here -------------------------------*/
      switch (Usart_RxCOB.port_num)
      {
      case 1:
        break;
      case 2:
        break;
      case 3:
        break;
      case 4
        break;
      case 5:
        break;
      case 6:
        break;
      default:
        break;
      }
      /* User Code End Here ---------------------------------*/
    }
  }
}
```

​	而关于串口发送，用户将串口号、数据地址、数据长度存进`USART_COB`中通过队列`USART_TxPort`发送，任务`Task_UsartTransmit`在接收到队列后就会调用SRML的串口发送API进行数据发送。之所以使用队列发送给任务`Task_UsartTransmit`，而不是之间使用SRML的API发送，主要是为了隔离性考虑，将所有串口发送集中在一个地方，方便日后改动串口发送的代码等等。

​	这种隔离性在初探H7系列芯片时，为SRML适配H7提供了很大助力。

```c++
void Task_UsartTransmit(void *arg)
{
  /* Cache for Task */
  static USART_COB Usart_TxCOB;
  /* Pre-Load for task */
  /* Infinite loop */
  for (;;)
  {
    /* Usart Receive Port*/
    if (xQueueReceive(USART_TxPort, &Usart_TxCOB, portMAX_DELAY) == pdPASS)
    {
      /* User Code Begin Here -------------------------------*/
      switch (Usart_TxCOB.port_num)
      {
      default:
        break;
      }
      /* User Code End Here ---------------------------------*/
      SRML_UART_Transmit_DMA(&Usart_TxCOB);
    }
  }
}
```

## 4. CAN初始化与收发逻辑

​	CAN的初始化与收发与串口库异曲同工，除了初始化函数外均使用外设号来代替HAL库句柄，数据的收发逻辑也与串口一致，不再过多讲述。
