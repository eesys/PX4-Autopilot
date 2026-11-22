# 开发指南

这份文档给没有无人机软件背景的人读。它解释飞控在做什么、这棵学习树为什么这样长，以及下一轮该怎么动手。

三份文档的分工：

| 文件 | 作用 |
| --- | --- |
| [README.md](README.md) | 这棵树是什么，以及怎么编译、怎么跑测试 |
| [ROADMAP.md](ROADMAP.md) | 每一轮对照 `main` 的哪些文件、保住了什么、先放下了什么 |
| [GUIDE.md](GUIDE.md) | 背景知识、当前代码怎么读、做下一轮的规矩 |

`README.md`、`ROADMAP.md`、`GUIDE.md` 和 `.cursor/` 是学习笔记，不是 PX4 的源码。迭代结束、这棵树与 `main` 完全一致时，这些笔记要离开源码树。对照上游时不要把它们算进「源码是否一致」。

这是学习代码。不要拿去飞。

## 1. 这棵树在学什么

旁边的 `../PX4-Autopilot` 是 [PX4 Autopilot](https://px4.io) 的检出，停在 `main`。PX4 是开源飞控软件：它跑在飞行控制器上，读传感器，算出每个电机该出多大力，把飞机稳住并飞向目标。

本仓库是同一份 git 仓库里的孤立分支 `learn/from-scratch`，目录和 `PX4-Autopilot` 并列，不带 `main` 的历史。学习方式是小步重写：每一轮只做一件能测的事，代码放进 `main` 已经有的路径。省略的行为记成欠账，后面补回。全部补完时，这里的源码和 `main` 一致。

当前第一阶段只走一条多旋翼姿态闭环，并且先不接仿真器：

```text
话题总线 → 真值姿态 → 最小解锁 → 姿态环 → 角速度环 → Quad-X 混控 → 简易机体
```

「真值姿态」的意思是：这一阶段自己积分出一个姿态发给控制器，先不实现估计器。估计器是后面的 EKF2。

## 2. 飞控在整架飞机里的位置

一架多旋翼上和软件有关的部分可以分成四层。

1. **传感器。** 加速度计和陀螺仪组成 IMU，测量比力和角速度。磁力计、气压计、GNSS 提供方向和位置。这一阶段不读真实传感器。
2. **飞控软件。** 也就是 PX4。它周期性地看「现在的姿态」和「想要的姿态」，算出电机指令。
3. **电机和螺旋桨。** 四个（或更多）电机改变拉力。拉力差产生转动力矩，总拉力抵消重力。
4. **机体。** 力矩改变角速度，角速度积累成姿态。这一阶段用一段数值积分代替真实飞机，也先不接 Gazebo。

飞控里还有一条更长的链路：位置 → 速度 → 姿态 → 角速度 → 电机。第一阶段只做后半段，也就是姿态闭环。位置、摇杆、参数系统、真实操作系统和驱动都排在第 9 步之后。

## 3. 读代码前要有的力学词汇

### 3.1 两个坐标系

PX4 的机体和本地导航用两套右手系。轴的名字要记死，后面每个消息都依赖它们。

**NED（北东地）** 贴在地面上，原点在估计器启动的位置：

- X 指向北
- Y 指向东
- Z 指向地心，也就是向下为正

**FRD（前右下）** 固定在飞机上：

- X 指向机头
- Y 指向右翼
- Z 指向机腹，向下为正

飞机水平、机头朝北时，FRD 和 NED 重合。这和很多机器人框架不一样：那些框架常常把 Z 朝上。PX4 这里 Z 朝下，所以「向上的力」在 Z 轴上是负数。

三个转动的名字：

| 名字 | 绕哪根轴 | 正向用右手定则看到的效果 |
| --- | --- | --- |
| 滚转 roll | 机头 X | 右翼向下 |
| 俯仰 pitch | 右翼 Y | 机头向上 |
| 偏航 yaw | 机腹 Z | 机头向右，从上往下看是顺时针 |

右手定则：拇指指向轴的正方向，四指弯曲的方向就是正转。

航向（heading）是机头在水平面上的指向，从真北顺时针量。PX4 里航向和偏航角通常指同一件事。它不表示飞机正在往哪飞：多旋翼可以头朝北、身子往东平移。第一阶段不涉及位置，只要分清「机头指向」和「角速度」即可。

更细的导航用词在上游 `docs/en/concept/navigation_terminology.md`。第 5 步动笔前要读那一页。

### 3.2 姿态用四元数，不用三张欧拉角

姿态就是「FRD 相对 NED 转了多少」。三个欧拉角（滚转、俯仰、偏航）好直觉，但俯仰接近 ±90° 时会失去一个自由度，插值也不干净。PX4 对外发布的姿态是四元数。

`vehicle_attitude.q` 有四个浮点数，顺序是 `(w, x, y, z)`，用 Hamilton 约定。它表示把 **FRD 机体向量转到 NED** 的旋转。

- `q = (1, 0, 0, 0)` 是单位旋转：机头朝北，右翼朝东，机腹朝下。
- 后面三个分量指出转轴，`w` 和转过的角度有关。读消息时先确认下标 0 是 `w`，不要写成别的顺序。

第 5 步的姿态环把「当前四元数」和「目标四元数」的误差变成机体角速度设定。上游实现对应 ETH 的笔记 *Nonlinear Quadrocopter Attitude Control*（Brescianini、Hehn、D'Andrea，2013）。那一轮先只留比例项。

### 3.3 角速度、力矩、推力

- **角速度** 是飞机正在转动的快慢，单位 rad/s，沿 FRD 三轴。消息是 `vehicle_angular_velocity.xyz`。
- **角速度设定** 是姿态环希望飞机转到的快慢。消息是 `vehicle_rates_setpoint` 的 `roll`、`pitch`、`yaw`，单位同样是 rad/s。
- **力矩设定** 是角速度环希望机体受到的转动力，沿 FRD，在 PX4 里先写成归一化的数，不是牛顿米。消息是 `vehicle_torque_setpoint.xyz`。
- **推力设定** 是归一化的力，范围大约 -1 到 1，沿机体轴。多旋翼的螺旋桨朝上推，而 +Z 朝下，所以油门需求写在 **Z 分量的负数** 上。`thrust_body[0]` 和 `thrust_body[1]` 通常是 0，`thrust_body[2] = -0.5` 表示大约一半向上的油门。这个符号在 `VehicleAttitudeSetpoint.msg` 的注释里。

### 3.4 四个电机怎么产生滚转

Quad-X 是四个电机摆成 X 形：从上看，电机在前左、前右、后左、后右，机臂沿对角线。每个电机的拉力同时影响滚转和俯仰。

纯滚转力矩让每一对对角线上的两个电机反向变化：一侧加力、另一侧减力，总拉力尽量不变，飞机就绕机头轴转。具体系数以第 7 步要读的 `ActuatorEffectivenessMultirotor.hpp` 为准，不要在文档里先发明一套混控。

电机指令在 `actuator_motors.control` 里，最多 12 个数，范围大约 -1 到 1。第一阶段只用前 4 个。`NaN` 表示这个电机上锁、停转。

### 3.5 解锁

上锁（disarmed）时电机必须停。解锁（armed）之后控制器的输出才允许送到电机。上游常量是：

- `ARMING_STATE_DISARMED = 1`
- `ARMING_STATE_ARMED = 2`

判断写在 `Commander.hpp` 的 `isArmed()`：`arming_state == ARMING_STATE_ARMED`。第 8 步只做这两个状态。上锁时电机指令为 0。

`vehicle_control_mode` 里的标志告诉各模块这一拍要不要工作，例如姿态环、角速度环、混控是否启用。第一阶段会用到 `flag_armed`、`flag_control_attitude_enabled`、`flag_control_rates_enabled`、`flag_control_allocation_enabled`。

## 4. 姿态闭环里谁对谁说话

把第 9 步验收时的数据流先看成一条流水线。每一格的输入输出都是 uORB 消息。

```text
简易机体
  发布 vehicle_attitude、vehicle_angular_velocity
        │
        ▼
姿态环
  读当前姿态和 vehicle_attitude_setpoint
  发布 vehicle_rates_setpoint
        │
        ▼
角速度环
  读当前角速度和角速度设定
  发布 vehicle_torque_setpoint、vehicle_thrust_setpoint
        │
        ▼
Quad-X 混控
  发布 actuator_motors
        │
        ▼
简易机体再积分下一拍
```

旁边还有两份状态：

- `vehicle_status`：现在是上锁还是解锁。
- `vehicle_control_mode`：哪些控制器允许跑。

第 9 步的验收标准：飞机一开始有一个滚转倾角，姿态设定是水平。若干秒后倾角和角速度回到 0 附近。

上游真实系统不是这根单线程管子。姿态一发布，`mc_att_control` 里的 `SubscriptionCallbackWorkItem` 就把姿态环唤醒。估计器、指挥官、位置环、驱动各自跑在工作队列上。那些调度要等第一阶段的行为站住，再按 `main` 的原样补回来。

## 5. uORB：飞控内部的话题总线

模块之间不互相调用函数来传姿态。发布方把一份 C 结构体交出去，订阅方稍后拷走。这条总线叫 **uORB**（micro Object Request Broker）。上游头文件里的说法是 lightweight object broker。

这样拆开的原因：姿态环不需要知道姿态是估计器算的还是这一阶段的简易机体算的。它只订阅 `vehicle_attitude`。

### 5.1 现在这棵树上的行为

一种 C++ 类型对应进程里一个 `DeviceNode`。`topic<T>()` 用函数里的 `static` 节点保住它。结构体必须能按字节拷贝（`std::is_trivially_copyable`）。

`Publication<T>::publish` 把结构体 `memcpy` 进节点，并把 `_generation` 加 1。`_generation` 是发布次数，只增不减。

`Subscription<T>` 自己记一份 `_last_generation`。几个订阅者互不影响。

| 调用 | 行为 |
| --- | --- |
| `updated()` | 发布次数比这个订阅者上次看到的更大 |
| `update(dst)` | 有新发布才拷贝，并把订阅者的计数追到当前。没有新发布就返回 false，`dst` 不动 |
| `copy(dst)` | 已经发布过就拷贝当前这一份，并把订阅者的计数追上 |
| `updates_available()` | 节点计数减去订阅者计数，也就是这个订阅者还没读的发布次数 |

队列深度是 1：节点里只留最新一份结构体。订阅者来晚了，读到的是最新值，漏掉的次数留在 generation 的差里。

`test_two_publishes_keep_latest_and_gap` 就是这条约定。订阅者先建好（计数 0），连续发布 `{1}` 和 `{2}`：

- 缓冲区里只剩 `2`
- `updates_available()` 是 2
- `update` 读到 `2`，订阅者的计数变成 2

后创建的订阅者仍能读到已经发布的最新一份。节点有数据时，初始计数是当前 generation 减 1，所以它只看到一次待读更新，看不到更早的历史。

测试里每种情况用一个不同的小结构体，是因为节点按类型共享，而且没有清节点的接口。两个测试如果共用一个类型，会看到彼此留下的 generation。

### 5.2 和 `main` 的对应关系

| 这棵树上的写法 | `main` 上的东西 |
| --- | --- |
| `topic<T>()` 的静态 `DeviceNode` | `orb_metadata` / `ORB_ID`，按消息名而不是按 C++ 类型 |
| `publish_data` | `DeviceNode::write`。上游还要文件描述符，并用原子操作选槽 |
| `copy` | `o_queue == 1` 时的 `DeviceNode::copy` |
| `Subscription::update` / `copy` | `uORBManager::orb_data_copy` 的 `only_if_updated` 为真或为假 |

上游的 `DeviceNode` 还继承字符设备，能做多实例话题、深度大于 1 的队列、回调和锁。这些都记在 `ROADMAP.md` 第 1 步的欠账里。

## 6. 消息：闭环里搬动的结构体

PX4 不手写这些结构体。源码是 `msg/` 下的 `.msg` 文件，构建时生成 C++ 头文件。包含方式是：

```cpp
#include <uORB/topics/vehicle_attitude.h>
```

生成结果在 `build/uORB/topics/`，不进 git。结构体名字是 `vehicle_attitude_s` 这种蛇形加 `_s`。`px4::msg::VehicleAttitude` 是它的别名。

第 2 步留下的字段：

| 消息 | 这一阶段用到的内容 |
| --- | --- |
| `vehicle_attitude` | `timestamp`，`q[4]`（Hamilton，FRD → NED） |
| `vehicle_attitude_setpoint` | `timestamp`，`q_d[4]`，`thrust_body[3]` |
| `vehicle_angular_velocity` | `timestamp`，`xyz[3]`（FRD，rad/s） |
| `vehicle_rates_setpoint` | `timestamp`，`roll` / `pitch` / `yaw`，`thrust_body[3]` |
| `vehicle_torque_setpoint` | `timestamp`，`xyz[3]` |
| `vehicle_thrust_setpoint` | `timestamp`，`xyz[3]`。某一轴是 NaN 时，影响该轴的电机停下 |
| `actuator_motors` | `timestamp`，`NUM_CONTROLS = 12`，`control[12]`。NaN 表示上锁停转 |
| `vehicle_status` | `timestamp`，`arming_state`，以及上面两个解锁常量 |
| `vehicle_control_mode` | `timestamp`，`flag_armed` 和姿态、角速度、混控三个使能标志 |

`timestamp` 是系统启动后的微秒数。上游常用它做时间戳和控制周期。

每个文件里还没写的字段、`TOPICS` 别名，以及生成器还没做的事，以 `ROADMAP.md` 第 2 步的欠账为准。生成器现在只是 `Tools/msg/px_generate_uorb_topic_files.py` 的一个能用的子集：字段按 `.msg` 里的顺序输出，不重排，也不生成 `ORB_DECLARE`。

## 7. `main` 的源码地图

读上游时按这个地图找，不要在学习树里新造目录。

| 路径 | 是什么 |
| --- | --- |
| `platforms/nuttx` | 飞控板上的实时操作系统 NuttX |
| `platforms/posix` | 在电脑上跑的软件在环（SITL） |
| `platforms/qurt`、`platforms/ros2` | 另外两种运行环境 |
| `platforms/common` | 各平台共用的部分。uORB、高精度时间、工作队列、模块基类在这里 |
| `boards/` | 具体飞控板。板子选择自己的 `PX4_PLATFORM` |
| `msg/`、`msg/versioned/` | 话题的字段定义 |
| `src/modules/mc_att_control` | 多旋翼姿态环 |
| `src/modules/mc_rate_control` | 多旋翼角速度环 |
| `src/modules/control_allocator` | 把力矩和推力分配到电机、舵机 |
| `src/modules/commander` | 解锁、飞行模式、失效保护 |
| `src/modules/ekf2` | 状态估计。第一阶段跳过 |
| `cmake/px4_add_module.cmake` | 把一个模块加进构建 |

`platforms` 这个名字指操作系统和运行时，不是某一块板。

## 8. 每一轮怎么做

`ROADMAP.md` 开头的规矩就是工作方式：

1. 只做路线图上的下一步。读它列出的上游文件，再写能测的最小行为。
2. 新代码只能放在 `main` 已经有的路径里。`src/uorb.hpp` 这种上游不存在的文件不要再出现。
3. 可以在已有文件里暂时少写行为。少写的部分必须记成欠账，不能把简化版当成最终设计，也不能改架构的名字。
4. 这一轮的末尾更新 `ROADMAP.md`：保住了什么、放下了什么。
5. 跑下面的构建。下一步另开一轮，不要在同一次改动里顺手做下去。

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build
ctest --test-dir build --output-on-failure
```

提交只在你明确说提交时做。推送也一样。

对照上游用目录比较，不要凭记忆改名字。类名已经有的，跟 `main` 走，例如节点类是 `DeviceNode`。

## 9. 当前树怎么读

第 1 步和第 2 步已经落在树上。从测试读最快。

```text
CMakeLists.txt                         根构建。打开测试，进入 msg/ 和 platforms/
cmake/px4_add_module.cmake             现在把 MAIN 编成可执行文件，并加上头文件搜索路径
msg/**/*.msg                           九个闭环消息
Tools/msg/px_generate_uorb_topic_files.py
                                       把 .msg 写成 build/uORB/topics/*.h
platforms/CMakeLists.txt               进入 common
platforms/common/CMakeLists.txt        不是 io-v2 板就进入 uORB
platforms/common/uORB/CMakeLists.txt   PX4_TESTING 打开时进入测试
platforms/common/uORB/uORB.h           topic<T>()
platforms/common/uORB/uORBDeviceNode.hpp
platforms/common/uORB/Publication.hpp
platforms/common/uORB/Subscription.hpp
platforms/common/uORB/uORB_tests/      uorb_tests 的入口和用例
```

根目录的 `CMakeLists.txt` 和 `px4_add_module.cmake` 为了能在主机上跑测试，比 `main` 少了很多东西。例如 `main` 的 `px4_add_module` 产出的是名为 `MODULE` 的静态库，测试模块也不会单独被 `add_test`。这些差异写在文件注释和路线图里，后面要用上游的构建换掉。

测试入口是 `uorb_tests_main`。文件末尾有一个主机 `main`，只是转发给它，这样这个二进制可以在 PX4 模块系统还没建起来时直接跑。

`uORBTest_UnitTest.cpp` 前六个用例测总线本身：没人发布、发布一次、发布两次仍只留最新值、没有新发布时 `update` 不改写调用方的缓冲区、两个订阅者各记各的计数、晚加入的订阅者只读到最新一份。最后的 `test_closed_loop_messages` 测第 2 步的九个消息：四元数长度和顺序、解锁常量、12 个电机槽、NaN 能原样拷贝，以及推力 Z 分量的负数。

编辑器如果报 `'uORB/topics/....h' file not found`，先确认已经构建过。生成头在 `build/` 里。仓库根的 `.clangd` 让 clangd 读取 `build/compile_commands.json`。红线还在时，在命令面板里执行 **clangd: Restart language server**。`.clangd` 也是编辑器配置，不是 PX4 源码。

## 10. 第一阶段剩下的步骤

每一格都是单独一轮。细节和要读的文件在 `ROADMAP.md`。

| 步 | 要得到的行为 |
| --- | --- |
| 3 | 一个线程里按固定顺序调用各模块的 `Run(dt)`。上游是话题发布唤醒工作项 |
| 4 | 刚体积分。常值力矩让角速度变大。发布姿态和角速度 |
| 5 | 姿态误差的比例项变成角速度设定。动笔前读导航术语 |
| 6 | 角速度误差变成力矩 |
| 7 | Quad-X 混控。纯滚转让对角电机反向 |
| 8 | 上锁 / 解锁。上锁时电机指令为 0 |
| 9 | 有初始滚转、水平设定，倾角和角速度回到 0 附近 |

第 9 步之后第一阶段结束。位置环、EKF2、摇杆、参数、NuttX、驱动、Gazebo 锁步，以及每一轮记下的欠账，继续按同样的方式补，直到源码与 `main` 一致。

## 11. 术语表

| 词 | 含义 |
| --- | --- |
| PX4 | 这套开源飞控软件 |
| SITL | Software In The Loop，飞控跑在电脑上，对着仿真或模型 |
| NuttX | 飞控板上常用的实时操作系统 |
| EKF2 | PX4 的状态估计器，融合 IMU 等传感器。第一阶段跳过 |
| uORB | 进程内的发布/订阅总线 |
| 话题 topic | 一种消息的共享节点。上游用 `ORB_ID` 区分 |
| generation | 该话题已经发布的次数 |
| 队列深度 | 节点保留的历史份数。深度 1 只留最新一份 |
| NED | 北、东、下 |
| FRD | 前、右、下，固定在机体上 |
| Hamilton 四元数 | `(w, x, y, z)`。这里的 `q` 把 FRD 转到 NED |
| 姿态环 | 姿态误差 → 角速度设定。模块 `mc_att_control` |
| 角速度环 | 角速度误差 → 力矩。模块 `mc_rate_control` |
| 混控 / control allocator | 力矩和推力 → 每个电机的指令 |
| Quad-X | 四电机 X 布局 |
| 上锁 / 解锁 | disarmed / armed。上锁时电机停转 |
| 欠账 | 某一轮在已有文件里故意还没写的上游行为 |
| `main` | `../PX4-Autopilot` 的 `main` 分支，学习的对照树 |
