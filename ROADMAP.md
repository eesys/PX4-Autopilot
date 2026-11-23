# 路线图

目的是学会 `main` 的源码。每一步只做一件事：先读这一步列出的上游文件，再写能测的最小行为，最后在这里记下保住了什么、先放下了什么。先放下的部分是欠账，后面的步骤要补回。全部补完时，这棵树的源码与 `main` 完全一致。下一步另开一轮。

上游检出：`../PX4-Autopilot`（`main`）。

## 1. 最小 uORB（这一刀）

对照：

- `platforms/common/uORB/uORB.h`
- `platforms/common/uORB/Publication.hpp`
- `platforms/common/uORB/Subscription.hpp`
- `platforms/common/uORB/uORBDeviceNode.hpp`（`copy`，`o_queue == 1`）
- `platforms/common/uORB/uORBDeviceNode.cpp`（`write`、`get_initial_generation`）
- `platforms/common/uORB/uORBManager.cpp`（`orb_data_copy`）

保住的行为：

- 每种消息一个进程内共享节点，对应上游的 `orb_metadata` / `ORB_ID`
- `publish` 按字节拷贝结构体，generation 是单调的发布次数
- 队列深度为 1：落后的订阅者读到最新一份，漏掉的次数留在 generation 差值里
- 每个 `Subscription` 记自己的 generation
- `update()` 只在有新发布时拷贝；`copy()` 在已发布过后总能拿到最新一份，并把订阅者追到当前 generation
- 后创建的订阅者仍能读到已经发布的最新一份；有数据时初始 generation 是当前值减 1

代码只放在 `main` 已有的路径里：`uORB.h`、`Publication.hpp`、`Subscription.hpp`、`uORBDeviceNode.hpp`。类名是 `DeviceNode`。测试在 `platforms/common/uORB/uORB_tests/`，入口是 `uorb_tests_main`。

先放下：

- 多实例话题、`o_queue > 1`、`SubscriptionCallback` / `WorkItem`、`/obj` 设备节点、`.msg` 代码生成、原子操作和节点锁。闭环先放在一个线程里
- `DeviceNode` 不继承 `cdev::CDev`，`publish_data` 代替带 `filp` 的 `write`
- 每种 C++ 类型一个节点，代替 `orb_metadata` / `ORB_ID`
- 同目录其余文件还没建。`write` 和 `orb_data_copy` 的行为先写在上面的头文件里
- `uORB_tests_main.cpp` 末尾的主机 `main` 只转发到 `uorb_tests_main`
- 根 `CMakeLists.txt` 先把 `PX4_TESTING` 打开；`add_subdirectory(platforms)` 没有 `EXCLUDE_FROM_ALL`；用 `include(CTest)` 和 `add_test` 让 README 里的 `ctest` 能跑
- `cmake/px4_add_module.cmake` 先把 `MAIN` 编成可执行文件。`main` 上它编的是名为 `MODULE` 的静态库
- `platforms/common/CMakeLists.txt` 和 `platforms/common/uORB/CMakeLists.txt` 只有进入子目录的那几行，库本身还没建

## 2. 闭环用的结构体

对照：

- `msg/versioned/VehicleAttitude.msg`（Hamilton 四元数，FRD 到 NED）
- `msg/versioned/VehicleAttitudeSetpoint.msg`
- `msg/versioned/VehicleAngularVelocity.msg`
- `msg/VehicleRatesSetpoint.msg` 或对应的 versioned 文件
- `msg/VehicleTorqueSetpoint.msg`
- `msg/VehicleThrustSetpoint.msg` 或对应的 versioned 文件
- `msg/versioned/ActuatorMotors.msg`
- `msg/versioned/VehicleStatus.msg`（`ARMING_STATE_DISARMED = 1`，`ARMING_STATE_ARMED = 2`）
- `msg/VehicleControlMode.msg`

保住的行为：

- 这九个消息在 `main` 的路径上。头文件生成到 `build/uORB/topics`，包含方式是 `<uORB/topics/*.h>`，别名是 `px4::msg::...`
- `vehicle_attitude.q`：Hamilton 四元数，顺序 `(w, x, y, z)`，表示 FRD 机体到 NED
- `vehicle_attitude_setpoint`：`q_d`，以及 `thrust_body`（多旋翼里 z 轴负向是油门需求）
- `vehicle_angular_velocity.xyz`：FRD 角速度，单位 rad/s
- `vehicle_rates_setpoint`：`roll`、`pitch`、`yaw`，以及 `thrust_body`
- `vehicle_torque_setpoint.xyz`、`vehicle_thrust_setpoint.xyz`
- `actuator_motors`：`NUM_CONTROLS = 12`，`control` 里的 NaN 表示上锁停转
- `vehicle_status`：`ARMING_STATE_DISARMED = 1`，`ARMING_STATE_ARMED = 2`
- `vehicle_control_mode`：`flag_armed`、`flag_control_attitude_enabled`、`flag_control_rates_enabled`、`flag_control_allocation_enabled`
- 这些结构体经现有的 `Publication` / `Subscription` 按字节拷贝

先放下：

- `VehicleAttitude`：`MESSAGE_VERSION`、`timestamp_sample`、`delta_q_reset`、`quat_reset_counter`、`TOPICS`
- `VehicleAttitudeSetpoint`：`MESSAGE_VERSION`、`yaw_sp_move_rate`、`TOPICS`
- `VehicleAngularVelocity`：`MESSAGE_VERSION`、`timestamp_sample`、`xyz_derivative`、`TOPICS`
- `VehicleRatesSetpoint`：`MESSAGE_VERSION`、`reset_integral`。`main` 上这个文件在 `msg/versioned/`
- `VehicleTorqueSetpoint`、`VehicleThrustSetpoint`：`timestamp_sample`、`TOPICS`
- `ActuatorMotors`：`MESSAGE_VERSION`、`timestamp_sample`、`reversible_flags`、`ACTUATOR_FUNCTION_MOTOR1`
- `VehicleStatus`：`MESSAGE_VERSION`，以及 `timestamp` 和 `arming_state` 以外的字段
- `VehicleControlMode`：上面四个标志以外的标志
- `Tools/msg/px_generate_uorb_topic_files.py` 不跑 EmPy 模板：字段按文件顺序，不加 padding，不生成 `ORB_DECLARE`、`msg.cpp`、`uORBTopics`、IDL
- `TOPICS` 别名还没有。一种 C++ 类型仍是一个节点
- `msg/CMakeLists.txt` 只列出这九个文件，不建 `uorb_msgs`

## 3. 固定步长的单线程调度

对照：

- `platforms/common/include/px4_platform_common/px4_work_queue/WorkItem.hpp`（`Run`）
- `platforms/common/include/px4_platform_common/px4_work_queue/WorkQueue.hpp`（`Run` 按队列调用）
- `platforms/common/uORB/SubscriptionCallback.hpp`（`SubscriptionCallbackWorkItem::call` 里 `ScheduleNow`）
- `src/modules/mc_att_control/mc_att_control.hpp`（`_vehicle_attitude_sub` 唤醒姿态环）

保住的行为：

- `WorkItem::Run(float dt)` 是模块的一步
- `WorkQueue::Add` 按调用顺序记下模块
- `WorkQueue::Run(dt)` 在调用它的线程上按这个顺序各调用一次，每个模块拿到同一次传入的 `dt`
- 后 `Add` 的模块从下一次 `Run` 起排在末尾
- 测试在 `platforms/common/px4_work_queue/test/`，入口是 `wqueue_test_main`

先放下：

- 上游的 `Run()` 没有 `dt` 参数。模块自己用 `hrt` 算周期
- `SubscriptionCallbackWorkItem` 还没建。姿态发布不会 `ScheduleNow()`，没有工作队列线程、锁、信号量
- 没有 `ScheduleNow` / `ScheduleClear`、按名字排序、多条 `WorkQueue`、`hrt`、lockstep
- `WorkItem.cpp`、`WorkQueue.cpp` 还没建。行为写在上面两个头文件里
- `wqueue_test.h` 还不是 `WQueueTest`。`wqueue_main.cpp` 末尾的主机 `main` 只转发到 `wqueue_test_main`
- `platforms/common/CMakeLists.txt` 仍没有 `px4_platform` 库。`px4_work_queue/CMakeLists.txt` 还没编那个库
- 根 `CMakeLists.txt` 用 `add_test` 跑 `wqueue_test`。`main` 不这么做

## 4. 刚体机体

积分力矩和推力，发布姿态和角速度。开环测试：常值力矩让角速度增大。先不接 Gazebo。

## 5. 姿态环

对照 `src/modules/mc_att_control/AttitudeControl/AttitudeControl.hpp`。四元数误差到机体角速度设定，先只留比例项。动笔前读 `docs/en/concept/navigation_terminology.md`。

## 6. 角速度环

对照 `src/modules/mc_rate_control`。角速度误差到力矩。

## 7. Quad-X 混控

对照 `src/modules/control_allocator/VehicleActuatorEffectiveness/ActuatorEffectivenessMultirotor.hpp`。四个电机。纯滚转力矩让对角电机反向。

## 8. 最小 commander

对照 `src/modules/commander/Commander.hpp` 的 `isArmed()`。两态：上锁、解锁。上锁时电机指令为 0。

## 9. 闭环验收

初始有滚转倾角，姿态设定是水平。若干秒后倾角和角速度回到附近。到这里第一阶段结束。

第一阶段到此为止。位置环、EKF2、摇杆、参数、NuttX、驱动、Gazebo 锁步，以及前面各步先放下的部分，继续按同样的方式补回，直到源码与 `main` 完全一致。
