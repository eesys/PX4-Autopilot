# 路线图

每一步只做一件事：先读这一步列出的上游文件，再写能测的最小行为，最后在这里记下保住了什么、先放下了什么。下一步另开一轮。

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

先放下：多实例话题、`o_queue > 1`、`SubscriptionCallback` / `WorkItem`、`/obj` 设备节点、`.msg` 代码生成、原子操作和节点锁。闭环先放在一个线程里。

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

## 3. 固定步长的单线程调度

模块按固定顺序 `Run(dt)`。对照 `mc_att_control` 里的 `SubscriptionCallbackWorkItem`：上游是姿态发布把控制器唤醒。

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

位置环、EKF2、摇杆、参数、NuttX、驱动、Gazebo 锁步排在这之后。
