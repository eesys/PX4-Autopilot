# px4-from-scratch

学习用的 PX4 重写。上游源码在旁边的 `PX4-Autopilot` 检出（`main`），这棵树从孤立分支 `learn/from-scratch` 开始，不带上游历史。

目标是一条多旋翼姿态闭环：话题总线、真值姿态、最小解锁、姿态环、角速度环、Quad-X 混控、简易机体。步骤和每一刀对照的上游文件写在 [ROADMAP.md](ROADMAP.md)。

这是学习代码，不上飞机。

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build
ctest --test-dir build --output-on-failure
```
