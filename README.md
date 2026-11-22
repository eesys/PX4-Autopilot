# px4-from-scratch

用小步重写来学习旁边 `PX4-Autopilot` 检出里的 `main`。这棵树从孤立分支 `learn/from-scratch` 开始，不带上游历史。迭代结束时，这里的源码要和 `main` 完全一致。

中间步骤可以先写最小行为，省略的部分记成欠账，后面补回。当前先走一条多旋翼姿态闭环：话题总线、真值姿态、最小解锁、姿态环、角速度环、Quad-X 混控、简易机体。步骤和每一刀对照的上游文件写在 [ROADMAP.md](ROADMAP.md)。

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build
ctest --test-dir build --output-on-failure
```
