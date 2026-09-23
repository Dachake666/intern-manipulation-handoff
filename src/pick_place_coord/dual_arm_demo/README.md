# 双臂同步仿真

`demo.py` 实现两臂同步接近、夹取、抬起、分区转运、放置和返回的运动学动画。输入为 `dual_arm_simulation.json`，证据为 `validation_report.json`。当前1624帧计划的完整工具碰撞资格为 **REJECT**，没有双臂运动执行入口。

## 播放与检查

以下从仓库根、仿真环境执行：

```bash
(cd src && XIFENG_ALLOW_REAL_MOTION=0 python3 -B pick_place_coord/dual_arm_demo/demo.py --gui --exit-after-replay)
```

蓝色为左臂、橙色为右臂。`--loop` 循环、`--speed 0.5` 半速；循环重置不是规划好的真实恢复路线。重建会改变计划与报告，应先导出 B1 到 `.runtime/` 新目录，再在该副本用 `--build`，不要覆盖参考结果。

## 场景与依赖

- 父参考为 Track A SDKALIGNED 轨迹，SHA `e5fd11010815020dcc67deebfe774381b391e5462486eb67f8e5ee9150ec9c59`；只保留抓取前导，后续路径重新规划。
- 坐标为 `URDF_WORLD_SIM_ONLY`；桌高、瓶尺寸和放置目标是仿真假设，不是当前 SDK 世界坐标。
- 右臂关于中面对称，用完整姿态 IK 求解；关节取反仅作初值。共同时间轴验证仿真调度，不证明 SDK 并发或同步到达。
- 夹持物刚性跟随末端，释放落下为显示假设，无摩擦/夹持力/落瓶稳定性验收。
- 使用共享 `arm_profiles` 和只读厂商模型，绑定代码、配置、求解器和父轨迹 SHA。依赖改变时旧报告应失效。

## 当前限制

跨臂、桌面、退离回程的采样检查不能消除腕部网格和工具/物体交叠。几何代理仍不完全匹配真实开闭夹爪，右臂真实限位/TCP、工位配准和 SDK 并发语义未确认。碰撞 REJECT 不应通过豁免非相邻链接、删除工具几何或放宽关节限位消除。

`test_demo.py` 检查契约、同步、父参考、场景与依赖漂移；测试通过不授予真机资格。`src/frame_calibration/robot_side/precheck_dual_arm.py` 是单独的设备只读工具，由发布器 DP 模式生成；限位读取 PASS 不是完整运动资格。现场使用前按根 `robot/README.md` 确认设备行为。
