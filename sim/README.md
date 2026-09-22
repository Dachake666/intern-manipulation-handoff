# Mac / PyBullet 仿真端

源码在 ../src，保持现有模块布局。运行类型由实际行为决定；Debian上的PyBullet报告同样归仿真。

## 环境与第一次检查

首版在已有Mac Python环境做离线验证；环境实测版本及结果见根verification.json。
新机器可先建立独立环境，安装 src/pick_place_coord/mpc_experiment/requirements.txt、
src/requirements/planning-py310.lock.txt 及src/requirements/vision-py310.lock.txt所列依赖，
注意后两份是历史Python3.10/Linux声明，不是Mac通用锁文件；实测Mac版本见 src/requirements/mac-tested.txt。

```bash
# 从交接根目录执行；python需已装相应依赖
python3 tools/offline_checks.py
cd src
XIFENG_ALLOW_REAL_MOTION=0 python3 -B pick_place_coord/verify_left_track_c.py
```

Track C核对应得到0.4度1633帧、0.2度3232帧及固定点流摘要；只计算，不连接设备。

## 保留的功能

| 功能 | 源码/数据入口 | 边界 |
|---|---|---|
| 模型、IK、规划 | pick_place_coord/pick_place_coord.py、left_arm_ik.py、xifeng_pb.py、XF0112048/ | 共享限位、标定和坐标角色必须继续沿用 |
| 双次抓放 | tasks/task_2pairs_table72.json、trajectories/verified/traj_multi_2grasp_20260804_REALVERIFIED.json | 历史文件名REALVERIFIED不替代缺失的完整原日志 |
| 桌面/瓶子/Servo | gen_tabletop_hybrid_candidate.py、gen_taught_tcp_candidate.py、相配任务/候选/审核 | 当前Mac候选与Debian实跑副本不一定相同 |
| MPC仿真 | pick_place_coord/mpc_experiment/{controller.py,run_experiment.py,results/} | 六组仿真，零重力/估计惯量；不提供完整碰撞或真机资格 |
| 双臂 | pick_place_coord/dual_arm_demo/ | 1624帧，碰撞资格REJECT，右臂工具和现场限制未闭合 |
| 视觉与任务 | vision/vision_system、robot_mission、frame_calibration/analysis/fit_eye_to_hand.py | 离线观测与规划接口，手眼质量门待修 |

MPC批量复跑会写results；建议先用tools/export_version.py B1恢复到新目录再实验，保留本包基准数据。
GUI需人工观看，本轮离线测试不产生GUI人工PASS；启动方式见各模块原README，个人绝对路径改为恢复目录的src。
唯一模型位于src/XF0112048，环境/工具与坐标配置位于src/arm_profiles*、schemas和calib_common.py。
