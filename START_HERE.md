# 机器人工作交接 · 第一版

2026-09-22。范围：Mac/PyBullet仿真与Debian/SDK真机；旧workspace和灵巧手已排除。
这是一份可恢复的开发交接，不是对所有轨迹和环境的新真机验收。

## 先从这里读

1. [仿真端](sim/README.md)：模型、Track C双次抓放、MPC与双臂离线入口。
2. [真机端](robot/README.md)：收到的各条现场线、证据边界与版本恢复方法。
3. [版本](docs/VERSIONS.md)：7类基础节点、3类缺陷及补充历史；可离线恢复。
4. [待办](docs/KNOWN_ISSUES.md)：真实缺口与修复顺序。

## 当前进展

| 工作线 | 当前已知结果 |
|---|---|
| Track A固定场景 | 9/8版共5份完整成功JSON：1次5%、4次10%，精确文件组合与收尾记录匹配 |
| Track C双次抓放 | 133运动点＋4夹爪事件，Mac/收到轨迹一致；五轮成功只有汇总，证据LIMITED |
| 瓶子示教/HOME | 238/275运动点两核心轨迹保留；五轮运动原日志缺失，证据LIMITED |
| Servo/VAJ3/MPC | 保留74份现代JSON、现场源码及逐轮映射；固定6 Shadow＋4 Active可复算，历史核心/样条哈希缺口仍在 |
| Mac MPC/双臂 | MPC六组离线实验；双臂1624帧演示碰撞资格REJECT，均不当真机授权 |
| 视觉/手眼 | 必要C++/Python核心与39组历史样本已交接；高残差质量门和接口适配待修，未闭合视觉抓放 |

## 包内布局

- src/：Mac当前开发源码及对应SDK消费者，保留原相对布局，使导入、模型和测试路径可用。
- sim/、robot/：两端阅读入口；共享配置、schemas和唯一URDF/STL在src内，不为整理复制安全常量。
- evidence/：Debian脱敏记录和MPC复算工具；src/frame_calibration/records为原Mac证据副本。
- extensions/debian_vision/：必要ROS/C++视觉源码、接口、配置及模型；不是已验证安装环境。
- data/handeye/：39组旧手眼配对数据，按原四组保留，不自动批准外参。
- sdk/：CPython3.10/Linux x86_64厂商wheel。Mac不可安装；视觉历史构建则为ARM/Humble。
- docs/：版本、问题、环境参考和机器清单。旧内层README保留当时语境，当前状态以这五份入口为准。
- .git/：新建的精简交付历史，未携带原仓库对象；39组大样本只随总包保存一份，Git记录身份清单。所有新提交日期是整理日，原来源见SOURCE_MAP与选择清单。

## 拿到包先做

```bash
python3 tools/verify_handoff.py
git status --short
git tag --list 'handoff/*'
```

然后按仿真说明建立环境，跑明确的离线检查。实机IP仍未现场确认；任何硬件连接/使能/运动另做现场检查。
实际本次检查结果见 [verification.json](verification.json)，不是把历史测试数字冒充本次结果。

原始workspace.zip SHA：426a10feb929fdf05eeaae51cb57f4d0dcff0a1d810ecd9ff39cfa687fe70842。
原件由用户保管，曾位于Downloads；日常源码、关键日志与样本已在本包内，不依赖该路径运行。
大PCAP、崩溃转储、安装缓存和可重建生成物留在原件中；必要取用位置见docs选择清单。
