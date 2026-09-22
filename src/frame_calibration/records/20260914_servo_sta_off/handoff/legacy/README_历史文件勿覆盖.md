# 历史文件：只作来源追溯，不能作为最新现场源码

- `trackA_servo_20260909_original.zip`：原始上传字节未改；含原 candidate、原 GUI review、原公共执行器与依赖，缺最新 20ms/25ms/VAJ3 field wrappers。
- `README_from_original_Servo_package.md`：从上面原 ZIP 提取的说明。
- `Servo_真机控制与网络长尾排查完整归档_20260911.md`：旧归档原样保留。其阶段结论、网络恢复方式、待跑状态已被本包新主文档明确修订。

原 ZIP 声明 SOFTWARE_CANDIDATE / MOTION_BLOCKED，并明确限定 GUI、场景/持物碰撞、真实 Servo 时序资格。原件里存在 PASS 字样不表示所有资格均 PASS。不要把历史说明里的命令作为本次授权。

旧 MD 把简单 ifconfig down 当作保持的 STA_OFF 状态，并有较强因果断言；后续实际发现 STA 会重新出现。新文档采用“观测时点/未连续监测/因果仍有限”表述，并记录 UCI 新实验。旧 MD 的 powersave 与完整恢复描述也要结合新的变更清单核验。
