# 开发、登记与发布

新负责人在当前 `src/` 开发；历史复现使用独立 `.runtime/` 目录。B1 按 `handoff/current` 标签导出，尚未提交或未更新标签的修改不会进入 B1。冻结代码、轨迹和原始日志保持身份稳定，修改后的实现登记为新版本。

## 新功能或实验

1. 先在 `docs/CONTENTS.md` 找职责模块，复用共享臂配置、坐标契约和安全常量。新算法不能顺带复制另一套 SDK 会话、限位或 Home。
2. 建立开发分支，例如 `git switch -c codex/new-experiment`。实验输出放 `.runtime/实验名/`；需要批跑已有 MPC 时先导出 B1，避免覆盖参考 `results/`。
3. 明确输入、运行入口、输出、适用场景和失败条件。新增候选必须有独立路径，记录源/输入 SHA、参数、依赖、解释器和结果。
4. 运行对应的明确离线测试；新增测试检查有意义的行为和拒绝条件。不要自动广泛发现所有 `test_*.py`，硬件探针必须单独人工使用。
5. 需要进入真机验证时，先完成标定有效性、IK/限位/首点、完整场景/持物/开爪/撤退检查和绑定轨迹 SHA 的人工 GUI PASS，再由现场负责人确认。

## 新运行与版本

- 成功和失败都归档。记录机器人/臂/夹爪身份、现场标定、起点、场景、参数、完整执行器/wrapper/轨迹/动态算法/依赖 SHA 和收尾状态。
- 使用 `src/frame_calibration/robot_side/scrub_log.py` 处理副本，原件不改。终端日志和 JSON 一起保存；不要只留下成功截图或 README。
- 新版本在 `docs/version_refs.json` 登记节点与不可变提交，成果说明同步 `docs/VERSIONS.md`。需要逐轮恢复时，在 `docs/debian_selection.json` 的 `run_bindings` 登记 `report_relative`、`source_relative`、`restore_name`、SHA、节点和未绑定项。
- 不把单次运行完成升级为通用 REALVERIFIED；证据不足使用 LIMITED。未记录的历史算法哈希保持未知，不能从后来同名文件推断。
- 新发布包用 `src/releases/make_release.py` 的明确模式生成。退出默认树的实验先确认有可恢复节点和路径映射，再修改引用与发布清单；不删除原始记录。

## 更新默认文件清单

`docs/DELIVERY_SELECTION.json` 的 `active` 列表是交付允许清单。新增/移动文件时登记相对路径、用途及来源信息，并同步目录说明。退役文件从默认列表退出前必须有历史恢复位置。不要把 `.runtime/`、虚拟环境、缓存、系统元数据或未经审查的文件加入允许清单。

`tools/build_package.py` 拒绝未分类文件、缺失项和源码验证身份漂移。只修改说明也应刷新根 manifest；只修改源码则必须重新记录离线检查。根 `HANDOFF_MANIFEST.json` 和 `SHA256SUMS` 由工具生成，不手写。

## 重新生成完整包

从仓库根执行，使用仿真环境解释器：

```bash
python3 tools/offline_checks.py --record
python3 tools/check_history.py --record
python3 tools/test_package_tools.py
python3 tools/build_package.py --refresh-manifest
python3 tools/build_package.py --check
git diff --check
git status --short
```

离线套件的 `--record` 写入 `verification.json`、`evidence/handoff_checks/offline_tests.log`，并绑定选定 `src/` 文件的 SHA。历史检查写入 `evidence/handoff_checks/history_checks.json`，验证节点、退役原字节与逐轮装配；未绑定依赖仍保持 PARTIAL。`build_package.py --check` 可在提交前执行；只有生成ZIP要求Git干净且current=HEAD。

修改视觉模型定位或工作站配置时，另运行 `python3 tools/test_vision_model_paths.py`，默认报告在 `.runtime/vision_model_checks.json`。该工具需要C++17编译器，仅编译生产代码中的模型路径解析块并测试路径，不编译完整ROS、不加载ONNX推理、不访问相机。需要随包保留新报告时先将其登记进允许清单。以上检查不等于全仓库、SDK安装、人工GUI、完整ROS/C++构建或真机验收。

审阅源码、说明、清单与验证日志后，按项目授权提交明确文件；示例提交信息为“完善抓放候选校验并更新发布清单”。随后在完整仓库中更新当前交付标签并打包：

```bash
# 完成已审阅变更的 git add / git commit 后
git tag -f -a handoff/current -m '当前交付版本'
python3 tools/build_package.py --output ../robot_handoff.zip
```

打包要求独立的 `.git/` 目录（不支持链接式 Git worktree）、工作区干净且 `handoff/current` 指向 HEAD；B2～B7、D1～D3、history、PRE_PRUNE 等历史节点不移动。输出 ZIP 必须位于仓库目录外，旁边会生成 `.sha256`。完整包包含所选文件与 Git 历史；`data/handeye/` 仍随包提供。

在新空目录解压，运行 `python3 tools/verify_handoff.py`、`python3 tools/build_package.py --check`，并抽查所需节点/逐轮恢复。交付对象为完整 ZIP 和校验文件；不能只复制可见源码目录或单独 Git 克隆而遗漏样本。
