# PilotSDK 使能异常 / varget.log / TCP 8080 排查记录

日期：2026-09-01

## 现象

真机运行时偶发：

```text
armInitData: 0
link: (0, True)
enable status: (0, False)
servo: (0, False)
```

但机器人登录返回中已经有：

```text
"svo_on": true
```

成功运行时会额外出现：

```text
文件上传成功: uploads/varget.log
```

## 根因

容器中残留了多个旧的 PilotSDK / Python 机器人程序，并且它们虽然处于暂停状态（`STAT=T`），仍同时监听：

```text
0.0.0.0:8080
```

发现的旧进程包括：

- `test_move_worlds.py`
- `execute_worlds_10x.py`
- `execute_trajectory.py`

这会干扰当前 PilotSDK 的 `varget.log` 回传 / 状态初始化，使当前进程读到：

```text
enable=False
servo=False
```

因此这不是机器人本体真的没有使能，也不是简单增加 `sleep(3)` 能解决的问题。

## 验证

清理旧的 8080 监听进程后：

```text
文件上传成功: uploads/varget.log
enable=True
servo=True
```

诊断脚本连续轮询 10 秒全部保持 `True`。

随后正式真机轨迹连续运行 **5 次成功**。

## 后续遇到同类问题先检查

```bash
netstat -ltnp 2>/dev/null | grep ':8080'
```

检查旧机器人 Python 进程：

```bash
ps -ef | grep -E 'python3.*(test_move_worlds|execute_worlds|execute_trajectory|execute_servo)' | grep -v grep
```

注意：

- 不要长期用 `Ctrl+Z` 挂起 PilotSDK 机器人程序。
- `Ctrl+Z` 只暂停程序，进程和监听端口仍可能保留。
- 清理旧进程前先确认它们确实不再需要。
- 不需要为了这个问题修改 `sdk_session.py` 增加固定 `sleep`。

## 本次验证轨迹

```text
traj_bottle_taught_tcp_home_start_20260831_RUN.json
```

这是 Debian 现场使用的文件名；其内容 SHA 与工作区现行发布文件
`traj_bottle_taught_tcp_20260831_RUN.json` 相同。

SHA256：

```text
cce2a83fae21468f80f0aad487087b61650eac6f92dea65f9560091d0239ad2b
```

执行器：

```text
execute_servo_grasp.py
脚本 SHA 前16位: bc05bd76c2584dc6
夹爪力度参数: 8000
```

## 诊断资料归档目录

```text
debug_archive/arm_enable_varget_8080_20260901/
```

这是 Debian 现场目录，其中保存了状态同步诊断脚本、运行日志和本次问题记录；
该目录目前尚未复制回工作1，也不包含在本发布包中。

## 证据边界

本文用于记录使能异常的根因、现场处理方法和操作者报告的五次连续成功。
当前工作区未收到这五次运行从启动到正常退出的完整日志，也未收到 Debian
现场 `execute_servo_grasp.py` 文件，因此不能仅凭本文把当前发布组合标为
`REAL_VERIFIED`。若需要正式升级验收状态，至少补一份完整运行日志和当次执行器文件即可。
