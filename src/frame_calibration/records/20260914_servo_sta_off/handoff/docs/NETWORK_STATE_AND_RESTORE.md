# 网络改动、影响与恢复核对

**状态截止：最近一次用户输出，不是实时监测。本次打包没有登录路由器、修改网络或完成恢复。**

## 1. 三个环境不能混用

| 提示符 | 环境 | 本项目涉及对象 |
|---|---|---|
| `dev@debian` | Debian 宿主机 | `enp3s0`、`wlp2s0`、NetworkManager、sing-box |
| `dev@humble` | ROS2 Humble 容器 | SDK、现场 Servo Python、JSON 报告 |
| `root@GL-SFT1200` | GL 路由器 | UCI、`wlan-sta0`、`wlan1`、`wlan0` |

不能在路由器上套宿主机 `enp3s0`、GNU timeout/sudo 命令。此次打包附两个不同环境的**只读** snapshot 脚本。

## 2. 真实拓扑与互联网影响

```text
Debian enp3s0 (.244) ──网线── GL-SFT1200 (.1)
                               │
                               ├─ phy1 / wlan1：GL-SFT1200-71f-5G ──机器人 .148
                               ├─ phy1 / wlan-sta0：STA ──Ultirobotics_5G（禁用对象）
                               └─ phy0 / wlan0：GL-SFT1200-71f（2.4 GHz AP）
```

关闭的是上游客户端，不是 GL 5G AP。手机仍可连接 GL 5G、访问本地允许访问的机器人/局域网。若该 STA 是唯一互联网入口，**所有依赖此 GL 出口的客户——5G、2.4G、有线——都可能失去互联网**，不只 5G 手机。若有另一条正常出口且路由已正确配置，才可能继续上网。

此前“STA down 后还能聊天”没有核验实际公网出口，且后来发现 STA 重回 UP；不能据此断言另有独立 WAN。sing-box TUN 也不是无物理网络的互联网来源。

2.4 GHz 客户端不等于 2.4 GHz 上游；单把手机换到 GL 的 2.4 GHz SSID，并不保证其互联网流量不再通过 phy1 的 STA。即使改用有线 WAN，多个 5GHz 客户端仍共享 `wlan1` 的空口；不能称绝对互不干扰。

## 3. 变更台账

| 项目 | 原始/较早记录 | 测试状态 | 恢复状态 |
|---|---|---|---|
| 路由器 runtime STA | 上游关联中 | `ifconfig wlan-sta0 down`；后来又 UP | 单独 up/down 不足以管理后续 UCI 状态 |
| `wireless.sta.disabled` | 用户输出中原本无此 option | 添加 `'1'`，`wifi reload`；未见 commit | 尚未有撤销和重新关联验收 |
| Debian Wi-Fi | GL 5G 已连，.185 | 最后观测断开 | 尚未有恢复连接验收 |
| Debian Ethernet profile | manual `172.24.1.8/24`，gateway 空，metric `-1` | DHCP、清空手动地址/gateway/DNS、metric `50`，获得 `.244/24` | 原值仅部分已知，未恢复 |
| Debian Wi-Fi powersave | 原值未知 | 旧 MD/回复称改为 `2` | 实际当前值和原值待核验 |
| EEE | 查询到 enabled / inactive | 未见成功修改证据 | 不擅自设置 |
| singbox_tun / policy route | 本来存在 | 未见本轮修改记录 | 不停代理、不删规则 |
| SSH known_hosts | 路由器 host key 未知 | 接受并写入 RSA 指纹 | 非网络连通性修改；不清空其他 host keys |
| 软件安装 | 工具缺失 | 容器装 iproute2/tcpdump；Debian 装 ethtool | 不为“恢复”盲目卸载；容器 tcpdump 配置失败需单独检查 |

## 4. 已执行的 UCI 禁用（历史记录，不要求现在再执行）

```sh
# root@GL-SFT1200；仅在无机器人任务且协调联网用户后
uci set wireless.sta.disabled='1'
wifi reload
sleep 5
```

用户随后确认：`iw dev` 不再列出 wlan-sta0；wlan1 留在 channel149、80MHz；机器人关联仍在，单站报告 TX130Mbps、RX78Mbps（20MHz）。这些是采样，不是吞吐基准或持续监视。

UCI set 的 pending change 和实际 reload 生效不是一回事；“不 commit”也不表示“不影响当前运行”。官方 UCI 文档说明 set/delete 等改动先进入暂存，commit 才写入配置，revert 可按 option 撤销。若其他管理服务另行保存或重启，必须重新检查。[W1]

## 5. 恢复最近这轮 STA 实验

**不要在机器人执行/持物中运行 wifi reload。**它可能打断无线客户端和互联网。先在本地有线管理终端协调停机、记录现状；不要自动全局 `uci revert wireless`。

只读查看：

```sh
# root@GL-SFT1200
uci -q get wireless.sta.disabled
uci changes wireless | grep 'wireless.sta.disabled'
iw dev
iwinfo wlan1 assoclist
```

以下仅适用于：原 disabled option 缺省、确实只有本次未 commit 的单项变更，期间没有他人保存覆盖。

```sh
# 撤销这一项 pending change；不要撤销整个 wireless 配置
uci revert wireless.sta.disabled
wifi reload
sleep 5
uci -q get wireless.sta.disabled || echo 'disabled option absent'
iw dev
iw dev wlan-sta0 link
iwinfo wlan1 assoclist
```

预期 STA 再次出现并关联 `Ultirobotics_5G`，机器人仍在 wlan1。若 `revert` 提示无 pending change、恢复后仍 disabled、或者上游未关联，先停下读持久配置/管理器状态，不能靠连续 reload 或 commit 猜修。历史对话给过 `uci delete wireless.sta.disabled`；它也需要区分原缺省/已持久化情况，不能无条件套用。

回 Debian 验证：

```bash
ip route get 192.168.8.148
ping -c 5 192.168.8.1
ping -c 5 192.168.8.148
```

另需验证真正公网和受影响客户端；能 ping 机器人不等于能上互联网。恢复上游后时序可能退回较差状态，但没有充分证据断言每次必定重现秒级 stall。

## 6. 恢复 Debian Wi-Fi 与更早的有线基线

最近实验前 Wi-Fi 恢复可用已存连接名：

```bash
# dev@debian，本地终端；不要在运行中的控制任务期间切链路
nmcli connection up 'GL-SFT1200-71f-5G'
nmcli device status
ip -br addr
ip route get 192.168.8.148
```

**回到最初接线前**还需恢复 `有线连接 1` 的已知字段，并把线缆接回相应 172.24.1.x 网络；这不同于保留当前 `.244` 控制路径。不在当前 GL LAN 上盲目设置原工业网段地址。

```bash
# 仅在明确要回到最初172.24.1.x网络、停止控制并核对接线后执行
sudo nmcli connection modify '有线连接 1' \
  ipv4.method manual ipv4.addresses '172.24.1.8/24' \
  ipv4.gateway '' ipv4.route-metric -1
sudo nmcli connection up '有线连接 1'
```

DNS 原值未提供，以上**只能恢复已知字段**，不构成全配置字节级恢复。回原 Wi-Fi 控机器人时 LOCAL_IP 也要跟随实际路由重新核对，不能继续照抄 `.244`。

powersave 先读：

```bash
nmcli -g 802-11-wireless.powersave connection show 'GL-SFT1200-71f-5G'
```

官方语义：0 使用全局默认，1 不改当前设置，2 关闭省电，3 开启省电。[W2] **原值未知，不能默认设为 0 后宣称恢复。**从原备份/管理员确认原值再单独处理。

## 7. 恢复完成的记录要求

保存恢复前后安全字段快照、实际出口/机器人路由和时间；确认无线用户能否上网、机器人连接是否正常、UCI pending 是否只剩原先内容。当前交接状态写 `RESTORATION_NOT_VERIFIED`，不得写已经全部还原。

[W1]: https://openwrt.org/docs/guide-user/base-system/uci
[W2]: https://www.networkmanager.dev/docs/api/latest/settings-802-11-wireless.html
