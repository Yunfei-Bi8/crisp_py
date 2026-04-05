# Ridgeback 机器人启动说明整理

本文档根据以下代码库中的现有文档与脚本整理：

- `/home/yunfei/crisp_py`
- `/home/yunfei/clearpath_remote_ws`
- `/home/yunfei/clearpath_robot_ws`
- `/home/yunfei/clearpath_simulation`

重点回答两个问题：

1. 这些代码库里有没有关于如何开启/启动 Ridgeback 机器人的说明
2. 如果有，原文是什么，以及应该如何实际使用

---

## 结论

找到了，而且最关键的说明主要集中在以下文件：

- `/home/yunfei/clearpath_remote_ws/docs/operation.md`
- `/home/yunfei/clearpath_remote_ws/docs/robot-software.md`
- `/home/yunfei/clearpath_remote_ws/docs/modifications.md`
- `/home/yunfei/clearpath_remote_ws/src/tum09_ridgeback/tum09_custom/scripts/headless_startup.py`
- `/home/yunfei/crisp_py/examples/ridgeback/README.md`

其中：

- `operation.md` 给出了最直接的人工开机、解除急停、启动机械臂、进入 ROS 控制的操作步骤
- `robot-software.md` 解释了机器人上电后，哪些 systemd 服务会自动启动
- `modifications.md` 说明了当前 `headless: true` 实际上并没有被 Clearpath 生成器真正启用
- `headless_startup.py` 给出了“理论上的无触摸屏启动流程”的脚本实现
- `crisp_py/examples/ridgeback/README.md` 说明了运行 `RidgebackRobot` 相关脚本前，机器人必须已经上电并且 bringup 正在运行

---

## 原文摘录

以下是代码库中与“如何开启/启动 Ridgeback 机器人”最相关的原文。

### 1. Ridgeback 底盘上电

来源：`/home/yunfei/clearpath_remote_ws/docs/operation.md`

```md
## Starting the Robot

1. Press the power button at the base of the robot
2. Wait for it to boot — it will first show solid red, then flashing red
```

### 2. 急停解除流程

来源：`/home/yunfei/clearpath_remote_ws/docs/operation.md`

```md
## E-Stop Procedure

The robot will not move until all e-stops are released and the remote e-stop is armed.

1. Pull out / disable all e-stop buttons on the Ridgeback base
2. Take the Autec remote e-stop
3. Pull out / disable the e-stop button on the remote
4. Hold the **start button** on the remote until the LED lights up — it should blink green fast
5. Press the **start button** once — the LED should blink green slowly — the remote e-stop is now active
6. Press the **E-stop reset** button at the back of the Ridgeback base

The robot should now show solid red at the back and white at the front — it is ready to operate.
```

### 3. UR10e 机械臂上电

来源：`/home/yunfei/clearpath_remote_ws/docs/operation.md`

```md
## UR10e Arm

### Powering On

1. Press the button labeled **"UR Power"** on the top of the plate above the touchpad
2. Wait until the touchpad shows the arm has fully booted up
3. Turn on the arm via the touchpad
```

### 4. 机械臂进入 ROS 控制模式

来源：`/home/yunfei/clearpath_remote_ws/docs/operation.md`

```md
### Operating Modes

**ROS Control** — the arm ships with a single program saved on it. Press the
**Play** button on the touchpad to run it. This enables the arm to be controlled
via ROS.
```

### 5. 外部控制的两种方式

来源：`/home/yunfei/clearpath_remote_ws/docs/operation.md`

```md
There are two ways to enable external (ROS2) control of the arm:

**Option 1 — Headless mode (full remote control)**

The driver takes full control of the arm including power on/off. No interaction
with the touchpad is needed after initial setup.

**Option 2 — External Control URCap (currently used)**

1. Power on the arm via the touchpad
2. Press **Play** on the touchpad to run the saved External Control program
3. The `clearpath-manipulators.service` connects to the arm and takes over
```

### 6. 机器人上电后的软件启动链路

来源：`/home/yunfei/clearpath_remote_ws/docs/robot-software.md`

```md
## Boot Sequence Step by Step

When the robot powers on:

1. Linux boots, systemd starts
2. **`clearpath-robot.service`** starts
3. **`clearpath-robot-generate`** runs first
4. **`clearpath-robot-check`** runs
5. `clearpath-robot.service` is now marked **active**
6. All sub-services start in parallel
```

### 7. 机器人服务管理命令

来源：`/home/yunfei/clearpath_remote_ws/docs/robot-software.md`

```bash
# Check status of all clearpath services
systemctl status clearpath-*.service

# Restart everything (re-runs generator from robot.yaml)
sudo systemctl restart clearpath-robot.service

# Restart just the arm (faster, no re-generation)
sudo systemctl restart clearpath-manipulators.service

# Restart just the base
sudo systemctl restart clearpath-platform.service
```

### 8. 关于 headless 模式的现实情况

来源：`/home/yunfei/clearpath_remote_ws/docs/modifications.md`

```md
`headless: true` was added ... with the intention of enabling full remote control ...

However, investigation showed that the Clearpath generator **silently ignores**
this parameter ... The driver launches in non-headless mode regardless of this setting.
```

### 9. `crisp_py` 侧对启动状态的要求

来源：`/home/yunfei/crisp_py/examples/ridgeback/README.md`

```md
## Prerequisites

1. Robot is powered on and bringup is running
2. Dev machine is connected to the robot network (Ethernet or WiFi)
```

---

## 中文详细说明

### 一、这套系统现在到底是怎么“开机可用”的

按目前仓库中的文档和实现，Ridgeback 机器人可用的真实流程不是“完全无屏自动启动”，而是：

1. 底盘上电
2. 解除所有急停
3. 启动 UR10e 机械臂
4. 在机械臂触摸屏上运行 External Control 程序
5. 让机器人上的 Clearpath 服务接管底盘和机械臂
6. 开发机接入机器人网络，通过 ROS2 与它通信

这里最重要的一点是：

- 虽然 `robot.yaml` 里写了 `headless: true`
- 但文档明确说明 Clearpath 的生成器当前会忽略这个参数
- 所以你不能把它当成“机器人上电后机械臂会自动无屏启动”

换句话说，当前实际使用时，机械臂仍然需要走触摸屏上的启动和 `Play` 流程。

---

## 二、实际操作步骤

### 第 1 步：给 Ridgeback 底盘上电

根据原文：

```md
1. Press the power button at the base of the robot
2. Wait for it to boot — it will first show solid red, then flashing red
```

中文解释：

1. 按下 Ridgeback 底盘上的电源按钮
2. 等待系统启动
3. 启动过程中，灯光状态应先表现为常亮红色，再变成闪烁红色

这一步主要是让机器人本体电脑、底盘相关电子系统和 Clearpath 的系统服务开始启动。

---

### 第 2 步：解除急停，准备允许运动

根据原文，机器人在所有急停都释放之前不会运动。

原文步骤是：

```md
1. Pull out / disable all e-stop buttons on the Ridgeback base
2. Take the Autec remote e-stop
3. Pull out / disable the e-stop button on the remote
4. Hold the **start button** on the remote until the LED lights up — it should blink green fast
5. Press the **start button** once — the LED should blink green slowly — the remote e-stop is now active
6. Press the **E-stop reset** button at the back of the Ridgeback base
```

中文解释：

1. 检查底盘本体上的所有急停按钮，确保都已经弹起、解除
2. 拿起 Autec 遥控急停器
3. 把遥控器上的急停按钮也解除
4. 长按遥控器的 `start` 键，直到 LED 亮起，并且快速绿色闪烁
5. 再按一次 `start` 键，此时 LED 应变成缓慢绿色闪烁，表示遥控急停已经激活
6. 按底盘后部的 `E-stop reset` 按钮

完成后，文档说：

```md
The robot should now show solid red at the back and white at the front — it is ready to operate.
```

也就是：

- 机器人后部显示红灯
- 前部显示白灯
- 表示底盘已经允许进入可操作状态

如果这一步没做对，后面即使 ROS2 连接正常，机器人通常也不会真正运动。

---

### 第 3 步：启动 UR10e 机械臂

原文：

```md
1. Press the button labeled **"UR Power"** on the top of the plate above the touchpad
2. Wait until the touchpad shows the arm has fully booted up
3. Turn on the arm via the touchpad
```

中文解释：

1. 找到触摸屏上方那块板子上的 `UR Power` 按钮
2. 按下后等待 UR10e 控制器完全启动
3. 等触摸屏界面显示机械臂已经完成启动后，再在触摸屏上执行机械臂开机

这一步的重点是：Ridgeback 底盘和 UR10e 机械臂是两个层级的系统。底盘上电不代表机械臂已经进入 ROS 可控状态，机械臂还要单独启动。

---

### 第 4 步：让机械臂进入 ROS 控制

当前文档明确写的是 `External Control URCap (currently used)`，也就是“目前实际使用”的方法不是完全 headless，而是触摸屏上的 External Control 程序。

原文：

```md
1. Power on the arm via the touchpad
2. Press **Play** on the touchpad to run the saved External Control program
3. The `clearpath-manipulators.service` connects to the arm and takes over
```

中文解释：

1. 先在触摸屏上把机械臂正常开机
2. 再按触摸屏上的 `Play`
3. 这个 `Play` 启动的是机械臂里已经保存好的 External Control 程序
4. 当这个程序运行后，机器人上的 `clearpath-manipulators.service` 才能接管机械臂，并通过 ROS2 驱动它

如果你只做了机械臂上电，但没有按 `Play` 运行这个程序，那么 ROS2 侧通常不会真正拿到控制权。

---

### 第 5 步：理解机器人上的 bringup 是否已经自动运行

从 `robot-software.md` 的描述来看，机器人一旦上电并正常启动系统，Clearpath 的主服务会自动拉起各个子服务。

启动链路是：

1. Linux 启动
2. `clearpath-robot.service` 启动
3. 它先运行生成器和检查逻辑
4. 然后并行拉起多个子服务

这些子服务包括：

- `clearpath-platform.service`
- `clearpath-manipulators.service`
- `clearpath-sensors.service`
- `clearpath-discovery.service`
- `clearpath-vcan.service`

这意味着在正常情况下：

- 你不需要自己手工写一长串 `ros2 launch ...`
- 机器人本机在上电完成后，应该已经由 systemd 把主要 bringup 带起来了

如果你怀疑服务没起来，可以在机器人电脑上检查：

```bash
systemctl status clearpath-*.service
```

如果需要重启：

重启整套系统：

```bash
sudo systemctl restart clearpath-robot.service
```

只重启机械臂相关：

```bash
sudo systemctl restart clearpath-manipulators.service
```

只重启底盘相关：

```bash
sudo systemctl restart clearpath-platform.service
```

---

### 第 6 步：开发机连上机器人网络

`crisp_py` 和 `clearpath_remote_ws` 的文档都说明了一个前提：

- 机器人要已经上电并且 bringup 正在运行
- 你的开发机还必须连接到机器人网络

网络文档里推荐使用以太网，机器人主机地址为：

- `192.168.131.1`

如果你用有线网络，开发机通常应配置成同一网段，例如：

- `192.168.131.10`
- 子网掩码 `255.255.255.0`
- 网关 `192.168.131.1`

开发机上可以用：

```bash
ssh robot@192.168.131.1
```

进入机器人本机。

---

### 第 7 步：在开发机上配置 ROS2 环境

在 `/home/yunfei/clearpath_remote_ws` 里已经有给开发机准备的环境脚本和说明。

最直接的方式：

```bash
cd /home/yunfei/clearpath_remote_ws
pixi shell
ros2 daemon stop && ros2 daemon start
ros2 topic list
```

这样做的目的，是把开发机的 ROS2 环境指向机器人的 Discovery Server。

`tools/setup_robot_env.sh` 里写得很清楚，核心环境变量包括：

- `ROS_DOMAIN_ID=0`
- `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`
- `ROS_DISCOVERY_SERVER=192.168.131.1:11811;`（有线网络时）
- `ROS_SUPER_CLIENT=True`

如果这些变量没配好，开发机上即使装了 ROS2，也未必能看到机器人的 topics、services 和 actions。

---

### 第 8 步：验证 bringup 已经可用

最简单的验证方式之一，是在开发机上执行：

```bash
ros2 topic list
```

如果你已经正确接入网络、配置好环境，并且机器人本机服务已经起来，你应该能看到大量来自机器人系统的 ROS2 topic。

另一种更贴近你当前项目的验证方式，是直接运行 `crisp_py` 里的 Ridgeback 示例。

根据 `/home/yunfei/crisp_py/examples/ridgeback/README.md`：

```md
1. Robot is powered on and bringup is running
2. Dev machine is connected to the robot network (Ethernet or WiFi)
```

也就是说，只要机器人已经处于前面说的“开机+bringup 正常”的状态，你就可以在开发机上运行：

```bash
cd /home/yunfei/crisp_py
pixi run --environment jazzy python3 examples/ridgeback/00_check_state.py
```

这通常是一个很好的只读检查入口，因为它先验证连接和状态读取，而不是一上来就执行明显运动。

---

## 三、关于 headless 启动脚本的说明

仓库里还有一个脚本：

- `/home/yunfei/clearpath_remote_ws/src/tum09_ridgeback/tum09_custom/scripts/headless_startup.py`

这个脚本表达的设计思路是：

1. 检查 UR 机械臂是否处于 Remote Control 模式
2. 清除安全错误、保护停机、保护弹窗
3. 通过 Dashboard Server 执行 `power on`
4. 执行 `brake release`
5. 加载并运行 `external_control.urp`

脚本原文中对这个流程的总结是：

```py
Full headless startup sequence:
  1. Verify remote control mode
  2. Clear any safety faults / protective stops
  3. Power on the robot
  4. Release brakes
  5. Load and start the External Control program
```

但要注意，这个脚本只能说明“仓库里有人设计过如何做无屏启动”，并不等于当前整套 Clearpath bringup 已经默认按这个模式工作。

相反，`modifications.md` 已经明确说明：

- `headless: true` 当前被生成器忽略
- 所以系统现实上仍按非 headless 模式运行

也就是说：

- 这个脚本是一个已有的辅助工具或实验性实现
- 不是当前你最应该依赖的标准开机流程
- 当前最可靠的做法仍然是人工完成底盘上电、急停解除、机械臂上电、触摸屏 `Play`

---

## 四、推荐你现在实际采用的启动流程

如果你的目标是“把 RidgebackRobot 真正启动到可以被 `crisp_py` 使用”，建议按下面流程执行：

1. 按底盘电源键，等待底盘启动完成
2. 释放底盘和遥控器上的所有急停
3. 用 Autec 遥控器完成 `start` 激活
4. 按底盘后部 `E-stop reset`
5. 按 `UR Power` 启动机械臂
6. 等触摸屏完全启动后，在触摸屏上开机械臂
7. 在触摸屏上按 `Play`，运行已保存的 External Control 程序
8. 确认机器人上的 Clearpath 服务已经运行
9. 开发机通过以太网或 WiFi 连接机器人网络
10. 在开发机中进入 `clearpath_remote_ws` 的 `pixi shell`
11. 执行 `ros2 daemon stop && ros2 daemon start`
12. 用 `ros2 topic list` 检查能否看到机器人 ROS2 图
13. 再进入 `crisp_py` 跑 `examples/ridgeback/00_check_state.py`

这样做最符合当前仓库里的真实实现状态，也最不容易踩到“文档里写了 headless，但实际没生效”的坑。

---

## 五、最终结论

这些代码库里确实有关于如何开启这个 Ridgeback 机器人的说明，而且说明是比较完整的。

最关键的事实是：

- 底盘和 ROS bringup 会随着机器上电由 systemd 自动启动
- 但 UR10e 机械臂当前并不是完全 headless 自动接管
- 现实使用中，仍然要人工完成急停解除、机械臂上电和触摸屏 `Play`
- 在这之后，`clearpath-manipulators.service` 才会接手机械臂
- 开发机还必须正确连接机器人网络并设置 ROS2 环境，`crisp_py` 的 `RidgebackRobot` 才能正常使用

