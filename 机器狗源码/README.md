# NextPal

NextPal 是一个基于 STM32F103 的四足机器人控制项目，集成 FreeRTOS 多任务调度、舵机 PWM 驱动、逆运动学步态控制、蓝牙通信、超声波避障、OLED 显示以及手势/外部指令控制等功能。机器人采用 12 个舵机构成四肢运动系统，通过状态机切换站立、前进、后退、左右平移、左右转向和踏步等动作。

项目采用模块化结构：状态机负责解析控制指令和切换运动模式，步态模块生成脚尖轨迹，逆运动学模块将足端坐标转换为各关节角度，舵机控制模块完成 PWM 输出和平滑处理。当前版本重点优化了四足运动的动作表现，降低默认步幅和抬脚高度，加入更柔和的步态相位曲线，并对舵机输出进行滤波，使前进、后退和右转等动作更加清楚、稳定。

本工程适合作为嵌入式机器人、STM32 外设驱动、FreeRTOS 实践和四足步态控制的学习案例，也可继续扩展 IMU 自稳、视觉识别、远程控制和更复杂的运动策略。

---

## 📥 下载方法

### 方式一：直接下载（推荐）

点击页面右上角的 **`code`** 按钮，再点击 **`Download ZIP`** 即可下载整个项目；也可以 **Fork** 到自己仓库后再编辑。

![点击 code 后选择 Download ZIP](https://jacakebum.github.io/NextPalCode/images/step1-download.png)

### 方式二：下载以往版本代码

即使项目以后更新，也可以通过提交哈希（commit SHA）下载指定版本。公开仓库任何人都能下载；私有仓库需要读取权限，无需管理员权限。

#### 历史版本记录

| 提交备注 | 完整提交哈希 | 下载 |
| --- | --- | --- |
| 11月24上传天问语音包和Windows控制台代码 | `961386128185def7ecbcb2782fa148faea756736` | [下载该版本 ZIP](https://github.com/Jacakebum/NextPalCode/archive/961386128185def7ecbcb2782fa148faea756736.zip) |

#### 方法一：直接下载 ZIP（无需安装 Git）

1. 点击上表中的“下载该版本 ZIP”。
2. 下载完成后，右键 ZIP 文件，选择“全部解压缩”。
3. 打开解压后的文件夹，即可查看这一版代码。

该链接固定指向上表中的版本，不会随着后续提交变化；前提是仓库及该提交仍可访问。

#### 方法二：使用 Git 下载指定版本

先安装 Git，然后在准备存放代码的目录中打开 PowerShell 或终端，依次执行：

```powershell
git clone https://github.com/Jacakebum/NextPalCode.git NextPalCode-1124
cd NextPalCode-1124
git switch -c restore-1124 961386128185def7ecbcb2782fa148faea756736
```

第一行把仓库下载到新文件夹 `NextPalCode-1124`；第二行进入该文件夹；第三行创建 `restore-1124` 本地分支，并切换到指定版本，可以在此基础上继续修改代码。这些命令不会修改 GitHub 上的仓库。

执行前请确认目标目录中没有同名的 `NextPalCode-1124` 文件夹；如有，请换一个文件夹名称，并同步修改 `cd` 命令。

下载其他历史版本时，将最后一行的哈希替换为对应提交的完整哈希，并相应调整文件夹名和分支名。建议保存仓库地址、完整哈希和提交备注；ZIP 解压包不包含 Git 历史。

### 提示：GitHub 加载不出来时

如果网页打不开或下载慢，安装 **WattToolkit** 加速器（免费好用），或者使用梯子 / VPN（收费）。

![WattToolkit 加速器](https://jacakebum.github.io/NextPalCode/images/step2-accelerator.png)

打开 Watt Toolkit，进入「**网络加速**」，勾选 **GitHub**，再点击「**一键加速**」即可：

![在网络加速中勾选 GitHub 并一键加速](https://jacakebum.github.io/NextPalCode/images/step3-install.jpg)
