# FOC 工程代码执行流程梳理（从头到尾）

本文档按“上电 → 初始化 → 主循环 → 状态机 → 中断”顺序梳理当前工程的主干逻辑。

---

## 一、上电入口与一次性初始化（main.c）

### 1.1 入口

- **文件**：`Application/.../Src/main.c`
- **入口**：`int main(void)`

### 1.2 初始化顺序（只执行一次）

| 顺序 | 调用 | 作用 |
|------|------|------|
| 1 | `dbg_periph_enable(DBG_TIMER0_HOLD)` / `DBG_TIMER2_HOLD` | 调试时保持 TIMER0/TIMER2 运行 |
| 2 | `systick_config()` | 配置 SysTick 定时器（1ms 节拍、delay、周期任务） |
| 3 | `comm_init()` | 通信（如 USART/DMA）初始化 |
| 4 | `eval_board_init()` | 板级外设：按键、EC11 旋钮、LED、OLED |
| 5 | `uart4_init(2500000)` | UART4 初始化（如编码器/上位机） |

### 1.3 eval_board_init() 内部（gd32_bldc_foc_eval.c）

- `key_init()`：KEY_RUN、KEY_ENTER 的 GPIO 配成输入（轮询，无 EXTI）
- `ec_init()`：EC11 的 A/B 脚配成输入 + EXTI 双边沿，使能 EXTI10_15 中断（旋钮用中断）
- `led_init()`：LED GPIO 输出
- `oled_init()`：OLED（SH1106）I2C 初始化

**说明**：此时 **TIMER0 未启动、ADC 插入通道未触发**，FOC 电流环尚未运行。

---

## 二、主循环 while(1)（main.c）

初始化完成后进入 **死循环**，每轮固定顺序执行：

```
1. motor_running_mode_process()   // 按运行模式配置角度/udq/idq 等
2. [可选] oled_display()          // 当前工程中已注释
3. key_process()                  // 按键轮询，设置 motor.command / 菜单
4. motor_state_machine()          // 状态机：INIT/IDLE/RUNNING/BRAKE/FAULT
5. led_sacn_state()              // LED 按状态闪烁
6. [可选] 其它（如 printf、串口解析等）
→ 回到 1，无限重复
```

- **motor_running_mode_process**：根据 `motor.running_mode` 在“第一次或模式变化”时配置 `rotor_angle.pos_mode`、`motor.udq_mode`、`motor.idqref_mode`、`motor.startup_mode` 等（开环 VF/IF、SMO、AFLUX、编码器、霍尔等）。
- **key_process**：轮询 KEY_RUN、KEY_ENTER，防抖后设置 `motor.command`（MC_START / MC_STOP / MC_BREAK）或确认菜单并改 `motor.running_mode`、`motor.state` 等。
- **motor_state_machine**：根据 `motor.state` 和 `motor.command` 做状态转移并执行启停/刹车/故障处理。
- **led_sacn_state**：根据 `motor.state` 控制 LED 亮灭（INIT/IDLE 闪烁，RUNNING/BRAKE 常亮，FAULT 按故障次数闪）。

---

## 三、电机状态机（main.c :: motor_state_machine）

### 3.1 故障优先

- 若 `motor.fault != FAULT_NONE`，先强制 `motor.state = MC_STATE_FAULT`。

### 3.2 状态与动作

| 状态 | 触发条件 / 动作 |
|------|------------------|
| **MC_STATE_INIT** | 执行一次：`motor_init`、`pwm_current_peripheral_init`、`motor_protect_peripheral_init`、`motor_protect_var_init`，然后 **motor.state = MC_STATE_IDLE**，command 清空。 |
| **MC_STATE_IDLE** | 若 command == MC_DIR_CHANGE：更新方向；若 command == MC_START：初始化速度/转矩/磁链 PID、保护变量、角度模块，**motor.state = MC_STATE_RUNNING**，调用 **motor_start(&motor)**，清 command。 |
| **MC_STATE_RUNNING** | 若 command == MC_STOP：**motor.state = MC_STATE_INIT**，**motor_stop**，延时 100ms，清 command；若 command == MC_BREAK：**motor.state = MC_STATE_BRAKE**，**motor_break**，清 command。 |
| **MC_STATE_BRAKE** | 若 command == MC_BREAK：**motor_stop**，**motor.state = MC_STATE_INIT**，清 command。 |
| **MC_STATE_FAULT** | 先 **motor_stop**；若 `motor.fault == FAULT_NONE` 则 **motor.state = MC_STATE_INIT**。 |

### 3.3 启停与硬件（motor_ctrl_driver.c）

- **motor_start(&motor)** 内部调用 **start_ctrl()**：
  - 使能 TIMER0 的 CH0/CH1/CH2 及互补输出；
  - `timer_enable(TIMER0)`；
  - 使能 ADC 插入通道外部触发（TIMER0 触发）及 EOIC 中断。
- **motor_stop(&motor)** 内部调用 **stop_ctrl()**：
  - 关闭 TIMER0 所有通道输出，`timer_disable(TIMER0)`；
  - 关闭 ADC 插入触发与 EOIC 中断；
  - 顺带关霍尔相关定时器中断。

**结论**：只有进入 **MC_STATE_RUNNING** 并执行 **motor_start** 后，TIMER0 和 ADC 插入转换才会周期触发，FOC 电流环中断才会运行。

---

## 四、按键与旋钮（gd32_bldc_foc_eval.c）

### 4.1 按键（轮询，无中断）

- **key_process()** 在主循环中调用。
- 通过 **key_read_debouncing() → key_read() → gpio_input_bit_get()** 读 KEY_RUN、KEY_ENTER。
- 防抖逻辑内含“等松手”的 `while`，会阻塞主循环直到松键。
- 根据 **motor.state** 分支：
  - **INIT**：仅同步菜单索引到主页。
  - **IDLE**：KEY_RUN → `motor.command = MC_START`；KEY_ENTER → 根据当前菜单项设置 `motor.running_mode` / 方向 / 电流采样 / PWM 模式等，并 **motor.state = MC_STATE_INIT**、进入子菜单。
  - **RUNNING**：KEY_RUN → `motor.command = MC_STOP`；KEY_ENTER → `motor.command = MC_BREAK`。
  - **BRAKE**：KEY_ENTER → `motor.command = MC_BREAK`。
  - **FAULT**：不处理按键。

### 4.2 EC11 旋钮（EXTI 中断）

- **EXTI10_15_IRQHandler**（宏定义为 **EC11_IRQ_HANDLE**）在 EXTI 线 13/14（EC11 A/B）边沿触发。
- 中断里根据 A/B 边沿顺序判定方向，设置 **ec_orientation**（DIRECTION_INCREASE / DIRECTION_DECREASE），然后调用 **ec11_handle()**。
- **ec11_handle()** 根据 **motor.state**：
  - **MC_STATE_INIT**：当前实现为直接 break（若已按前面讨论改为与 IDLE 一致，则也会更新菜单/速度）。
  - **MC_STATE_IDLE**：旋转更新 **func_index**（菜单项），在主页时更新 **motor.speed_ref**。
  - **MC_STATE_RUNNING**：仅更新 **motor.speed_ref**。
  - **MC_STATE_BRAKE**：不处理。
  - **MC_STATE_FAULT**：不处理。

---

## 五、运行模式配置（main.c :: motor_running_mode_process）

- 仅在 **首次进入** 或 **motor.running_mode 变化** 时更新（用 `first_flag` 和 `running_mode_pre` 判断）。
- 根据 **motor.running_mode** 设置：
  - **rotor_angle.pos_mode**：角度来源（ANGLE_CUSTOM_MODE / SMO_MODE / AFLUX_MODE / ENCODER_xxx / HALL_xxx 等）
  - **motor.udq_mode**：电压给定方式（UDQ_CUSTOM / UDQ_MTPA_IF / UDQ_OBSERVER 等）
  - **motor.idqref_mode**：电流给定方式（IDQ_CUSTOM / IDQ_SPEED_OBSERVER / IDQ_SPEED_MTPA 等）
  - **motor.startup_mode**：VF_STARTUP / IF_STARTUP（若适用）
  - 开环时可能设置 **motor.motor_openloop.d_var/q_var**、**motor.id_ref/iq_ref** 等。

这样，主循环只负责“选模式、选菜单、调速度、状态机、LED”，**不**做电流环计算。

---

## 六、中断（与主循环并行）

### 6.1 SysTick_Handler（gd32_foc_it.c）

- 周期由 **systick_config()** 决定（通常 1ms）。
- 每次中断：**delay_decrement()**、**speed_get()**、**motor_idqref_calc()**、**motor_protect_check()**、**comm_usart_motor_state_data_transmit()**。
- 作用：软件延时计数、速度计算、电流给定计算、保护检测、串口上报状态。

### 6.2 FOC_CONTORL_IRQHandler（ADC 插入转换完成）

- 仅在 **start_ctrl()** 使能 TIMER0 和 ADC 插入触发后**才会周期触发**（即电机已启动、处于 RUNNING 时）。
- 顺序大致为：清 ADC 标志 → **get_iab()** 采样电流 → 缺相检测 → 若为 ANGLE_CUSTOM_MODE 则 UART4 请求编码器角度并更新电角度 → **angle_get()** → Clarke/Park → **motor_udq_calc()** → 过调制 → 逆 Park → **pwm_generation()**。
- 即：**电流环 + 角度 + 电压给定 + PWM 输出** 都在本中断内完成，是 FOC 的“实时控制循环”。

### 6.3 EXTI10_15_IRQHandler（EC11）

- 旋钮 A/B 脚 EXTI 边沿触发，调用 **EC11_IRQ_HANDLE()**，内部 **ec_delay(10000)** 后根据边沿顺序调用 **ec11_handle()**（见第四节）。

### 6.4 HALL_ENCODER_TIMER_IRQHandler

- 霍尔/编码器定时器中断，调用 **angle_sensor_interrupt_handler()**，用于霍尔或编码器模式下的角度/位置更新。

### 6.5 其它

- **COMM_RECEIVE_IRQHandler**：串口接收（如 DMA 完成）处理。
- **UART4_IRQHandler**：UART4 中断（如编码器通信）。

---

## 七、数据与配置来源（gdfoc_config.c 等）

- **motor**：全局 `motor_struct motor`，在 **gdfoc_config.c** 中初始化（参数、PID 指针、motor_start/stop/break 函数指针等），**state 未在初始化列表显式赋值，相当于 0，即 MC_STATE_INIT**。
- **pwm_current**：电流采样与 PWM 生成结构，在 **pwm_current_peripheral_init()**（INIT 状态里调用）中与具体硬件（1/2/3 shunt、DSVPWM/CSVPWM）绑定。
- **rotor_angle**：角度与速度观测，在 **angle_init()**（MC_START 时调用）等处根据 **rotor_angle.pos_mode** 初始化。

---

## 八、整体流程简图（文字）

```
上电
  → 调试使能、systick、comm_init、eval_board_init、uart4_init（只执行一次）
  → 进入 while(1)
      1. motor_running_mode_process()  // 按 motor.running_mode 配参数
      2. [可选] oled_display()         // 当前注释掉
      3. key_process()                 // 按键 → motor.command / 菜单 / running_mode
      4. motor_state_machine()         // 状态转移 + motor_start/motor_stop/motor_break
      5. led_sacn_state()              // LED
      6. printf 等
      → 回到 1

并行中断：
  · SysTick        → delay、speed_get、motor_idqref_calc、保护、串口上报
  · ADC EOIC       → FOC 电流环（仅 motor 启动后）
  · EXTI10_15      → EC11 旋钮 → ec11_handle()
  · HALL/ENCODER   → 角度/位置
  · COMM/UART4     → 串口接收
```

---

## 九、小结（执行顺序与“头尾”）

1. **头**：`main()` 里依次完成调试、SysTick、通信、板级、UART4 初始化；**不**启动 TIMER0，FOC 未运行。
2. **主循环**：固定顺序为 模式处理 → （OLED）→ 按键 → 状态机 → LED → 其它；状态机根据 **state + command** 决定是否 **motor_start / motor_stop / motor_break**。
3. **尾（实时环）**：**motor_start** 后 TIMER0 和 ADC 插入通道周期工作，**FOC_CONTORL_IRQHandler** 以 ADC 周期为节拍运行电流环；**motor_stop** 后该中断不再触发。
4. **旋钮**：EC11 在 INIT 下若未单独处理会“无效”；若在 **ec11_handle()** 中为 **MC_STATE_INIT** 增加与 IDLE 相同的菜单/速度逻辑，则未启动 FOC 时旋钮也可用。

按上述顺序即可从“头”到“尾”把当前工程的主干执行流程理清。
