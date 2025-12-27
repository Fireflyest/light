# OLED 显示系统重构设计思路

## 1. 总体架构设计 (Layered Architecture)

采用分层架构，由底向上分为四层：

*   **L1 - 驱动层 (Driver Layer)**: 负责与 OLED 硬件通信，核心是 **显存（FrameBuffer）** 和 **DMA 传输**。
*   **L2 - 图形层 (GFX Layer)**: 负责在显存上画点、线、面、字符，以及 **3D 投影计算**。
*   **L3 - UI 框架层 (UI Framework)**: 定义控件（Widget）、页面（Page）、焦点管理、事件分发。
*   **L4 - 应用层 (App Layer)**: 具体的页面布局、业务逻辑（如显示 PWM 值）。

---

## 2. 核心模块设计思路

### A. 解决“快速刷新” (针对要求 1)
*   **全屏缓冲 (FrameBuffer)**: 在内存中开辟一个 `128 * 64 / 8 = 1024 Bytes` 的数组 `u8 FrameBuffer[1024]`。
*   **DMA 传输**: 所有的绘图操作只修改内存中的 `FrameBuffer`。绘制完成后，启动 DMA 将这 1KB 数据一次性推送到 OLED。
*   **双缓冲 (可选)**: 如果出现画面撕裂，可以开两个 Buffer，一个用于显示，一个用于绘制，绘制完后交换指针（STM32F4 内存足够）。
*   **脏矩形 (Dirty Rect) 优化**: 如果不需要 3D 动画，只刷新变化的区域。但考虑到要做 3D，建议直接全屏刷新，F4 的 SPI DMA 足够快达到 60FPS。

### B. 解决“控件嵌套与互不干扰” (针对要求 3, 5)
采用 **组合模式 (Composite Pattern)** 的思想。
*   **基类设计**: 定义一个结构体 `UI_Widget`。
    ```c
    typedef struct UI_Widget {
        int x, y, w, h;           // 相对父控件的坐标和尺寸
        struct UI_Widget* parent; // 父控件指针
        struct UI_Widget* child;  // 第一个子控件
        struct UI_Widget* next;   // 下一个兄弟控件
        void (*draw)(struct UI_Widget*); // 虚函数：绘制
        void (*onKey)(struct UI_Widget*, u8 key); // 虚函数：按键处理
        u8 isVisible;
    } UI_Widget;
    ```
*   **坐标系**: 绘制时，子控件的坐标 = `父控件绝对坐标` + `子控件相对坐标`。
*   **裁剪 (Clipping)**: 绘图函数需要支持裁剪区域，防止子控件画到父控件外面。

### C. 解决“焦点与变化显示” (针对要求 6)
*   **焦点管理**: 全局维护一个指针 `UI_Widget* g_focusWidget`。
*   **状态绘制**: 在控件的 `draw` 函数中判断：
    ```c
    if (this == g_focusWidget) {
        // 绘制反色背景或虚线边框
        GFX_DrawRect(..., Color_Invert); 
    } else {
        // 正常绘制
    }
    ```
*   **焦点切换**: 按键事件触发时，通过链表找到 `next` 兄弟节点，更新 `g_focusWidget`。

### D. 解决“页面切换” (针对要求 4)
*   **页面栈**: 维护一个页面栈或状态机。
*   **页面结构**: `Page` 本质上是一个全屏的 `UI_Widget`（容器）。
*   **切换逻辑**:
    1.  清空当前 FrameBuffer。
    2.  将 `CurrentPage` 指针指向新的页面根节点。
    3.  重置焦点到新页面的默认控件。

### E. 解决“3D 转 2D” (针对要求 7)
这是一个纯数学问题，STM32F4 有硬件浮点单元 (FPU)，计算毫无压力。
*   **数据结构**: 定义 `Point3d {x, y, z}`。
*   **变换流水线**:
    1.  **模型变换**: 旋转、平移（矩阵乘法或欧拉角公式）。
    2.  **投影变换 (透视投影)**:
        $$ x_{2d} = \frac{x_{3d} \times FocalLength}{z_{3d} + CameraDistance} + ScreenCenterX $$
        $$ y_{2d} = \frac{y_{3d} \times FocalLength}{z_{3d} + CameraDistance} + ScreenCenterY $$
*   **绘制**: 将转换后的 2D 点连接起来，调用 GFX 层的 `DrawLine`。

---

## 3. 代码重构步骤 (Roadmap)

建议按以下顺序实现：

### 第一阶段：基础图形库 (GFX)
1.  实现 `OLED_Init_DMA()`。
2.  实现 `GFX_ClearBuffer()` 和 `GFX_UpdateScreen()`。
3.  实现原子操作 `GFX_DrawPixel(x, y, color)`。
4.  基于画点，实现 `DrawLine` (Bresenham算法), `DrawRect`, `DrawCircle`。
5.  实现 `DrawString` (支持中英文字库)。

### 第二阶段：UI 核心
1.  定义 `UI_Widget` 结构体。
2.  实现 `UI_AddChild(parent, child)` 建立树状关系。
3.  实现 `UI_DrawTree(root)`：递归遍历树，计算绝对坐标并绘制。
4.  实现具体控件：`Label` (文本), `Button` (带边框文本), `Chart` (波形图)。

### 第三阶段：交互与逻辑
1.  实现 `Input_Process(key)`：根据按键改变 `g_focusWidget` 或调用 `g_focusWidget->onKey()`。
2.  实现 `Page_Manager`：管理不同的界面根节点。

### 第四阶段：3D 引擎插件
1.  编写 `Math3D` 模块。
2.  创建一个特殊的控件 `UI_View3D`。
3.  在 `UI_View3D` 的 `draw` 函数中，执行 3D 点旋转和投影，然后画线。

---

## 4. 伪代码演示 (主循环逻辑)

重构后的 `main` 函数应该非常干净：

```c
int main() {
    Hardware_Init(); // 时钟, GPIO, I2C/SPI, DMA
    GFX_Init();      // OLED初始化
    UI_Init();       // 初始化页面和控件树

    while (1) {
        // 1. 处理输入
        if (Key_Pressed) {
            UI_ProcessInput(Key_Value);
        }

        // 2. 逻辑更新 (如 3D 旋转角度增加，PWM 数据刷新)
        CurrentPage->Update(); 

        // 3. 渲染流程
        GFX_ClearBuffer();       // 清空显存
        UI_DrawTree(CurrentPage); // 递归绘制当前页面所有控件
        GFX_UpdateScreen();      // DMA 推送

        // 4. 帧率控制
        Delay_ms(16); // 约 60FPS
```
# OLED 显示系统重构设计思路

## 1. 总体架构设计 (Layered Architecture)

采用分层架构，由底向上分为四层：

*   **L1 - 驱动层 (Driver Layer)**: 负责与 OLED 硬件通信，核心是 **显存（FrameBuffer）** 和 **DMA 传输**。
*   **L2 - 图形层 (GFX Layer)**: 负责在显存上画点、线、面、字符，以及 **3D 投影计算**。
*   **L3 - UI 框架层 (UI Framework)**: 定义控件（Widget）、页面（Page）、焦点管理、事件分发。
*   **L4 - 应用层 (App Layer)**: 具体的页面布局、业务逻辑（如显示 PWM 值）。

---

## 2. 核心模块设计思路

### A. 解决“快速刷新” (针对要求 1)
*   **全屏缓冲 (FrameBuffer)**: 在内存中开辟一个 `128 * 64 / 8 = 1024 Bytes` 的数组 `u8 FrameBuffer[1024]`。
*   **DMA 传输**: 所有的绘图操作只修改内存中的 `FrameBuffer`。绘制完成后，启动 DMA 将这 1KB 数据一次性推送到 OLED。
*   **双缓冲 (可选)**: 如果出现画面撕裂，可以开两个 Buffer，一个用于显示，一个用于绘制，绘制完后交换指针（STM32F4 内存足够）。
*   **脏矩形 (Dirty Rect) 优化**: 如果不需要 3D 动画，只刷新变化的区域。但考虑到要做 3D，建议直接全屏刷新，F4 的 SPI DMA 足够快达到 60FPS。

### B. 解决“控件嵌套与互不干扰” (针对要求 3, 5)
采用 **组合模式 (Composite Pattern)** 的思想。
*   **基类设计**: 定义一个结构体 `UI_Widget`。
    ```c
    typedef struct UI_Widget {
        int x, y, w, h;           // 相对父控件的坐标和尺寸
        struct UI_Widget* parent; // 父控件指针
        struct UI_Widget* child;  // 第一个子控件
        struct UI_Widget* next;   // 下一个兄弟控件
        void (*draw)(struct UI_Widget*); // 虚函数：绘制
        void (*onKey)(struct UI_Widget*, u8 key); // 虚函数：按键处理
        u8 isVisible;
    } UI_Widget;
    ```
*   **坐标系**: 绘制时，子控件的坐标 = `父控件绝对坐标` + `子控件相对坐标`。
*   **裁剪 (Clipping)**: 绘图函数需要支持裁剪区域，防止子控件画到父控件外面。

### C. 解决“焦点与变化显示” (针对要求 6)
*   **焦点管理**: 全局维护一个指针 `UI_Widget* g_focusWidget`。
*   **状态绘制**: 在控件的 `draw` 函数中判断：
    ```c
    if (this == g_focusWidget) {
        // 绘制反色背景或虚线边框
        GFX_DrawRect(..., Color_Invert); 
    } else {
        // 正常绘制
    }
    ```
*   **焦点切换**: 按键事件触发时，通过链表找到 `next` 兄弟节点，更新 `g_focusWidget`。

### D. 解决“页面切换” (针对要求 4)
*   **页面栈**: 维护一个页面栈或状态机。
*   **页面结构**: `Page` 本质上是一个全屏的 `UI_Widget`（容器）。
*   **切换逻辑**:
    1.  清空当前 FrameBuffer。
    2.  将 `CurrentPage` 指针指向新的页面根节点。
    3.  重置焦点到新页面的默认控件。

### E. 解决“3D 转 2D” (针对要求 7)
这是一个纯数学问题，STM32F4 有硬件浮点单元 (FPU)，计算毫无压力。
*   **数据结构**: 定义 `Point3d {x, y, z}`。
*   **变换流水线**:
    1.  **模型变换**: 旋转、平移（矩阵乘法或欧拉角公式）。
    2.  **投影变换 (透视投影)**:
        $$ x_{2d} = \frac{x_{3d} \times FocalLength}{z_{3d} + CameraDistance} + ScreenCenterX $$
        $$ y_{2d} = \frac{y_{3d} \times FocalLength}{z_{3d} + CameraDistance} + ScreenCenterY $$
*   **绘制**: 将转换后的 2D 点连接起来，调用 GFX 层的 `DrawLine`。

---

## 3. 代码重构步骤 (Roadmap)

建议按以下顺序实现：

### 第一阶段：基础图形库 (GFX)
1.  实现 `OLED_Init_DMA()`。
2.  实现 `GFX_ClearBuffer()` 和 `GFX_UpdateScreen()`。
3.  实现原子操作 `GFX_DrawPixel(x, y, color)`。
4.  基于画点，实现 `DrawLine` (Bresenham算法), `DrawRect`, `DrawCircle`。
5.  实现 `DrawString` (支持中英文字库)。

### 第二阶段：UI 核心
1.  定义 `UI_Widget` 结构体。
2.  实现 `UI_AddChild(parent, child)` 建立树状关系。
3.  实现 `UI_DrawTree(root)`：递归遍历树，计算绝对坐标并绘制。
4.  实现具体控件：`Label` (文本), `Button` (带边框文本), `Chart` (波形图)。

### 第三阶段：交互与逻辑
1.  实现 `Input_Process(key)`：根据按键改变 `g_focusWidget` 或调用 `g_focusWidget->onKey()`。
2.  实现 `Page_Manager`：管理不同的界面根节点。

### 第四阶段：3D 引擎插件
1.  编写 `Math3D` 模块。
2.  创建一个特殊的控件 `UI_View3D`。
3.  在 `UI_View3D` 的 `draw` 函数中，执行 3D 点旋转和投影，然后画线。

---

## 4. 伪代码演示 (主循环逻辑)

重构后的 `main` 函数应该非常干净：

```c
int main() {
    Hardware_Init(); // 时钟, GPIO, I2C/SPI, DMA
    GFX_Init();      // OLED初始化
    UI_Init();       // 初始化页面和控件树

    while (1) {
        // 1. 处理输入
        if (Key_Pressed) {
            UI_ProcessInput(Key_Value);
        }

        // 2. 逻辑更新 (如 3D 旋转角度增加，PWM 数据刷新)
        CurrentPage->Update(); 

        // 3. 渲染流程
        GFX_ClearBuffer();       // 清空显存
        UI_DrawTree(CurrentPage); // 递归绘制当前页面所有控件
        GFX_UpdateScreen();      // DMA 推送

        // 4. 帧率控制
        Delay_ms(16); // 约 60FPS
```