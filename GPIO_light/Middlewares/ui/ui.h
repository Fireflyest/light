#ifndef __UI_H
#define __UI_H

#include "render.h"

// 组件类型枚举
typedef enum {
    UI_LABEL,      // 文本标签
    UI_BUTTON,     // 按钮
    UI_CHECKBOX,   // 复选框
    UI_PROGRESS,   // 进度条
    UI_ICON,       // 图标
    UI_WINDOW,     // 窗口/面板
    UI_MENU        // 菜单
} UIType;

typedef enum {
    ALIGN_LEFT,    // 左对齐
    ALIGN_CENTER,  // 居中对齐
    ALIGN_RIGHT    // 右对齐
} UIAlignment;

typedef enum {
    FONT_6X8,      // 6x8字体
    FONT_8X16,     // 8x16字体
    FONT_12X24     // 12x24字体
} UIFont;


// UI对象基本结构体
typedef struct UIObject {
    UIType type;                  // 组件类型
    uint8_t id;                   // 组件唯一ID
    uint8_t x, y;                 // 位置
    uint8_t width, height;        // 尺寸
    uint8_t visible : 1;          // 可见性
    uint8_t active : 1;           // 是否激活
    uint8_t dirty : 1;            // 是否需要重绘
    void (*draw)(struct UIObject*); // 绘制函数
    void (*update)(struct UIObject*, void* data); // 更新函数
    void (*handleEvent)(struct UIObject*, uint8_t event); // 事件处理函数
    struct UIObject* parent;      // 父对象
    struct UIObject* next;        // 链表中的下一个对象
} UIObject;


// 标签组件
typedef struct {
    UIObject base;            // 基类
    uint8_t text[6];              // 文本内容
    uint8_t font;            // 字体
    uint8_t alignment;       // 对齐方式
} UILabel;

// 按钮组件
typedef struct {
    UIObject base;            // 基类
    uint8_t text[6];              // 按钮文本
    uint8_t pressed : 1;     // 是否被按下
    void (*onClick)(void);   // 点击回调函数
} UIButton;

// 复选框组件
typedef struct {
    UIObject base;            // 基类
    uint8_t checked : 1;     // 是否被选中
    uint8_t text[6];              // 复选框文本
    void (*onToggle)(uint8_t state); // 状态改变回调
} UICheckbox;

// 进度条组件
typedef struct {
    UIObject base;           // 基类
    uint8_t progress;        // 进度值(0-100)
    uint8_t style;           // 样式
} UIProgressBar;

// 窗口/面板组件
typedef struct {
    UIObject base;          // 基类
    uint8_t title[6];            // 窗口标题
    UIObject* children;     // 子组件链表
    uint8_t border : 1;    // 边框样式
} UIWindow;


// UI管理器
typedef struct {
    UIObject* root;            // 根组件(通常是屏幕)
    UIObject* focusedObject;   // 当前获得焦点的对象
    uint8_t needsRefresh : 1;  // 是否需要完全刷新
    void (*refresh)(void);     // 刷新函数
} UIManager;

// 全局UI管理器实例
extern UIManager g_uiManager;
extern uint8_t g_uiIdCounter; // ID计数器

void UI_Init(UIWindow* window);
void UI_AddChild(UIObject* parent, UIObject* child);
void UI_RefreshScreen(void);
void UI_DrawObject(UIObject* obj);
uint8_t UI_GetNextId(void);

void UI_CreateWindow(UIWindow* window,uint8_t x, uint8_t y, uint8_t width, uint8_t height, uint8_t* title, uint8_t border);
void UI_CreateLabel(UILabel* label, uint8_t x, uint8_t y, uint8_t width, uint8_t height, uint8_t* text);
void UI_CreateButton(UIButton* button, uint8_t x, uint8_t y, uint8_t width, uint8_t height, uint8_t* text, void (*onClick)(void));
void UI_CreateCheckbox(UICheckbox* checkbox, uint8_t x, uint8_t y, uint8_t width, uint8_t height, uint8_t* text, void (*onToggle)(uint8_t state));
void UI_CreateProgressBar(UIProgressBar* progressbar, uint8_t x, uint8_t y, uint8_t width, uint8_t height, uint8_t progress, uint8_t style);

void UI_DrawLabel(UIObject* obj);
void UI_DrawButton(UIObject* obj);
void UI_DrawCheckbox(UIObject* obj);
void UI_DrawProgressBar(UIObject* obj);
void UI_DrawWindow(UIObject* obj);
void UI_DrawMenu(UIObject* obj);


#endif /* __UI_H */
