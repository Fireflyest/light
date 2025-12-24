#include "ui.h"
#include <string.h>

UIManager g_uiManager;
uint8_t g_uiIdCounter = 0; // ID计数器



void UI_Init(UIWindow* window) {
    // 初始化UI管理器
    g_uiManager.root = 0;
    g_uiManager.focusedObject = 0;
    g_uiManager.needsRefresh = 1;
    g_uiManager.refresh = UI_RefreshScreen;
    g_uiManager.root = (UIObject*)window;
}

void UI_AddChild(UIObject* parent, UIObject* child) {
    if (!parent || !child) return;
    
    // 设置父子关系
    child->parent = parent;
    
    // 找到链表末尾并添加
    if (parent->type == UI_WINDOW) {
        UIWindow* window = (UIWindow*)parent;
        if (!window->children) {
            window->children = child;
        } else {
            UIObject* last = window->children;
            while (last->next) {
                last = last->next;
            }
            last->next = child;
        }
    }
    
    // 标记需要刷新
    parent->dirty = 1;
}

void UI_RefreshScreen(void) {
    // 绘制根组件及其所有子组件
    UI_DrawObject(g_uiManager.root);
    
    // 清除刷新标记
    g_uiManager.needsRefresh = 0;
}

void UI_DrawObject(UIObject* obj) {
    if (!obj || !obj->visible) return;
    
    // 绘制当前对象
    if (obj->draw && obj->dirty) {
        obj->draw(obj);
    }
    
    // 如果是容器，绘制所有子对象
    if (obj->type == UI_WINDOW) {
        UIWindow* window = (UIWindow*)obj;
        UIObject* child = window->children;
        while (child) {
            UI_DrawObject(child);
            child = child->next;
        }
    }
}

uint8_t UI_GetNextId() {
    return g_uiIdCounter++;
}

void UI_CreateWindow(UIWindow* window, uint8_t x, uint8_t y, uint8_t width, uint8_t height, uint8_t* title, uint8_t border) {
    window->base.type = UI_WINDOW;
    window->base.id = UI_GetNextId();  // 获取唯一ID
    window->base.x = x;
    window->base.y = y;
    window->base.width = width;
    window->base.height = height;
    window->base.visible = 1;
    window->base.active = 0;
    window->base.dirty = 1;  // 初次创建时标记为需要绘制
    window->base.draw = UI_DrawWindow;
    window->base.update = 0;
    window->base.handleEvent = 0;  // 窗口不处理事件
    window->base.parent = 0;
    window->base.next = 0;
    
    // 初始化窗口特有部分
    memcpy(window->title, title, 6);
    window->children = 0;  // 初始没有子组件
    window->border = border;  // 默认实线边框
}

void UI_CreateLabel(UILabel* label, uint8_t x, uint8_t y, uint8_t width, uint8_t height, uint8_t* text) {
    label->base.type = UI_LABEL;
    label->base.id = UI_GetNextId();  // 获取唯一ID
    label->base.x = x;
    label->base.y = y;
    label->base.width = width;
    label->base.height = height;
    label->base.visible = 1;
    label->base.active = 0;
    label->base.dirty = 1;  // 初次创建时标记为需要绘制
    label->base.draw = UI_DrawLabel;
    label->base.update = 0;  // 标签不需要更新函数
    label->base.handleEvent = 0;  // 标签不处理事件
    label->base.parent = 0;
    label->base.next = 0;
    
    // 初始化标签特有部分
    memcpy(label->text, text, 6);
    label->font = FONT_6X8;  // 默认字体
    label->alignment = ALIGN_LEFT;  // 默认左对齐
}

void UI_CreateButton(UIButton* button, uint8_t x, uint8_t y, uint8_t width, uint8_t height, uint8_t* text, void (*onClick)(void)) {
    button->base.type = UI_BUTTON;
    button->base.id = UI_GetNextId();  // 获取唯一ID
    button->base.x = x;
    button->base.y = y;
    button->base.width = width;
    button->base.height = height;
    button->base.visible = 1;
    button->base.active = 0;
    button->base.dirty = 1;  // 初次创建时标记为需要绘制
    button->base.draw = UI_DrawButton;
    button->base.update = 0;  // 按钮不需要更新函数
    button->base.handleEvent = 0;  // 按钮不处理事件
    button->base.parent = 0;
    button->base.next = 0;
    
    // 初始化按钮特有部分
    memcpy(button->text, text, 6);
    button->pressed = 0;  // 默认未按下
    button->onClick = onClick;  // 点击回调
}

void UI_CreateCheckbox(UICheckbox* checkbox, uint8_t x, uint8_t y, uint8_t width, uint8_t height, uint8_t* text, void (*onToggle)(uint8_t state)) {
    checkbox->base.type = UI_CHECKBOX;
    checkbox->base.id = UI_GetNextId();  // 获取唯一ID
    checkbox->base.x = x;
    checkbox->base.y = y;
    checkbox->base.width = width;
    checkbox->base.height = height;
    checkbox->base.visible = 1;
    checkbox->base.active = 0;
    checkbox->base.dirty = 1;  // 初次创建时标记为需要绘制
    checkbox->base.draw = UI_DrawCheckbox;
    checkbox->base.update = 0;  // 复选框不需要更新函数
    checkbox->base.handleEvent = 0;  // 复选框不处理事件
    checkbox->base.parent = 0;
    checkbox->base.next = 0;
    
    // 初始化复选框特有部分
    memcpy(checkbox->text, text, 6);
    checkbox->checked = 0;  // 默认未选中
    checkbox->onToggle = onToggle;  // 切换回调
}

void UI_CreateProgressBar(UIProgressBar* progressBar, uint8_t x, uint8_t y, uint8_t width, uint8_t height, uint8_t progress, uint8_t style) {
    progressBar->base.type = UI_PROGRESS;
    progressBar->base.id = UI_GetNextId();  // 获取唯一ID
    progressBar->base.x = x;
    progressBar->base.y = y;
    progressBar->base.width = width;
    progressBar->base.height = height;
    progressBar->base.visible = 1;
    progressBar->base.active = 0;
    progressBar->base.dirty = 1;  // 初次创建时标记为需要绘制
    progressBar->base.draw = UI_DrawProgressBar;
    progressBar->base.update = 0;  // 进度条不需要更新函数
    progressBar->base.handleEvent = 0;  // 进度条不处理事件
    progressBar->base.parent = 0;
    progressBar->base.next = 0;
    
    // 初始化进度条特有部分
    progressBar->progress = progress;  // 设置初始进度
    progressBar->style = style;  // 设置样式
}

void UI_DrawLabel(UIObject* obj) {
    UILabel* label = (UILabel*)obj;
    
    // 如果不可见则不绘制
    if (!obj->visible) return;
    
    // 计算绘制位置
    uint8_t x = obj->x;
    uint8_t y = obj->y;
    
    // 如果有父对象，加上父对象的偏移
    if (obj->parent) {
        x += obj->parent->x;
        y += obj->parent->y;
    }
    
    // 根据对齐方式调整位置
    uint8_t text_width = strlen((char*)label->text) * 6;  // 假设6x8字体
    if (label->alignment == ALIGN_CENTER) {
        x += (obj->width - text_width) / 2;
    } else if (label->alignment == ALIGN_RIGHT) {
        x += obj->width - text_width;
    }
    
    // 绘制文本
    uint8_t row = y >> 3;  // 转换为页地址
    
    // 绘制文本内容
    for (uint8_t i = 0; i < strlen((char*)label->text); i++) {
        char c = label->text[i];
        uint8_t char_offset = c - ' ';
        if (label->font == FONT_6X8) {
            for (uint8_t j = 0; j < 6; j++) {
                Draw_Byte(x + i*6 + j, y, F6x8[char_offset][j], DRAW_ON);
            }
        }
    }
    
    // 清除脏标记
    obj->dirty = 0;
}

void UI_DrawButton(UIObject* obj) {
    UIButton* button = (UIButton*)obj;
    
    // 如果不可见则不绘制
    if (!obj->visible) return;
    
    // 计算绘制位置
    uint8_t x = obj->x;
    uint8_t y = obj->y;
    
    // 如果有父对象，加上父对象的偏移
    if (obj->parent) {
        x += obj->parent->x;
        y += obj->parent->y;
    }
    
    // 绘制按钮边框
    LineType lineStyle = button->base.active ? LINE_SOLID : LINE_DASHED;
    Draw_Rectangle(x, y, x + obj->width - 1, y + obj->height - 1, lineStyle, 
                   button->pressed ? FILL_SOLID : FILL_NONE, DRAW_ON);
    
    // 绘制文本
    uint8_t text_width = strlen((char*)button->text) * 6;  // 假设6x8字体
    uint8_t text_x = x + (obj->width - text_width) / 2;
    uint8_t text_y = y + (obj->height - 8) / 2;
    
    for (uint8_t i = 0; i < strlen((char*)button->text); i++) {
        char c = button->text[i];
        uint8_t char_offset = c - ' ';
        for (uint8_t j = 0; j < 6; j++) {
            Draw_Byte(text_x + i*6 + j, text_y, F6x8[char_offset][j], DRAW_ON);
        }
    }
    
    // 清除脏标记
    obj->dirty = 0;
}

void UI_DrawCheckbox(UIObject* obj) {
    UICheckbox* checkbox = (UICheckbox*)obj;
    
    // 如果不可见则不绘制
    if (!obj->visible) return;
    
    // 计算绘制位置
    uint8_t x = obj->x;
    uint8_t y = obj->y;
    
    // 如果有父对象，加上父对象的偏移
    if (obj->parent) {
        x += obj->parent->x;
        y += obj->parent->y;
    }
    
    // 绘制复选框边框
    Draw_Rectangle(x, y, x + 16 - 1, y + 16 - 1, LINE_SOLID, checkbox->checked ? FILL_SOLID : FILL_NONE, DRAW_ON);
    
    // 绘制复选框文本
    uint8_t text_x = x + 20;  // 偏移文本位置
    uint8_t text_y = y + (16 - 8) / 2;  // 垂直居中
    
    for (uint8_t i = 0; i < strlen((char*)checkbox->text); i++) {
        char c = checkbox->text[i];
        uint8_t char_offset = c - ' ';
        for (uint8_t j = 0; j < 6; j++) {
            Draw_Byte(text_x + i*6 + j, text_y, F6x8[char_offset][j], DRAW_ON);
        }
    }
    
    // 清除脏标记
    obj->dirty = 0;
}

void UI_DrawProgressBar(UIObject* obj) {
    UIProgressBar* progressBar = (UIProgressBar*)obj;
    
    // 如果不可见则不绘制
    if (!obj->visible) return;
    
    // 计算绘制位置
    uint8_t x = obj->x;
    uint8_t y = obj->y;
    
    // 如果有父对象，加上父对象的偏移
    if (obj->parent) {
        x += obj->parent->x;
        y += obj->parent->y;
    }
    
    // 绘制进度条边框
    Draw_Rectangle(x, y, x + obj->width - 1, y + obj->height - 1, LINE_SOLID, FILL_NONE, DRAW_ON);
    
    // 绘制进度条填充部分
    uint8_t fill_width = (obj->width - 2) * progressBar->progress / 100;  // 填充宽度
    Draw_Rectangle(x + 1, y + 1, x + fill_width, y + obj->height - 2, LINE_SOLID, FILL_SOLID, DRAW_ON);
    
    // 清除脏标记
    obj->dirty = 0;
}

void UI_DrawWindow(UIObject* obj) {
    UIWindow* window = (UIWindow*)obj;

    // 如果不可见则不绘制
    if (!obj->visible) return;

    // 计算绘制位置
    uint8_t x = obj->x;
    uint8_t y = obj->y;

    // 如果有父对象，加上父对象的偏移
    if (obj->parent) {
        x += obj->parent->x;
        y += obj->parent->y;
    }
    
    // 绘制窗口边框
    Draw_Rectangle(x, y, x + obj->width - 1, y + obj->height - 1, window->border, FILL_NONE, DRAW_ON);
    
    // 绘制窗口标题
    uint8_t title_x = x + (obj->width - strlen((char*)window->title) * 6) / 2;  // 居中显示标题
    uint8_t title_y = y + 2;  // 偏移标题位置
    
    for (uint8_t i = 0; i < strlen((char*)window->title); i++) {
        char c = window->title[i];
        uint8_t char_offset = c - ' ';
        for (uint8_t j = 0; j < 6; j++) {
            Draw_Byte(title_x + i*6 + j, title_y, F6x8[char_offset][j], DRAW_ON);
        }
    }
    
    // 清除脏标记
    obj->dirty = 0;
}

void UI_DrawMenu(UIObject* obj) {
    
}

