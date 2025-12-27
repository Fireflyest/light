#include "ui.h"
#include <stddef.h>
#include <string.h>

UI_Widget* g_focusWidget = NULL;

void UI_Init(void) {
    g_focusWidget = NULL;
}

void UI_AddChild(UI_Widget* parent, UI_Widget* child) {
    child->parent = parent;
    child->next = NULL;
    
    if (parent->child == NULL) {
        parent->child = child;
    } else {
        UI_Widget* p = parent->child;
        while (p->next != NULL) {
            p = p->next;
        }
        p->next = child;
    }
}

// Global offset for rendering
static int g_uiOffsetX = 0;
static int g_uiOffsetY = 0;

void UI_DrawWidget(UI_Widget* widget) {
    if (widget->draw) {
        widget->draw(widget);
    }
}

void UI_DrawTree_Recursive(UI_Widget* root, int x, int y) {
    if (!root->isVisible) return;
    
    int oldX = g_uiOffsetX;
    int oldY = g_uiOffsetY;
    
    g_uiOffsetX = x + root->x;
    g_uiOffsetY = y + root->y;
    
    UI_DrawWidget(root);
    
    UI_Widget* child = root->child;
    while (child) {
        UI_DrawTree_Recursive(child, g_uiOffsetX, g_uiOffsetY);
        child = child->next;
    }
    
    g_uiOffsetX = oldX;
    g_uiOffsetY = oldY;
}

void UI_DrawTree(UI_Widget* root, int absX, int absY) {
    UI_DrawTree_Recursive(root, absX, absY);
}

void UI_SetFocus(UI_Widget* widget) {
    g_focusWidget = widget;
}

void UI_ProcessKey(uint8_t key) {
    if (g_focusWidget && g_focusWidget->onKey) {
        g_focusWidget->onKey(g_focusWidget, key);
    }
}

// --- Widget Implementations ---

void UI_Label_Draw(UI_Widget* w) {
    UI_Label* label = (UI_Label*)w;
    GFX_DrawString(g_uiOffsetX, g_uiOffsetY, label->text, GFX_COLOR_WHITE);
}

void UI_Label_Init(UI_Label* label, int x, int y, const char* text) {
    label->base.x = x;
    label->base.y = y;
    label->base.w = 0; // Auto width?
    label->base.h = 8;
    label->base.parent = NULL;
    label->base.child = NULL;
    label->base.next = NULL;
    label->base.draw = UI_Label_Draw;
    label->base.onKey = NULL;
    label->base.isVisible = 1;
    label->base.isFocusable = 0;
    label->text = text;
}

void UI_Button_Draw(UI_Widget* w) {
    UI_Button* btn = (UI_Button*)w;
    GFX_Color color = (w == g_focusWidget) ? GFX_COLOR_INVERT : GFX_COLOR_WHITE;
    
    GFX_DrawRect(g_uiOffsetX, g_uiOffsetY, w->w, w->h, GFX_COLOR_WHITE);
    GFX_DrawString(g_uiOffsetX + 2, g_uiOffsetY + 2, btn->text, color);
}

void UI_Button_OnKey(UI_Widget* w, uint8_t key) {
    UI_Button* btn = (UI_Button*)w;
    if (key == 1 && btn->onClick) { // Assume key 1 is Enter/Click
        btn->onClick();
    }
}

void UI_Button_Init(UI_Button* button, int x, int y, int w, int h, const char* text, void (*onClick)(void)) {
    button->base.x = x;
    button->base.y = y;
    button->base.w = w;
    button->base.h = h;
    button->base.parent = NULL;
    button->base.child = NULL;
    button->base.next = NULL;
    button->base.draw = UI_Button_Draw;
    button->base.onKey = UI_Button_OnKey;
    button->base.isVisible = 1;
    button->base.isFocusable = 1;
    button->text = text;
    button->onClick = onClick;
}

static void UI_TextList_Draw(UI_Widget* widget) {
    UI_TextList* list = (UI_TextList*)widget;

    int startX = widget->x;
    int startY = widget->y;
    int lineHeight = 8;

    for (int i = 0; i < list->lineCount; i++) {
        if ((i + 1) * lineHeight > widget->h - 4) break;
        GFX_DrawString(startX, startY + (i * lineHeight), list->buffer[i], GFX_COLOR_WHITE);
    }
}

void UI_TextList_Init(UI_TextList* list, int x, int y, int w, int h) {
    list->base.x = x;
    list->base.y = y;
    list->base.w = w;
    list->base.h = h;
    list->base.parent = NULL;
    list->base.child = NULL;
    list->base.next = NULL;
    list->base.isVisible = 1;
    list->base.isFocusable = 0;
    list->base.draw = UI_TextList_Draw;
    list->base.onKey = NULL;

    list->lineCount = 0;
    list->maxVisibleLines = (h - 4) / 8; 
    if (list->maxVisibleLines > TEXTLIST_MAX_LINES) {
        list->maxVisibleLines = TEXTLIST_MAX_LINES;
    }
    
    memset(list->buffer, 0, sizeof(list->buffer));
}

void UI_TextList_AddLine(UI_TextList* list, const char* text) {
    if (list->lineCount < list->maxVisibleLines) {
        strncpy(list->buffer[list->lineCount], text, TEXTLIST_MAX_CHARS - 1);
        list->buffer[list->lineCount][TEXTLIST_MAX_CHARS - 1] = '\0';
        list->lineCount++;
    } else {
        for (int i = 0; i < list->maxVisibleLines - 1; i++) {
            strcpy(list->buffer[i], list->buffer[i+1]);
        }
        
        strncpy(list->buffer[list->maxVisibleLines - 1], text, TEXTLIST_MAX_CHARS - 1);
        list->buffer[list->maxVisibleLines - 1][TEXTLIST_MAX_CHARS - 1] = '\0';
    }
}

void UI_TextList_Clear(UI_TextList* list) {
    list->lineCount = 0;
    memset(list->buffer, 0, sizeof(list->buffer));
}
