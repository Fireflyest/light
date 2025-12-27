#ifndef __UI_H
#define __UI_H

#include "gfx.h"

#define TEXTLIST_MAX_LINES 10
#define TEXTLIST_MAX_CHARS 20

typedef struct UI_Widget UI_Widget;

struct UI_Widget {
    int x, y, w, h;           // Relative coordinates
    UI_Widget* parent;
    UI_Widget* child;
    UI_Widget* next;
    
    // Virtual functions
    void (*draw)(UI_Widget*);
    void (*onKey)(UI_Widget*, uint8_t key);
    
    uint8_t isVisible;
    uint8_t isFocusable;
};

// Global Focus
extern UI_Widget* g_focusWidget;

// Core functions
void UI_Init(void);
void UI_AddChild(UI_Widget* parent, UI_Widget* child);
void UI_DrawTree(UI_Widget* root, int absX, int absY);
void UI_ProcessKey(uint8_t key);
void UI_SetFocus(UI_Widget* widget);

// Standard Widgets
typedef struct {
    UI_Widget base;
    const char* text;
} UI_Label;

typedef struct {
    UI_Widget base;
    const char* text;
    void (*onClick)(void);
} UI_Button;

typedef struct {
    UI_Widget base;
    char buffer[TEXTLIST_MAX_LINES][TEXTLIST_MAX_CHARS];
    int lineCount;
    int maxVisibleLines;
} UI_TextList;


// Widget Constructors
void UI_Label_Init(UI_Label* label, int x, int y, const char* text);
void UI_Button_Init(UI_Button* button, int x, int y, int w, int h, const char* text, void (*onClick)(void));

void UI_TextList_Init(UI_TextList* list, int x, int y, int w, int h);
void UI_TextList_AddLine(UI_TextList* list, const char* text);
void UI_TextList_Clear(UI_TextList* list);

#endif /* __UI_H */
