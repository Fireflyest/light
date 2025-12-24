#ifndef __EVENT_H
#define __EVENT_H


// 事件类型
typedef enum {
    EVENT_NONE,
    EVENT_KEY_UP,
    EVENT_KEY_DOWN,
    EVENT_KEY_LEFT,
    EVENT_KEY_RIGHT,
    EVENT_KEY_SELECT,
    EVENT_KEY_BACK
} UIEvent;

// void UI_HandleEvent(UIEvent event) {
//     // 如果有焦点对象且它有事件处理函数
//     if (g_uiManager.focusedObject && g_uiManager.focusedObject->handleEvent) {
//         g_uiManager.focusedObject->handleEvent(g_uiManager.focusedObject, event);
//     } else {
//         // 默认焦点导航行为
//         switch (event) {
//             case EVENT_KEY_DOWN:
//                 UI_FocusNext();
//                 break;
//             case EVENT_KEY_UP:
//                 UI_FocusPrevious();
//                 break;
//             // 其他事件处理...
//         }
//     }
// }

#endif /* __EVENT_H */