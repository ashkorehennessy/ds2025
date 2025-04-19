//
// Created by ashkore on 2024/2/1.
//

#include "string.h"
#include "stdio.h"
#include "UI.h"
#include "cmsis_os2.h"
#include <tgmath.h>
#include "pool.h"
#include "st7735.h"
#include "switch.h"

char buf[32];
UI_item items[8][8];
int empty = 0;
int cursor_pos = 0;
int exponent = 0;
int ui_state = 0;  // 0:移动光标 1:修改数值 2:修改数值的指数
int key_pressed = 0;
char buf[32];

void UI_item_init(UI_item *item, const char *name, int type, void *var_ptr) {
    strcpy(item->name, name);
    item->type = type;
    switch (item->type) {
        case INT32:
            item->var_p.int32_p = (int32_t *) var_ptr;
            break;
        case INT16:
            item->var_p.int16_p = (int16_t *) var_ptr;
            break;
        case INT8:
            item->var_p.int8_p = (int8_t *) var_ptr;
            break;
        case UINT32:
            item->var_p.uint32_p = (uint32_t *) var_ptr;
            break;
        case UINT16:
            item->var_p.uint16_p = (uint16_t *) var_ptr;
            break;
        case UINT8:
            item->var_p.uint8_p = (uint8_t *) var_ptr;
            break;
        case DOUBLE:
            item->var_p.double_p = (double *) var_ptr;
            break;
        case FLOAT:
            item->var_p.float_p = (float *) var_ptr;
            break;
        case EMPTY:
            item->var_p.int8_p = (int8_t *) var_ptr;
            break;
        case FUNC:
            item->var_p.func_p = var_ptr;
            break;
        case CHAR:
            item->var_p.char_p = (char *) var_ptr;
            break;
    }
}

double UI_item_get_value(const UI_item *item) {
    switch (item->type) {
        case INT32:
            return (double) *item->var_p.int32_p;
        case INT16:
            return (double) *item->var_p.int16_p;
        case INT8:
            return (double) *item->var_p.int8_p;
        case UINT32:
            return (double) *item->var_p.uint32_p;
        case UINT16:
            return (double) *item->var_p.uint16_p;
        case UINT8:
            return (double) *item->var_p.uint8_p;
        case DOUBLE:
            return *item->var_p.double_p;
        case FLOAT:
            return (double) *item->var_p.float_p;
        case FUNC:
            return (double) item->var_p.func_p();
        case CHAR:
            return (double) *item->var_p.char_p;
    }
    return 0;
}

void UI_item_set_value(UI_item *item, double value) {
    if(item->type == EMPTY) return;
    switch (item->type) {
        case INT32:
            *item->var_p.int32_p = (int32_t) value;
            break;
        case INT16:
            *item->var_p.int16_p = (int16_t) value;
            break;
        case INT8:
            *item->var_p.int8_p = (int8_t) value;
            break;
        case UINT32:
            *item->var_p.uint32_p = (uint32_t) value;
            break;
        case UINT16:
            *item->var_p.uint16_p = (uint16_t) value;
            break;
        case UINT8:
            *item->var_p.uint8_p = (uint8_t) value;
            break;
        case DOUBLE:
            *item->var_p.double_p = value;
            break;
        case FLOAT:
            *item->var_p.float_p = (float) value;
            break;
        case CHAR:
            *item->var_p.char_p = (char) value;
            break;
    }
}

void UI_item_show_name(UI_item *item, uint16_t x, uint16_t y, FontDef font) {
    if(item->type == EMPTY) {
        ST7735_WriteString(x, y, "          ",font, ST7735_WHITE, ST7735_BLACK);
    }
    ST7735_WriteString(x, y, item->name, font, ST7735_YELLOW, ST7735_BLACK);
}

void UI_item_show_value(const UI_item *item, uint16_t x, uint16_t y, FontDef font) {
    if(item->type == EMPTY) return;
    double value = UI_item_get_value(item);
    switch (item->type) {
        case INT32:
        case INT16:
        case INT8:
        case UINT32:
        case UINT16:
        case UINT8:
        case FUNC:
            if(value > 9999999999 || value < -999999999){
                sprintf(buf, " Out range");
            } else {
                sprintf(buf, "%8.0f", value);
            }
            break;
        case DOUBLE:
        case FLOAT:
            if(value > 9999999.99 || value < -999999.99){
                sprintf(buf, " Out range");
            } else {
                sprintf(buf, "%8.2f", UI_item_get_value(item));
            }
            break;
        case CHAR:
            sprintf(buf, "%8c", (char) value);
            break;
    }
    ST7735_WriteString(x, y, buf, font, ST7735_WHITE, ST7735_BLACK);
}

void UI_init(){
    // 初始化所有的item为EMPTY
    for(uint8_t page = 0; page < 8; page++){
        for(int item = 0; item < SCREEN_H / FONT_H - 1; item++){
            items[page][item].type = EMPTY;
        }
    }
    UI_item_init(&items[0][0], "adang", FLOAT, &angle_adc);
    UI_item_init(&items[0][1], "ywang", FLOAT, &angle_yaw);
    UI_item_init(&items[0][2], "shang", FLOAT, &angle_top);
    UI_item_init(&items[0][3], "Speed", FLOAT, &speed);
    UI_item_init(&items[0][4], "adraw", UINT32, &adc_raw);
    UI_item_init(&items[1][0], "ywang", FLOAT, &angle_yaw);
    UI_item_init(&items[1][1], "ywout", FLOAT, &yaw_pidout);
    UI_item_init(&items[1][2], "targe", FLOAT, &target_angle_yaw);
    UI_item_init(&items[1][3], "Speed", FLOAT, &speed);
    UI_item_init(&items[1][4], "run  ", INT32, &task_running);
    UI_item_init(&items[1][5], "index", INT32, &task_index);
    UI_item_init(&items[4][0], "shang", FLOAT, &angle_top);
    UI_item_init(&items[4][1], "ywang", FLOAT, &angle_yaw);
    UI_item_init(&items[4][2], "Speed", FLOAT, &speed);
    UI_item_init(&items[4][3], "ywout", FLOAT, &yaw_pidout);
    UI_item_init(&items[4][4], "run  ", INT32, &task_running);
    UI_item_init(&items[4][5], "off  ", FLOAT, &offset);


}

void UI_show(){
    static uint8_t last_dip_switch = 8;
    static uint8_t last_ui_state = 3;
    uint8_t dip_switch = DIP_SWITCH;
    uint8_t show_static_part = (dip_switch != last_dip_switch) || (ui_state != last_ui_state) || key_pressed;

    // 显示静态部分
    if(show_static_part) {
        last_dip_switch = dip_switch;
        last_ui_state = ui_state;
        key_pressed = 0;

        ST7735_FillScreenFast(ST7735_BLACK);
        sprintf(buf, "Page%d     10%+d", dip_switch, exponent);
        ST7735_WriteString(0, 0, buf, Font_11x18, ST7735_GREEN, ST7735_BLACK);
        for(int i = 0; i < 9; i++) {
            ST7735_WriteString(0, FONT_H * i + FONT_H, "|", Font_11x18, ST7735_GREEN, ST7735_BLACK);
        }
        // 显示光标
        switch (ui_state) {
            case 0:
                ST7735_WriteString(0, FONT_H * cursor_pos + FONT_H, ">", Font_11x18, ST7735_GREEN, ST7735_BLACK);
                break;
            case 1:
                ST7735_WriteString(0, FONT_H * cursor_pos + FONT_H, "*", Font_11x18, ST7735_GREEN, ST7735_BLACK);
                break;
            case 2:
                ST7735_WriteString(0, FONT_H * cursor_pos + FONT_H, "^", Font_11x18, ST7735_GREEN, ST7735_BLACK);
                sprintf(buf, "%+d", exponent);
                ST7735_WriteString(131, 0, buf, Font_11x18, ST7735_RED, ST7735_BLACK);
                break;
        }
        // 显示所有的item
        for(int i = 0; i < SCREEN_H / FONT_H - 1; i++){
            UI_item_show_name(&items[dip_switch][i], FONT_W, FONT_H * i + FONT_H, Font_11x18);
        }
    }

    // 显示值
    for(int i = 0; i < SCREEN_H / FONT_H - 1; i++){
        UI_item_show_value(&items[dip_switch][i], FONT_W * 6, FONT_H * i + FONT_H, Font_11x18);
    }
    // 显示自定义部分
    UI_show_custom_part();

}

void UI_show_custom_part(){
}
void UI_key_process(){
    static int8_t key_forward_pressed = 0;
    static int8_t key_up_pressed = 0;
    static int8_t key_down_pressed = 0;
    static int8_t key_back_pressed = 0;

    uint8_t dip_switch = DIP_SWITCH;
    UI_item *item = &items[dip_switch][cursor_pos];
    // 切换模式
    if(KEY_FORWARD && !key_forward_pressed){
        key_forward_pressed = 1;
        key_pressed = 1;
        switch (ui_state) {
            case 0:
                ui_state = 1;
                break;
            case 1:
                ui_state = 2;
                break;
            case 2:
                ui_state = 0;
                break;
        }
    } else if(!KEY_FORWARD && key_forward_pressed){
        key_forward_pressed = 0;
    }

    // 0:上移光标 1:增加数值 2:增加数值的指数
    if(KEY_UP && !key_up_pressed){
        key_up_pressed = 1;
        key_pressed = 1;
        switch (ui_state) {
            case 0:
                cursor_pos = (cursor_pos + SCREEN_H / FONT_H - 2) % (SCREEN_H / FONT_H - 1);
                break;
            case 1:
                UI_item_set_value(item, UI_item_get_value(item) + pow(10, exponent));
                break;
            case 2:
                if(exponent < 7) exponent++;
                break;
        }
    } else if(!KEY_UP && key_up_pressed){
        key_up_pressed = 0;
    }

    // 0:下移光标 1:减少数值 2:减少数值的指数
    if(KEY_DOWN && !key_down_pressed){
        key_down_pressed = 1;
        key_pressed = 1;
        switch (ui_state) {
            case 0:
                cursor_pos = (cursor_pos + 1) % (SCREEN_H / FONT_H - 1);
                break;
            case 1:
                UI_item_set_value(item, UI_item_get_value(item) - pow(10, exponent));
                break;
            case 2:
                if(exponent > -7) exponent--;
                break;
        }
    } else if(!KEY_DOWN && key_down_pressed){
        key_down_pressed = 0;
    }

    // 启动任务
    if(KEY_BACK && !key_back_pressed){
        key_back_pressed = 1;
        key_pressed = 1;
        if (task_running == 0) {
            start_task(task_index);
        }
    } else if(!KEY_BACK && key_back_pressed){
        key_back_pressed = 0;
    }
}

