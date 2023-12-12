/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include "lvgl.h"
#include <stdio.h>
void example_lvgl_demo_ui(lv_disp_t *disp, uint8_t selec)
{   

    lv_obj_t *scr = lv_disp_get_scr_act(disp);
    lv_obj_t *label = lv_label_create(scr);
    lv_label_set_long_mode(label, LV_LABEL_LONG_DOT); /* Circular scroll */
    
    lv_label_set_text(label, "SISTEMA:    \0");
    lv_label_cut_text(label, 8, 4);
    switch (selec)
    {
        case (0): lv_label_ins_text(label, 8, " ON\0"); break;
        case (1): lv_label_ins_text(label, 8, "OFF\0"); break;
    }
 

    /* Size of the screen (if you use rotation 90 or 270, please set disp->driver->ver_res) */
    lv_obj_set_width(label, disp->driver->hor_res);
    lv_obj_align(label, LV_ALIGN_TOP_MID, 0, 0);
}
