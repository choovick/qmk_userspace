/**
 * Copyright 2021 Charly Delay <charly@codesink.dev> (@0xcharly)
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include QMK_KEYBOARD_H

#ifdef CHARYBDIS_AUTO_POINTER_LAYER_TRIGGER_ENABLE
#    include "timer.h"
#endif // CHARYBDIS_AUTO_POINTER_LAYER_TRIGGER_ENABLE

/* TODO: Keymap re-arrangement */
enum charybdis_keymap_layers {
    LAYER_BASE = 0,
    LAYER_POINTER,
    LAYER_LOWER,
    LAYER_RAISE,
    LAYER_SYMBOLS,
};

/** \brief Automatically enable sniping-mode on the pointer layer. */
// TODO: Disable this feature as it conflict the my user of layer_state_set_user function
//#define CHARYBDIS_AUTO_SNIPING_ON_LAYER LAYER_POINTER

#ifdef CHARYBDIS_AUTO_POINTER_LAYER_TRIGGER_ENABLE
static uint16_t auto_pointer_layer_timer = 0;

#    ifndef CHARYBDIS_AUTO_POINTER_LAYER_TRIGGER_TIMEOUT_MS
#        define CHARYBDIS_AUTO_POINTER_LAYER_TRIGGER_TIMEOUT_MS 1000
#    endif // CHARYBDIS_AUTO_POINTER_LAYER_TRIGGER_TIMEOUT_MS

#    ifndef CHARYBDIS_AUTO_POINTER_LAYER_TRIGGER_THRESHOLD
#        define CHARYBDIS_AUTO_POINTER_LAYER_TRIGGER_THRESHOLD 8
#    endif // CHARYBDIS_AUTO_POINTER_LAYER_TRIGGER_THRESHOLD
#endif     // CHARYBDIS_AUTO_POINTER_LAYER_TRIGGER_ENABLE

#define LOWER MO(LAYER_LOWER)
#define RAISE MO(LAYER_RAISE)
#define PT_Z LT(LAYER_POINTER, KC_Z)
#define PT_SLSH LT(LAYER_POINTER, KC_SLSH)

#ifndef POINTING_DEVICE_ENABLE
#    define DRGSCRL KC_NO
#    define DPI_MOD KC_NO
#    define S_D_MOD KC_NO
#    define SNIPING KC_NO
#endif // !POINTING_DEVICE_ENABLE

// TODO: Tap Dance Declarations
enum {
  TD_ESC_CAPS = 0,
  TD_LGUI_CTRLB
};
//Tap Dance Definitions
tap_dance_action_t tap_dance_actions[] = {
  //Tap once for Esc, twice for Caps Lock
  [TD_ESC_CAPS]  = ACTION_TAP_DANCE_DOUBLE(KC_ESC, KC_CAPS),
  // Other declarations would go here, separated by commas, if you have them
  [TD_LGUI_CTRLB]  = ACTION_TAP_DANCE_DOUBLE(KC_LGUI, LCTL(KC_B))
};

// clang-format off
const uint16_t PROGMEM keymaps[][MATRIX_ROWS][MATRIX_COLS] = {
  [LAYER_BASE] = LAYOUT(
  // ╭──────────────────────────────────────────────────────╮ ╭──────────────────────────────────────────────────────╮
      KC_ESC,  KC_1,    KC_2,    KC_3,    KC_4,    KC_5,       KC_6,   KC_7,   KC_8,     KC_9,    KC_0,    KC_DEL,
  // ├──────────────────────────────────────────────────────┤ ├──────────────────────────────────────────────────────┤
      KC_TAB,  KC_Q,    KC_W,    KC_E,    KC_R,    KC_T,       KC_Y,   KC_U,   KC_I,     KC_O,    KC_P,    KC_BSLS,
  // ├──────────────────────────────────────────────────────┤ ├──────────────────────────────────────────────────────┤
      KC_RCTL, KC_A,    KC_S,    KC_D,    KC_F,    KC_G,       KC_H,   KC_J,   KC_K,     KC_L,    KC_SCLN, MT(MOD_LCTL | MOD_RCTL, KC_QUOT),
  // ├──────────────────────────────────────────────────────┤ ├──────────────────────────────────────────────────────┤
      KC_LSFT, KC_Z,    KC_X,    KC_C,    KC_V,    KC_B,       KC_N,   KC_M,   KC_COMM,  KC_DOT,  KC_SLSH, KC_RSFT,
  // ╰──────────────────────────────────────────────────────┤ ├──────────────────────────────────────────────────────╯
              MO(LAYER_POINTER), TD(TD_LGUI_CTRLB), KC_LALT,   LT(3, KC_BSPC), LT(LAYER_SYMBOLS, KC_SPACE),
                                   KC_RCTL, MO(LAYER_LOWER),   LT(LAYER_LOWER, KC_ENTER)
  //                            ╰───────────────────────────╯ ╰──────────────────╯
  ),

  [LAYER_POINTER] = LAYOUT(
  // ╭──────────────────────────────────────────────────────╮ ╭──────────────────────────────────────────────────────╮
      KC_GRV,  KC_F1,   KC_F2,   KC_F3,   KC_F4,   KC_F5,      KC_F6,   KC_F7,   KC_F8,   KC_F9,   KC_F10,  _______,
  // ├──────────────────────────────────────────────────────┤ ├──────────────────────────────────────────────────────┤
      A(KC_TAB),KC_LGUI,KC_LALT, SNIPING, KC_BTN4, KC_BTN5,    KC_HOME, KC_PGDN, KC_PGUP, KC_END,  KC_F11,  KC_F12,
  // ├──────────────────────────────────────────────────────┤ ├──────────────────────────────────────────────────────┤
      KC_RCTL, G(KC_A), S(KC_BTN3),KC_BTN2,KC_BTN1,KC_BTN3,    KC_LEFT, KC_DOWN, KC_UP,   KC_RIGHT,KC_MINUS,KC_EQUAL,
  // ├──────────────────────────────────────────────────────┤ ├──────────────────────────────────────────────────────┤
      KC_LSFT, G(KC_Z), G(KC_X), G(KC_C), G(KC_V), DRGSCRL,    RCS(KC_TAB), C(KC_TAB), KC_INS,  KC_INS,  KC_LBRC, KC_RBRC,
  // ╰──────────────────────────────────────────────────────┤ ├──────────────────────────────────────────────────────╯
                                  _______, _______, _______,     _______, _______,
                                           _______, _______,     _______
  //                            ╰───────────────────────────╯ ╰──────────────────╯
),

  [LAYER_LOWER] = LAYOUT(
  // ╭──────────────────────────────────────────────────────╮ ╭──────────────────────────────────────────────────────╮
      KC_ESC, LCAG(KC_F1),LCAG(KC_F2),LCAG(KC_F3),LCAG(KC_F4), LCAG(KC_F5), LCAG(KC_F6), LCAG(KC_F7), LCAG(KC_F8), LCAG(KC_F9), LCAG(KC_F10), _______,
  // ├──────────────────────────────────────────────────────┤ ├──────────────────────────────────────────────────────┤
      XXXXXXX,LCA(KC_Q),LCA(KC_W),LCA(KC_E),LCA(KC_R),LCA(KC_T),LCA(KC_Y), LCA(KC_U), LCA(KC_I), LCA(KC_O), LCAG(KC_F11),LCAG(KC_F12),
  // ├──────────────────────────────────────────────────────┤ ├──────────────────────────────────────────────────────┤
      KC_CAPS,LCA(KC_A),LCA(KC_S),LCA(KC_D),LCA(KC_F),LCA(KC_G),LCA(KC_H), LCA(KC_J), LCA(KC_K), LCA(KC_L), LCA(KC_SCLN),LCA(KC_QUOT),
  // ├──────────────────────────────────────────────────────┤ ├──────────────────────────────────────────────────────┤
      _______,LCA(KC_Z),LCA(KC_X),LCA(KC_C),LCA(KC_V),MEH(KC_F1),LCA(KC_N),LCA(KC_M), LCA(KC_COMMA), LCA(KC_DOT), LCA(KC_SLSH), _______,
  // ╰──────────────────────────────────────────────────────┤ ├──────────────────────────────────────────────────────╯
                                  _______, _______, _______,    _______, _______,
                                           _______, _______,    _______
  //                            ╰─────────────────────── ────╯ ╰──────────────────╯
  ),

  [LAYER_RAISE] = LAYOUT(
  // ╭──────────────────────────────────────────────────────╮ ╭──────────────────────────────────────────────────────╮
      KC_ESC,  KC_F1,   KC_F2,   KC_F3,   KC_F4,   KC_F5,      KC_F6,   KC_F7,   KC_F8,   KC_F9,   KC_F10,  _______,
  // ├──────────────────────────────────────────────────────┤ ├──────────────────────────────────────────────────────┤
      _______, QK_RBT, LCAG(KC_W), KC_VOLU, G(KC_MINS),G(KC_EQL), RGB_SPI, RGB_HUI, RGB_SAI, RGB_VAI, KC_F11,  KC_F12,
  // ├──────────────────────────────────────────────────────┤ ├──────────────────────────────────────────────────────┤
      _______, XXXXXXX, KC_MPRV, KC_MPLY, KC_MNXT,  G(KC_0),   XXXXXXX, RGB_RMOD,RGB_TOG, RGB_MOD, XXXXXXX, EE_CLR,
  // ├──────────────────────────────────────────────────────┤ ├──────────────────────────────────────────────────────┤
      _______, XXXXXXX, XXXXXXX, KC_VOLD,  XXXXXXX, XXXXXXX,   XXXXXXX, XXXXXXX, RGB_M_P, XXXXXXX, DPI_RMOD, DPI_MOD,
  // ╰──────────────────────────────────────────────────────┤ ├──────────────────────────────────────────────────────╯
                                  _______, _______, _______,    _______, _______,
                                           _______, _______,    _______
  //                            ╰───────────────────────────╯ ╰──────────────────╯
  ),

  [LAYER_SYMBOLS] = LAYOUT(
  // ╭──────────────────────────────────────────────────────╮ ╭──────────────────────────────────────────────────────╮
      KC_GRV , _______, _______, _______, _______, _______,    _______, _______, _______, _______, _______, _______,
  // ├──────────────────────────────────────────────────────┤ ├──────────────────────────────────────────────────────┤
     S(KC_GRV), KC_QUOT, KC_LT,   KC_GT,   KC_DQT,  KC_DOT,     KC_COMM, KC_AMPR, KC_LBRC, KC_RBRC, KC_PERC, _______,
  // ├──────────────────────────────────────────────────────┤ ├──────────────────────────────────────────────────────┤
      _______, KC_EXLM, KC_MINS, KC_PLUS, KC_EQL,  KC_UNDS,    KC_PIPE, KC_COLN, KC_LPRN, KC_RPRN, KC_QUES, _______,
  // ├──────────────────────────────────────────────────────┤ ├──────────────────────────────────────────────────────┤
      _______, KC_CIRC, KC_SLSH, KC_ASTR, KC_BSLS, KC_HASH,    KC_TILD, KC_DLR, KC_LCBR, KC_RCBR, KC_AT,   _______,
  // ╰──────────────────────────────────────────────────────┤ ├──────────────────────────────────────────────────────╯
                                  _______, _______, _______,    _______, _______,
                                           _______, _______,    _______
  //                            ╰───────────────────────────╯ ╰──────────────────╯
  ),
};
// clang-format on

#ifdef POINTING_DEVICE_ENABLE
#    ifdef CHARYBDIS_AUTO_POINTER_LAYER_TRIGGER_ENABLE
report_mouse_t pointing_device_task_user(report_mouse_t mouse_report) {
    if (abs(mouse_report.x) > CHARYBDIS_AUTO_POINTER_LAYER_TRIGGER_THRESHOLD || abs(mouse_report.y) > CHARYBDIS_AUTO_POINTER_LAYER_TRIGGER_THRESHOLD) {
        if (auto_pointer_layer_timer == 0) {
            layer_on(LAYER_POINTER);
#        ifdef RGB_MATRIX_ENABLE
            rgb_matrix_mode_noeeprom(RGB_MATRIX_NONE);
            rgb_matrix_sethsv_noeeprom(HSV_GREEN);
#        endif // RGB_MATRIX_ENABLE
        }
        auto_pointer_layer_timer = timer_read();
    }
    return mouse_report;
}

void matrix_scan_user(void) {
    if (auto_pointer_layer_timer != 0 && TIMER_DIFF_16(timer_read(), auto_pointer_layer_timer) >= CHARYBDIS_AUTO_POINTER_LAYER_TRIGGER_TIMEOUT_MS) {
        auto_pointer_layer_timer = 0;
        layer_off(LAYER_POINTER);
#        ifdef RGB_MATRIX_ENABLE
        rgb_matrix_mode_noeeprom(RGB_MATRIX_DEFAULT_MODE);
#        endif // RGB_MATRIX_ENABLE
    }
}
#    endif // CHARYBDIS_AUTO_POINTER_LAYER_TRIGGER_ENABLE

#    ifdef CHARYBDIS_AUTO_SNIPING_ON_LAYER
layer_state_t layer_state_set_user(layer_state_t state) {
    charybdis_set_pointer_sniping_enabled(layer_state_cmp(state, CHARYBDIS_AUTO_SNIPING_ON_LAYER));
    return state;
}
#    endif // CHARYBDIS_AUTO_SNIPING_ON_LAYER
#endif     // POINTING_DEVICE_ENABLE

#ifdef RGB_MATRIX_ENABLE
// Forward-declare this helper function since it is defined in rgb_matrix.c.
void rgb_matrix_update_pwm_buffers(void);
#endif


// TODO: Custom RGB Matrix Effects
layer_state_t layer_state_set_user(layer_state_t state) {
    uint8_t layer      = get_highest_layer(state); // layer ID
    //uint8_t saturation = rgblight_get_sat();       // Current saturated color
    //uint8_t value      = rgblight_get_val();       // Current brightness value

    if (layer == 1) {
        rgb_matrix_mode_noeeprom(RGB_MATRIX_CUSTOM_layer_1_effect);
    } else if (layer == 2) {
        rgb_matrix_mode_noeeprom(RGB_MATRIX_CUSTOM_layer_2_effect);
    } else if (layer == 3) {
        rgb_matrix_mode_noeeprom(RGB_MATRIX_CUSTOM_layer_3_effect);
    } else if (layer == 4) {
        rgb_matrix_mode_noeeprom(RGB_MATRIX_CUSTOM_layer_4_effect);
    } else {
        // default layer
        rgb_matrix_mode_noeeprom(RGB_MATRIX_CUSTOM_base_effect);
    }
    return state;
}