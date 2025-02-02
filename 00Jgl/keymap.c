#include QMK_KEYBOARD_H
#include "version.h"
#include "i18n.h"
#define MOON_LED_LEVEL LED_LEVEL
#define ML_SAFE_RANGE SAFE_RANGE

enum custom_keycodes {
  RGB_SLD = ML_SAFE_RANGE,
  ST_MACRO_0,
  ST_MACRO_1,
  ST_MACRO_2,
  ST_MACRO_3,
  ST_MACRO_4,
  ST_MACRO_5,
  ST_MACRO_6,
  ST_MACRO_7,
  ST_MACRO_8,
  ST_MACRO_9,
  ST_MACRO_10,
  ST_MACRO_11,
  ST_MACRO_12,
  ST_MACRO_13,
  JIGGLE,
};



enum tap_dance_codes {
  DANCE_0,
  DANCE_1,
  DANCE_2,
};

const uint16_t PROGMEM keymaps[][MATRIX_ROWS][MATRIX_COLS] = {
  [0] = LAYOUT_voyager(
    DM_PLY1,        KC_1,           KC_2,           KC_3,           KC_4,           KC_5,                                           KC_6,           KC_7,           KC_8,           KC_9,           KC_0,           KC_INSERT,      
    KC_TAB,         KC_Q,           KC_W,           KC_F,           KC_P,           KC_B,                                           KC_J,           KC_L,           KC_U,           KC_Y,           SE_SCLN,        SE_BSLS,        
    KC_ESCAPE,      MT(MOD_LGUI, KC_A),MT(MOD_LALT, KC_R),MT(MOD_LSFT, KC_S),MT(MOD_LCTL, KC_T),KC_G,                                           KC_M,           MT(MOD_RCTL, KC_N),MT(MOD_RSFT, KC_E),MT(MOD_RALT, KC_I),MT(MOD_RGUI, KC_O),SE_APOS,        
    JIGGLE,          KC_Z,           KC_X,           KC_C,           KC_D,           KC_V,                                           KC_K,           KC_H,           KC_COMMA,       KC_DOT,         SE_SLSH,        OSL(4),         
                                                    LT(1,KC_SPACE), KC_ENTER,                                       KC_BSPC,        LT(2,KC_SPACE)
  ),
  [1] = LAYOUT_voyager(
    DM_REC1,        KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, 
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, SE_LBRC,        SE_RBRC,        ST_MACRO_0,                                     SE_SLSH,        SE_ASTR,        SE_PLUS,        SE_COLN,        KC_PERC,        SE_PIPE,        
    KC_TRANSPARENT, SE_MINS,        SE_UNDS,        SE_LPRN,        SE_RPRN,        SE_GRV,                                         SE_CIRC,        SE_LESS,        SE_GRTR,        SE_EQL,         SE_DLR,         SE_DQUO,        
    KC_TRANSPARENT, TD(DANCE_0),    SE_AMPR,        SE_LCBR,        SE_RCBR,        ST_MACRO_1,                                     ST_MACRO_2,     KC_EXLM,        SE_AT,          KC_HASH,        TD(DANCE_1),    KC_TRANSPARENT, 
                                                    KC_TRANSPARENT, KC_NO,                                          KC_TRANSPARENT, KC_TRANSPARENT
  ),
  [2] = LAYOUT_voyager(
    KC_F1,          KC_F2,          KC_F3,          KC_F4,          KC_F5,          KC_F6,                                          KC_F7,          KC_F8,          KC_F9,          KC_F10,         KC_F11,         KC_F12,         
    KC_TRANSPARENT, KC_NO,          KC_7,           KC_8,           KC_9,           KC_PAGE_UP,                                     KC_MINUS,       KC_HOME,        KC_UP,          KC_END,         KC_TRANSPARENT, KC_TRANSPARENT, 
    KC_TRANSPARENT, KC_NO,          KC_4,           KC_5,           KC_6,           KC_PGDN,                                        KC_NO,          KC_LEFT,        KC_DOWN,        KC_RIGHT,       SE_SLSH,        KC_TRANSPARENT, 
    TD(DANCE_2),    KC_LEFT_ALT,    KC_1,           KC_2,           KC_3,           KC_NO,                                          KC_0,           KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_RIGHT_ALT,   KC_TRANSPARENT, 
                                                    KC_TRANSPARENT, MT(MOD_LSFT, KC_0),                                KC_NO,          KC_TRANSPARENT
  ),
  [3] = LAYOUT_voyager(
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, RGB_MODE_FORWARD,RGB_TOG,        
    KC_NO,          KC_NO,          KC_NO,          KC_NO,          KC_NO,          KC_NO,                                          KC_NO,          ST_MACRO_3,     ST_MACRO_4,     ST_MACRO_5,     ST_MACRO_6,     ST_MACRO_7,     
    KC_TRANSPARENT, KC_NO,          KC_NO,          KC_NO,          KC_NO,          KC_NO,                                          ST_MACRO_8,     ST_MACRO_9,     ST_MACRO_10,    ST_MACRO_11,    ST_MACRO_12,    ST_MACRO_13,    
    KC_NO,          KC_NO,          KC_NO,          KC_NO,          KC_NO,          KC_NO,                                          KC_NO,          LGUI(KC_1),     LGUI(KC_2),     LGUI(KC_3),     LGUI(KC_4),     LGUI(KC_5),     
                                                    KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_TRANSPARENT
  ),
  [4] = LAYOUT_voyager(
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, RGB_SPI,        RGB_VAI,        RGB_TOG,        
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, RGB_SPD,        RGB_VAD,        TOGGLE_LAYER_COLOR,
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_TRANSPARENT, SE_ADIA,        SE_OSLH,        SE_AA,          RGB_MODE_FORWARD,
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, 
                                                    KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_TRANSPARENT
  ),
};


uint16_t get_tapping_term(uint16_t keycode, keyrecord_t *record) {
    switch (keycode) {
        case KC_X:
            return 0;
        case KC_C:
            return 0;
        case KC_D:
            return 0;
        case KC_H:
            return 0;
        case KC_COMMA:
            return 0;
        case KC_DOT:
            return 0;
        case KC_LEFT:
            return 0;
        case KC_DOWN:
            return 0;
        default:
            return TAPPING_TERM;
    }
}

extern rgb_config_t rgb_matrix_config;

void keyboard_post_init_user(void) {
  rgb_matrix_enable();
}

const uint8_t PROGMEM ledmap[][RGB_MATRIX_LED_COUNT][3] = {
    [0] = { {161,181,225}, {161,181,225}, {161,181,225}, {161,181,225}, {161,181,225}, {161,181,225}, {161,181,225}, {161,181,225}, {161,181,225}, {161,181,225}, {161,181,225}, {161,181,225}, {161,181,225}, {161,181,225}, {161,181,225}, {161,181,225}, {161,181,225}, {161,181,225}, {161,181,225}, {161,181,225}, {161,181,225}, {161,181,225}, {161,181,225}, {161,181,225}, {161,181,225}, {161,181,225}, {161,181,225}, {161,181,225}, {161,181,225}, {161,181,225}, {161,181,225}, {161,181,225}, {161,181,225}, {161,181,225}, {161,181,225}, {161,181,225}, {161,181,225}, {161,181,225}, {161,181,225}, {161,181,225}, {161,181,225}, {161,181,225}, {161,181,225}, {161,181,225}, {161,181,225}, {161,181,225}, {161,181,225}, {161,181,225}, {161,181,225}, {161,181,225}, {161,181,225}, {161,181,225} },

    [1] = { {172,50,255}, {172,50,255}, {172,50,255}, {172,50,255}, {172,50,255}, {172,50,255}, {172,50,255}, {172,50,255}, {172,50,255}, {172,50,255}, {172,50,255}, {172,50,255}, {172,50,255}, {172,50,255}, {172,50,255}, {172,50,255}, {172,50,255}, {172,50,255}, {172,50,255}, {172,50,255}, {172,50,255}, {172,50,255}, {172,50,255}, {172,50,255}, {172,50,255}, {172,50,255}, {172,50,255}, {172,50,255}, {172,50,255}, {172,50,255}, {172,50,255}, {172,50,255}, {172,50,255}, {172,50,255}, {172,50,255}, {172,50,255}, {172,50,255}, {172,50,255}, {172,50,255}, {172,50,255}, {172,50,255}, {172,50,255}, {172,50,255}, {172,50,255}, {172,50,255}, {172,50,255}, {172,50,255}, {172,50,255}, {172,50,255}, {172,50,255}, {172,50,255}, {172,50,255} },

    [2] = { {41,255,255}, {41,255,255}, {41,255,255}, {41,255,255}, {41,255,255}, {41,255,255}, {41,255,255}, {41,255,255}, {41,255,255}, {41,255,255}, {41,255,255}, {41,255,255}, {41,255,255}, {41,255,255}, {41,255,255}, {41,255,255}, {41,255,255}, {41,255,255}, {41,255,255}, {41,255,255}, {41,255,255}, {41,255,255}, {41,255,255}, {41,255,255}, {41,255,255}, {41,255,255}, {41,255,255}, {41,255,255}, {41,255,255}, {41,255,255}, {41,255,255}, {41,255,255}, {41,255,255}, {41,255,255}, {41,255,255}, {41,255,255}, {41,255,255}, {41,255,255}, {41,255,255}, {41,255,255}, {41,255,255}, {41,255,255}, {41,255,255}, {41,255,255}, {41,255,255}, {41,255,255}, {41,255,255}, {41,255,255}, {41,255,255}, {41,255,255}, {41,255,255}, {41,255,255} },

    [4] = { {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {14,237,224}, {14,237,224}, {14,237,224}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {14,237,224}, {14,237,224}, {0,0,0}, {0,0,0}, {207,236,224}, {207,236,224}, {207,236,224}, {14,237,224}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0} },

};

void set_layer_color(int layer) {
  for (int i = 0; i < RGB_MATRIX_LED_COUNT; i++) {
    HSV hsv = {
      .h = pgm_read_byte(&ledmap[layer][i][0]),
      .s = pgm_read_byte(&ledmap[layer][i][1]),
      .v = pgm_read_byte(&ledmap[layer][i][2]),
    };
    if (!hsv.h && !hsv.s && !hsv.v) {
        rgb_matrix_set_color( i, 0, 0, 0 );
    } else {
        RGB rgb = hsv_to_rgb( hsv );
        float f = (float)rgb_matrix_config.hsv.v / UINT8_MAX;
        rgb_matrix_set_color( i, f * rgb.r, f * rgb.g, f * rgb.b );
    }
  }
}

bool rgb_matrix_indicators_user(void) {
  if (rawhid_state.rgb_control) {
      return false;
  }
  if (keyboard_config.disable_layer_led) { return false; }
  switch (biton32(layer_state)) {
    case 0:
      set_layer_color(0);
      break;
    case 1:
      set_layer_color(1);
      break;
    case 2:
      set_layer_color(2);
      break;
    case 4:
      set_layer_color(4);
      break;
   default:
    if (rgb_matrix_get_flags() == LED_FLAG_NONE)
      rgb_matrix_set_color_all(0, 0, 0);
    break;
  }
  return true;
}

bool process_record_user(uint16_t keycode, keyrecord_t *record) {


  // Custom QMK code
  if (record->event.pressed) {
    static deferred_token token = INVALID_DEFERRED_TOKEN;
    static report_mouse_t report = {0};
    if (token) {
      // If jiggler is currently running, stop when any key is pressed.
      cancel_deferred_exec(token);
      token = INVALID_DEFERRED_TOKEN;
      report = (report_mouse_t){};  // Clear the mouse.
      host_mouse_send(&report);
    } else if (keycode == JIGGLE) {

      uint32_t jiggler_callback(uint32_t trigger_time, void* cb_arg) {
        // Deltas to move in a circle of radius 20 pixels over 32 frames.
        static const int8_t deltas[32] = {
            0, -1, -2, -2, -3, -3, -4, -4, -4, -4, -3, -3, -2, -2, -1, 0,
            0, 1, 2, 2, 3, 3, 4, 4, 4, 4, 3, 3, 2, 2, 1, 0};
        static uint8_t phase = 0;
        // Get x delta from table and y delta by rotating a quarter cycle.
        report.x = deltas[phase];
        report.y = deltas[(phase + 8) & 31];
        phase = (phase + 1) & 31;
        host_mouse_send(&report);
        return 16;  // Call the callback every 16 ms.
      }

      token = defer_exec(1, jiggler_callback, NULL);  // Schedule callback.
    }
  }
  // End of custom QMK code


  switch (keycode) {
    case ST_MACRO_0:
    if (record->event.pressed) {
      SEND_STRING(SS_LSFT(SS_TAP(X_6)) SS_DELAY(100) SS_LSFT(SS_TAP(X_6)));
    }
    break;
    case ST_MACRO_1:
    if (record->event.pressed) {
      SEND_STRING(SS_LALT(SS_LCTL(SS_TAP(X_NUBS))) SS_DELAY(100) SS_LALT(SS_LCTL(SS_TAP(X_NUBS))));
    }
    break;
    case ST_MACRO_2:
    if (record->event.pressed) {
      SEND_STRING(SS_TAP(X_RIGHT_SHIFT) SS_DELAY(100) SS_TAP(X_INSERT));
    }
    break;
    case ST_MACRO_3:
    if (record->event.pressed) {
      SEND_STRING(SS_LCTL(SS_TAP(X_B)) SS_DELAY(100) SS_TAP(X_S) SS_DELAY(100) SS_TAP(X_0));
    }
    break;
    case ST_MACRO_4:
    if (record->event.pressed) {
      SEND_STRING(SS_LCTL(SS_TAP(X_B)) SS_DELAY(100) SS_TAP(X_S) SS_DELAY(100) SS_TAP(X_1));
    }
    break;
    case ST_MACRO_5:
    if (record->event.pressed) {
      SEND_STRING(SS_LCTL(SS_TAP(X_B)) SS_DELAY(100) SS_TAP(X_S) SS_DELAY(100) SS_TAP(X_2));
    }
    break;
    case ST_MACRO_6:
    if (record->event.pressed) {
      SEND_STRING(SS_LCTL(SS_TAP(X_B)) SS_DELAY(100) SS_TAP(X_S) SS_DELAY(100) SS_TAP(X_3));
    }
    break;
    case ST_MACRO_7:
    if (record->event.pressed) {
      SEND_STRING(SS_LCTL(SS_TAP(X_B)) SS_DELAY(100) SS_TAP(X_S) SS_DELAY(100) SS_TAP(X_4));
    }
    break;
    case ST_MACRO_8:
    if (record->event.pressed) {
      SEND_STRING(SS_LCTL(SS_TAP(X_B)) SS_DELAY(100) SS_TAP(X_LBRC));
    }
    break;
    case ST_MACRO_9:
    if (record->event.pressed) {
      SEND_STRING(SS_LCTL(SS_TAP(X_B)) SS_DELAY(100) SS_TAP(X_1));
    }
    break;
    case ST_MACRO_10:
    if (record->event.pressed) {
      SEND_STRING(SS_LCTL(SS_TAP(X_B)) SS_DELAY(100) SS_TAP(X_2));
    }
    break;
    case ST_MACRO_11:
    if (record->event.pressed) {
      SEND_STRING(SS_LCTL(SS_TAP(X_B)) SS_DELAY(100) SS_TAP(X_3));
    }
    break;
    case ST_MACRO_12:
    if (record->event.pressed) {
      SEND_STRING(SS_LCTL(SS_TAP(X_B)) SS_DELAY(100) SS_TAP(X_4));
    }
    break;
    case ST_MACRO_13:
    if (record->event.pressed) {
      SEND_STRING(SS_LCTL(SS_TAP(X_B)) SS_DELAY(100) SS_TAP(X_5));
    }
    break;

    case RGB_SLD:
      if (record->event.pressed) {
        rgblight_mode(1);
      }
      return false;
  }
  return true;
}


typedef struct {
    bool is_press_action;
    uint8_t step;
} tap;

enum {
    SINGLE_TAP = 1,
    SINGLE_HOLD,
    DOUBLE_TAP,
    DOUBLE_HOLD,
    DOUBLE_SINGLE_TAP,
    MORE_TAPS
};

static tap dance_state[3];

uint8_t dance_step(tap_dance_state_t *state);

uint8_t dance_step(tap_dance_state_t *state) {
    if (state->count == 1) {
        if (state->interrupted || !state->pressed) return SINGLE_TAP;
        else return SINGLE_HOLD;
    } else if (state->count == 2) {
        if (state->interrupted) return DOUBLE_SINGLE_TAP;
        else if (state->pressed) return DOUBLE_HOLD;
        else return DOUBLE_TAP;
    }
    return MORE_TAPS;
}


void on_dance_0(tap_dance_state_t *state, void *user_data);
void dance_0_finished(tap_dance_state_t *state, void *user_data);
void dance_0_reset(tap_dance_state_t *state, void *user_data);

void on_dance_0(tap_dance_state_t *state, void *user_data) {
    if(state->count == 3) {
        tap_code16(SE_TILD);
        tap_code16(SE_TILD);
        tap_code16(SE_TILD);
    }
    if(state->count > 3) {
        tap_code16(SE_TILD);
    }
}

void dance_0_finished(tap_dance_state_t *state, void *user_data) {
    dance_state[0].step = dance_step(state);
    switch (dance_state[0].step) {
        case SINGLE_TAP: register_code16(SE_TILD); break;
        case SINGLE_HOLD: register_code16(KC_LEFT_ALT); break;
        case DOUBLE_TAP: register_code16(SE_TILD); register_code16(SE_TILD); break;
        case DOUBLE_SINGLE_TAP: tap_code16(SE_TILD); register_code16(SE_TILD);
    }
}

void dance_0_reset(tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[0].step) {
        case SINGLE_TAP: unregister_code16(SE_TILD); break;
        case SINGLE_HOLD: unregister_code16(KC_LEFT_ALT); break;
        case DOUBLE_TAP: unregister_code16(SE_TILD); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(SE_TILD); break;
    }
    dance_state[0].step = 0;
}
void on_dance_1(tap_dance_state_t *state, void *user_data);
void dance_1_finished(tap_dance_state_t *state, void *user_data);
void dance_1_reset(tap_dance_state_t *state, void *user_data);

void on_dance_1(tap_dance_state_t *state, void *user_data) {
    if(state->count == 3) {
        tap_code16(SE_QUES);
        tap_code16(SE_QUES);
        tap_code16(SE_QUES);
    }
    if(state->count > 3) {
        tap_code16(SE_QUES);
    }
}

void dance_1_finished(tap_dance_state_t *state, void *user_data) {
    dance_state[1].step = dance_step(state);
    switch (dance_state[1].step) {
        case SINGLE_TAP: register_code16(SE_QUES); break;
        case SINGLE_HOLD: register_code16(KC_RIGHT_ALT); break;
        case DOUBLE_TAP: register_code16(SE_QUES); register_code16(SE_QUES); break;
        case DOUBLE_SINGLE_TAP: tap_code16(SE_QUES); register_code16(SE_QUES);
    }
}

void dance_1_reset(tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[1].step) {
        case SINGLE_TAP: unregister_code16(SE_QUES); break;
        case SINGLE_HOLD: unregister_code16(KC_RIGHT_ALT); break;
        case DOUBLE_TAP: unregister_code16(SE_QUES); break;
        case DOUBLE_SINGLE_TAP: unregister_code16(SE_QUES); break;
    }
    dance_state[1].step = 0;
}
void dance_2_finished(tap_dance_state_t *state, void *user_data);
void dance_2_reset(tap_dance_state_t *state, void *user_data);

void dance_2_finished(tap_dance_state_t *state, void *user_data) {
    dance_state[2].step = dance_step(state);
    switch (dance_state[2].step) {
        case DOUBLE_HOLD: register_code16(KC_LEFT_CTRL); break;
    }
}

void dance_2_reset(tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[2].step) {
        case DOUBLE_HOLD: unregister_code16(KC_LEFT_CTRL); break;
    }
    dance_state[2].step = 0;
}

tap_dance_action_t tap_dance_actions[] = {
        [DANCE_0] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_0, dance_0_finished, dance_0_reset),
        [DANCE_1] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_1, dance_1_finished, dance_1_reset),
        [DANCE_2] = ACTION_TAP_DANCE_FN_ADVANCED(NULL, dance_2_finished, dance_2_reset),
};



// Custom QMK here
const key_override_t delete_key_override = 
    ko_make_basic(MOD_MASK_SHIFT, KC_BSPC, KC_DEL);

const key_override_t **key_overrides = (const key_override_t *[]){
	&delete_key_override,
	NULL
};
