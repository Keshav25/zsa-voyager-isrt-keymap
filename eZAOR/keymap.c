#include QMK_KEYBOARD_H
#include "version.h"
#include "keymap_steno.h"
#define MOON_LED_LEVEL LED_LEVEL
#ifndef ZSA_SAFE_RANGE
#define ZSA_SAFE_RANGE SAFE_RANGE
#endif

enum custom_keycodes {
  RGB_SLD = ZSA_SAFE_RANGE,
};



enum tap_dance_codes {
  DANCE_0,
};

#define DUAL_FUNC_0 LT(13, KC_F24)
#define DUAL_FUNC_1 LT(12, KC_V)
#define DUAL_FUNC_2 LT(6, KC_D)

const uint16_t PROGMEM keymaps[][MATRIX_ROWS][MATRIX_COLS] = {
  [0] = LAYOUT_voyager(
    DUAL_FUNC_0,    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_EQUAL,       
    KC_TRANSPARENT, KC_Y,           KC_C,           KC_L,           KC_M,           KC_K,                                           KC_Z,           KC_F,           KC_U,           KC_COMMA,       KC_QUOTE,       KC_BSLS,        
    LT(4, KC_BSPC), LT(3, KC_I),    MT(MOD_LALT, KC_S),LT(6, KC_R),    KC_TRANSPARENT, KC_G,                                           KC_P,           MT(MOD_RSFT, KC_N),MT(MOD_RCTL, KC_E),MT(MOD_RALT, KC_A),LT(3, KC_O),    LT(4, KC_SCLN), 
    TO(5),          KC_Q,           KC_V,           KC_W,           LT(4, KC_D),    KC_J,                                           KC_B,           KC_H,           KC_SLASH,       KC_DOT,         KC_X,           KC_TRANSPARENT, 
                                                    TD(DANCE_0),    LT(2, KC_TAB),                                  MT(MOD_LSFT, KC_BSPC),KC_ENTER
  ),
  [1] = LAYOUT_voyager(
    KC_ESCAPE,      KC_F1,          KC_F2,          KC_F3,          KC_F4,          KC_F5,                                          KC_F6,          KC_F7,          KC_F8,          KC_F9,          KC_F10,         KC_F11,         
    KC_GRAVE,       KC_EXLM,        KC_AT,          KC_HASH,        KC_DLR,         KC_PERC,                                        KC_SLASH,       KC_7,           KC_8,           KC_9,           KC_MINUS,       KC_F12,         
    KC_TRANSPARENT, DUAL_FUNC_1,    DUAL_FUNC_2,    KC_TRANSPARENT, KC_LEFT_SHIFT,  KC_TRANSPARENT,                                 KC_ASTR,        KC_4,           KC_5,           KC_6,           KC_PLUS,        KC_TRANSPARENT, 
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_MINUS,       KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_EQUAL,       KC_1,           KC_2,           KC_3,           KC_SPACE,       KC_ENTER,       
                                                    KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_DOT,         KC_0
  ),
  [2] = LAYOUT_voyager(
    RGB_TOG,        TOGGLE_LAYER_COLOR,RGB_MODE_FORWARD,RGB_SLD,        RGB_VAD,        RGB_VAI,                                        KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, 
    KC_TRANSPARENT, KC_TRANSPARENT, KC_AUDIO_VOL_DOWN,KC_AUDIO_VOL_UP,KC_AUDIO_MUTE,  KC_PSCR,                                        KC_PAGE_UP,     KC_HOME,        KC_UP,          KC_END,         KC_TRANSPARENT, KC_TRANSPARENT, 
    KC_TRANSPARENT, KC_MEDIA_PREV_TRACK,KC_MEDIA_NEXT_TRACK,KC_MEDIA_STOP,  KC_MEDIA_PLAY_PAUSE,KC_SCRL,                                        KC_PGDN,        KC_LEFT,        KC_DOWN,        KC_RIGHT,       KC_TRANSPARENT, KC_TRANSPARENT, 
    KC_TRANSPARENT, KC_MEDIA_REWIND,KC_MEDIA_FAST_FORWARD,KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_CAPS,        CW_TOGG,        KC_MS_WH_UP,    KC_MS_WH_DOWN,  KC_TRANSPARENT, KC_TRANSPARENT, 
                                                    KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_TRANSPARENT
  ),
  [3] = LAYOUT_voyager(
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, 
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TAB,         KC_TRANSPARENT, KC_TRANSPARENT, 
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_ENTER,       KC_ESCAPE,      KC_BSPC,        KC_TRANSPARENT, KC_TRANSPARENT, 
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, 
                                                    KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_TRANSPARENT
  ),
  [4] = LAYOUT_voyager(
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, 
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_TRANSPARENT, KC_PIPE,        KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, 
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_LPRN,        KC_LBRC,        KC_LCBR,        KC_GRAVE,       KC_TRANSPARENT, 
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_RPRN,        KC_RBRC,        KC_RCBR,        KC_SLASH,       KC_TRANSPARENT, 
                                                    KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_TRANSPARENT
  ),
  [5] = LAYOUT_voyager(
    QK_STENO_GEMINI,STN_N1,         STN_N2,         STN_N3,         STN_N4,         STN_N5,                                         STN_N6,         STN_N7,         STN_N8,         STN_N9,         STN_NA,         STN_NB,         
    STN_RES1,       STN_S1,         STN_TL,         STN_PL,         STN_HL,         STN_ST1,                                        STN_ST3,        STN_FR,         STN_PR,         STN_LR,         STN_TR,         STN_DR,         
    TO(0),          STN_S2,         STN_KL,         KC_TRANSPARENT, STN_RL,         STN_ST2,                                        STN_ST4,        STN_RR,         STN_BR,         STN_GR,         STN_SR,         STN_ZR,         
    TO(0),          STN_RES2,       STN_NC,         STN_A,          STN_O,          KC_TRANSPARENT,                                 KC_TRANSPARENT, STN_E,          STN_U,          STN_NC,         STN_FN,         STN_PWR,        
                                                    STN_A,          STN_O,                                          STN_E,          STN_U
  ),
  [6] = LAYOUT_voyager(
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, 
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, 
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_N,           KC_E,           KC_A,           KC_O,           KC_TRANSPARENT, 
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, 
                                                    KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_TRANSPARENT
  ),
  [7] = LAYOUT_voyager(
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, 
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, 
    KC_TRANSPARENT, KC_I,           KC_S,           KC_TRANSPARENT, KC_T,           KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, 
    KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, KC_TRANSPARENT, 
                                                    KC_TRANSPARENT, KC_TRANSPARENT,                                 KC_TRANSPARENT, KC_TRANSPARENT
  ),
};



uint16_t get_tapping_term(uint16_t keycode, keyrecord_t *record) {
    switch (keycode) {
        case TD(DANCE_0):
            return TAPPING_TERM + 150;
        default:
            return TAPPING_TERM;
    }
}

extern rgb_config_t rgb_matrix_config;

void keyboard_post_init_user(void) {
  rgb_matrix_enable();
}

const uint8_t PROGMEM ledmap[][RGB_MATRIX_LED_COUNT][3] = {
    [0] = { {174,163,255}, {174,163,255}, {174,163,255}, {174,163,255}, {174,163,255}, {174,163,255}, {174,163,255}, {174,163,255}, {174,163,255}, {174,163,255}, {174,163,255}, {174,163,255}, {174,163,255}, {174,163,255}, {174,163,255}, {174,163,255}, {174,163,255}, {174,163,255}, {174,163,255}, {174,163,255}, {174,163,255}, {174,163,255}, {174,163,255}, {174,163,255}, {174,163,255}, {174,163,255}, {174,163,255}, {174,163,255}, {174,163,255}, {174,163,255}, {174,163,255}, {174,163,255}, {174,163,255}, {174,163,255}, {174,163,255}, {174,163,255}, {174,163,255}, {174,163,255}, {174,163,255}, {174,163,255}, {174,163,255}, {174,163,255}, {174,163,255}, {174,163,255}, {174,163,255}, {174,163,255}, {174,163,255}, {174,163,255}, {174,163,255}, {174,163,255}, {174,163,255}, {174,163,255} },

    [2] = { {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {100,163,255}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0} },

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
    case 2:
      set_layer_color(2);
      break;
   default:
    if (rgb_matrix_get_flags() == LED_FLAG_NONE)
      rgb_matrix_set_color_all(0, 0, 0);
    break;
  }
  return true;
}


bool process_record_user(uint16_t keycode, keyrecord_t *record) {
  switch (keycode) {

    case DUAL_FUNC_0:
      if (record->tap.count > 0) {
        if (record->event.pressed) {
          register_code16(KC_MINUS);
        } else {
          unregister_code16(KC_MINUS);
        }
      } else {
        if (record->event.pressed) {
          register_code16(KC_ESCAPE);
        } else {
          unregister_code16(KC_ESCAPE);
        }  
      }  
      return false;
    case DUAL_FUNC_1:
      if (record->tap.count > 0) {
        if (record->event.pressed) {
          register_code16(KC_CIRC);
        } else {
          unregister_code16(KC_CIRC);
        }
      } else {
        if (record->event.pressed) {
          register_code16(KC_LEFT_GUI);
        } else {
          unregister_code16(KC_LEFT_GUI);
        }  
      }  
      return false;
    case DUAL_FUNC_2:
      if (record->tap.count > 0) {
        if (record->event.pressed) {
          register_code16(KC_AMPR);
        } else {
          unregister_code16(KC_AMPR);
        }
      } else {
        if (record->event.pressed) {
          register_code16(KC_LEFT_ALT);
        } else {
          unregister_code16(KC_LEFT_ALT);
        }  
      }  
      return false;
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

static tap dance_state[1];

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
        tap_code16(KC_SPACE);
        tap_code16(KC_SPACE);
        tap_code16(KC_SPACE);
    }
    if(state->count > 3) {
        tap_code16(KC_SPACE);
    }
}

void dance_0_finished(tap_dance_state_t *state, void *user_data) {
    dance_state[0].step = dance_step(state);
    switch (dance_state[0].step) {
        case SINGLE_TAP: register_code16(KC_SPACE); break;
        case SINGLE_HOLD: layer_on(1); break;
        case DOUBLE_TAP: register_code16(KC_SPACE); register_code16(KC_SPACE); break;
        case DOUBLE_HOLD: layer_on(1); break;
        case DOUBLE_SINGLE_TAP: tap_code16(KC_SPACE); register_code16(KC_SPACE);
    }
}

void dance_0_reset(tap_dance_state_t *state, void *user_data) {
    wait_ms(10);
    switch (dance_state[0].step) {
        case SINGLE_TAP: unregister_code16(KC_SPACE); break;
        case SINGLE_HOLD:
          layer_off(1);
        break;
        case DOUBLE_TAP: unregister_code16(KC_SPACE); break;
              case DOUBLE_HOLD: 
                layer_off(1);
                break;
        case DOUBLE_SINGLE_TAP: unregister_code16(KC_SPACE); break;
    }
    dance_state[0].step = 0;
}

tap_dance_action_t tap_dance_actions[] = {
        [DANCE_0] = ACTION_TAP_DANCE_FN_ADVANCED(on_dance_0, dance_0_finished, dance_0_reset),
};
