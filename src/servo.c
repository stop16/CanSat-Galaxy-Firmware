#include "servo.h"
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "hardware/clocks.h"
#include <string.h> // memset 사용

// 디버그 메시지 활성화 (필요 시 주석 해제)
// #define DEBUG_SERVO

#ifdef DEBUG_SERVO
#include <stdio.h>
#endif

// --- 내부 상태 구조체 ---
typedef struct {
    uint16_t gpio_num;
    uint16_t slice_num;
    uint16_t chan_num; // A=0, B=1
    uint16_t wrap_val;
    uint16_t min_pulse_us;
    uint16_t max_pulse_us;
    bool is_initialized;
    bool is_attached; // PWM 슬라이스가 활성화되어 있는지 여부
} servo_info_t;

// --- 상태 저장 배열 ---
static servo_info_t servo_state[MAX_SERVOS];
static bool servo_state_initialized = false; // 배열 초기화 여부 플래그

// --- 내부 함수 ---

// servo_state 배열 초기화
static void initialize_servo_state() {
    if (!servo_state_initialized) {
        memset(servo_state, 0, sizeof(servo_state)); // 모든 멤버를 0 또는 false로 초기화
        // is_initialized 는 false 로 초기화됨
        servo_state_initialized = true;
#ifdef DEBUG_SERVO
        printf("Servo state array initialized.\n");
#endif
    }
}

// GPIO 번호로 servo_state 배열 인덱스 찾기
static int find_servo_index(uint16_t gpio_num) {
    initialize_servo_state(); // 배열이 초기화되었는지 확인
    for (int i = 0; i < MAX_SERVOS; ++i) {
        if (servo_state[i].is_initialized && servo_state[i].gpio_num == gpio_num) {
            return i; // 찾음
        }
    }
    return -1; // 못 찾음
}

// servo_state 배열에서 빈 슬롯 인덱스 찾기
static int find_free_index() {
    initialize_servo_state();
    for (int i = 0; i < MAX_SERVOS; ++i) {
        if (!servo_state[i].is_initialized) {
            return i; // 빈 슬롯 찾음
        }
    }
    return -1; // 빈 슬롯 없음 (MAX_SERVOS 초과)
}


// PWM 파라미터 계산 (이전과 거의 동일, 약간의 개선)
static bool calculate_pwm_params(uint32_t freq_hz, uint16_t *wrap_val, uint16_t *clk_div_int, uint16_t *clk_div_frac) {
    uint32_t sys_clk_hz = clock_get_hz(clk_sys);
    if (sys_clk_hz == 0) return false; // 클럭이 아직 설정되지 않았을 수 있음

    // 목표 분주비 계산 (wrap 값 최대 65535 고려)
    float divider = (float)sys_clk_hz / (freq_hz * 65536.0f);

    // Pico PWM 분주기는 1.0 ~ 255.9375 범위
    if (divider < 1.0f) divider = 1.0f;
    const float MAX_DIVIDER = 255.0f + (15.0f / 16.0f);
    if (divider > MAX_DIVIDER) {
#ifdef DEBUG_SERVO
        printf("Error: Cannot achieve %lu Hz with sys_clk %lu Hz. Required divider %.2f > %.2f\n",
               freq_hz, sys_clk_hz, divider, MAX_DIVIDER);
#endif
        return false; // 요청된 주파수 생성 불가
    }

    *clk_div_int = (uint16_t)divider;
    *clk_div_frac = (uint16_t)((divider - *clk_div_int) * 16.0f);

    // 실제 적용될 분주비로 wrap 값 계산
    float effective_divider = *clk_div_int + (*clk_div_frac / 16.0f);
    *wrap_val = (uint16_t)(((float)sys_clk_hz / (effective_divider * freq_hz)) - 1.0f);

    // wrap 값이 0이거나 너무 크면 문제 발생 가능성 있음 (실제로는 거의 발생 안함)
    if (*wrap_val == 0 || *wrap_val > 65535) { // wrap은 uint16_t이므로 > 65535는 불필요하나 명시적 표현
#ifdef DEBUG_SERVO
        printf("Warning: Calculated wrap value (%u) is out of optimal range.\n", *wrap_val);
#endif
         // 실제로는 분주기 제한으로 인해 이 조건에 도달하기 어려움
         if (*wrap_val == 0) return false; // 0이면 PWM 생성 불가
         *wrap_val = 65535; // 최대값으로 설정 (최선책)
    }
    return true;
}

// 각도를 PWM 레벨로 변환 (상태 구조체 사용)
static uint16_t angle_to_level(uint8_t angle, const servo_info_t *servo) {
    if (!servo || !servo->is_initialized) return 0; // 안전장치

    // 각도 제한
    if (angle > 180) {
        angle = 180;
    }

    // 각도(0-180) -> 펄스 폭(us) 변환 (캘리브레이션 값 사용)
    float pulse_us = servo->min_pulse_us + ((float)angle / 180.0f) * (servo->max_pulse_us - servo->min_pulse_us);

    // 펄스 폭(us) -> PWM 레벨 변환
    float period_us = 1000000.0f / SERVO_PWM_FREQ_HZ;
    uint16_t level = (uint16_t)((pulse_us / period_us) * (servo->wrap_val + 1));

    // 레벨 값이 wrap 값보다 크지 않도록 보장
    if (level > servo->wrap_val) {
        level = servo->wrap_val;
    }

    return level;
}


// --- 라이브러리 함수 구현 ---

bool servo_init(uint16_t gpio_num, uint16_t min_pulse_us, uint16_t max_pulse_us) {
    initialize_servo_state(); // 상태 배열 초기화 (최초 1회)

    // 1. 빈 슬롯 찾기
    int index = find_free_index();
    if (index == -1) {
#ifdef DEBUG_SERVO
        printf("Error: Maximum number of servos (%d) reached.\n", MAX_SERVOS);
#endif
        return false; // 슬롯 없음
    }

    // 2. 이미 초기화된 GPIO인지 확인
    if (find_servo_index(gpio_num) != -1) {
#ifdef DEBUG_SERVO
        printf("Error: Servo on GPIO %d already initialized.\n", gpio_num);
#endif
        return false; // 이미 초기화됨
    }

    // 3. 펄스 폭 유효성 검사 (선택 사항)
    if (min_pulse_us == 0 || max_pulse_us == 0 || min_pulse_us >= max_pulse_us) {
#ifdef DEBUG_SERVO
        printf("Error: Invalid pulse width settings for GPIO %d (min: %u, max: %u)\n", gpio_num, min_pulse_us, max_pulse_us);
#endif
        return false;
    }

    // 4. GPIO -> PWM 슬라이스/채널 정보 얻기
    uint16_t slice_num = pwm_gpio_to_slice_num(gpio_num);
    // Pico SDK는 잘못된 GPIO에 대해 UINT_MAX 반환 가능성 있음 (또는 함수 자체에서 assert)
    // 여기서는 pwm_init 등에서 내부적으로 처리될 것으로 기대
    uint16_t chan_num = pwm_gpio_to_channel(gpio_num); // PWM_CHAN_A or PWM_CHAN_B

    // 5. PWM 파라미터 계산
    uint16_t wrap_val, clk_div_int, clk_div_frac;
    if (!calculate_pwm_params(SERVO_PWM_FREQ_HZ, &wrap_val, &clk_div_int, &clk_div_frac)) {
#ifdef DEBUG_SERVO
        printf("Error: Could not calculate PWM parameters for GPIO %d.\n", gpio_num);
#endif
        return false; // 파라미터 계산 실패
    }

    // 6. GPIO를 PWM 기능으로 설정
    gpio_set_function(gpio_num, GPIO_FUNC_PWM);

    // 7. PWM 설정 구성
    pwm_config config = pwm_get_default_config();
    pwm_config_set_wrap(&config, wrap_val);
    pwm_config_set_clkdiv_int_frac(&config, clk_div_int, clk_div_frac);

    // 8. PWM 슬라이스 초기화 (슬라이스당 한 번만 실행되어야 함)
    //    주의: 다른 서보가 이미 같은 슬라이스를 사용 중일 수 있음.
    //    pwm_init은 내부적으로 해당 슬라이스가 이미 초기화되었는지 확인하지 않음.
    //    같은 슬라이스를 다른 설정으로 재초기화하면 문제가 발생할 수 있음.
    //    여기서는 모든 서보가 동일한 주파수(SERVO_PWM_FREQ_HZ)를 사용한다고 가정.
    //    만약 다른 주파수가 필요하다면, 슬라이스별 설정 관리 로직이 더 복잡해짐.
    pwm_init(slice_num, &config, true); // true: PWM 즉시 시작 (attached 상태)

    // 9. 상태 정보 저장
    servo_info_t *servo = &servo_state[index];
    servo->gpio_num = gpio_num;
    servo->slice_num = slice_num;
    servo->chan_num = chan_num;
    servo->wrap_val = wrap_val;
    servo->min_pulse_us = min_pulse_us;
    servo->max_pulse_us = max_pulse_us;
    servo->is_initialized = true;
    servo->is_attached = true; // 초기화 시 바로 attach

    // 10. 초기 각도(0도) 설정
    uint16_t initial_level = angle_to_level(0, servo);
    pwm_set_gpio_level(gpio_num, initial_level); // 또는 pwm_set_chan_level(slice_num, chan_num, initial_level);

#ifdef DEBUG_SERVO
    printf("Servo on GPIO %d initialized (Slice: %d, Chan: %d, Wrap: %u, MinPulse: %u, MaxPulse: %u).\n",
           gpio_num, slice_num, chan_num, wrap_val, min_pulse_us, max_pulse_us);
#endif

    return true; // 성공
}

// 기본값 사용하는 초기화 함수
bool servo_init_default(uint16_t gpio_num) {
    return servo_init(gpio_num, DEFAULT_SERVO_MIN_PULSE_US, DEFAULT_SERVO_MAX_PULSE_US);
}


bool servo_set(uint16_t gpio_num, uint8_t angle) {
    int index = find_servo_index(gpio_num);
    if (index == -1) {
#ifdef DEBUG_SERVO
        printf("Error: Servo on GPIO %d not initialized for set().\n", gpio_num);
#endif
        return false; // 초기화되지 않음
    }

    servo_info_t *servo = &servo_state[index];

    // 1. 만약 detach 상태였다면 re-attach (PWM 활성화)
    if (!servo->is_attached) {
        pwm_set_enabled(servo->slice_num, true);
        servo->is_attached = true;
#ifdef DEBUG_SERVO
        printf("Servo on GPIO %d re-attached (Slice %d enabled).\n", gpio_num, servo->slice_num);
#endif
    }

    // 2. 각도를 레벨로 변환
    uint16_t level = angle_to_level(angle, servo);

    // 3. PWM 레벨 설정
    pwm_set_gpio_level(servo->gpio_num, level);

#ifdef DEBUG_SERVO
    // printf("Servo on GPIO %d set to angle %u (Level: %u).\n", gpio_num, angle, level);
#endif

    return true; // 성공
}

bool servo_detach(uint16_t gpio_num) {
    int index = find_servo_index(gpio_num);
    if (index == -1) {
#ifdef DEBUG_SERVO
        printf("Error: Servo on GPIO %d not initialized for detach().\n", gpio_num);
#endif
        return false; // 초기화되지 않음
    }

    servo_info_t *servo = &servo_state[index];

    if (!servo->is_attached) {
#ifdef DEBUG_SERVO
        printf("Info: Servo on GPIO %d already detached.\n", gpio_num);
#endif
        return true; // 이미 detach 상태면 성공으로 간주
    }

    // PWM 슬라이스 비활성화
    // 주의: 이 슬라이스를 사용하는 다른 GPIO의 PWM 출력도 중단됨
    pwm_set_enabled(servo->slice_num, false);
    servo->is_attached = false;

#ifdef DEBUG_SERVO
    printf("Servo on GPIO %d detached (Slice %d disabled).\n", gpio_num, servo->slice_num);
#endif

    return true; // 성공
}

bool servo_attach(uint16_t gpio_num) {
     int index = find_servo_index(gpio_num);
    if (index == -1) {
#ifdef DEBUG_SERVO
        printf("Error: Servo on GPIO %d not initialized for attach().\n", gpio_num);
#endif
        return false; // 초기화되지 않음
    }

    servo_info_t *servo = &servo_state[index];

    if (servo->is_attached) {
#ifdef DEBUG_SERVO
        printf("Info: Servo on GPIO %d already attached.\n", gpio_num);
#endif
        return true; // 이미 attach 상태면 성공으로 간주
    }

    // PWM 슬라이스 활성화
    pwm_set_enabled(servo->slice_num, true);
    servo->is_attached = true;

#ifdef DEBUG_SERVO
    printf("Servo on GPIO %d attached (Slice %d enabled).\n", gpio_num, servo->slice_num);
#endif

    // 참고: attach만 하고 각도를 설정하지 않으면 마지막 설정값 또는 0으로 설정된 레벨이 출력될 것임
    // 필요하다면 여기서 특정 각도로 설정하는 로직 추가 가능

    return true; // 성공
}