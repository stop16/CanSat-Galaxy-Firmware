#ifndef SERVO_H_
#define SERVO_H_

#include <stdint.h>
#include <stdbool.h> // bool 타입 사용

// --- 설정값 ---
// 라이브러리가 관리할 수 있는 최대 서보 개수
#define MAX_SERVOS 8

// 기본 서보 펄스 폭 (마이크로초) - 일반적인 값
#define DEFAULT_SERVO_MIN_PULSE_US 1000
#define DEFAULT_SERVO_MAX_PULSE_US 2000

// 서보 모터 PWM 주파수 (Hz)
#define SERVO_PWM_FREQ_HZ 50

/**
 * @brief 지정된 GPIO 핀을 서보 모터 제어용으로 초기화합니다.
 *
 * PWM을 설정하고, 사용자가 제공한 캘리브레이션 값(펄스 폭)을 저장합니다.
 * 초기 각도는 0도로 설정됩니다.
 *
 * @param gpio_num 서보 모터를 연결할 GPIO 핀 번호.
 * @param min_pulse_us 0도에 해당하는 펄스 폭 (마이크로초).
 * @param max_pulse_us 180도에 해당하는 펄스 폭 (마이크로초).
 * @return 초기화 성공 시 true, 실패 시 false (잘못된 GPIO, 슬롯 부족 등).
 */
bool servo_init(uint16_t gpio_num, uint16_t min_pulse_us, uint16_t max_pulse_us);

/**
 * @brief 기본 펄스 폭(1000us, 2000us)을 사용하여 서보를 초기화합니다.
 *
 * servo_init(gpio_num, DEFAULT_SERVO_MIN_PULSE_US, DEFAULT_SERVO_MAX_PULSE_US)와 동일합니다.
 *
 * @param gpio_num 서보 모터를 연결할 GPIO 핀 번호.
 * @return 초기화 성공 시 true, 실패 시 false.
 */
bool servo_init_default(uint16_t gpio_num);


/**
 * @brief 지정된 GPIO 핀에 연결된 서보 모터의 각도를 설정합니다.
 *
 * 서보가 detach 상태였다면 자동으로 re-attach (PWM 활성화) 됩니다.
 * 각도는 0도에서 180도 사이로 제한됩니다.
 *
 * @param gpio_num 서보 모터가 연결된 GPIO 핀 번호.
 * @param angle 설정할 각도 (0 ~ 180).
 * @return 설정 성공 시 true, 실패 시 false (초기화되지 않은 서보 등).
 */
bool servo_set(uint16_t gpio_num, uint8_t angle);

/**
 * @brief 지정된 GPIO 핀에 연결된 서보 모터의 PWM 출력을 비활성화합니다 (Detach).
 *
 * 모터는 더 이상 힘을 유지하지 않습니다.
 * 참고: 동일한 PWM 슬라이스를 사용하는 다른 서보도 함께 비활성화될 수 있습니다.
 *
 * @param gpio_num 서보 모터가 연결된 GPIO 핀 번호.
 * @return 성공 시 true, 실패 시 false (초기화되지 않은 서보 등).
 */
bool servo_detach(uint16_t gpio_num);

/**
 * @brief 지정된 GPIO 핀에 연결된 서보 모터의 PWM 출력을 다시 활성화합니다 (Attach).
 *
 * 일반적으로 servo_set() 호출 시 자동으로 처리되지만, 명시적으로 활성화할 때 사용 가능합니다.
 *
 * @param gpio_num 서보 모터가 연결된 GPIO 핀 번호.
 * @return 성공 시 true, 실패 시 false (초기화되지 않은 서보 등).
 */
bool servo_attach(uint16_t gpio_num);


#endif // SERVO_H_