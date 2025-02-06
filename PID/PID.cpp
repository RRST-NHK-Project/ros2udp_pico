#include "pico/stdlib.h"
#include "pico/time.h"
#include <stdio.h>
#include <stdlib.h>

// エンコーダに使用するGPIOピンの定義
constexpr uint ENCODER_A_PIN = 14;
constexpr uint ENCODER_B_PIN = 15;

// エンコーダのカウント値（符号付き整数）
volatile int32_t encoder_count = 0;
// 前回の A, B の状態（2ビットで表現）
volatile uint8_t lastEncoded = 0;

// PID用の変数（ここではチャネル１のみ）
double target = 360.0;  // 目標角度（例: 90°）
double deg = 0.0;      // 現在の角度
double Kp = 0.01;      // Pゲイン
double Ki = 0.0;      // Iゲイン
double Kd = 0.0;      // Dゲイン
double Error = 0.0;
double last_Error = 0.0;
double Integral = 0.0;
double Differential = 0.0;
double Output = 0.0;
double deg_limit = 360.0; // 最大角度（1回転＝360°）
double pwm_limit = 0.6;   // 出力の上限（安全装置の役割）

// dt用の変数（秒単位）
uint32_t dt_us = 0;
double dt_d = 0.0;

// エンコーダの状態変化を処理するIRQコールバック関数
// ※ A相、B相いずれかにエッジが発生した場合に呼ばれます
void gpio_callback(uint gpio, uint32_t events) {
    // 現在の A, B の状態を取得
    uint8_t a = gpio_get(ENCODER_A_PIN);
    uint8_t b = gpio_get(ENCODER_B_PIN);
    uint8_t encoded = (a << 1) | b;  // 2ビットの状態
    // 4ビットの状態遷移として判定（前回状態 << 2 | 現在の状態）
    uint8_t sum = (lastEncoded << 2) | encoded;

    // 状態遷移に応じたカウントアップ／ダウンの例
    switch(sum) {
        // 時計回りの例
        case 0b0001:
        case 0b0111:
        case 0b1110:
        case 0b1000:
            encoder_count++;
            break;
        // 反時計回りの例
        case 0b0010:
        case 0b0100:
        case 0b1101:
        case 0b1011:
            encoder_count--;
            break;
        default:
            // その他の遷移はノイズまたは不正な状態として無視
            break;
    }
    lastEncoded = encoded;
}

int main() {
    // 標準入出力（シリアル）初期化
    stdio_init_all();

    // エンコーダ A相用GPIOの初期化（入力モード、内部プルアップ有効）
    gpio_init(ENCODER_A_PIN);
    gpio_set_dir(ENCODER_A_PIN, GPIO_IN);
    gpio_pull_up(ENCODER_A_PIN);

    // エンコーダ B相用GPIOの初期化（入力モード、内部プルアップ有効）
    gpio_init(ENCODER_B_PIN);
    gpio_set_dir(ENCODER_B_PIN, GPIO_IN);
    gpio_pull_up(ENCODER_B_PIN);

    // A相に対してIRQコールバックを登録
    gpio_set_irq_enabled_with_callback(ENCODER_A_PIN,
                                       GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL,
                                       true, gpio_callback);
    // B相もエッジ検出する（コールバックは A相で登録済みのものが呼ばれます）
    gpio_set_irq_enabled(ENCODER_B_PIN,
                         GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL,
                         true);

    // 初期状態の保存
    lastEncoded = ((gpio_get(ENCODER_A_PIN) << 1) | gpio_get(ENCODER_B_PIN));

    // PIDループ用のタイマー開始
    absolute_time_t last_time = get_absolute_time();

    while (true) {
        // 現在時刻を取得し、前回からの経過時間 dt を計算（マイクロ秒単位）
        absolute_time_t now = get_absolute_time();
        dt_us = absolute_time_diff_us(last_time, now);
        dt_d = dt_us / 1000000.0;  // 秒に変換
        last_time = now;

        // エンコーダのカウント値から現在角度（度）を算出
        // 例：1回転あたり 8192 カウントの場合
        deg = (double)encoder_count / 8192.0 * 360.0;

        // PID計算
        Error = target - deg;
        // 積分項は台形則による近似
        Integral += (Error + last_Error) * dt_d / 2.0;
        Differential = (Error - last_Error) / dt_d;
        last_Error = Error;

        // PID出力の累積（シンプルな実装例）
        Output += (Kp * Error) + (Ki * Integral) + (Kd * Differential);

        // 安全のため、出力値の絶対値に対して上限を設ける
        double pid_output = Output;
        if (pid_output > 0) {
            pid_output = pid_output > pwm_limit ? pwm_limit : pid_output;
        } else {
            pid_output = (-pid_output > pwm_limit) ? -pwm_limit : pid_output;
        }

        // シリアル出力：エンコーダカウント、角度、誤差、PID出力
        printf("Encoder Count: %d, Angle: %.2f, Error: %.2f, PID Output: %.2f\n",encoder_count, deg, Error, pid_output);
    }

    return 0;
}
