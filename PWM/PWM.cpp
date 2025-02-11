#include "pico/stdlib.h"
#include "hardware/pwm.h"

int main() {
    // 標準入出力の初期化（必要に応じてシリアルコンソール出力など）
    stdio_init_all();

    // PWM出力に使用するGPIOピンの設定（ここではGPIO15を使用）
    const uint PWM_PIN = 16;
    gpio_set_function(PWM_PIN, GPIO_FUNC_PWM);

    // 対応するPWMスライス番号とチャネル番号を取得
    uint slice_num = pwm_gpio_to_slice_num(PWM_PIN);
    uint channel = pwm_gpio_to_channel(PWM_PIN);

    // PWM周期（ラップ値）の設定
    // ここでは 8 ビット分解能の例として 0～255 の範囲になるように設定
    pwm_set_wrap(slice_num, 65535);

    // PWM出力を有効化
    pwm_set_enabled(slice_num, true);
    pwm_set_chan_level(slice_num, channel,  65535);
    // メインループ：デューティサイクルを変化させて LED の明るさをフェードさせる
    // while (true) {
    //     // 0 から 255 まで徐々に明るくする（フェードイン）
    //     for (int duty = 0; duty <= 65535; duty=duty+100) {
    //         pwm_set_chan_level(slice_num, channel, duty);
    //         sleep_ms(1);
    //     }
    //     // 255 から 0 まで徐々に暗くする（フェードアウト）
    //     for (int duty = 65535; duty >= 0; duty=duty-100) {
    //         pwm_set_chan_level(slice_num, channel, duty);
    //         sleep_ms(10);
    //     }
    // }

    return 0;
}
