#pragma once //多重読み込み防止

//PIDコントローラークラス

class PIDController {
    public ://外部からアクセスできる部分

        /**
         * @brief PIDコントローラのコンストラクタ
         * @param [in] kp 比例ゲイン
         * @param [in] ki 積分ゲイン
         * @param [in] kd 微分ゲイン
         */
        PIDController(double kp, double ki, double kd);

        /**
         * @brief PID計算処理
         * @param [in] target_speed 目標速度
         * @param [in] current_speed 現在の速度
         * @return PID計算による出力値
         */
        double calculate(double target_speed, double current_speed);

    private ://カプセル化
    //PIDゲイン
    double kp_;
    double ki_;
    double kd_;

    double pre_error_; //前回の誤差
    double integral_; //誤差の積分値
};