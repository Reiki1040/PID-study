#include "PIDController.h" //ヘッダファイル読み込み

//コンストラクタ、インスタンス生成時のセットアップ関数
PIDController::PIDController(double kp, double ki, double kd)://PIDControllerクラスのPIDController()関数の処理
    kp_(kp), ki_(ki), kd_(kd), pre_error_(0.0), integral_(0.0){
    }

//制御量を計算する関数実装
double PIDController::calculate(double target_speed, double current_speed){

    //誤差の計算
    double error = target_speed - current_speed;

    //積分項の計算 誤差の蓄積
    integral_ += error;

    //微分項の計算 誤差の変化量
    double derivative = error - pre_error_;

    //PID制御による出力値の計算
    double output = (kp_ * error) + (ki_ * integral_) * (kd_ * derivative);

    //次回計算用に現在の誤差をpre_errorに格納
    pre_error_ = error;

    return output;
}
