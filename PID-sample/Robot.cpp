#include "Robot.h"
#include <iostream>

Robot::Robot()
    //メンバ変数初期化
    : target_wheel_speeds_(4, 0.0),
      target_wheel_angles_(4, 0.0),
      current_wheel_speeds_(4, 0.0),
      current_wheel_angles_(4, 0.0)
{
    //PIDゲイン
    const double KP = 0.5, KI = 0.1, KD = 0.2;

    //4輪分の駆動用PIDコントローラを生成
    for (int i = 0; i < 4; ++i) {
        drive_pid_controllers_.emplace_back(KP, KI, KD);
    }
}

void Robot::setTargetVelocity(double linear_x, double z_angle) {
    calculateWheelOutputs(linear_x, z_angle);
}

/**
 * @brief 逆運動学計算 (自動車モデル).
 */
void Robot::calculateWheelOutputs(double linear_x, double angular_z) {
    // 並進速度がほぼ0の場合、旋回せず停止.
    if (std::abs(linear_x) < 0.01) {
        for(int i = 0; i < 4; ++i) {
            target_wheel_speeds_[i] = 0.0;
        }
        return;
    }

    double steering_angle = 0.0;

    // 1. 旋回運動がある場合、ステアリング角度を計算 (バイシクルモデル).
    //    旋回半径 R = v / ω
    //    tan(δ) = L / R  (δ: 操舵角, L: 軸距)
    //    よって δ = atan(L * ω / v)
    if (std::abs(angular_z) > 0.01) {
        // L * ω / v
        steering_angle = std::atan(WHEEL_BASE_X * angular_z / linear_x);
    }

    // 2. 各車輪の目標角度を設定.
    //    前輪のみ操舵し、後輪は直進方向(0)で固定.
    target_wheel_angles_[0] = steering_angle; // 右前
    target_wheel_angles_[1] = steering_angle; // 左前
    target_wheel_angles_[2] = 0.0;            // 右後
    target_wheel_angles_[3] = 0.0;            // 左後

    //各車輪の目標回転速度を設定
    //簡単のため全車輪を同じ速度とする
    double wheel_speed_rads = linear_x / WHEEL_RADIUS;
    for (int i = 0; i < 4; ++i) {
        target_wheel_speeds_[i] = wheel_speed_rads;
    }
}

/**
 * @brief ロボットの状態更新
 */
void Robot::update() {
    //センサーからの現在値取得をシミュレート
    for (size_t i = 0; i < 4; ++i) {
        current_wheel_speeds_[i] += (target_wheel_speeds_[i] - current_wheel_speeds_[i]) * 0.1;
        current_wheel_angles_[i] += (target_wheel_angles_[i] - current_wheel_angles_[i]) * 0.1;
    }

    // 各車輪にPID制御を適用.
    for (int i = 0; i < 4; ++i) {
        //タイヤ回転速度のPID制御.
        double drive_output = drive_pid_controllers_[i].calculate(target_wheel_speeds_[i], current_wheel_speeds_[i]);
        //タイヤ角度のPID制御も別途必要

        std::cout << "Wheel " << i << " Angle: " << target_wheel_angles_[i]
                  << ", Speed: " << target_wheel_speeds_[i]
                  << ", Drive Output: " << drive_output << std::endl;
    }
}

/**
 * @brief メイン関数
 */
int main() {
    Robot my_robot;

    //前進しながら左にカーブする動きを指示.
    std::cout << "--- Commanding a left curve ---" << std::endl;
    my_robot.setTargetVelocity(1.0, 0.5);

    // 10回分の更新ループを実行.
    for (int i = 0; i < 10; ++i) {
        std::cout << "--- Update " << i + 1 << " ---" << std::endl;
        my_robot.update();
    }
    return 0;
}

