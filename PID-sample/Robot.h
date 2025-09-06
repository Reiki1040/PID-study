#pragma once

#include "PIDController.h"
#include <vector>
#include <cmath>

/**
 * @brief 四輪走行ロボットクラス(自動車的な)
 */
class Robot{
public:
    /**
     * @brief コンストラクタ
     */
    Robot();

    /**
     * @brief ロボットの状態更新
     */
    void update();

    /**
     * @brief 目標速度を設定
     * @param [in] linear_x 前後方向の速度 [m/s]
     * @param [in] z_angle Z軸周りの角速度 [rad/s]
     */
    void setTargetVelocity(double x_speed, double z_angle);

private:
    /**
     * @brief 逆運動学を計算 (自動車モデル)
     * @details 前輪の操舵角と全輪の回転速度を逆算する
     */
    void calculateWheelOutputs(double linear_x, double z_angle);

    //ロボット物理パラメータ
    const double WHEEL_RADIUS = 0.05; // 車輪の半径 [m].
    const double WHEEL_BASE_X = 0.20; // 前後車輪間の距離の半分[m].

    //メンバ変数
    std::vector<PIDController> drive_pid_controllers_; //車輪回転用PID

    std::vector<double> target_wheel_speeds_;    ///< 各車輪の目標回転速度 [rad/s].
    std::vector<double> target_wheel_angles_;    ///< 各車輪の目標角度 [rad].
    std::vector<double> current_wheel_speeds_;   ///< 各車輪の現在回転速度 [rad/s].
    std::vector<double> current_wheel_angles_;   ///< 各車輪の現在角度 [rad].
};

