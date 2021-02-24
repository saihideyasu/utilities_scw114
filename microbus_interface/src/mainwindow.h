#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <ros/ros.h>
#include <iomanip>
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int8.h>
#include <std_msgs/UInt8.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <can_msgs/Frame.h>
#include <mobileye_560_660_msgs/AftermarketLane.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_datatypes.h>
#include <autoware_can_msgs/MicroBusCan501.h>
#include <autoware_can_msgs/MicroBusCan502.h>
#include <autoware_can_msgs/MicroBusCan503.h>
#include <autoware_msgs/LocalizerMatchStat.h>
#include <autoware_can_msgs/MicroBusCanSenderStatus.h>
#include <autoware_msgs/DifferenceToWaypointDistance.h>
#include <autoware_config_msgs/ConfigMicroBusCan111SCW.h>
#include <autoware_can_msgs/MicroBusCanVelocityParam.h>
#include <autoware_msgs/WaypointParam.h>
#include <autoware_msgs/GnssStandardDeviation.h>
#include <autoware_msgs/NDTStat.h>
#include <autoware_system_msgs/Date.h>
#include <autoware_msgs/StopperDistance.h>
#include <autoware_msgs/TrafficLight.h>
#include <autoware_msgs/VehicleCmd.h>
#include <autoware_msgs/LaneArray.h>
#include <QFileDialog>
#include <QStandardPaths>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(ros::NodeHandle nh, ros::NodeHandle p_nh, QWidget *parent = nullptr);
    ~MainWindow();

    void window_updata();
private:
    const short PEDAL_VOLTAGE_CENTER_ = 1024;//ペダルをニュートラルにしている際の電圧値
    static const int TRAFFIC_LIGHT_RED = 0;
    static const int TRAFFIC_LIGHT_GREEN  = 1;
    static const int TRAFFIC_LIGHT_UNKNOWN = 2;

    static const int LOAD_NAME_MAX = 4;
    int load_name_count_;
    Ui::MainWindow *ui;

    ros::NodeHandle nh_, private_nh_;

    ros::Publisher pub_unlock_;//デバイス立ち上がり時のLOCKを解除
    ros::Publisher pub_drive_mode_, pub_steer_mode_;//autoモードとmanualモードのチェンジ
    ros::Publisher pub_drive_control_;//driveのコントロールモード(velocity操作とstroke操作の切り替え)
    ros::Publisher pub_drive_input_, pub_steer_input_;//programモード時の自動入力と手動入力の切り替え
    ros::Publisher pub_drive_clutch_, pub_steer_clutch_;//クラッチの状態変更フラグ
    ros::Publisher pub_blinker_right_, pub_blinker_left_, pub_blinker_stop_; //ウィンカー
    ros::Publisher pub_error_lock_;//エラーがでている場合、canアプリにロック情報を送る
    ros::Publisher pub_use_safety_localizer_;//localizer関連のセーフティのチェック
    ros::Publisher pub_log_write_, pub_log_stop, pub_log_folder_;//ログ出力通知
    ros::Publisher pub_use_distance_localizer_;//距離のフェイルセーブを使用するか

    ros::Subscriber sub_can501_, sub_can502_, sub_can503_;//マイクロバスcanのID501,502
    ros::Subscriber sub_can_status_;//canステータス情報
    ros::Subscriber sub_distance_angular_check_, sub_distance_angular_check_ndt_, sub_distance_angular_check_ekf_, sub_distance_angular_check_gnss_;//経路と自車位置のチェック用
    ros::Subscriber sub_config_;
    ros::Subscriber sub_localizer_select_;//localizerの遷移状態 
    ros::Subscriber sub_localizer_match_stat_;//localizerのマッチング状態
    ros::Subscriber sub_can_velocity_param_;//canの速度情報
    ros::Subscriber sub_stopper_distance_;//停止線の位置情報
    ros::Subscriber sub_waypoint_param_;//経路の埋め込み情報
    ros::Subscriber sub_imu_;//IMU情報
    ros::Subscriber sub_gnss_pose_, sub_gnss_deviation_, sub_ndt_stat_, sub_gnss_stat_, sub_ndt_stat_string_;
    ros::Subscriber sub_stroke_routine_;
    ros::Subscriber sub_mobileye_frame_;//mobileyeからのcanの生データ
    ros::Subscriber sub_gnss_time_;//gnssの時間
    ros::Subscriber sub_light_color_;//信号色
    ros::Subscriber sub_period_signal_takeover_;//定時信号での緑でのテイクオーバー
    ros::Subscriber sub_signal_change_time_;//次の信号の切り替わり時間
    ros::Subscriber sub_automode_mileage_;//自動走行時の距離
    ros::Subscriber sub_vehicle_cmd_;//canで処理される速度とステアのコマンド
    ros::Subscriber sub_cmd_select_;//ctrl_rawとtwist_rawをpublishしているノードの種類
    ros::Subscriber sub_load_name_;
    ros::Subscriber sub_base_waypoints_;

    void callbackCan501(const autoware_can_msgs::MicroBusCan501 &msg);//マイコン応答ID501
    void callbackCan502(const autoware_can_msgs::MicroBusCan502 &msg);//マイコン応答ID502
    void callbackCan503(const autoware_can_msgs::MicroBusCan503 &msg);//マイコン応答ID502
    void callbackCanStatus(const autoware_can_msgs::MicroBusCanSenderStatus &msg);//canステータス
    void callbackDistanceAngularCheck(const autoware_msgs::DifferenceToWaypointDistance &msg);
    void callbackDistanceAngularCheckNdt(const autoware_msgs::DifferenceToWaypointDistance &msg);
    void callbackDistanceAngularCheckEkf(const autoware_msgs::DifferenceToWaypointDistance &msg);
    void callbackDistanceAngularCheckGnss(const autoware_msgs::DifferenceToWaypointDistance &msg);
    void callbackConfig(const autoware_config_msgs::ConfigMicroBusCan111SCW &msg);
    void callbackLocalizerSelect(const std_msgs::Int32 &msg);//localizerの遷移状態 
    void callbackLocalizerMatchStat(const autoware_msgs::LocalizerMatchStat &msg);
    void callbackCanVelocityParam(const autoware_can_msgs::MicroBusCanVelocityParam &msg);
    void callbackStopperDistance(const autoware_msgs::StopperDistance &msg);
    void callbackWaypointParam(const autoware_msgs::WaypointParam &msg);
    void callbackImu(const sensor_msgs::Imu &msg);
    void callbackGnssPose(const geometry_msgs::PoseStamped &msg);
    void callbackGnssDeviation(const autoware_msgs::GnssStandardDeviation &msg);
    void callbackNdtStat(const autoware_msgs::NDTStat &msg);
    void callbackGnssStat(const std_msgs::UInt8 &msg);
    void callbackNdtStatString(const std_msgs::String &msg);
    void callbackStrokeRoutine(const std_msgs::String &msg);
    void callbackMobileyeCan(const can_msgs::Frame &msg);
    void callbackGnssTime(const autoware_system_msgs::Date &msg);
    void callbackLightColor(const autoware_msgs::TrafficLight &msg);
    void callbackSignalChangeTime(const std_msgs::Float64 &msg);
    void callbackPeriodSignalTakeover(const std_msgs::Bool &msg);
    void callbackAutomodeMileage(const std_msgs::Float64 &msg);
    void callbackVehicleCmd(const autoware_msgs::VehicleCmd &msg);
    void callbackCmdSelect(const std_msgs::Int32 &msg);
    void callbackLoadName(const std_msgs::String &msg);
    void callbackBaseWaypoints(const autoware_msgs::LaneArray &msg);

    void killWaypointsNode();
    void runWaypointsNode();

    autoware_can_msgs::MicroBusCan501 can501_;//マイコン応答ID501
    autoware_can_msgs::MicroBusCan502 can502_;//マイコン応答ID502
    autoware_can_msgs::MicroBusCan503 can503_;//マイコン応答ID503
    autoware_can_msgs::MicroBusCanSenderStatus can_status_;//canステータス
    autoware_msgs::DifferenceToWaypointDistance distance_angular_check_, distance_angular_check_ndt_, distance_angular_check_ekf_, distance_angular_check_gnss_;
    geometry_msgs::TwistStamped current_velocity_;//autowareからの現在の速度
    autoware_config_msgs::ConfigMicroBusCan111SCW config_;
    int localizer_select_;
    autoware_msgs::LocalizerMatchStat localizer_match_stat_;
    autoware_can_msgs::MicroBusCanVelocityParam can_velocity_param_;
    bool error_text_lock_;
    autoware_msgs::StopperDistance stopper_distance_;
    autoware_msgs::WaypointParam waypoint_param_;
    geometry_msgs::PoseStamped gnss_pose_;
    autoware_msgs::GnssStandardDeviation gnss_deviation_;
    autoware_system_msgs::Date gnss_time_;
    autoware_msgs::NDTStat ndt_stat_;
    unsigned char gnss_stat_;
    std::string ndt_stat_string_, stroke_routine_;
    sensor_msgs::Imu imu_;
    mobileye_560_660_msgs::AftermarketLane mobileye_lane_;
    std::string log_folder_;
    autoware_msgs::TrafficLight light_color_;
    double signal_change_time_;
    bool period_signal_takeover_;
    double automode_mileage_;
    autoware_msgs::VehicleCmd vehicle_cmd_;
    int cmd_select_;//ctrl_rawとtwist_rawをpublishしているノードの種類

    //タイマー
    ros::Time timer_error_lock_;

    //label color
    QPalette palette_drive_mode_ok_, palette_steer_mode_ok_;//autoモード表示テキストボックスのバックグラウンドカラーOK
    QPalette palette_drive_mode_error_, palette_steer_mode_error_;//autoモード表示テキストボックスのバックグラウンドカラーerror
    QPalette palette_drive_clutch_connect_, palette_drive_clutch_cut_;//ドライブクラッチのテキストボックスパレット
    QPalette palette_steer_clutch_connect_, palette_steer_clutch_cut_;//ハンドルクラッチのテキストボックスパレット
    QPalette palette_distance_angular_ok_, palette_distance_angular_error_;//経路との距離と角度チェックのテキストボックスパレット
    QPalette palette_localizer_select_ok_, palette_localizer_select_error_;//localizerの遷移状態のテキストボックスパレット
    QPalette palette_gnss_deviation_ok_, palette_gnss_deviation_error_;//gnssの誤差
    QPalette palette_score_ok_, palette_score_error_;
    QPalette palette_current_localizer_, palette_lb_normal_, palette_lb_localize_;
    QPalette palette_signal_text_green_, palette_signal_text_red_, palette_signal_text_unknown_, palette_period_signal_takeover_;//信号関連
    QPalette palette_stop_line_non_, palette_stop_line_middle_, palette_stop_line_stop_;
    QPalette palette_auto_check_ok_, palette_auto_check_error_;

    std::string gnss_time_str();
    void error_view(std::string error_message);

    double signal_red_green_time_, signal_green_yellow_time_, signal_yellow_red_time_, signal_red_green_time2_;
private slots:
    void publish_emergency_clear();
    void publish_Dmode_manual();
    void publish_Dmode_program();
    void publish_Smode_manual();
    void publish_Smode_program();
    void publish_Dmode_velocity();
    void publish_Dmode_stroke();
    void publish_Dmode_input_direct();
    void publish_Dmode_input_auto();
    void publish_Smode_input_direct();
    void publish_Smode_input_auto();
    void publish_drive_clutch_connect();
    void publish_drive_clutch_cut();
    void publish_steer_clutch_connect();
    void publish_steer_clutch_cut();
    void publish_blinker_right();
    void publish_blinker_left();
    void publish_blinker_stop();
    void publish_use_safety_localizer();
    void publish_use_distance_localizer();
    void publish_log_write();
    void publish_log_stop();
    void click_error_text_reset();
    void click_signal_time();
    void click_signal_time_clear();
    void click_log_folder();
    void click_load_next();
    void click_load_back();
};

#endif // MAINWINDOW_H
