// 本文件提供 ToN 实验的 Visual Studio 手动入口，集中设置公共参数并按需选择单个实验。
// 默认不启动任何实验，避免直接运行程序时意外覆盖已经完成的结果。
#include "localization_experiments.h"

#ifndef TON_RERUN_TESTING
/// @brief 配置固定 ToN 批次，并提供 EXP1--EXP5 的基础手动调用入口。
/// @return 实验正常结束或未选择实验时返回 0；实验抛出异常时返回 1。
int main() {
    try {
        ExperimentRunOptions options;
        options.instance_count = 10;
        options.master_seed = 20260905u;
        options.rounding_trials = 2;
        options.ton_epsilon = 0.1;
        options.input_root = (fs::path(experimentDataPath) / "data_ToN" / "2026-09-07").string();
        options.output_name = "run_ton_01";
        options.conditions.clear();
        options.reuse_exp1_root.clear();

        // 需要运行时仅取消对应一行的注释；默认不执行，EXP4 也继续保持关闭。
        // exp1_different_user_number(options);
        // exp2_different_uav_number(options);
        // exp3_different_hard_user_ratio(options);
        // exp4_different_total_bandwidth(options);
        // exp5_different_location_error(options);

        return 0;
    } catch (const std::exception& error) {
        std::cerr << "实验执行失败：" << error.what() << std::endl;
        return 1;
    }
}
#endif // TON_RERUN_TESTING
