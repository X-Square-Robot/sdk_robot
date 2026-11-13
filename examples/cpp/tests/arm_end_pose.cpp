// 直接验证示例的到位逻辑，不启动 RPC 或连接机器人。
#define main arm_example_main
#include "../quanta_x3/arm_control.cpp"
#undef main

struct FeedbackArm
{
    geometry_msgs::PoseStamped feedback;
    int writes = 0;
    bool reject = false;
    SdkResult<void> SetEndPose(const geometry_msgs::Pose&, std::chrono::milliseconds)
    {
        ++writes;
        if (reject) return SdkResult<void>::Err({});
        return SdkResult<void>::Success();
    }
    SdkResult<geometry_msgs::PoseStamped> GetEndPose(std::chrono::milliseconds)
    {
        return SdkResult<geometry_msgs::PoseStamped>::Success(feedback);
    }
};

int main()
{
    geometry_msgs::Pose target;
    target.mutable_orientation()->set_w(1);
    FeedbackArm arm, other, waist;
    for (auto* endpoint : {&arm, &other, &waist}) *endpoint->feedback.mutable_pose() = target;
    const auto started = std::chrono::steady_clock::now();
    if (!WaitEndPose(arm, other, waist, target, target, target) || std::chrono::steady_clock::now() - started < 500ms ||
        arm.writes < 2 || other.writes != arm.writes || waist.writes != arm.writes)
        return 1;

    auto opposite_sign = target;
    opposite_sign.mutable_orientation()->set_w(-1);
    if (!PoseReached(opposite_sign, target)) return 2;
    auto invalid = target;
    invalid.mutable_position()->set_z(std::numeric_limits<double>::quiet_NaN());
    if (PoseReached(invalid, target)) return 3;
    auto rotated = target;
    rotated.mutable_orientation()->set_w(0);
    rotated.mutable_orientation()->set_x(1);
    if (PoseReached(rotated, target)) return 4;

    // 所选臂已到位，但保持臂没到位，仍须超时失败。
    other.feedback.mutable_pose()->mutable_position()->set_z(.1);
    if (WaitEndPose(arm, other, waist, target, target, target)) return 5;
    other.feedback.mutable_pose()->mutable_position()->set_z(0);
    waist.reject = true;
    if (WaitEndPose(arm, other, waist, target, target, target)) return 6;
    return 0;
}
