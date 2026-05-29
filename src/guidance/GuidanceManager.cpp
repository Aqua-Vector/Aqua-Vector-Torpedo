#include "GuidanceManager.hpp"

namespace guidance {

GuidanceManager::GuidanceManager() {
        reset();
}

void GuidanceManager::reset() {
        gpm_.reset();
        pnc_.reset();
}

ControlState GuidanceManager::update(const torpedo::domain::EskfState& my_state, const std::optional<Eigen::Vector2f>& lidar_pos, bool terminal_trigger, float dt) {
        ControlState cmd = {0.0f, 0.0f, 0.0f, 0};

        if (lidar_pos.has_value()) {
                tse_.updateFromLidar(*lidar_pos, dt);
        } else {
                tse_.predictVirtualState(dt);
        }

        if (!tse_.getState().is_valid) {
                return cmd;
        }

        float dist_to_target = (tse_.getState().pos - my_state.p.head<2>()).norm();
        GuidancePhase phase = gpm_.evaluatePhase(dist_to_target, terminal_trigger);

        if (phase == GuidancePhase::INTERCEPTED) {
                cmd.velocity = 0.0f;
                cmd.rudder = 0.0f;
        } else if (phase == GuidancePhase::STANDBY) {
                cmd.velocity = 0.0f;
                cmd.rudder = 0.0f;
        } else {
                cmd.velocity = DEFAULT_SPEED;

                float my_speed = my_state.v.head<2>().norm();
                Eigen::Vector2f pip = tse_.calculatePIP(my_state.p.head<2>(), my_speed);

                cmd.rudder = pnc_.calculateSteering(my_state, pip, dt);
        }

        return cmd;
}

} // namespace guidance
