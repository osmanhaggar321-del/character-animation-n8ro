#include "SimCharAnimCustomModelPlugin.h"

#include <model/AnimationModel.h>
#include <model/IModel.h>
#include <model/ModelFactoryRegistry.h>
#include <plugin/IModelPluginService.h>
#include <plugin/PluginContext.h>
#include <plugin/IPluginServices.h>

#include <cmath>
#include <memory>
#include <string>
#include <unordered_set>

namespace arkheon::sample::simcharanimcustommodel {
    namespace {

        // Smooth interpolation: maps t in [0,1] to a smooth curve
        static double smoothstep(double t) {
            t = std::max(0.0, std::min(1.0, t));
            return t * t * (3.0 - 2.0 * t);
        }

        class CustomAnimationModel final : public arkheon::astsim::IModel, public arkheon::astsim::IAnimationModel {
        public:
            [[nodiscard]] std::string getTypeName() const override {
                return "animationModelCustom";
            }
            [[nodiscard]] std::unique_ptr<arkheon::astsim::IModel> clone() const override {
                return std::make_unique<CustomAnimationModel>(*this);
            }

            [[nodiscard]] bool evaluate(
                const arkheon::astsim::AnimationModelInput& input,
                arkheon::astsim::AnimationModelOutput& output) override {

                const std::string& code = input.entity.activeAnimationCode;
                const double t = input.simulationTimeSeconds;

                output.clearExistingJointOverrides = true;
                output.jointOverrides.clear();

                std::unordered_set<std::string> joints;
                joints.reserve(input.entity.joints.size());
                for (const auto& j : input.entity.joints)
                    joints.insert(j.jointId);

                // ─────────────────────────────────────────────────────
                // STATE 1: Idle Breathing
                // Reference: Natural standing posture biomechanics
                // - Slow chest rise/fall simulating breath (0.4 Hz)
                // - Subtle lateral body sway (0.25 Hz)
                // - Arms hang naturally with slight abduction (~8 deg)
                // - Knees slightly flexed (~2 deg) for natural stance
                // ─────────────────────────────────────────────────────
                if (code == "Idle Breathing") {
                    const double breathe = std::sin(t * 0.4) * 0.03;   // chest rise ~1.7 deg
                    const double sway = std::sin(t * 0.25) * 0.01;  // lateral sway ~0.6 deg

                    if (hasJoint(joints, "LeftArm"))
                        output.jointOverrides.push_back({ "LeftArm",     0.0 + breathe, 0.0,  0.14 + sway });
                    if (hasJoint(joints, "RightArm"))
                        output.jointOverrides.push_back({ "RightArm",    0.0 + breathe, 0.0, -0.14 - sway });
                    if (hasJoint(joints, "LeftForeArm"))
                        output.jointOverrides.push_back({ "LeftForeArm",  0.10, 0.0, 0.0 });
                    if (hasJoint(joints, "RightForeArm"))
                        output.jointOverrides.push_back({ "RightForeArm", 0.10, 0.0, 0.0 });
                    if (hasJoint(joints, "LeftLeg"))
                        output.jointOverrides.push_back({ "LeftLeg",  0.03, 0.0, 0.0 });
                    if (hasJoint(joints, "RightLeg"))
                        output.jointOverrides.push_back({ "RightLeg", 0.03, 0.0, 0.0 });
                }

                // ─────────────────────────────────────────────────────
                // STATE 2: Idle Neutral
                // Reference: Human gait biomechanics (Winter, 2009)
                // - Gait cycle: ~0.9 Hz (normal walking cadence)
                // - Hip flexion/extension: ±25 deg (0.44 rad)
                // - Knee flexion: 0 to 60 deg (1.05 rad)
                // - Ankle dorsi/plantarflexion: ±15 deg (0.26 rad)
                // - Arm swing opposite to legs (contralateral pattern)
                // - Elbow flexed ~23 deg during swing
                // ─────────────────────────────────────────────────────
                else if (code == "Idle Neutral") {
                    const double cycle = t * 2.0 * 3.14159265 * 0.9;
                    const double hipSwing = std::sin(cycle) * 0.44;
                    const double kneeSwing = (std::sin(cycle) + 1.0) * 0.5 * 1.05;
                    const double ankleSwing = std::sin(cycle + 0.5) * 0.26;
                    const double armSwing = std::sin(cycle) * 0.30;

                    if (hasJoint(joints, "LeftUpLeg"))
                        output.jointOverrides.push_back({ "LeftUpLeg",   hipSwing,  0.0, 0.0 });
                    if (hasJoint(joints, "RightUpLeg"))
                        output.jointOverrides.push_back({ "RightUpLeg", -hipSwing,  0.0, 0.0 });
                    if (hasJoint(joints, "LeftLeg"))
                        output.jointOverrides.push_back({ "LeftLeg",  std::max(0.0, kneeSwing), 0.0, 0.0 });
                    if (hasJoint(joints, "RightLeg"))
                        output.jointOverrides.push_back({ "RightLeg", std::max(0.0, -kneeSwing + 1.05), 0.0, 0.0 });
                    if (hasJoint(joints, "LeftFoot"))
                        output.jointOverrides.push_back({ "LeftFoot",   ankleSwing,  0.0, 0.0 });
                    if (hasJoint(joints, "RightFoot"))
                        output.jointOverrides.push_back({ "RightFoot", -ankleSwing,  0.0, 0.0 });
                    if (hasJoint(joints, "LeftArm"))
                        output.jointOverrides.push_back({ "LeftArm",  -armSwing, 0.0,  0.10 });
                    if (hasJoint(joints, "RightArm"))
                        output.jointOverrides.push_back({ "RightArm",  armSwing, 0.0, -0.10 });
                    if (hasJoint(joints, "LeftForeArm"))
                        output.jointOverrides.push_back({ "LeftForeArm",  0.40, 0.0, 0.0 });
                    if (hasJoint(joints, "RightForeArm"))
                        output.jointOverrides.push_back({ "RightForeArm", 0.40, 0.0, 0.0 });
                }

                // ─────────────────────────────────────────────────────
                // STATE 3: Idle Shake
                // Reference: Deep squat biomechanics
                // - Hip flexion: ~60 deg (1.05 rad)
                // - Knee flexion: ~90 deg (1.57 rad)
                // - Ankle dorsiflexion: ~-20 deg (-0.35 rad)
                // - Arms extend forward for balance (counterbalance)
                // - Smooth transition via smoothstep over 1 second
                // ─────────────────────────────────────────────────────
                else if (code == "Idle Shake") {
                    const double localT = std::fmod(t, 8.0);
                    const double blend = smoothstep(std::min(localT, 1.0));

                    if (hasJoint(joints, "LeftUpLeg"))
                        output.jointOverrides.push_back({ "LeftUpLeg",  blend * 1.05, 0.0, 0.0 });
                    if (hasJoint(joints, "RightUpLeg"))
                        output.jointOverrides.push_back({ "RightUpLeg", blend * 1.05, 0.0, 0.0 });
                    if (hasJoint(joints, "LeftLeg"))
                        output.jointOverrides.push_back({ "LeftLeg",  blend * 1.57, 0.0, 0.0 });
                    if (hasJoint(joints, "RightLeg"))
                        output.jointOverrides.push_back({ "RightLeg", blend * 1.57, 0.0, 0.0 });
                    if (hasJoint(joints, "LeftFoot"))
                        output.jointOverrides.push_back({ "LeftFoot",  blend * (-0.35), 0.0, 0.0 });
                    if (hasJoint(joints, "RightFoot"))
                        output.jointOverrides.push_back({ "RightFoot", blend * (-0.35), 0.0, 0.0 });
                    if (hasJoint(joints, "LeftArm"))
                        output.jointOverrides.push_back({ "LeftArm",  blend * 0.52, 0.0,  0.15 });
                    if (hasJoint(joints, "RightArm"))
                        output.jointOverrides.push_back({ "RightArm", blend * 0.52, 0.0, -0.15 });
                    if (hasJoint(joints, "LeftForeArm"))
                        output.jointOverrides.push_back({ "LeftForeArm",  blend * 0.40, 0.0, 0.0 });
                    if (hasJoint(joints, "RightForeArm"))
                        output.jointOverrides.push_back({ "RightForeArm", blend * 0.40, 0.0, 0.0 });
                }

                // ─────────────────────────────────────────────────────
                // STATE 4: CustomIdle
                // Reference: Natural turning biomechanics
                // - Lower body rotates in turn direction (~15 deg)
                // - Upper body counter-rotates (natural dissociation)
                // - Small weight-shift steps during turn
                // - Arms maintain balance via slight counter-swing
                // ─────────────────────────────────────────────────────
                else if (code == "CustomIdle") {
                    const double turnCycle = std::sin(t * 0.7) * 0.26;  // ±15 deg rotation
                    const double stepShift = std::sin(t * 1.4) * 0.15;  // weight shift steps

                    if (hasJoint(joints, "LeftUpLeg"))
                        output.jointOverrides.push_back({ "LeftUpLeg",   stepShift,  turnCycle, 0.0 });
                    if (hasJoint(joints, "RightUpLeg"))
                        output.jointOverrides.push_back({ "RightUpLeg", -stepShift,  turnCycle, 0.0 });
                    if (hasJoint(joints, "LeftLeg"))
                        output.jointOverrides.push_back({ "LeftLeg",  std::max(0.0,  stepShift) * 0.5, 0.0, 0.0 });
                    if (hasJoint(joints, "RightLeg"))
                        output.jointOverrides.push_back({ "RightLeg", std::max(0.0, -stepShift) * 0.5, 0.0, 0.0 });
                    // Upper body counter-rotates naturally
                    if (hasJoint(joints, "LeftArm"))
                        output.jointOverrides.push_back({ "LeftArm",  0.05, -turnCycle * 0.5,  0.10 });
                    if (hasJoint(joints, "RightArm"))
                        output.jointOverrides.push_back({ "RightArm", 0.05, -turnCycle * 0.5, -0.10 });
                    if (hasJoint(joints, "LeftForeArm"))
                        output.jointOverrides.push_back({ "LeftForeArm",  0.25, 0.0, 0.0 });
                    if (hasJoint(joints, "RightForeArm"))
                        output.jointOverrides.push_back({ "RightForeArm", 0.25, 0.0, 0.0 });
                }

                else {
                    return false;
                }

                return !output.jointOverrides.empty();
            }

        private:
            [[nodiscard]] static bool hasJoint(
                const std::unordered_set<std::string>& joints,
                const char* id) {
                if (!id || *id == '\0') return false;
                if (joints.empty()) return true;
                return joints.find(id) != joints.end();
            }
        };

    } // namespace

    int SimCharAnimCustomModelPlugin::getInterfaceVersion() const { return 1; }

    arkheon::astlib::PluginMetadata SimCharAnimCustomModelPlugin::getMetadata() const {
        arkheon::astlib::PluginMetadata metadata;
        metadata.setPluginId("sim-char-anim-custom-model");
        metadata.setVersion("1.0.0");
        metadata.setAuthor("Arkheon Sample");
        return metadata;
    }

    void SimCharAnimCustomModelPlugin::initialize(arkheon::astlib::PluginContext& context) {
        initialized_ = true;
        shutdown_ = false;
        modelRegistered_ = false;
        modelType_ = "animationModelCustom";
        modelFactoryRegistry_ = nullptr;
        if (context.services) {
            auto* rawService = context.services->getService(arkheon::astsim::IModelPluginService::kPluginServiceId);
            auto* service = static_cast<arkheon::astsim::IModelPluginService*>(rawService);
            modelFactoryRegistry_ = service ? &service->modelFactoryRegistry() : nullptr;
        }
        if (!modelFactoryRegistry_) return;
        modelRegistered_ = modelFactoryRegistry_->registerFactory(
            modelType_, std::make_unique<CustomAnimationModel>());
    }

    void SimCharAnimCustomModelPlugin::tick(double dt) {
        static_cast<void>(dt);
        if (!initialized_ || shutdown_) return;
    }

    void SimCharAnimCustomModelPlugin::shutdown() {
        if (modelFactoryRegistry_ && modelRegistered_)
            static_cast<void>(modelFactoryRegistry_->unregisterFactory(modelType_));
        modelRegistered_ = false;
        shutdown_ = true;
        modelFactoryRegistry_ = nullptr;
    }

} // namespace arkheon::sample::simcharanimcustommodel

extern "C" {
    ARKHEON_ASTLIB_API arkheon::astlib::IPlugin* create_plugin() {
        return new arkheon::sample::simcharanimcustommodel::SimCharAnimCustomModelPlugin();
    }
    ARKHEON_ASTLIB_API void destroy_plugin(arkheon::astlib::IPlugin* plugin) {
        if (plugin) delete plugin;
    }
    ARKHEON_ASTLIB_API const char* get_plugin_signature() {
        return "ARKHEON_PLUGIN_V1";
    }
} // extern "C"