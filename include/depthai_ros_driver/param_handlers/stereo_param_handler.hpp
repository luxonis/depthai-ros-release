#pragma once

#include <string>
#include <unordered_map>
#include <vector>

#include "depthai/common/DeviceModelZoo.hpp"
#include "depthai/pipeline/datatype/NeuralDepthConfig.hpp"
#include "depthai/pipeline/datatype/StereoDepthConfig.hpp"
#include "depthai/pipeline/node/StereoDepth.hpp"
#include "depthai_ros_driver/param_handlers/base_param_handler.hpp"

namespace dai {
class CameraFeatures;
namespace node {
class NeuralDepth;
}
}  // namespace dai

namespace rclcpp {
class Node;
class Parameter;
}  // namespace rclcpp

namespace depthai_ros_driver {
namespace param_handlers {
enum class StereoType {
    Stereo,
    NeuralDepth,
    StereoAndNeuralDepth,
};
class StereoParamHandler : public BaseParamHandler {
   public:
    explicit StereoParamHandler(std::shared_ptr<rclcpp::Node> node, const std::string& name, const std::string& deviceName, bool rsCompat);
    ~StereoParamHandler();
    void declareParams(std::shared_ptr<dai::node::StereoDepth> stereo);
    void declareParams(std::shared_ptr<dai::node::NeuralDepth> neuralDepth);
    void updateSocketsFromParams(dai::CameraBoardSocket& left, dai::CameraBoardSocket& right, dai::CameraBoardSocket& align);
    dai::DeviceModelZoo getModel();
    std::shared_ptr<dai::NeuralDepthConfig> setRuntimeParams(const std::vector<rclcpp::Parameter>& params);

   private:
    std::unordered_map<std::string, dai::node::StereoDepth::PresetMode> depthPresetMap;
    std::unordered_map<std::string, dai::StereoDepthConfig::CostMatching::DisparityWidth> disparityWidthMap;
    std::unordered_map<std::string, dai::DeviceModelZoo> neuralModelTypeMap;
    std::unordered_map<std::string, dai::StereoDepthConfig::PostProcessing::DecimationFilter::DecimationMode> decimationModeMap;
    std::unordered_map<std::string, dai::StereoDepthConfig::PostProcessing::TemporalFilter::PersistencyMode> temporalPersistencyMap;
    std::unordered_map<std::string, StereoType> stereoTypeMap;
    dai::CameraBoardSocket alignSocket;
    dai::DeviceModelZoo model;
};
}  // namespace param_handlers
}  // namespace depthai_ros_driver
