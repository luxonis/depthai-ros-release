#pragma once

#include <memory>
#include <pluginlib/class_loader.hpp>
#include <string>
#include <unordered_map>
#include <vector>

#include "depthai_ros_driver/dai_nodes/base_node.hpp"
#include "depthai_ros_driver/pipeline/base_pipeline.hpp"

namespace dai {
class Pipeline;
class Device;
}  // namespace dai

namespace rclcpp {
class Node;
}

namespace depthai_ros_driver {
namespace param_handlers {
class PipelineGenParamHandler;
}  // namespace param_handlers
namespace pipeline_gen {
enum class PipelineType { RGB, RGBD, RGBStereo, Stereo, Depth, CamArray, DepthToF, StereoToF, ToF, RGBToF, Thermal };

class PipelineGenerator {
   public:
    PipelineGenerator();
    ~PipelineGenerator();
    /**
     * @brief      Validates the pipeline type. If the pipeline type is not valid for the number of sensors, it will be changed to the default type.
     *
     * @param      node       The node used for logging
     * @param[in]  type       The type
     * @param[in]  sensorNum  The sensor number
     *
     * @return     The validated pipeline type.
     */
    std::string validatePipeline(std::shared_ptr<rclcpp::Node> node, const std::string& typeStr, int sensorNum, const std::string& deviceName);
    /**
     * @brief      Creates the pipeline by using a plugin. Plugin types need to be of type depthai_ros_driver::pipeline_gen::BasePipeline.
     *
     * @param      node          The node
     * @param      device        The device
     * @param      pipeline      The pipeline
     * @param[in]  pipelineType  The pipeline type name (plugin name or one of the default types)
     * @param[in]  nnType        The neural network type (none, rgb, spatial)
     * @param[in]  enableImu     Indicates if IMU is enabled
     *
     * @return     Vector BaseNodes created.
     */
    void createPipeline(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<dai::Device> device, std::shared_ptr<dai::Pipeline> pipeline, bool rsCompat);

    void updateParams(const std::vector<rclcpp::Parameter>& params);

   protected:
    std::unordered_map<std::string, std::string> pluginTypeMap;
    std::unordered_map<std::string, PipelineType> pipelineTypeMap;

   private:
    std::shared_ptr<param_handlers::PipelineGenParamHandler> ph;
    std::shared_ptr<pluginlib::ClassLoader<BasePipeline>> pipelineLoader;
    std::vector<std::unique_ptr<dai_nodes::BaseNode>> daiNodes;
};
}  // namespace pipeline_gen
}  // namespace depthai_ros_driver
