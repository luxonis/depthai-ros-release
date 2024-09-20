#pragma once

#include <memory>
#include <string>

#include "depthai-shared/common/CameraBoardSocket.hpp"
#include "depthai/pipeline/Node.hpp"
#include "depthai_ros_driver/parametersConfig.h"
#include "depthai_ros_driver/utils.hpp"

namespace dai {
class Pipeline;
class Device;
namespace node {
class XLinkOut;
class VideoEncoder;
}  // namespace node
}  // namespace dai

namespace ros {
class NodeHandle;
class Parameter;
}  // namespace ros

namespace depthai_ros_driver {
namespace dai_nodes {
namespace sensor_helpers {
class ImagePublisher;
struct VideoEncoderConfig;
}  // namespace sensor_helpers
class BaseNode {
   public:
    /**
     * @brief      Constructor of the class BaseNode. Creates a node in the pipeline.
     *
     * @param[in]  daiNodeName  The dai node name
     * @param      node         The node
     * @param      pipeline     The pipeline
     */
    BaseNode(const std::string& daiNodeName, ros::NodeHandle node, std::shared_ptr<dai::Pipeline> /*pipeline*/);
    virtual ~BaseNode();
    virtual void updateParams(parametersConfig& config);
    virtual void link(dai::Node::Input in, int linkType = 0);
    virtual dai::Node::Input getInput(int linkType = 0);
    virtual dai::Node::Input getInputByName(const std::string& name = "");
    virtual std::vector<std::shared_ptr<sensor_helpers::ImagePublisher>> getPublishers();
    virtual void setupQueues(std::shared_ptr<dai::Device> device) = 0;
    /**
     * @brief      Sets the names of the queues.
     */
    virtual void setNames() = 0;
    /**
     * @brief      Link inputs and outputs.
     *
     * @param      pipeline  The pipeline
     */
    virtual void setXinXout(std::shared_ptr<dai::Pipeline> pipeline) = 0;
    std::shared_ptr<sensor_helpers::ImagePublisher> setupOutput(std::shared_ptr<dai::Pipeline> pipeline,
                                                                const std::string& qName,
                                                                std::function<void(dai::Node::Input input)> nodeLink,
                                                                bool isSynced = false,
                                                                const utils::VideoEncoderConfig& encoderConfig = {});
    virtual void closeQueues() = 0;

    std::shared_ptr<dai::node::XLinkOut> setupXout(std::shared_ptr<dai::Pipeline> pipeline, const std::string& name);

    void setNodeName(const std::string& daiNodeName);
    void setROSNodePointer(ros::NodeHandle node);
    /**
     * @brief      Gets the ROS node pointer.
     *
     * @return     The ROS node pointer.
     */
    ros::NodeHandle getROSNode();
    /**
     * @brief      Gets the name of the node.
     *
     * @return     The name.
     */
    std::string getName();
    /**
     * @brief    Append ROS node name to the frameName given.
     *
     * @param[in]  frameName  The frame name
     */
    std::string getTFPrefix(const std::string& frameName = "");
    /**
     * @brief    Append ROS node name to the frameName given and append optical frame suffix to it.
     *
     * @param[in]  frameName  The frame name
     */
    std::string getOpticalTFPrefix(const std::string& frameName = "");
    std::string getSocketName(dai::CameraBoardSocket socket);
    bool rsCompabilityMode();

   private:
    ros::NodeHandle baseNode;
    std::string baseDAINodeName;
};
}  // namespace dai_nodes
}  // namespace depthai_ros_driver
