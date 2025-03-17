#include "depthai_ros_driver/dai_nodes/sensors/img_pub.hpp"

#include "camera_info_manager/camera_info_manager.h"
#include "depthai-shared/properties/VideoEncoderProperties.hpp"
#include "depthai/device/Device.hpp"
#include "depthai/pipeline/Pipeline.hpp"
#include "depthai/pipeline/node/VideoEncoder.hpp"
#include "depthai/pipeline/node/XLinkOut.hpp"
#include "depthai_bridge/ImageConverter.hpp"
#include "depthai_ros_driver/dai_nodes/sensors/sensor_helpers.hpp"
#include "depthai_ros_driver/utils.hpp"
#include "image_transport/image_transport.h"

namespace depthai_ros_driver {
namespace dai_nodes {
namespace sensor_helpers {
ImagePublisher::ImagePublisher(ros::NodeHandle node,
                               std::shared_ptr<dai::Pipeline> pipeline,
                               const std::string& qName,
                               std::function<void(dai::Node::Input in)> linkFunc,
                               bool synced,
                               const utils::VideoEncoderConfig& encoderConfig)
    : node(node), it(node), encConfig(encoderConfig), qName(qName), synced(synced) {
    if(!synced) {
        xout = utils::setupXout(pipeline, qName);
    }

    linkCB = linkFunc;
    if(encoderConfig.enabled) {
        encoder = createEncoder(pipeline, encoderConfig);
        linkFunc(encoder->input);

        if(!synced) {
            if(encoderConfig.profile == dai::VideoEncoderProperties::Profile::MJPEG) {
                encoder->bitstream.link(xout->input);
            } else {
                encoder->out.link(xout->input);
            }
        } else {
            if(encoderConfig.profile == dai::VideoEncoderProperties::Profile::MJPEG) {
                linkCB = [&](dai::Node::Input input) { encoder->bitstream.link(input); };
            } else {
                linkCB = [&](dai::Node::Input input) { encoder->out.link(input); };
            }
        }
    } else {
        if(!synced) {
            linkFunc(xout->input);
        }
    }
}
void ImagePublisher::setup(std::shared_ptr<dai::Device> device, const utils::ImgConverterConfig& convConf, const utils::ImgPublisherConfig& pubConf) {
    convConfig = convConf;
    pubConfig = pubConf;
    createImageConverter(device);
    createInfoManager(device);
    if(pubConfig.topicName.empty()) {
        throw std::runtime_error("Topic name cannot be empty!");
    }
    if(pubConfig.publishCompressed) {
        if(encConfig.profile == dai::VideoEncoderProperties::Profile::MJPEG) {
            compressedImgPub = node.advertise<sensor_msgs::CompressedImage>(pubConfig.topicName + pubConfig.compressedTopicSuffix, 10);
        } else {
            ffmpegPub = node.advertise<depthai_ros_msgs::FFMPEGPacket>(pubConfig.topicName + pubConfig.compressedTopicSuffix, 10);
        }
        infoPub = node.advertise<sensor_msgs::CameraInfo>(pubConfig.topicName + pubConfig.infoSuffix + "/camera_info", 10);
    } else {
        imgPubIT = it.advertiseCamera(pubConfig.topicName + pubConfig.topicSuffix, 10);
    }
    if(!synced) {
        dataQ = device->getOutputQueue(getQueueName(), pubConf.maxQSize, pubConf.qBlocking);
        addQueueCB(dataQ);
    }
}

void ImagePublisher::createImageConverter(std::shared_ptr<dai::Device> device) {
    converter = std::make_shared<dai::ros::ImageConverter>(convConfig.tfPrefix, convConfig.interleaved, convConfig.getBaseDeviceTimestamp);
    converter->setUpdateRosBaseTimeOnToRosMsg(convConfig.updateROSBaseTimeOnRosMsg);
    if(convConfig.lowBandwidth) {
        converter->convertFromBitstream(convConfig.encoding);
    }
    if(convConfig.addExposureOffset) {
        converter->addExposureOffset(convConfig.expOffset);
    }
    if(convConfig.reverseSocketOrder) {
        converter->reverseStereoSocketOrder();
    }
    if(convConfig.alphaScalingEnabled) {
        converter->setAlphaScaling(convConfig.alphaScaling);
    }
    if(convConfig.isStereo && !convConfig.outputDisparity) {
        auto calHandler = device->readCalibration();
        double baseline = calHandler.getBaselineDistance(pubConfig.leftSocket, pubConfig.rightSocket, false);
        if(convConfig.reverseSocketOrder) {
            baseline = calHandler.getBaselineDistance(pubConfig.rightSocket, pubConfig.leftSocket, false);
        }
        converter->convertDispToDepth(baseline);
    }
    converter->setFFMPEGEncoding(convConfig.ffmpegEncoder);
}

std::shared_ptr<dai::node::VideoEncoder> ImagePublisher::createEncoder(std::shared_ptr<dai::Pipeline> pipeline,
                                                                       const utils::VideoEncoderConfig& encoderConfig) {
    auto enc = pipeline->create<dai::node::VideoEncoder>();
    enc->setQuality(encoderConfig.quality);
    enc->setProfile(encoderConfig.profile);
    if(encoderConfig.profile != dai::VideoEncoderProperties::Profile::MJPEG) {
        enc->setBitrate(encoderConfig.bitrate);
        enc->setKeyframeFrequency(encoderConfig.frameFreq);
    }
    return enc;
}
void ImagePublisher::createInfoManager(std::shared_ptr<dai::Device> device) {
    infoManager = std::make_shared<camera_info_manager::CameraInfoManager>(ros::NodeHandle(node, pubConfig.daiNodeName + pubConfig.infoMgrSuffix),
                                                                           "/" + pubConfig.daiNodeName + pubConfig.infoMgrSuffix);
    if(pubConfig.calibrationFile.empty()) {
        auto info = sensor_helpers::getCalibInfo(converter, device, pubConfig.socket, pubConfig.width, pubConfig.height);
        if(pubConfig.rectified) {
            std::fill(info.D.begin(), info.D.end(), 0.0);
            info.R[0] = info.R[4] = info.R[8] = 1.0;
        }
        infoManager->setCameraInfo(info);
    } else {
        infoManager->loadCameraInfo(pubConfig.calibrationFile);
    }
}
ImagePublisher::~ImagePublisher() {
    closeQueue();
}

void ImagePublisher::closeQueue() {
    if(dataQ) dataQ->close();
}
void ImagePublisher::link(dai::Node::Input in) {
    linkCB(in);
}
std::shared_ptr<dai::DataOutputQueue> ImagePublisher::getQueue() {
    return dataQ;
}
void ImagePublisher::addQueueCB(const std::shared_ptr<dai::DataOutputQueue>& queue) {
    dataQ = queue;
    qName = queue->getName();
    cbID = dataQ->addCallback([this](const std::shared_ptr<dai::ADatatype>& data) { publish(data); });
}

std::string ImagePublisher::getQueueName() {
    return qName;
}
std::shared_ptr<Image> ImagePublisher::convertData(const std::shared_ptr<dai::ADatatype>& data) {
    auto info = infoManager->getCameraInfo();
    auto img = std::make_shared<Image>();
    if(pubConfig.publishCompressed) {
        if(encConfig.profile == dai::VideoEncoderProperties::Profile::MJPEG) {
            auto daiImg = std::dynamic_pointer_cast<dai::ImgFrame>(data);
            auto rawMsg = converter->toRosCompressedMsg(daiImg);
            img->compressedImg = std::make_unique<sensor_msgs::CompressedImage>(rawMsg);
        } else {
            auto daiImg = std::dynamic_pointer_cast<dai::EncodedFrame>(data);
            auto rawMsg = converter->toRosFFMPEGPacket(daiImg);
            img->ffmpegPacket = std::make_unique<depthai_ros_msgs::FFMPEGPacket>(rawMsg);
        }
    } else {
        auto daiImg = std::dynamic_pointer_cast<dai::ImgFrame>(data);
        auto rawMsg = converter->toRosMsgRawPtr(daiImg, info);
        info.header = rawMsg.header;
        sensor_msgs::Image::Ptr msg = std::make_unique<sensor_msgs::Image>(rawMsg);
        img->image = std::move(msg);
    }
    sensor_msgs::CameraInfo::Ptr infoMsg = std::make_unique<sensor_msgs::CameraInfo>(info);
    img->info = std::move(infoMsg);
    return img;
}
void ImagePublisher::publish(std::shared_ptr<Image> img) {
    if(pubConfig.publishCompressed) {
        if(encConfig.profile == dai::VideoEncoderProperties::Profile::MJPEG) {
            compressedImgPub.publish(std::move(img->compressedImg));
        } else {
            ffmpegPub.publish(std::move(img->ffmpegPacket));
        }
        infoPub.publish(std::move(img->info));
    } else {
        if(!pubConfig.lazyPub || imgPubIT.getNumSubscribers() > 0) imgPubIT.publish(*img->image, *img->info);
    }
}
void ImagePublisher::publish(std::shared_ptr<Image> img, ros::Time timestamp) {
    img->info->header.stamp = timestamp;
    if(pubConfig.publishCompressed) {
        if(encConfig.profile == dai::VideoEncoderProperties::Profile::MJPEG) {
            img->compressedImg->header.stamp = timestamp;
        } else {
            img->ffmpegPacket->header.stamp = timestamp;
        }
    } else {
        img->image->header.stamp = timestamp;
    }
    publish(img);
}

void ImagePublisher::publish(const std::shared_ptr<dai::ADatatype>& data) {
    if(ros::ok()) {
        auto img = convertData(data);
        publish(img);
    }
}

bool ImagePublisher::detectSubscription(const ros::Publisher pub, const ros::Publisher infoPub) {
    return (pub.getNumSubscribers() > 0 || infoPub.getNumSubscribers() > 0);
}
}  // namespace sensor_helpers
}  // namespace dai_nodes
}  // namespace depthai_ros_driver
