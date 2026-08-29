#include <libobsensor/ObSensor.hpp>

#include <algorithm>
#include <cstdint>
#include <cstdlib>
#include <future>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

namespace {

struct CameraConfig {
    const char* role;
    const char* serial;
    uint32_t fps;
    OBFormat format;
};

struct Camera {
    CameraConfig config;
    std::shared_ptr<ob::Device> device;
    std::unique_ptr<ob::Pipeline> pipeline;
};

}  // namespace

int main(int argc, char** argv) try {
    ob::Context::setLoggerToFile(OB_LOG_SEVERITY_OFF, "");
    ob::Context::setLoggerToConsole(OB_LOG_SEVERITY_ERROR);
    const int frame_count = argc > 1 ? std::max(1, std::atoi(argv[1])) : 30;
    const std::vector<CameraConfig> requested{
        {"base_0_rgb", "CP28563000AJ", 30, OB_FORMAT_MJPG},
        {"left_wrist_0_rgb", "CV2L761000KT", 30, OB_FORMAT_YUYV},
    };
    ob::Context context;
    const auto devices = context.queryDeviceList();
    std::vector<Camera> cameras;
    cameras.reserve(requested.size());
    for(const auto& config : requested) {
        auto device = devices->getDeviceBySN(config.serial);
        auto pipeline = std::make_unique<ob::Pipeline>(device);
        auto stream_config = std::make_shared<ob::Config>();
        stream_config->enableVideoStream(OB_STREAM_COLOR, 1280, 800, config.fps, config.format);
        pipeline->start(stream_config);
        cameras.push_back({config, std::move(device), std::move(pipeline)});
    }

    uint64_t max_skew_us = 0;
    uint64_t discarded_frames = 0;
    std::vector<std::shared_ptr<ob::FrameSet>> current(cameras.size());
    auto fetch = [&cameras](std::size_t index) {
        auto frameset = cameras[index].pipeline->waitForFrameset(2000);
        if(!frameset) throw std::runtime_error(std::string("frame timeout: ") + cameras[index].config.role);
        return frameset;
    };
    for(int frame_index = 0; frame_index < frame_count; ++frame_index) {
        std::vector<std::future<std::shared_ptr<ob::FrameSet>>> pending;
        pending.reserve(cameras.size());
        for(std::size_t index = 0; index < cameras.size(); ++index) {
            pending.push_back(std::async(std::launch::async, fetch, index));
        }
        for(std::size_t index = 0; index < cameras.size(); ++index) {
            current[index] = pending[index].get();
        }
        while(true) {
            std::vector<uint64_t> timestamps;
            for(const auto& frameset : current) timestamps.push_back(frameset->getColorFrame()->getSystemTimeStampUs());
            const auto [minimum, maximum] = std::minmax_element(timestamps.begin(), timestamps.end());
            const uint64_t skew_us = *maximum - *minimum;
            if(skew_us <= 20000) {
                max_skew_us = std::max(max_skew_us, skew_us);
                break;
            }
            const auto oldest = static_cast<std::size_t>(std::distance(timestamps.begin(), minimum));
            current[oldest] = fetch(oldest);
            if(++discarded_frames > 1000) throw std::runtime_error("could not pair camera timestamps within 20 ms");
        }
        for(std::size_t index = 0; index < cameras.size(); ++index) {
            auto& camera = cameras[index];
            auto frame = current[index]->getColorFrame();
            if(!frame) throw std::runtime_error(std::string("missing color frame: ") + camera.config.role);
            const auto video = frame->as<ob::VideoFrame>();
            if(video->width() != 1280 || video->height() != 800) {
                throw std::runtime_error(std::string("unexpected color size: ") + camera.config.role);
            }
            if(frame_index == 0) {
                std::cout << camera.config.role << " serial=" << camera.config.serial << " size=" << video->width()
                          << 'x' << video->height() << " format=" << static_cast<int>(video->format()) << '\n';
            }
        }
    }
    for(auto& camera : cameras) camera.pipeline->stop();
    std::cout << "frame_pairs=" << frame_count << " discarded_frames=" << discarded_frames
              << " max_system_timestamp_skew_ms=" << max_skew_us / 1000.0 << '\n';
    return 0;
}
catch(const ob::Error& error) {
    std::cerr << "Orbbec SDK error: function=" << error.getFunction() << " message=" << error.what() << '\n';
    return 2;
}
catch(const std::exception& error) {
    std::cerr << "error: " << error.what() << '\n';
    return 1;
}
