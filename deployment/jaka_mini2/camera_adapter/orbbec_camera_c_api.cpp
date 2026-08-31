#include "orbbec_camera_c_api.h"

#include <libobsensor/ObSensor.hpp>

#include <algorithm>
#include <array>
#include <cstdlib>
#include <cstring>
#include <future>
#include <memory>
#include <stdexcept>
#include <string>

namespace {

struct Stream {
    std::string serial;
    OBFormat format;
    OBConvertFormat conversion;
    std::shared_ptr<ob::Device> device;
    std::unique_ptr<ob::Pipeline> pipeline;
    std::unique_ptr<ob::FormatConvertFilter> converter;
    std::shared_ptr<ob::FrameSet> current;
};

class DualCamera {
public:
    DualCamera(std::string base_serial, std::string wrist_serial, uint32_t width, uint32_t height, uint32_t fps,
               uint64_t max_skew_us)
        : width_(width), height_(height), fps_(fps), max_skew_us_(max_skew_us),
          streams_{Stream{std::move(base_serial), OB_FORMAT_MJPG, FORMAT_MJPG_TO_RGB},
                   Stream{std::move(wrist_serial), OB_FORMAT_YUYV, FORMAT_YUYV_TO_RGB}} {}

    ~DualCamera() { close_impl(); }

    int open() {
        return guard([this] {
            if(open_) return;
            ob::Context::setLoggerToFile(OB_LOG_SEVERITY_OFF, "");
            ob::Context::setLoggerToConsole(OB_LOG_SEVERITY_ERROR);
            ob::Context context;
            auto devices = context.queryDeviceList();
            try {
                for(auto& stream : streams_) {
                    stream.device = devices->getDeviceBySN(stream.serial.c_str());
                    configure_color(stream);
                    stream.pipeline = std::make_unique<ob::Pipeline>(stream.device);
                    auto config = std::make_shared<ob::Config>();
                    config->enableVideoStream(OB_STREAM_COLOR, width_, height_, fps_, stream.format);
                    stream.pipeline->start(config);
                    stream.converter = std::make_unique<ob::FormatConvertFilter>();
                    stream.converter->setFormatConvertType(stream.conversion);
                }
            }
            catch(...) {
                close_impl();
                throw;
            }
            open_ = true;
        });
    }

    int capture(uint8_t* base_rgb, size_t base_size, uint8_t* wrist_rgb, size_t wrist_size, uint64_t* timestamp_us,
                uint64_t* skew_us) {
        return guard([&] {
            if(!open_) throw std::runtime_error("dual camera is not open");
            const size_t required = static_cast<size_t>(width_) * height_ * 3;
            if(base_rgb == nullptr || wrist_rgb == nullptr || timestamp_us == nullptr || skew_us == nullptr
               || base_size < required || wrist_size < required) {
                throw std::invalid_argument("invalid RGB output buffer");
            }
            std::array<std::future<std::shared_ptr<ob::FrameSet>>, 2> pending{
                std::async(std::launch::async, [this] { return fetch(0); }),
                std::async(std::launch::async, [this] { return fetch(1); }),
            };
            for(size_t index = 0; index < streams_.size(); ++index) streams_[index].current = pending[index].get();
            uint64_t discarded = 0;
            while(true) {
                const auto base_time = system_timestamp(0);
                const auto wrist_time = system_timestamp(1);
                const auto minimum = std::min(base_time, wrist_time);
                const auto maximum = std::max(base_time, wrist_time);
                if(maximum - minimum <= max_skew_us_) {
                    *timestamp_us = maximum;
                    *skew_us = maximum - minimum;
                    break;
                }
                const size_t oldest = base_time < wrist_time ? 0 : 1;
                streams_[oldest].current = fetch(oldest);
                if(++discarded > 1000) throw std::runtime_error("could not pair camera frames within skew limit");
            }
            copy_rgb(0, base_rgb, required);
            copy_rgb(1, wrist_rgb, required);
        });
    }

    int close() { return guard([this] { close_impl(); }); }

    const char* last_error() const { return last_error_.c_str(); }

private:
    static int configured_int(const char* name, int fallback) {
        const char* raw = std::getenv(name);
        if(raw == nullptr) return fallback;
        const int value = std::stoi(raw);
        if(value < 0 || value > 100) throw std::invalid_argument(std::string(name) + " must be in 0..100");
        return value;
    }

    static void configure_color(Stream& stream) {
        const char* variable = stream.serial == "CV2L761000KT" ? "JAKA_WRIST_COLOR_SHARPNESS"
                                                               : "JAKA_BASE_COLOR_SHARPNESS";
        const int sharpness = configured_int(variable, 50);
        if(!stream.device->isPropertySupported(OB_PROP_COLOR_SHARPNESS_INT, OB_PERMISSION_WRITE)) {
            if(std::getenv(variable) != nullptr) throw std::runtime_error(std::string(variable) + " is unsupported");
            return;
        }
        const auto range = stream.device->getIntPropertyRange(OB_PROP_COLOR_SHARPNESS_INT);
        if(sharpness < range.min || sharpness > range.max || (sharpness - range.min) % range.step != 0) {
            throw std::invalid_argument(std::string(variable) + " is outside the camera range");
        }
        stream.device->setIntProperty(OB_PROP_COLOR_SHARPNESS_INT, sharpness);
    }

    template <typename Function> int guard(Function&& function) {
        try {
            function();
            last_error_.clear();
            return 0;
        }
        catch(const ob::Error& error) {
            last_error_ = std::string("Orbbec SDK: ") + error.getFunction() + ": " + error.what();
        }
        catch(const std::exception& error) {
            last_error_ = error.what();
        }
        catch(...) {
            last_error_ = "unknown camera error";
        }
        return -1;
    }

    std::shared_ptr<ob::FrameSet> fetch(size_t index) {
        auto frameset = streams_[index].pipeline->waitForFrameset(2000);
        if(!frameset || !frameset->getColorFrame()) throw std::runtime_error("timed out waiting for true color frame");
        return frameset;
    }

    uint64_t system_timestamp(size_t index) const {
        return streams_[index].current->getColorFrame()->getSystemTimeStampUs();
    }

    void copy_rgb(size_t index, uint8_t* output, size_t required) {
        auto color = streams_[index].current->getColorFrame();
        auto rgb = color->format() == OB_FORMAT_RGB ? color : streams_[index].converter->process(color);
        auto video = rgb->as<ob::VideoFrame>();
        if(video->format() != OB_FORMAT_RGB || video->width() != width_ || video->height() != height_
           || video->dataSize() < required) {
            throw std::runtime_error("Orbbec RGB conversion returned an unexpected frame");
        }
        std::memcpy(output, video->data(), required);
    }

    void close_impl() noexcept {
        for(auto& stream : streams_) {
            if(stream.pipeline) {
                try {
                    stream.pipeline->stop();
                }
                catch(...) {
                }
            }
            stream.current.reset();
            stream.converter.reset();
            stream.pipeline.reset();
            stream.device.reset();
        }
        open_ = false;
    }

    uint32_t width_;
    uint32_t height_;
    uint32_t fps_;
    uint64_t max_skew_us_;
    std::array<Stream, 2> streams_;
    bool open_ = false;
    std::string last_error_;
};

DualCamera* camera(void* handle) {
    return static_cast<DualCamera*>(handle);
}

}  // namespace

extern "C" {

void* orbbec_dual_camera_create(const char* base_serial, const char* wrist_serial, uint32_t width, uint32_t height,
                                uint32_t fps, uint64_t max_skew_us) {
    if(base_serial == nullptr || wrist_serial == nullptr) return nullptr;
    try {
        return new DualCamera(base_serial, wrist_serial, width, height, fps, max_skew_us);
    }
    catch(...) {
        return nullptr;
    }
}

void orbbec_dual_camera_destroy(void* handle) {
    delete camera(handle);
}

int orbbec_dual_camera_open(void* handle) {
    return handle == nullptr ? -1 : camera(handle)->open();
}

int orbbec_dual_camera_capture(void* handle, uint8_t* base_rgb, size_t base_size, uint8_t* wrist_rgb,
                               size_t wrist_size, uint64_t* timestamp_us, uint64_t* skew_us) {
    return handle == nullptr
               ? -1
               : camera(handle)->capture(base_rgb, base_size, wrist_rgb, wrist_size, timestamp_us, skew_us);
}

int orbbec_dual_camera_close(void* handle) {
    return handle == nullptr ? -1 : camera(handle)->close();
}

const char* orbbec_dual_camera_last_error(void* handle) {
    return handle == nullptr ? "invalid camera handle" : camera(handle)->last_error();
}

}  // extern "C"
