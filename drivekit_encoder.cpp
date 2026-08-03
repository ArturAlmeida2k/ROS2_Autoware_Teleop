//////////////////////////////////////////////////////////////////////////////////////////////////////
//  IT Research Lab - Embedded Systems
//  Capture GMSL camera frames -> process via ISP -> encode H264 via DW serializer -> publish to ROS2
//////////////////////////////////////////////////////////////////////////////////////////////////////

#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>
#include <queue>
#include <mutex>

#include <dw/core/logger/Logger.h>
#include <dw/image/Image.h>
#include <dw/rig/Rig.h>
#include <dw/sensors/common/Sensors.h>
#include <dw/sensors/common/SensorSerializer.h>
#include <dw/sensors/camera/Camera.h>

#include <framework/Checks.hpp>
#include <framework/DriveWorksSample.hpp>
#include <framework/Log.hpp>
#include <framework/ProgramArguments.hpp>
#include <framework/SamplesDataPath.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>

#include <dw/image/Image.h>
#include <sensor_msgs/msg/image.hpp>

using namespace dw_samples::common;

#define MAX_CAMS 16

bool isResetLinkOnError = false;

void cameraEventHandling(dwCameraSIPLNotification* notificationData, dwSensorHandle_t sensor);

struct CameraSerializerCtx {
    rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr publisher;
    rclcpp::Clock clock{RCL_SYSTEM_TIME};
    uint32_t cameraIdx = 0;

    std::mutex ts_mutex;
    std::queue<uint64_t> capture_timestamps;
};

static uint64_t global_frame_id = 0;

//  Called by encoder when H264 chunk is ready
static void onEncodedH264Data(const uint8_t* const buffer,
                              const size_t          bufferSize,
                              void* const           userData) {

    if (!buffer || bufferSize == 0 || !userData) {
        std::cerr << "ERROR: Invalid callback parameters" << std::endl;
        return;
    }

    auto* ctx = reinterpret_cast<CameraSerializerCtx*>(userData);
    
    uint64_t current_id = ++global_frame_id;
    uint64_t ts_ns = 0;

    // Resgatar o timestamp exato de quando ESTE frame foi capturado pela câmara
    {
        std::lock_guard<std::mutex> lock(ctx->ts_mutex);
        if (!ctx->capture_timestamps.empty()) {
            ts_ns = ctx->capture_timestamps.front();
            ctx->capture_timestamps.pop();
        } else {
            // Fallback caso a fila falhe
            auto now = std::chrono::system_clock::now().time_since_epoch();
            ts_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(now).count();
        }
    }

    // 2. Criar a string do payload (Ex: "ID:1|TS:1680000000000")
    std::string payload_str = "ID:" + std::to_string(current_id) + "|TS:" + std::to_string(ts_ns);
    size_t payload_size = payload_str.length();

    // 3. Definir um UUID customizado de 16 bytes (não pode conter 0x00 para evitar conflitos)
    const uint8_t uuid[16] = {
        0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 
        0x99, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF, 0x11
    };

    // 4. Construir o NAL Unit SEI
    std::vector<uint8_t> sei_nalu;
    sei_nalu.push_back(0x00); sei_nalu.push_back(0x00); sei_nalu.push_back(0x00); sei_nalu.push_back(0x01); // Start Code
    sei_nalu.push_back(0x06); // NAL Type: 6 (SEI)
    sei_nalu.push_back(0x05); // Payload Type: 5 (User Data Unregistered)
    
    // O tamanho do payload é UUID (16) + tamanho da string
    sei_nalu.push_back(static_cast<uint8_t>(16 + payload_size)); 
    
    sei_nalu.insert(sei_nalu.end(), uuid, uuid + 16); // Inserir UUID
    sei_nalu.insert(sei_nalu.end(), payload_str.begin(), payload_str.end()); // Inserir texto
    sei_nalu.push_back(0x80); // rbsp_trailing_bits (fecha o pacote corretamente)

    // 5. Preparar a mensagem ROS2
    sensor_msgs::msg::CompressedImage msg;
    msg.header.stamp.sec = ts_ns / 1000000000ULL;
    msg.header.stamp.nanosec = ts_ns % 1000000000ULL;

    msg.header.frame_id = "camera_" + std::to_string(ctx->cameraIdx);
    msg.format          = "h264";
    
    // 6. Juntar o nosso SEI ao buffer de vídeo original
    msg.data.resize(sei_nalu.size() + bufferSize);
    std::memcpy(msg.data.data(), sei_nalu.data(), sei_nalu.size());                 // Copiar SEI
    std::memcpy(msg.data.data() + sei_nalu.size(), buffer, bufferSize);             // Copiar Video H.264

    ctx->publisher->publish(std::move(msg));
}


class CameraRosPublisher : public DriveWorksSample {
private:
    dwContextHandle_t  m_context   = DW_NULL_HANDLE;
    dwSALHandle_t      m_sal       = DW_NULL_HANDLE;
    dwRigHandle_t      m_rigConfig{};

    uint32_t           m_totalCameras = 0;
    dwSensorHandle_t   m_camera[MAX_CAMS];

    dwSensorSerializerHandle_t m_serializer[MAX_CAMS];
    CameraSerializerCtx        m_serializerCtx[MAX_CAMS];

    //  ROS2
    rclcpp::Node::SharedPtr m_rosNode;
    std::vector<rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr> m_publishers;
    dwImageStreamerHandle_t m_rawStreamer[MAX_CAMS];
    std::vector<rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr> m_rawPublishers;

    //  ROS2 Publisher Multi-Threading
    std::atomic<bool>        m_running{false};
    std::vector<std::thread> m_cameraThreads;
    std::thread              m_rosSpinThread;

public:
    CameraRosPublisher(const ProgramArguments& args)
        : DriveWorksSample(args) {
        for (auto& s : m_serializer) s = DW_NULL_HANDLE;
        for (auto& c : m_camera)     c = DW_NULL_HANDLE;
    }


    // -----------------------
    //  Initialize driveworks
    // -----------------------
    void initializeDriveWorks(dwContextHandle_t& context) const {
        CHECK_DW_ERROR(dwLogger_initialize(filteredLoggerCallback));
        CHECK_DW_ERROR(dwLogger_setLogLevel(DW_LOG_ERROR));

        dwContextParameters sdkParams = {};
        #ifdef VIBRANTE
            sdkParams.eglDisplay = getEGLDisplay();
        #endif
        
        CHECK_DW_ERROR(dwInitialize(&context, DW_VERSION, &sdkParams));
    }


    // ----------------------------------------------------------------
    //  Filter some timing errors that are irrelevant (Nvidia said so)
    // ----------------------------------------------------------------
    static void filteredLoggerCallback(dwContextHandle_t ctx,
                                   dwLoggerVerbosity  verbosity,
                                   const char*        msg) {
        if (msg && strstr(msg, "DW_SENSOR_STATE_DELTA_CURRENT_AND_HOST_TIME"))
            return;

        getConsoleLoggerCallback(true)(ctx, verbosity, msg);
    }


    // --------------------------------------------------------
    //  Setup the driveworks, sensors, reader threads and ROS2
    // --------------------------------------------------------
    bool onInitialize() override {

        dwLogger_initialize(filteredLoggerCallback);
        dwLogger_setLogLevel(DW_LOG_ERROR);

        initializeDriveWorks(m_context);
        CHECK_DW_ERROR(dwSAL_initialize(&m_sal, m_context));
        CHECK_DW_ERROR(dwRig_initializeFromFile(&m_rigConfig, m_context, getArgument("rig").c_str()));

        uint32_t camCount = 0;
        CHECK_DW_ERROR(dwRig_getSensorCountOfType(&camCount, DW_SENSOR_CAMERA, m_rigConfig));

        isResetLinkOnError        = (std::stoi(getArgument("reset-on-error")) > 0);
        bool isEnableUserCallback = (std::stoi(getArgument("enable-user-event-callbacks")) > 0);

        std::string fullParamStrings[MAX_CAMS];


        //  Setup camera params and create the cameras
        for (uint32_t i = 0; i < camCount; i++) {
            uint32_t sensorIdx = 0;
            CHECK_DW_ERROR(dwRig_findSensorByTypeIndex(&sensorIdx, DW_SENSOR_CAMERA, i, m_rigConfig));

            const char* protocol = nullptr;
            CHECK_DW_ERROR(dwRig_getSensorProtocol(&protocol, sensorIdx, m_rigConfig));

            const char* params = nullptr;
            CHECK_DW_ERROR(dwRig_getSensorParameterUpdatedPath(&params, sensorIdx, m_rigConfig));

            fullParamStrings[i] = std::string(params);

            dwSensorParams sp{};
            sp.protocol   = protocol;
            sp.parameters = params;

            std::cout << "Creating camera [" << i << "]" << std::endl;
            CHECK_DW_ERROR(dwSAL_createSensor(&m_camera[m_totalCameras], sp, m_sal));

            if (isEnableUserCallback) {
                CHECK_DW_ERROR(dwSensorCamera_setEventCallback(
                    cameraEventHandling, cameraEventHandling, m_camera[m_totalCameras]));
            }

            m_totalCameras++;
        }

        
        //  Start sensor abstraction layer
        CHECK_DW_ERROR(dwSAL_start(m_sal));

        
        //  Setup and start ROS2 publisher threads
        rclcpp::init(0, nullptr);
        m_rosNode = std::make_shared<rclcpp::Node>("dw_camera_publisher");

        rclcpp::QoS qos_profile(1);
        qos_profile.best_effort();
        qos_profile.keep_last(1);

        for (uint32_t i = 0; i < m_totalCameras; ++i) {
            std::string topic = "camera_" + std::to_string(i) + "/compressed_image";
            m_publishers.push_back(m_rosNode->create_publisher<sensor_msgs::msg::CompressedImage>(topic, qos_profile));
            
            std::cout << "ROS2 topic: " << topic << std::endl;
        }

        for (uint32_t i = 0; i < m_totalCameras; ++i) {
            std::string rawTopic = "camera_" + std::to_string(i) + "/image_raw";
            m_rawPublishers.push_back(m_rosNode->create_publisher<sensor_msgs::msg::Image>(rawTopic, 4));
        }

        for (uint32_t i = 0; i < m_totalCameras; ++i) {
            dwImageProperties rgbaProps{};
            CHECK_DW_ERROR(dwSensorCamera_getImageProperties(&rgbaProps, DW_CAMERA_OUTPUT_CUDA_RGBA_UINT8, m_camera[i]));
            CHECK_DW_ERROR(dwImageStreamer_initialize(&m_rawStreamer[i], &rgbaProps, DW_IMAGE_CPU, m_context));
        }


        //  Initialize serializers
        for (uint32_t i = 0; i < m_totalCameras; ++i) {
            m_serializerCtx[i].publisher = m_publishers[i];
            m_serializerCtx[i].cameraIdx = i;

            dwSerializerParams sp{};
            sp.parameters = fullParamStrings[i].c_str();
            sp.onData     = onEncodedH264Data;
            sp.userData   = &m_serializerCtx[i];

            CHECK_DW_ERROR(dwSensorSerializer_initialize(&m_serializer[i], &sp, m_camera[i]));
            
            std::cout << "Serializer [" << i << "] initialized" << std::endl;
        }


        //  NvSci attribute exchange
        for (uint32_t i = 0; i < m_totalCameras; ++i) {
            dwImageProperties imgProps{};

            CHECK_DW_ERROR(dwSensorCamera_getImageProperties(&imgProps, DW_CAMERA_OUTPUT_NATIVE_PROCESSED, m_camera[i]));
            CHECK_DW_ERROR(dwSensorCamera_appendAllocationAttributes(&imgProps, DW_CAMERA_OUTPUT_NATIVE_PROCESSED, m_camera[i]));
            CHECK_DW_ERROR(dwSensorSerializer_appendAllocationAttributes(&imgProps, m_serializer[i]));
            CHECK_DW_ERROR(dwSensorCamera_setImageAttributes(&imgProps, DW_CAMERA_OUTPUT_NATIVE_PROCESSED, m_camera[i]));
        }


        //  Start cameras
        for (uint32_t i = 0; i < m_totalCameras; ++i) {
            CHECK_DW_ERROR(dwSensor_start(m_camera[i]));
            std::cout << "Camera [" << i << "] started" << std::endl;
            //dwCameraProperties camProps{};
            //dwSensorCamera_getSensorProperties(&camProps, m_camera[i]);
            //std::cout << "Camera [" << i << "] framerate: " << camProps.framerate << std::endl;
        }


        //  Launch per-camera reader threads
        m_running = true;
        for (uint32_t i = 0; i < m_totalCameras; ++i) {
            m_cameraThreads.emplace_back(&CameraRosPublisher::cameraReaderThread, this, i);
            std::cout << "Camera [" << i << "] reader thread started" << std::endl;
        }


        //  Launch dedicated ROS spin thread
        m_rosSpinThread = std::thread([this]() {
            rclcpp::executors::SingleThreadedExecutor executor;
            executor.add_node(m_rosNode);
            std::cout << "ROS spin thread started" << std::endl;

            while (m_running.load()) {
                executor.spin_some();
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        });

        return true;
    }

    // --------------------------------------------------
    // Per-camera reader thread to drain ICP buffer ASAP
    // --------------------------------------------------
    void cameraReaderThread(uint32_t i) {
        while (m_running.load()) {
            dwCameraFrameHandle_t frame;
            dwStatus status = dwSensorCamera_readFrame(&frame, 16, m_camera[i]);

            //  ICP buffer empty, yield briefly and retry later
            if (status == DW_NOT_AVAILABLE || status == DW_TIME_OUT || status == DW_NOT_READY) {
                std::this_thread::sleep_for(std::chrono::microseconds(200));
                continue;
            }

            //  Transient error, log but keep going
            if (status != DW_SUCCESS) {
                std::cerr << "Camera [" << i << "] readFrame error: " << status << std::endl;
                std::this_thread::sleep_for(std::chrono::microseconds(200));
                continue;
            }

            auto now = std::chrono::system_clock::now().time_since_epoch();
            uint64_t capture_ts = std::chrono::duration_cast<std::chrono::nanoseconds>(now).count();

            {
                std::lock_guard<std::mutex> lock(m_serializerCtx[i].ts_mutex);
                m_serializerCtx[i].capture_timestamps.push(capture_ts);
            }

            //  Try encoding to h264, drop if encoder queue is full
            dwStatus encStatus = dwSensorSerializer_serializeCameraFrameAsync(frame, m_serializer[i]);

            if (encStatus != DW_SUCCESS) {
                std::lock_guard<std::mutex> lock(m_serializerCtx[i].ts_mutex);
                m_serializerCtx[i].capture_timestamps.pop(); 
            }

            dwImageHandle_t rgbaImg = DW_NULL_HANDLE;
            if (dwSensorCamera_getImage(&rgbaImg, DW_CAMERA_OUTPUT_CUDA_RGBA_UINT8, frame) == DW_SUCCESS) {

                dwImageStreamer_producerSend(rgbaImg, m_rawStreamer[i]);

                dwImageHandle_t cpuImgHandle = DW_NULL_HANDLE;
                if (dwImageStreamer_consumerReceive(&cpuImgHandle, 33000, m_rawStreamer[i]) == DW_SUCCESS) {

                    dwImageCPU* cpuImg = nullptr;
                    dwImage_getCPU(&cpuImg, cpuImgHandle);

                    auto msg = std::make_unique<sensor_msgs::msg::Image>();
                    msg->header.stamp    = m_serializerCtx[i].clock.now();
                    msg->header.frame_id = "camera_" + std::to_string(i);
                    msg->height   = cpuImg->prop.height;
                    msg->width    = cpuImg->prop.width;
                    msg->encoding = "rgba8";
                    msg->step     = cpuImg->pitch[0];
                    msg->data.resize(msg->step * msg->height);
                    std::memcpy(msg->data.data(), cpuImg->data[0], msg->data.size());

                    m_rawPublishers[i]->publish(std::move(msg));

                    dwImageStreamer_consumerReturn(&cpuImgHandle, m_rawStreamer[i]);
                }
                dwImageStreamer_producerReturn(nullptr, 33000, m_rawStreamer[i]);
            }

            CHECK_DW_ERROR(dwSensorCamera_returnFrame(&frame));
        }
    }

    // ------------------------------------------------------------------------------
    //  onProcess is now just a heartbeat
    // IF REMOVED WILL CONSUME A FULL CORE DUE TO ROS2 THREAD SPINNING ALL THE TIME!
    // ------------------------------------------------------------------------------
    void onProcess() override final {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }


    // --------------------------------------
    //  Release driveworks stuff and threads 
    // --------------------------------------
    void onRelease() override final {
        //  Signal threads to stop and wait for them
        m_running = false;

        //  Camera reader threads
        for (auto& t : m_cameraThreads)
            if (t.joinable()) t.join();

        //  ROS2 spin thread
        if (m_rosSpinThread.joinable())
            m_rosSpinThread.join();

        //  Stop and release all cameras
        for (uint32_t i = 0; i < m_totalCameras; ++i) {
            dwSensor_stop(m_camera[i]);

            if (m_serializer[i] != DW_NULL_HANDLE) {
                dwSensorSerializer_stop(m_serializer[i]);
                dwSensorSerializer_release(m_serializer[i]);
            }

            if (m_camera[i])
                dwSAL_releaseSensor(m_camera[i]);

            if (m_rawStreamer[i] != DW_NULL_HANDLE)
                dwImageStreamer_release(m_rawStreamer[i]);
        }

        //  Release rig, sal and context
        if (m_rigConfig) dwRig_release(m_rigConfig);
        if (m_sal)       dwSAL_release(m_sal);
        if (m_context)   dwRelease(m_context);


        //  Shutoff ROS2
        rclcpp::shutdown();
    }
};


// --------------------------------
//  Check camera events for errors
// --------------------------------
void cameraEventHandling(dwCameraSIPLNotification* notificationData, dwSensorHandle_t sensor) {
    if (notificationData->data.eNotifyType == DW_NOTIF_INFO_ICP_PROCESSING_DONE ||
        notificationData->data.eNotifyType == DW_NOTIF_INFO_ISP_PROCESSING_DONE ||
        notificationData->data.eNotifyType == DW_NOTIF_INFO_ACP_PROCESSING_DONE ||
        notificationData->data.eNotifyType == DW_NOTIF_INFO_CDI_PROCESSING_DONE) {
        return;
    }

    if (isResetLinkOnError) {
        dwSensorCamera_disableLink(sensor);
        dwSensorCamera_enableLink(sensor, false);
        std::cout << "Link reset OK." << std::endl;
    }
}


// ------------------
//  Params and start 
// ------------------
int main(int argc, const char** argv) {
    ProgramArguments args(argc, argv, {
            ProgramArguments::Option_t("rig",
                (dw_samples::SamplesDataPath::get() + "/samples/sensors/camera/camera/rig.json").c_str(),
                "Path to rig.json camera configuration file"),
            ProgramArguments::Option_t("reset-on-error", "0",
                "Set to 1 to reset the camera link on SIPL errors"),
            ProgramArguments::Option_t("enable-user-event-callbacks", "0",
                "Set to 1 to enable SIPL error callbacks"),
        },
        "GMSL camera -> ROS2 H264 (threaded, per-camera drain)");

    CameraRosPublisher app(args);
    app.initializeWindow("Camera ROS Publisher", 1, 1, true);
    return app.run();
}