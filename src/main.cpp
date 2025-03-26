#include <iostream>

// Include necessary header files for WebRTC functionality
#include "args.h"
#include "common/logging.h"
#include "common/utils.h"
#include "conductor.h"
#include "parser.h"
#include "recorder/recorder_manager.h"
#include "signaling/signaling_service.h"

// Conditional includes based on which signaling method is being used
#if USE_MQTT_SIGNALING
#include "signaling/mqtt_service.h"
#elif USE_HTTP_SIGNALING
#include "signaling/http_service.h"
#endif

int main(int argc, char *argv[]) {
    // Parse command line arguments into Args structure
    Args args;
    Parser::ParseArgs(argc, argv, args);

    // Create the main WebRTC conductor that handles peer connections
    std::shared_ptr<Conductor> conductor = Conductor::Create(args);
    std::unique_ptr<RecorderManager> recorder_mgr;

    // Initialize video/audio recording if the recording directory can be created
    if (Utils::CreateFolder(args.record_path)) {
        recorder_mgr =
            RecorderManager::Create(conductor->VideoSource(), conductor->AudioSource(), args);
        DEBUG_PRINT("Recorder is running!");
    } else {
        DEBUG_PRINT("Recorder is not started!");
    }

    // Create appropriate signaling service based on compile-time configuration
    // The signaling service handles WebRTC connection establishment and peer discovery
    auto signaling_service = ([args, conductor]() -> std::shared_ptr<SignalingService> {
#if USE_MQTT_SIGNALING
        // Use MQTT for signaling (Message Queuing Telemetry Transport protocol)
        return MqttService::Create(args, conductor);
#elif USE_HTTP_SIGNALING
        // Use HTTP-based signaling
        return HttpService::Create(args, conductor);
#else
        return nullptr;
#endif
    })();

    // Start the signaling service if one was successfully created
    if (signaling_service) {
        signaling_service->Start();
    } else {
        INFO_PRINT("There is no any signaling service found!");
    }

    return 0;
}
