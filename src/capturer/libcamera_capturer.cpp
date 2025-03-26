#include "libcamera_capturer.h"

#include <sys/mman.h>

#include "common/logging.h"

// Factory method to create and configure a LibcameraCapturer instance
// Takes configuration arguments and returns a fully initialized capturer
std::shared_ptr<LibcameraCapturer> LibcameraCapturer::Create(Args args) {
    auto ptr = std::make_shared<LibcameraCapturer>(args);
    ptr->Init(args.cameraId);
    ptr->SetFps(args.fps)
        .SetRotation(args.rotation_angle)
        .SetFormat(args.width, args.height)
        .StartCapture();
    return ptr;
}

// Constructor that initializes basic properties from provided arguments
LibcameraCapturer::LibcameraCapturer(Args args)
    : buffer_count_(2),      // Using 2 buffers for double-buffering
      format_(args.format),  // Store the desired pixel format
      config_(args) {}       // Store the complete configuration

// Initialize the camera system and acquire the specified camera
void LibcameraCapturer::Init(int deviceId) {
    // Create the libcamera camera manager
    cm_ = std::make_unique<libcamera::CameraManager>();
    int ret = cm_->start();
    if (ret) {
        throw std::runtime_error("Failed to start camera manager");
    }

    // Check if any cameras are available
    auto cameras = cm_->cameras();
    if (cameras.size() == 0) {
        throw std::runtime_error("No camera is available via libcamera.");
    }

    // Validate that the requested camera ID is valid
    if (config_.cameraId >= cameras.size()) {
        throw std::runtime_error("Selected camera is not available.");
    }

    // Get the camera by ID and acquire it for exclusive use
    std::string const &cam_id = cameras[config_.cameraId]->id();
    INFO_PRINT("camera id: %s", cam_id.c_str());
    camera_ = cm_->get(cam_id);
    camera_->acquire();
    
    // Generate a default configuration for video recording
    camera_config_ = camera_->generateConfiguration({libcamera::StreamRole::VideoRecording});
}

// Destructor that properly cleans up all camera resources
LibcameraCapturer::~LibcameraCapturer() {
    camera_->stop();                 // Stop capturing
    allocator_->free(stream_);       // Free allocated buffers
    allocator_.reset();              // Reset the allocator
    camera_config_.reset();          // Reset the configuration
    camera_->release();              // Release the camera
    camera_.reset();                 // Reset the camera smart pointer
    cm_->stop();                     // Stop the camera manager
}

// Getter methods to access capturer configuration
int LibcameraCapturer::fps() const { return fps_; }
int LibcameraCapturer::width() const { return width_; }
int LibcameraCapturer::height() const { return height_; }
bool LibcameraCapturer::is_dma_capture() const { return false; } // Not using DMA for capture
uint32_t LibcameraCapturer::format() const { return format_; }
Args LibcameraCapturer::config() const { return config_; }

// Configure the camera resolution and pixel format
LibcameraCapturer &LibcameraCapturer::SetFormat(int width, int height) {
    DEBUG_PRINT("camera original format: %s", camera_config_->at(0).toString().c_str());

    // Set the resolution if specified
    if (width && height) {
        libcamera::Size size(width, height);
        camera_config_->at(0).size = size;
    }

    // Configure for YUV420 format (commonly used for video)
    camera_config_->at(0).pixelFormat = libcamera::formats::YUV420;
    camera_config_->at(0).bufferCount = buffer_count_;

    // Validate the configuration and handle adjustments
    auto validation = camera_config_->validate();
    if (validation == libcamera::CameraConfiguration::Status::Valid) {
        INFO_PRINT("camera validated format: %s.", camera_config_->at(0).toString().c_str());
    } else if (validation == libcamera::CameraConfiguration::Status::Adjusted) {
        INFO_PRINT("camera adjusted format: %s.", camera_config_->at(0).toString().c_str());
    } else {
        ERROR_PRINT("Failed to validate camera configuration.");
        exit(1);
    }

    // Store the finalized dimensions and stride
    width_ = camera_config_->at(0).size.width;
    height_ = camera_config_->at(0).size.height;
    stride_ = camera_config_->at(0).stride;

    INFO_PRINT("  width: %d, height: %d, stride: %d", width_, height_, stride_);

    // Verify that stride equals width (no padding)
    if (width_ != stride_) {
        ERROR_PRINT("Stride is not equal to width");
        exit(1);
    }

    return *this;
}

// Set the desired frame rate
LibcameraCapturer &LibcameraCapturer::SetFps(int fps) {
    fps_ = fps;
    // Convert fps to frame duration in microseconds
    int64_t frame_time = 1000000 / fps;
    // Set both min and max frame duration to the same value to request exact fps
    controls_.set(libcamera::controls::FrameDurationLimits,
                  libcamera::Span<const int64_t, 2>({frame_time, frame_time}));
    DEBUG_PRINT("  Fps: %d", fps);

    return *this;
}

// Set camera control parameters in a thread-safe manner
LibcameraCapturer &LibcameraCapturer::SetControls(const int key, const int value) {
    std::lock_guard<std::mutex> lock(control_mutex_);
    DEBUG_PRINT("Set camera controls, key: %d, value: %d", key, value);
    controls_.set(key, value);
    return *this;
}

// Configure camera rotation
LibcameraCapturer &LibcameraCapturer::SetRotation(int angle) {
    // Map angle to libcamera orientation enum
    if (angle == 90) {
        camera_config_->orientation = libcamera::Orientation::Rotate90;
    } else if (angle == 180) {
        camera_config_->orientation = libcamera::Orientation::Rotate180;
    } else if (angle == 270) {
        camera_config_->orientation = libcamera::Orientation::Rotate270;
    }
    // Default is no rotation (0 degrees)

    DEBUG_PRINT("  Rotation: %d", angle);

    return *this;
}

// Allocate memory-mapped buffers for camera capture
void LibcameraCapturer::AllocateBuffer() {
    // Create a frame buffer allocator for this camera
    allocator_ = std::make_unique<libcamera::FrameBufferAllocator>(camera_);

    // Get the stream object from configuration
    stream_ = camera_config_->at(0).stream();
    int ret = allocator_->allocate(stream_);
    if (ret < 0) {
        ERROR_PRINT("Can't allocate buffers");
    }

    // Verify that we got the expected number of buffers
    auto &buffers = allocator_->buffers(stream_);
    if (buffer_count_ != buffers.size()) {
        ERROR_PRINT("Buffer counts not match allocated buffer number");
        exit(1);
    }

    // For each buffer, set up memory mapping and create camera request
    for (unsigned int i = 0; i < buffer_count_; i++) {
        auto &buffer = buffers[i];
        int fd = 0;
        int buffer_length = 0;
        
        // Get file descriptor and calculate total length across all planes
        for (auto &plane : buffer->planes()) {
            fd = plane.fd.get();
            buffer_length += plane.length;
        }
        
        // Map the buffer into process memory space
        void *memory = mmap(NULL, buffer_length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
        mapped_buffers_[fd] = std::make_pair(memory, buffer_length);
        DEBUG_PRINT("Allocated fd(%d) Buffer[%d] pointer: %p, length: %d", fd, i, memory,
                    buffer_length);

        // Create a request and add the buffer to it
        auto request = camera_->createRequest();
        if (!request) {
            ERROR_PRINT("Can't create camera request");
        }
        int ret = request->addBuffer(stream_, buffer.get());
        if (ret < 0) {
            ERROR_PRINT("Can't set buffer for request");
        }
        requests_.push_back(std::move(request));
    }
}

// Callback function that processes completed capture requests
void LibcameraCapturer::RequestComplete(libcamera::Request *request) {
    // Check if the request was cancelled
    if (request->status() == libcamera::Request::RequestCancelled) {
        DEBUG_PRINT("Request has been cancelled");
        exit(1);
    }

    // Get the buffer from the request
    auto &buffers = request->buffers();
    auto *buffer = buffers.begin()->second;

    // Get access to the buffer data
    auto &plane = buffer->planes()[0];
    int fd = plane.fd.get();
    void *data = mapped_buffers_[fd].first;
    int length = mapped_buffers_[fd].second;
    
    // Extract timestamp from buffer metadata
    timeval tv = {};
    tv.tv_sec = buffer->metadata().timestamp / 1000000000;
    tv.tv_usec = (buffer->metadata().timestamp % 1000000000) / 1000;

    // Create a V4L2Buffer representation and process it
    V4L2Buffer v4l2_buffer((uint8_t *)data, length, V4L2_BUF_FLAG_KEYFRAME, tv);
    NextBuffer(v4l2_buffer);

    // Reuse the request with the same buffer
    request->reuse(libcamera::Request::ReuseBuffers);

    // Apply any pending control changes (thread-safe)
    {
        std::lock_guard<std::mutex> lock(control_mutex_);
        request->controls() = controls_;
    }

    // Queue the request back to the camera
    camera_->queueRequest(request);
}

// Convert the current frame to WebRTC's I420 format
rtc::scoped_refptr<webrtc::I420BufferInterface> LibcameraCapturer::GetI420Frame() {
    return frame_buffer_->ToI420();
}

// Process a new buffer from the camera
void LibcameraCapturer::NextBuffer(V4L2Buffer &buffer) {
    // Create a frame buffer representation from the raw buffer
    frame_buffer_ = V4L2FrameBuffer::Create(width_, height_, buffer, format_);
    
    // Notify observers about the new frame
    NextFrameBuffer(frame_buffer_);
    NextRawBuffer(buffer);
}

// Start the capture process
void LibcameraCapturer::StartCapture() {
    // Apply the configuration to the camera
    int ret = camera_->configure(camera_config_.get());
    if (ret < 0) {
        ERROR_PRINT("Failed to configure camera");
        exit(1);
    }

    // Allocate buffers for capture
    AllocateBuffer();

    // Start the camera with initial controls
    ret = camera_->start(&controls_);
    if (ret) {
        ERROR_PRINT("Failed to start capturing");
        exit(1);
    }

    // Clear controls after applying them
    controls_.clear();
    
    // Connect the completion callback
    camera_->requestCompleted.connect(this, &LibcameraCapturer::RequestComplete);

    // Queue all initial requests
    for (auto &request : requests_) {
        ret = camera_->queueRequest(request.get());
        if (ret < 0) {
            ERROR_PRINT("Can't queue request");
            camera_->stop();
        }
    }
}
