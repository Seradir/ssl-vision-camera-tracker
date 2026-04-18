# OBS Ball Zoom Plugin

Vollständiges Grundgerüst für ein OBS-Plugin auf Linux, das als **Video-Filter** auf eine beliebige Videoquelle gelegt wird, UDP/Protobuf-Ballpositionen empfängt und einen zentrierten Zoom auf den Ball erzeugt.

## Dateistruktur

```text
obs-ball-zoom/
├── CMakeLists.txt
├── README.md
├── proto/
│   └── messages_robocup_ssl_detection_tracked.proto
└── src/
    ├── plugin-main.cpp
    ├── ball-zoom-filter.hpp
    ├── ball-zoom-filter.cpp
    ├── udp-tracker.hpp
    ├── udp-tracker.cpp
    ├── affine-map.hpp
    └── affine-map.cpp
```

---

## `CMakeLists.txt`

```cmake
cmake_minimum_required(VERSION 3.20)
project(obs-ball-zoom VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBOBS REQUIRED libobs)
pkg_check_modules(LIBAVUTIL REQUIRED libavutil)
pkg_check_modules(LIBSWSCALE REQUIRED libswscale)
find_package(Protobuf REQUIRED)
find_package(Threads REQUIRED)

set(PROTO_FILE ${CMAKE_CURRENT_SOURCE_DIR}/proto/messages_robocup_ssl_detection_tracked.proto)
protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS ${PROTO_FILE})

add_library(obs-ball-zoom MODULE
    src/plugin-main.cpp
    src/ball-zoom-filter.cpp
    src/udp-tracker.cpp
    src/affine-map.cpp
    ${PROTO_SRCS}
)

# Linux plugin naming: obs-ball-zoom.so
set_target_properties(obs-ball-zoom PROPERTIES
    PREFIX ""
    OUTPUT_NAME "obs-ball-zoom"
)

target_include_directories(obs-ball-zoom PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_BINARY_DIR}
    ${LIBOBS_INCLUDE_DIRS}
    ${LIBAVUTIL_INCLUDE_DIRS}
    ${LIBSWSCALE_INCLUDE_DIRS}
    ${Protobuf_INCLUDE_DIRS}
)

target_compile_definitions(obs-ball-zoom PRIVATE
    ${LIBOBS_CFLAGS_OTHER}
    ${LIBAVUTIL_CFLAGS_OTHER}
    ${LIBSWSCALE_CFLAGS_OTHER}
)

target_link_directories(obs-ball-zoom PRIVATE
    ${LIBOBS_LIBRARY_DIRS}
    ${LIBAVUTIL_LIBRARY_DIRS}
    ${LIBSWSCALE_LIBRARY_DIRS}
)

target_link_libraries(obs-ball-zoom PRIVATE
    ${LIBOBS_LIBRARIES}
    ${LIBAVUTIL_LIBRARIES}
    ${LIBSWSCALE_LIBRARIES}
    ${Protobuf_LIBRARIES}
    Threads::Threads
)

install(TARGETS obs-ball-zoom
    LIBRARY DESTINATION lib/obs-plugins
)
```

---

## `README.md`

```md
# OBS Ball Zoom Plugin

## Idee
Dieses Plugin registriert einen OBS-Video-Filter namens **Ball Zoom Filter**.
Der Filter:

- nimmt Frames einer bestehenden OBS-Quelle entgegen
- empfängt per UDP Protobuf-Nachrichten mit Ballpositionen
- mappt `(ball_x, ball_y)` per 2D-affiner Transformation auf Pixelkoordinaten
- schneidet einen Bereich um den Ball aus
- skaliert diesen wieder auf die ursprüngliche Ausgabegröße
- gibt das Ergebnis als gefilterten Stream zurück

## Wichtiger Architekturpunkt
Das Plugin ist **nicht DeckLink-spezifisch**. Es arbeitet als generischer OBS-Filter auf jeder Videoquelle.
Die DeckLink-Karte ist also nur die vorgeschaltete OBS-Quelle.

## Ubuntu-Abhängigkeiten

### Laufzeit / Build
```bash
sudo apt update
sudo apt install -y \
    build-essential \
    cmake \
    pkg-config \
    ninja-build \
    obs-studio \
    libobs-dev \
    protobuf-compiler \
    libprotobuf-dev \
    libswscale-dev \
    libavutil-dev
```

### Plugin bauen
```bash
mkdir -p build
cd build
cmake -G Ninja ..
ninja
```

### Plugin installieren
Je nach Distribution liegt der OBS-Plugin-Pfad typischerweise hier:

```bash
mkdir -p ~/.config/obs-studio/plugins/obs-ball-zoom/bin/64bit
cp obs-ball-zoom.so ~/.config/obs-studio/plugins/obs-ball-zoom/bin/64bit/
```

Alternativ systemweit:

```bash
sudo cp obs-ball-zoom.so /usr/lib/x86_64-linux-gnu/obs-plugins/
```

## OBS-Nutzung
1. DeckLink-Quelle oder andere Videoquelle in OBS hinzufügen.
2. Auf die Quelle einen Filter legen.
3. Filtertyp **Ball Zoom Filter** auswählen.
4. UDP-Port, Zoom und die vier Kalibrierpunkte setzen.
```

---

## `src/affine-map.hpp`

```cpp
#pragma once

#include <array>
#include <optional>

struct WorldPoint {
    double x = 0.0;
    double y = 0.0;
};

struct ImagePoint {
    double x = 0.0;
    double y = 0.0;
};

struct CalibrationPair {
    WorldPoint world;
    ImagePoint image;
};

class AffineMap {
public:
    bool solve(const std::array<CalibrationPair, 4>& pairs);
    std::optional<ImagePoint> map(double world_x, double world_y) const;
    bool valid() const { return valid_; }

private:
    // x_img = a11*x + a12*y + a13
    // y_img = a21*x + a22*y + a23
    double a11_ = 1.0;
    double a12_ = 0.0;
    double a13_ = 0.0;
    double a21_ = 0.0;
    double a22_ = 1.0;
    double a23_ = 0.0;
    bool valid_ = false;
};
```

---

## `src/affine-map.cpp`

```cpp
#include "affine-map.hpp"

#include <cmath>

namespace {

bool solve_3x3(double m[3][4], double out[3])
{
    for (int col = 0; col < 3; ++col) {
        int pivot = col;
        double best = std::fabs(m[col][col]);
        for (int row = col + 1; row < 3; ++row) {
            double v = std::fabs(m[row][col]);
            if (v > best) {
                best = v;
                pivot = row;
            }
        }

        if (best < 1e-12) {
            return false;
        }

        if (pivot != col) {
            for (int k = col; k < 4; ++k) {
                std::swap(m[col][k], m[pivot][k]);
            }
        }

        double div = m[col][col];
        for (int k = col; k < 4; ++k) {
            m[col][k] /= div;
        }

        for (int row = 0; row < 3; ++row) {
            if (row == col) {
                continue;
            }
            double factor = m[row][col];
            for (int k = col; k < 4; ++k) {
                m[row][k] -= factor * m[col][k];
            }
        }
    }

    out[0] = m[0][3];
    out[1] = m[1][3];
    out[2] = m[2][3];
    return true;
}

} // namespace

bool AffineMap::solve(const std::array<CalibrationPair, 4>& pairs)
{
    // Least squares from 4 points to 6 affine params.
    // Solve two independent systems based on normal equations.
    double ata[3][3] = {};
    double atbx[3] = {};
    double atby[3] = {};

    for (const auto& p : pairs) {
        const double x = p.world.x;
        const double y = p.world.y;
        const double b1 = p.image.x;
        const double b2 = p.image.y;
        const double r[3] = {x, y, 1.0};

        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                ata[i][j] += r[i] * r[j];
            }
            atbx[i] += r[i] * b1;
            atby[i] += r[i] * b2;
        }
    }

    double mx[3][4];
    double my[3][4];
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            mx[i][j] = ata[i][j];
            my[i][j] = ata[i][j];
        }
        mx[i][3] = atbx[i];
        my[i][3] = atby[i];
    }

    double ox[3];
    double oy[3];
    if (!solve_3x3(mx, ox) || !solve_3x3(my, oy)) {
        valid_ = false;
        return false;
    }

    a11_ = ox[0];
    a12_ = ox[1];
    a13_ = ox[2];
    a21_ = oy[0];
    a22_ = oy[1];
    a23_ = oy[2];
    valid_ = true;
    return true;
}

std::optional<ImagePoint> AffineMap::map(double world_x, double world_y) const
{
    if (!valid_) {
        return std::nullopt;
    }

    return ImagePoint{
        a11_ * world_x + a12_ * world_y + a13_,
        a21_ * world_x + a22_ * world_y + a23_
    };
}
```

---

## `src/udp-tracker.hpp`

```cpp
#pragma once

#include <atomic>
#include <cstdint>
#include <mutex>
#include <optional>
#include <thread>

struct LatestBallState {
    double x = 0.0;
    double y = 0.0;
    bool valid = false;
    uint64_t sequence = 0;
};

class UdpTracker {
public:
    UdpTracker();
    ~UdpTracker();

    bool start(int port);
    void stop();
    std::optional<LatestBallState> get_latest() const;

private:
    void thread_main(int port);

    mutable std::mutex mutex_;
    LatestBallState latest_;
    std::atomic<bool> running_{false};
    std::thread thread_;
    int socket_fd_ = -1;
};
```

---

## `src/udp-tracker.cpp`

```cpp
#include "udp-tracker.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <vector>

#include <obs-module.h>

#include "messages_robocup_ssl_detection_tracked.pb.h"

UdpTracker::UdpTracker() = default;

UdpTracker::~UdpTracker()
{
    stop();
}

bool UdpTracker::start(int port)
{
    stop();

    running_ = true;
    thread_ = std::thread(&UdpTracker::thread_main, this, port);
    return true;
}

void UdpTracker::stop()
{
    running_ = false;

    if (socket_fd_ >= 0) {
        shutdown(socket_fd_, SHUT_RDWR);
        close(socket_fd_);
        socket_fd_ = -1;
    }

    if (thread_.joinable()) {
        thread_.join();
    }
}

std::optional<LatestBallState> UdpTracker::get_latest() const
{
    std::scoped_lock lock(mutex_);
    if (!latest_.valid) {
        return std::nullopt;
    }
    return latest_;
}

void UdpTracker::thread_main(int port)
{
    GOOGLE_PROTOBUF_VERIFY_VERSION;

    socket_fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (socket_fd_ < 0) {
        blog(LOG_ERROR, "obs-ball-zoom: socket() failed: %s", std::strerror(errno));
        running_ = false;
        return;
    }

    int reuse = 1;
    setsockopt(socket_fd_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(static_cast<uint16_t>(port));
    addr.sin_addr.s_addr = htonl(INADDR_ANY);

    if (bind(socket_fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
        blog(LOG_ERROR, "obs-ball-zoom: bind(%d) failed: %s", port, std::strerror(errno));
        close(socket_fd_);
        socket_fd_ = -1;
        running_ = false;
        return;
    }

    std::vector<std::uint8_t> buffer(65536);

    while (running_) {
        ssize_t received = recv(socket_fd_, buffer.data(), buffer.size(), 0);
        if (received <= 0) {
            if (!running_) {
                break;
            }
            continue;
        }

        // Annahme: Datagramm enthält direkt TrackedFrame.
        // Falls bei dir TrackerWrapperPacket genutzt wird, muss hier auf das Wrapper-Proto umgestellt werden.
        TrackedFrame frame;
        if (!frame.ParseFromArray(buffer.data(), static_cast<int>(received))) {
            continue;
        }

        if (frame.balls_size() <= 0) {
            continue;
        }

        const auto& ball = frame.balls(0);
        if (!ball.has_pos()) {
            continue;
        }

        LatestBallState state;
        state.x = ball.pos().x();
        state.y = ball.pos().y();
        state.valid = true;

        {
            std::scoped_lock lock(mutex_);
            state.sequence = latest_.sequence + 1;
            latest_ = state;
        }
    }
}
```

---

## `src/ball-zoom-filter.hpp`

```cpp
#pragma once

#include <array>
#include <memory>
#include <mutex>
#include <optional>

extern "C" {
#include <libobs/obs-module.h>
}

#include <libswscale/swscale.h>

#include "affine-map.hpp"
#include "udp-tracker.hpp"

struct BallZoomFilter {
    obs_source_t* context = nullptr;

    int udp_port = 10020;
    double zoom_factor = 2.0;

    std::array<CalibrationPair, 4> calibration{};
    AffineMap affine;

    UdpTracker tracker;

    mutable std::mutex state_mutex;

    SwsContext* to_bgra_ctx = nullptr;
    SwsContext* scale_ctx = nullptr;

    int cached_src_width = 0;
    int cached_src_height = 0;
    AVPixelFormat cached_src_pixfmt = AV_PIX_FMT_NONE;

    int cached_crop_width = 0;
    int cached_crop_height = 0;
};

void* ball_zoom_create(obs_data_t* settings, obs_source_t* source);
void ball_zoom_destroy(void* data);
void ball_zoom_update(void* data, obs_data_t* settings);
obs_properties_t* ball_zoom_properties(void* data);
const char* ball_zoom_name(void* type_data);
obs_source_frame* ball_zoom_filter_video(void* data, obs_source_frame* frame);
```

---

## `src/ball-zoom-filter.cpp`

```cpp
#include "ball-zoom-filter.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <memory>
#include <vector>

extern "C" {
#include <libavutil/imgutils.h>
#include <libavutil/pixfmt.h>
}

namespace {

constexpr const char* SETTING_PORT = "udp_port";
constexpr const char* SETTING_ZOOM = "zoom_factor";

AVPixelFormat obs_to_av_pixfmt(enum video_format fmt)
{
    switch (fmt) {
    case VIDEO_FORMAT_BGRA:
    case VIDEO_FORMAT_BGRX:
        return AV_PIX_FMT_BGRA;
    case VIDEO_FORMAT_RGBA:
        return AV_PIX_FMT_RGBA;
    case VIDEO_FORMAT_YUY2:
        return AV_PIX_FMT_YUYV422;
    case VIDEO_FORMAT_UYVY:
        return AV_PIX_FMT_UYVY422;
    case VIDEO_FORMAT_NV12:
        return AV_PIX_FMT_NV12;
    case VIDEO_FORMAT_I420:
        return AV_PIX_FMT_YUV420P;
    default:
        return AV_PIX_FMT_NONE;
    }
}

void set_default_pairs(obs_data_t* settings)
{
    // Beispielwerte, müssen im OBS-Dialog überschrieben werden.
    for (int i = 0; i < 4; ++i) {
        const std::string idx = std::to_string(i + 1);
        obs_data_set_default_double(settings, ("world_x_" + idx).c_str(), i * 1000.0);
        obs_data_set_default_double(settings, ("world_y_" + idx).c_str(), i * 500.0);
        obs_data_set_default_double(settings, ("image_x_" + idx).c_str(), i * 100.0);
        obs_data_set_default_double(settings, ("image_y_" + idx).c_str(), i * 80.0);
    }
}

void load_pairs(std::array<CalibrationPair, 4>& pairs, obs_data_t* settings)
{
    for (int i = 0; i < 4; ++i) {
        const std::string idx = std::to_string(i + 1);
        pairs[i].world.x = obs_data_get_double(settings, ("world_x_" + idx).c_str());
        pairs[i].world.y = obs_data_get_double(settings, ("world_y_" + idx).c_str());
        pairs[i].image.x = obs_data_get_double(settings, ("image_x_" + idx).c_str());
        pairs[i].image.y = obs_data_get_double(settings, ("image_y_" + idx).c_str());
    }
}

obs_source_frame* alloc_bgra_frame(uint32_t width, uint32_t height, uint64_t timestamp)
{
    auto* out = static_cast<obs_source_frame*>(bzalloc(sizeof(obs_source_frame)));
    out->format = VIDEO_FORMAT_BGRA;
    out->width = width;
    out->height = height;
    out->timestamp = timestamp;
    out->linesize[0] = static_cast<uint32_t>(width * 4);
    out->data[0] = static_cast<uint8_t*>(bzalloc(static_cast<size_t>(out->linesize[0]) * height));
    return out;
}

void free_frame(obs_source_frame* frame)
{
    if (!frame) {
        return;
    }
    if (frame->data[0]) {
        bfree(frame->data[0]);
    }
    bfree(frame);
}

} // namespace

const char* ball_zoom_name(void*)
{
    return "Ball Zoom Filter";
}

void* ball_zoom_create(obs_data_t* settings, obs_source_t* source)
{
    auto* filter = new BallZoomFilter();
    filter->context = source;
    ball_zoom_update(filter, settings);
    return filter;
}

void ball_zoom_destroy(void* data)
{
    auto* filter = static_cast<BallZoomFilter*>(data);
    if (!filter) {
        return;
    }

    filter->tracker.stop();

    if (filter->to_bgra_ctx) {
        sws_freeContext(filter->to_bgra_ctx);
        filter->to_bgra_ctx = nullptr;
    }
    if (filter->scale_ctx) {
        sws_freeContext(filter->scale_ctx);
        filter->scale_ctx = nullptr;
    }

    delete filter;
}

void ball_zoom_update(void* data, obs_data_t* settings)
{
    auto* filter = static_cast<BallZoomFilter*>(data);
    if (!filter) {
        return;
    }

    std::scoped_lock lock(filter->state_mutex);

    filter->udp_port = static_cast<int>(obs_data_get_int(settings, SETTING_PORT));
    filter->zoom_factor = std::max(1.0, obs_data_get_double(settings, SETTING_ZOOM));

    load_pairs(filter->calibration, settings);
    filter->affine.solve(filter->calibration);

    filter->tracker.start(filter->udp_port);
}

obs_properties_t* ball_zoom_properties(void*)
{
    obs_properties_t* props = obs_properties_create();

    obs_properties_add_int(props, SETTING_PORT, "UDP Port", 1, 65535, 1);
    obs_properties_add_float(props, SETTING_ZOOM, "Zoom Factor", 1.0, 16.0, 0.1);

    for (int i = 0; i < 4; ++i) {
        const std::string idx = std::to_string(i + 1);
        obs_properties_add_float(props, ("world_x_" + idx).c_str(), ("World X " + idx).c_str(), -100000.0, 100000.0, 0.01);
        obs_properties_add_float(props, ("world_y_" + idx).c_str(), ("World Y " + idx).c_str(), -100000.0, 100000.0, 0.01);
        obs_properties_add_float(props, ("image_x_" + idx).c_str(), ("Image X " + idx).c_str(), -100000.0, 100000.0, 0.01);
        obs_properties_add_float(props, ("image_y_" + idx).c_str(), ("Image Y " + idx).c_str(), -100000.0, 100000.0, 0.01);
    }

    return props;
}

obs_source_frame* ball_zoom_filter_video(void* data, obs_source_frame* frame)
{
    auto* filter = static_cast<BallZoomFilter*>(data);
    if (!filter || !frame) {
        return frame;
    }

    std::optional<LatestBallState> latest;
    double zoom = 2.0;
    AffineMap affine;

    {
        std::scoped_lock lock(filter->state_mutex);
        latest = filter->tracker.get_latest();
        zoom = filter->zoom_factor;
        affine = filter->affine;
    }

    if (!latest || !affine.valid()) {
        return frame;
    }

    auto mapped = affine.map(latest->x, latest->y);
    if (!mapped) {
        return frame;
    }

    const int src_w = static_cast<int>(frame->width);
    const int src_h = static_cast<int>(frame->height);
    if (src_w <= 0 || src_h <= 0) {
        return frame;
    }

    const AVPixelFormat src_fmt = obs_to_av_pixfmt(frame->format);
    if (src_fmt == AV_PIX_FMT_NONE) {
        // Unbekanntes OBS-Format: wir reichen durch.
        return frame;
    }

    if (!filter->to_bgra_ctx ||
        filter->cached_src_width != src_w ||
        filter->cached_src_height != src_h ||
        filter->cached_src_pixfmt != src_fmt) {

        if (filter->to_bgra_ctx) {
            sws_freeContext(filter->to_bgra_ctx);
        }

        filter->to_bgra_ctx = sws_getContext(
            src_w, src_h, src_fmt,
            src_w, src_h, AV_PIX_FMT_BGRA,
            SWS_BILINEAR,
            nullptr, nullptr, nullptr
        );

        filter->cached_src_width = src_w;
        filter->cached_src_height = src_h;
        filter->cached_src_pixfmt = src_fmt;
    }

    if (!filter->to_bgra_ctx) {
        return frame;
    }

    std::vector<uint8_t> bgra_buffer(static_cast<size_t>(src_w) * src_h * 4);
    uint8_t* dst_data[4] = {bgra_buffer.data(), nullptr, nullptr, nullptr};
    int dst_linesize[4] = {src_w * 4, 0, 0, 0};

    const uint8_t* src_data[4] = {
        frame->data[0], frame->data[1], frame->data[2], frame->data[3]
    };
    int src_linesize[4] = {
        static_cast<int>(frame->linesize[0]),
        static_cast<int>(frame->linesize[1]),
        static_cast<int>(frame->linesize[2]),
        static_cast<int>(frame->linesize[3])
    };

    sws_scale(filter->to_bgra_ctx, src_data, src_linesize, 0, src_h, dst_data, dst_linesize);

    const int crop_w = std::max(1, static_cast<int>(std::llround(src_w / zoom)));
    const int crop_h = std::max(1, static_cast<int>(std::llround(src_h / zoom)));

    int center_x = static_cast<int>(std::llround(mapped->x));
    int center_y = static_cast<int>(std::llround(mapped->y));

    int crop_x = center_x - crop_w / 2;
    int crop_y = center_y - crop_h / 2;

    crop_x = std::clamp(crop_x, 0, std::max(0, src_w - crop_w));
    crop_y = std::clamp(crop_y, 0, std::max(0, src_h - crop_h));

    const uint8_t* crop_src[4] = {
        bgra_buffer.data() + static_cast<size_t>(crop_y) * src_w * 4 + static_cast<size_t>(crop_x) * 4,
        nullptr, nullptr, nullptr
    };
    int crop_linesize[4] = {src_w * 4, 0, 0, 0};

    if (!filter->scale_ctx ||
        filter->cached_crop_width != crop_w ||
        filter->cached_crop_height != crop_h) {

        if (filter->scale_ctx) {
            sws_freeContext(filter->scale_ctx);
        }

        filter->scale_ctx = sws_getContext(
            crop_w, crop_h, AV_PIX_FMT_BGRA,
            src_w, src_h, AV_PIX_FMT_BGRA,
            SWS_BICUBIC,
            nullptr, nullptr, nullptr
        );

        filter->cached_crop_width = crop_w;
        filter->cached_crop_height = crop_h;
    }

    if (!filter->scale_ctx) {
        return frame;
    }

    obs_source_frame* out = alloc_bgra_frame(frame->width, frame->height, frame->timestamp);
    uint8_t* out_data[4] = {out->data[0], nullptr, nullptr, nullptr};
    int out_linesize[4] = {static_cast<int>(out->linesize[0]), 0, 0, 0};

    sws_scale(filter->scale_ctx, crop_src, crop_linesize, 0, crop_h, out_data, out_linesize);

    // OBS übernimmt das zurückgegebene Frame nicht automatisch immer mit Ownership.
    // In vielen Plugins wird das Frame nach downstream-Verwendung wieder freigegeben.
    // Für ein minimales funktionales Gerüst geben wir das Frame direkt zurück.
    // Sollte sich bei deiner OBS-Version Ownership anders verhalten, muss ein eigener async output path gewählt werden.
    return out;
}
```

---

## `src/plugin-main.cpp`

```cpp
#include <obs-module.h>

#include "ball-zoom-filter.hpp"

OBS_DECLARE_MODULE()
OBS_MODULE_USE_DEFAULT_LOCALE("obs-ball-zoom", "en-US")

MODULE_EXPORT const char* obs_module_description(void)
{
    return "OBS filter plugin that zooms onto a tracked RoboCup SSL ball.";
}

bool obs_module_load(void)
{
    obs_source_info info{};
    info.id = "ball_zoom_filter";
    info.type = OBS_SOURCE_TYPE_FILTER;
    info.output_flags = OBS_SOURCE_VIDEO;
    info.get_name = ball_zoom_name;
    info.create = ball_zoom_create;
    info.destroy = ball_zoom_destroy;
    info.update = ball_zoom_update;
    info.get_properties = ball_zoom_properties;
    info.filter_video = ball_zoom_filter_video;

    obs_register_source(&info);
    blog(LOG_INFO, "obs-ball-zoom loaded successfully");
    return true;
}
```

---

## `proto/messages_robocup_ssl_detection_tracked.proto`

```proto
// Hier die Originaldatei aus ssl-vision ablegen:
// https://github.com/RoboCup-SSL/ssl-vision/blob/master/src/shared/proto/messages_robocup_ssl_detection_tracked.proto
```

---

## Hinweise zur Qualität / Review

```md
1. Das Plugin ist absichtlich als OBS-Filter gebaut, nicht als neue Capture-Quelle.
   Dadurch ist es generisch und kann auch mit DeckLink verwendet werden.

2. Die 4 Kalibrierpunkte werden hier als affine 2D-Approximation verwendet.
   Das ist mathematisch bewusst einfacher als eine vollständige Homographie.
   Wenn deine Kamera stark perspektivisch verzerrt ist, solltest du später auf Homographie wechseln.

3. Das UDP-Parsing nimmt aktuell an, dass direkt `TrackedFrame` gesendet wird.
   Falls bei dir `TrackerWrapperPacket` gesendet wird, musst du das Proto dafür zusätzlich einbinden.

4. Die swscale-Konvertierung macht das Plugin deutlich generischer gegenüber OBS-Frame-Formaten.
   Das ist wichtig, weil DeckLink in OBS nicht zwingend schon BGRA liefert.

5. Der heikelste technische Punkt ist die Ownership des zurückgegebenen `obs_source_frame`.
   Die API ist da nicht ganz so gemütlich wie man es gern hätte.
   Wenn du beim Test Speicherprobleme siehst, sollte man die Ausgabe auf einen expliziten async path umbauen.
```

