#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "htra_api.h"

struct Options {
    double start_hz = 2.4e9;
    double stop_hz = 2.5e9;
    double rbw_hz = 100e3;
    double ref_level_dbm = 0.0;
    double fps = 2.0;
    int frames = 0;
};

double parse_double(const char* text, const char* name) {
    char* end = nullptr;
    const double value = std::strtod(text, &end);
    if (!end || *end != '\0' || !std::isfinite(value)) {
        throw std::runtime_error(std::string("invalid ") + name);
    }
    return value;
}

int parse_int(const char* text, const char* name) {
    char* end = nullptr;
    const long value = std::strtol(text, &end, 10);
    if (!end || *end != '\0' || value < 0 || value > 2147483647L) {
        throw std::runtime_error(std::string("invalid ") + name);
    }
    return static_cast<int>(value);
}

Options parse_options(int argc, char** argv) {
    Options options;
    for (int index = 1; index < argc; index += 2) {
        if (index + 1 >= argc) {
            throw std::runtime_error(std::string("missing value for ") + argv[index]);
        }
        const std::string name(argv[index]);
        const char* value = argv[index + 1];
        if (name == "--start-hz") options.start_hz = parse_double(value, "start-hz");
        else if (name == "--stop-hz") options.stop_hz = parse_double(value, "stop-hz");
        else if (name == "--rbw-hz") options.rbw_hz = parse_double(value, "rbw-hz");
        else if (name == "--ref-level-dbm") options.ref_level_dbm = parse_double(value, "ref-level-dbm");
        else if (name == "--fps") options.fps = parse_double(value, "fps");
        else if (name == "--frames") options.frames = parse_int(value, "frames");
        else throw std::runtime_error("unknown option: " + name);
    }
    if (!(options.start_hz > 0 && options.start_hz < options.stop_hz)) {
        throw std::runtime_error("start-hz must be positive and lower than stop-hz");
    }
    if (!(options.rbw_hz > 0 && options.fps > 0)) {
        throw std::runtime_error("rbw-hz and fps must be positive");
    }
    return options;
}

void check_status(int status, const char* operation) {
    if (status != APIRETVAL_NoError) {
        std::ostringstream stream;
        stream << operation << " failed with HTRA status " << status;
        throw std::runtime_error(stream.str());
    }
}

class DeviceGuard {
public:
    void* device = nullptr;
    ~DeviceGuard() {
        if (device) Device_Close(&device);
    }
};

int main(int argc, char** argv) {
    try {
        const Options options = parse_options(argc, argv);
        DeviceGuard guard;
        BootProfile_TypeDef boot_profile{};
        BootInfo_TypeDef boot_info{};
        boot_profile.DevicePowerSupply = USBPortAndPowerPort;
        boot_profile.PhysicalInterface = USB;
        check_status(
            Device_Open(&guard.device, 0, &boot_profile, &boot_info), "Device_Open");

        SWP_Profile_TypeDef profile_in{};
        SWP_Profile_TypeDef profile_out{};
        SWP_TraceInfo_TypeDef trace_info{};
        check_status(
            SWP_ProfileDeInit(&guard.device, &profile_in), "SWP_ProfileDeInit");
        profile_in.StartFreq_Hz = options.start_hz;
        profile_in.StopFreq_Hz = options.stop_hz;
        profile_in.RefLevel_dBm = options.ref_level_dbm;
        profile_in.RBWMode = RBW_Manual;
        profile_in.RBW_Hz = options.rbw_hz;
        check_status(
            SWP_Configuration(
                &guard.device, &profile_in, &profile_out, &trace_info),
            "SWP_Configuration");

        const std::size_t partial_points =
            static_cast<std::size_t>(trace_info.PartialsweepTracePoints);
        const std::size_t hop_count =
            static_cast<std::size_t>(trace_info.TotalHops);
        const std::size_t allocated_points = partial_points * hop_count;
        if (partial_points == 0 || hop_count == 0 || allocated_points < 2) {
            throw std::runtime_error("HTRA returned invalid trace geometry");
        }
        std::vector<double> frequencies(allocated_points);
        std::vector<float> powers(allocated_points);
        MeasAuxInfo_TypeDef aux_info{};
        int hop_index = 0;
        int frame_index = 0;

        std::size_t first = static_cast<std::size_t>(trace_info.UserStartIndex);
        std::size_t last =
            (hop_count - 1) * partial_points
            + static_cast<std::size_t>(trace_info.UserStopIndex);
        if (first >= allocated_points || last < first || last >= allocated_points) {
            first = 0;
            last = std::min(
                allocated_points,
                static_cast<std::size_t>(trace_info.FullsweepTracePoints)) - 1;
        }

        std::cerr << "SAN-60 ready: " << (last - first + 1) << " display points, "
                  << options.start_hz / 1e6 << '-' << options.stop_hz / 1e6
                  << " MHz" << std::endl;
        const auto period = std::chrono::duration<double>(1.0 / options.fps);
        int sequence = 0;
        while (options.frames == 0 || sequence < options.frames) {
            const auto cycle_start = std::chrono::steady_clock::now();
            for (std::size_t hop = 0; hop < hop_count; ++hop) {
                const std::size_t offset = hop * partial_points;
                check_status(
                    SWP_GetPartialSweep(
                        &guard.device,
                        frequencies.data() + offset,
                        powers.data() + offset,
                        &hop_index,
                        &frame_index,
                        &aux_info),
                    "SWP_GetPartialSweep");
            }
            const auto peak = std::max_element(
                powers.begin() + static_cast<std::ptrdiff_t>(first),
                powers.begin() + static_cast<std::ptrdiff_t>(last + 1));
            const std::size_t peak_index =
                static_cast<std::size_t>(std::distance(powers.begin(), peak));
            const double captured_at = std::chrono::duration<double>(
                std::chrono::system_clock::now().time_since_epoch()).count();

            std::ostringstream output;
            output << std::setprecision(15)
                   << "{\"type\":\"spectrum\",\"captured_at\":" << captured_at
                   << ",\"start_hz\":" << frequencies[first]
                   << ",\"stop_hz\":" << frequencies[last]
                   << ",\"bin_hz\":" << trace_info.TraceBinBW_Hz
                   << ",\"rbw_hz\":" << profile_out.RBW_Hz
                   << ",\"ref_level_dbm\":" << profile_out.RefLevel_dBm
                   << ",\"temperature_c\":" << (0.01 * aux_info.Temperature)
                   << ",\"peak_hz\":" << frequencies[peak_index]
                   << ",\"peak_dbm\":" << *peak
                   << ",\"powers_dbm\":[";
            output << std::fixed << std::setprecision(3);
            for (std::size_t index = first; index <= last; ++index) {
                if (index != first) output << ',';
                output << powers[index];
            }
            output << "]}";
            std::cout << output.str() << '\n' << std::flush;
            ++sequence;
            const auto elapsed = std::chrono::steady_clock::now() - cycle_start;
            if (elapsed < period) std::this_thread::sleep_for(period - elapsed);
        }
        return 0;
    } catch (const std::exception& exc) {
        std::cerr << "capture error: " << exc.what() << std::endl;
        return 2;
    }
}
