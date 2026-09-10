// Explicit GPU test (not automatic CTest: requires the optional CUDA plugin/device).
#include <algorithm>
#include <cassert>
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <numeric>
#include <stdexcept>
#include "ORBextractor.h"

int main(int argc, char** argv)
{
    cv::Mat image;
    if (argc > 1) image = cv::imread(argv[1], cv::IMREAD_GRAYSCALE);
    else { image = cv::Mat(480, 640, CV_8UC1); cv::RNG(7).fill(image, cv::RNG::UNIFORM, 0, 256); }
    assert(!image.empty());
    ORB_SLAM3::ORBextractor extractor(1500, 1.2, 8, 20, 7);
    std::vector<int> overlap{0, 0};
    std::vector<cv::KeyPoint> cpu_points, gpu_points;
    cv::Mat cpu, gpu;
    int cases = 0;
    auto check = [&](const cv::Mat& sample, std::vector<int> area) {
        setenv("ORB_SLAM3_DESCRIPTOR_BACKEND", "cpu", 1);
        const int cpu_mono = extractor(sample, cv::Mat(), cpu_points, cpu, area);
        setenv("ORB_SLAM3_DESCRIPTOR_BACKEND", "cuda", 1);
        const int gpu_mono = extractor(sample, cv::Mat(), gpu_points, gpu, area);
        assert(cpu_mono == gpu_mono && cpu_points.size() == gpu_points.size());
        assert(cpu.size() == gpu.size() && cpu.type() == gpu.type());
        assert(cpu.empty() || cv::countNonZero(cpu != gpu) == 0);
        for (size_t i = 0; i < cpu_points.size(); ++i) {
            assert(cpu_points[i].pt == gpu_points[i].pt);
            assert(cpu_points[i].angle == gpu_points[i].angle);
            assert(cpu_points[i].octave == gpu_points[i].octave);
            assert(cpu_points[i].size == gpu_points[i].size);
            assert(cpu_points[i].response == gpu_points[i].response);
            const bool stereo = cpu_points[i].pt.x >= area[0] && cpu_points[i].pt.x <= area[1];
            assert(stereo == (int(i) >= cpu_mono));
        }
        ++cases;
    };
    for (const cv::Size size : {cv::Size(641, 479), cv::Size(321, 241), cv::Size(960, 720)}) {
        cv::Mat sample(size, CV_8UC1);
        cv::RNG(7).fill(sample, cv::RNG::UNIFORM, 0, 256);
        for (auto area : {std::vector<int>{0, 0}, std::vector<int>{size.width/3, 2*size.width/3},
                          std::vector<int>{0, size.width}})
            check(sample, area);
    }
    cv::Mat sparse = cv::Mat::zeros(480, 640, CV_8UC1);
    cv::Mat patch = sparse(cv::Rect(280, 200, 32, 32));
    cv::RNG(11).fill(patch, cv::RNG::UNIFORM, 0, 256);
    check(sparse, {250, 350});
    const cv::Mat blank = cv::Mat::zeros(480, 640, CV_8UC1);
    check(blank, overlap);
    assert(cpu_points.empty() && gpu.empty());
    for (const cv::Mat& sample : {blank, cv::Mat()}) {
        setenv("ORB_SLAM3_DESCRIPTOR_BACKEND", "invalid", 1);
        bool rejected = false;
        try { extractor(sample, cv::Mat(), gpu_points, gpu, overlap); }
        catch (const std::runtime_error&) { rejected = true; }
        assert(rejected);
    }
    check(image, overlap); // Reuse buffers after larger/smaller and featureless inputs.
    assert(!cpu_points.empty());
    std::cout << "{\"exact_descriptors\":true,\"cases\":" << cases
              << ",\"features\":" << cpu_points.size();
    for (const char* backend : {"cpu", "cuda"}) {
        setenv("ORB_SLAM3_DESCRIPTOR_BACKEND", backend, 1);
        std::vector<double> times;
        for (int i = 0; i < 43; ++i) {
            auto start = std::chrono::steady_clock::now();
            extractor(image, cv::Mat(), gpu_points, gpu, overlap);
            if (i >= 3) times.push_back(std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - start).count());
        }
        std::sort(times.begin(), times.end());
        std::cout << ",\"" << backend << "_extraction_ms\":{\"mean\":"
            << std::accumulate(times.begin(), times.end(), 0.) / times.size()
            << ",\"median\":" << times[times.size()/2] << ",\"p95\":" << times[38] << "}";
    }
    std::cout << "}\n";
}
