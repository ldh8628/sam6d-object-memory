// Single RealSense RGB-D camera -> ordered ORB-SLAM3 localization -> ROS poses.
// Frame loss is a recorded run failure, never a latest-frame optimization.
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <distributed_slam_interfaces/msg/state.hpp>
#include <distributed_slam_interfaces/msg/session.hpp>
#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
#include <opencv2/opencv.hpp>
#include <System.h>
#include <MapPoint.h>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <csignal>
#include <deque>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <sstream>
#include <thread>
#include <algorithm>
#include <cmath>
#include <tuple>
#include <ctime>
#include <sched.h>
#include <sys/resource.h>

using Clock = std::chrono::steady_clock;
static std::atomic<bool> interrupted{false};
static void onSignal(int) { interrupted.store(true); }
static int64_t steadyNs() { return std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now().time_since_epoch()).count(); }
static int64_t threadCpuNs() { timespec t{}; clock_gettime(CLOCK_THREAD_CPUTIME_ID,&t); return int64_t(t.tv_sec)*1000000000LL+t.tv_nsec; }
static int64_t wallNs() { return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count(); }
static std::string jsonEscape(const std::string& text) {
    std::ostringstream out;
    for(unsigned char c:text) {
        if(c=='"' || c=='\\') out << '\\' << c;
        else if(c<0x20) out << "\\u" << std::hex << std::setw(4) << std::setfill('0') << static_cast<int>(c);
        else out << c;
    }
    return out.str();
}
struct SensorOptions {
    std::vector<std::tuple<rs2::sensor,rs2_option,float>> saved;
    void set(rs2::sensor sensor,rs2_option option,float value) {
        const float previous=sensor.get_option(option);
        if(previous==value) return;
        saved.emplace_back(sensor,option,previous);
        sensor.set_option(option,value);
    }
    ~SensorOptions() {
        for(auto it=saved.rbegin();it!=saved.rend();++it)
            try { std::get<0>(*it).set_option(std::get<1>(*it),std::get<2>(*it)); }
            catch(const std::exception& e) { std::cerr << "Camera option restore: " << e.what() << std::endl; }
    }
};
struct Pair {
    cv::Mat rgb, depth;
    uint64_t sequence=0, rgb_number=0, depth_number=0;
    double stamp=0, depth_stamp=0;
    int64_t capture_ns=0, enqueue_ns=0, enqueue_steady=0;
    int64_t pair_ready_steady=0,aligned_steady=0;
};
struct InputQueue {
    std::mutex mutex;
    std::condition_variable ready;
    std::deque<Pair> pairs;
    size_t capacity=120, high_water=0;
    uint64_t received=0, accepted=0, rejected=0, rgb_gaps=0, depth_gaps=0;
    uint64_t raw_rgb_received=0,raw_depth_received=0;
    bool done=false;
    std::string error,camera_serial;
    void fail(const std::string& reason) {
        std::lock_guard<std::mutex> lock(mutex);
        if(error.empty()) error=reason;
        ready.notify_all();
    }
    bool stopped() { std::lock_guard<std::mutex> lock(mutex); return !error.empty() || interrupted.load(); }
    bool failed() { std::lock_guard<std::mutex> lock(mutex); return !error.empty(); }
    bool push(Pair pair) {
        std::lock_guard<std::mutex> lock(mutex);
        ++received;
        if(pairs.size()>=capacity) {
            ++rejected;
            if(error.empty()) error="input FIFO capacity exceeded";
            ready.notify_all();
            return false;
        }
        pair.sequence=++accepted;
        pairs.emplace_back(std::move(pair));
        high_water=std::max(high_water,pairs.size());
        ready.notify_one();
        return true;
    }
    bool pop(Pair& pair) {
        std::unique_lock<std::mutex> lock(mutex);
        ready.wait(lock,[&]{return !pairs.empty() || done;});
        if(pairs.empty()) return false;
        pair=std::move(pairs.front()); pairs.pop_front(); return true;
    }
    void finish() { std::lock_guard<std::mutex> lock(mutex); done=true; ready.notify_all(); }
};
// Keep filesystem latency (especially /mnt/c on WSL) off the tracking thread.
class FrameWriter {
    std::ofstream& csv; std::ofstream& trajectory; InputQueue& input;
    std::mutex mutex; std::condition_variable ready,space;
    std::deque<std::pair<std::string,std::string>> rows;
    bool done=false; std::thread worker;
public:
    FrameWriter(std::ofstream& c,std::ofstream& t,InputQueue& q):csv(c),trajectory(t),input(q),worker([this]{
        while(true) {
            std::pair<std::string,std::string> row;
            {
                std::unique_lock<std::mutex> lock(mutex);
                ready.wait(lock,[&]{return done || !rows.empty();});
                if(rows.empty()) break;
                row=std::move(rows.front()); rows.pop_front(); space.notify_one();
            }
            csv<<row.first; trajectory<<row.second;
            if(!csv || !trajectory) input.fail("telemetry output write failed");
        }
        csv.flush(); trajectory.flush();
        if(!csv || !trajectory) input.fail("telemetry output flush failed");
    }) {}
    void append(std::string row,std::string pose) {
        std::unique_lock<std::mutex> lock(mutex);
        if(rows.size()>=4096) {
            input.fail("telemetry writer backlog exceeded 4096 frames");
            // Stop capture, but preserve already processed frame records.
            space.wait(lock,[&]{return rows.size()<4096;});
        }
        rows.emplace_back(std::move(row),std::move(pose));ready.notify_one();
    }
    void finish() {
        { std::lock_guard<std::mutex> lock(mutex); done=true; ready.notify_one(); }
        if(worker.joinable()) worker.join();
    }
    ~FrameWriter() { finish(); }
};
static double quantile(std::vector<double> v,double p) {
    if(v.empty()) return 0;
    std::sort(v.begin(),v.end());
    return v[std::min(v.size()-1,static_cast<size_t>(std::ceil(p*v.size())-1))];
}
static void readDataset(const std::string& directory, double rate, int max_frames, InputQueue& queue) {
    try {
        std::ifstream input(directory+"/associations.txt");
        if(!input) throw std::runtime_error("dataset requires associations.txt: RGB_timestamp RGB_path depth_timestamp depth_path");
        std::string line; double first=0,last_stamp=-1,last_depth_stamp=-1; int index=0; cv::Size image_size; auto start=Clock::now();
        while(std::getline(input,line) && !queue.stopped()) {
            if(line.empty() || line[0]=='#') continue;
            std::istringstream row(line); Pair pair; std::string rgb_path,depth_path;
            if(!(row>>pair.stamp>>rgb_path>>pair.depth_stamp>>depth_path)) throw std::runtime_error("malformed dataset association");
            if(!std::isfinite(pair.stamp) || !std::isfinite(pair.depth_stamp) ||
               pair.stamp<=last_stamp || pair.depth_stamp<=last_depth_stamp)
                throw std::runtime_error("dataset timestamp reversal or duplicate");
            last_stamp=pair.stamp; last_depth_stamp=pair.depth_stamp;
            pair.rgb=cv::imread(directory+"/"+rgb_path,cv::IMREAD_COLOR);
            pair.depth=cv::imread(directory+"/"+depth_path,cv::IMREAD_UNCHANGED);
            if(pair.rgb.empty() || pair.depth.empty() || pair.depth.type()!=CV_16UC1 || pair.rgb.size()!=pair.depth.size())
                throw std::runtime_error("invalid RGB-D dataset image");
            if(index==0) { first=pair.stamp; start=Clock::now(); image_size=pair.rgb.size(); }
            if(pair.rgb.size()!=image_size) throw std::runtime_error("dataset resolution changed mid-stream");
            if(rate>0) {
                auto deadline=start+std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>((pair.stamp-first)/rate));
                while(Clock::now()<deadline && !queue.stopped())
                    std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
            if(queue.stopped()) break;
            pair.capture_ns=static_cast<int64_t>(pair.stamp*1e9);
            pair.enqueue_ns=wallNs(); pair.enqueue_steady=steadyNs();
            pair.pair_ready_steady=pair.aligned_steady=pair.enqueue_steady;
            pair.rgb_number=pair.depth_number=++index;
            if(!queue.push(std::move(pair))) break;
            if(max_frames>0 && index>=max_frames) break;
        }
        if(index==0 && !interrupted) throw std::runtime_error("dataset has no input frames");
    } catch(const std::exception& e) { queue.fail(e.what()); }
    queue.finish();
}
static void readCamera(const std::string& serial, const std::string& settings_path, const std::string& bag, bool realtime, int max_frames,
                       double exposure, double max_skew_ms, InputQueue& queue) {
    try {
        cv::FileStorage settings(settings_path,cv::FileStorage::READ);
        const int width=static_cast<int>(settings["Camera.width"]), height=static_cast<int>(settings["Camera.height"]);
        const int fps=static_cast<int>(settings["Camera.fps"]);
        const double factor=static_cast<double>(settings["RGBD.DepthMapFactor"]);
        if(width<=0 || height<=0 || fps<=0 || factor<=0) throw std::runtime_error("invalid RGB-D settings dimensions/fps/depth factor");
        rs2::context context;
        auto devices=context.query_devices();
        if(bag.empty() && devices.size()==0) throw std::runtime_error("no RealSense camera connected");
        if(bag.empty() && serial.empty() && devices.size()!=1) throw std::runtime_error("more than one RealSense camera: specify --serial");
        rs2::pipeline pipeline(context); rs2::config config;
        if(!bag.empty()) config.enable_device_from_file(bag,false);
        else if(!serial.empty()) config.enable_device(serial);
        config.enable_stream(RS2_STREAM_COLOR,width,height,RS2_FORMAT_BGR8,fps);
        config.enable_stream(RS2_STREAM_DEPTH,width,height,RS2_FORMAT_Z16,fps);
        auto resolved=config.resolve(pipeline);
        auto device=resolved.get_device();
        {
            std::lock_guard<std::mutex> lock(queue.mutex);
            queue.camera_serial=device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
        }
        SensorOptions options;
        // Set before start. No automatic FPS reduction to lengthen exposure.
        if(!bag.empty()) device.as<rs2::playback>().set_real_time(realtime);
        if(bag.empty()) for(auto sensor: device.query_sensors()) {
            if(sensor.supports(RS2_OPTION_GLOBAL_TIME_ENABLED)) options.set(sensor,RS2_OPTION_GLOBAL_TIME_ENABLED,1);
            if(sensor.supports(RS2_OPTION_AUTO_EXPOSURE_PRIORITY)) options.set(sensor,RS2_OPTION_AUTO_EXPOSURE_PRIORITY,0);
            if(sensor.supports(RS2_OPTION_FRAMES_QUEUE_SIZE)) {
                auto range=sensor.get_option_range(RS2_OPTION_FRAMES_QUEUE_SIZE);
                options.set(sensor,RS2_OPTION_FRAMES_QUEUE_SIZE,std::min(16.f,range.max));
            }
            if(exposure>0 && sensor.is<rs2::color_sensor>()) {
                auto range=sensor.get_option_range(RS2_OPTION_EXPOSURE);
                if(exposure<range.min || exposure>range.max) throw std::runtime_error("color exposure outside SDK range");
                options.set(sensor,RS2_OPTION_ENABLE_AUTO_EXPOSURE,0);
                options.set(sensor,RS2_OPTION_EXPOSURE,exposure);
            }
        }
        auto intr=resolved.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>().get_intrinsics();
        const double fx=static_cast<double>(settings["Camera1.fx"]),fy=static_cast<double>(settings["Camera1.fy"]);
        const double cx=static_cast<double>(settings["Camera1.cx"]),cy=static_cast<double>(settings["Camera1.cy"]);
        if(std::abs(fx-intr.fx)>0.5 || std::abs(fy-intr.fy)>0.5 || std::abs(cx-intr.ppx)>0.5 || std::abs(cy-intr.ppy)>0.5)
            throw std::runtime_error("camera intrinsics differ from supplied map settings (>0.5 pixels)");
        if(bag.empty()) {
            if(intr.model!=RS2_DISTORTION_NONE && intr.model!=RS2_DISTORTION_BROWN_CONRADY &&
               intr.model!=RS2_DISTORTION_MODIFIED_BROWN_CONRADY && intr.model!=RS2_DISTORTION_INVERSE_BROWN_CONRADY)
                throw std::runtime_error("camera distortion model is unsupported by these PinHole settings");
            const char* keys[]={"Camera1.k1","Camera1.k2","Camera1.p1","Camera1.p2","Camera1.k3"};
            double map_coeffs[5];
            for(int i=0;i<5;++i) {
                const double coefficient=settings[keys[i]].empty()?0.0:static_cast<double>(settings[keys[i]]);
                map_coeffs[i]=coefficient;
                if(std::abs(coefficient-intr.coeffs[i])>1e-4)
                    throw std::runtime_error("camera distortion differs from supplied map settings");
            }
            // D455 firmware can label color calibration inverse Brown-Conrady.
            // Compare the SDK's actual projection against the mapping camera's
            // OpenCV Brown model; do not reinterpret coefficients by label alone.
            for(int gy=0;gy<=12;++gy) for(int gx=0;gx<=16;++gx) {
                const double x=(gx*(width-1)/16.0-cx)/fx,y=(gy*(height-1)/12.0-cy)/fy;
                float point[]={static_cast<float>(x),static_cast<float>(y),1},pixel[2];
                rs2_project_point_to_pixel(pixel,&intr,point);
                const double r2=x*x+y*y;
                const double radial=1+map_coeffs[0]*r2+map_coeffs[1]*r2*r2+map_coeffs[4]*r2*r2*r2;
                const double u=fx*(x*radial+2*map_coeffs[2]*x*y+map_coeffs[3]*(r2+2*x*x))+cx;
                const double v=fy*(y*radial+2*map_coeffs[3]*x*y+map_coeffs[2]*(r2+2*y*y))+cy;
                if(!std::isfinite(pixel[0]) || !std::isfinite(pixel[1]) || std::hypot(u-pixel[0],v-pixel[1])>0.5)
                    throw std::runtime_error("SDK distortion projection is incompatible with the mapping PinHole model (>0.5 pixels)");
            }
        }
        const double unit=device.first<rs2::depth_sensor>().get_depth_scale();
        if(std::abs(unit*factor-1.0)>0.001) throw std::runtime_error("depth unit disagrees with RGBD.DepthMapFactor");
        // Subscribe to the two sensors directly. The pipeline syncer can reuse
        // or discard frames before an application callback sees their counters.
        // This path audits each raw stream and constructs each pair exactly once.
        struct Raw { rs2::video_frame frame{rs2::frame{}}; int64_t arrival_ns=0,arrival_steady=0; };
        std::deque<Raw> rgb_raw,depth_raw;
        std::mutex raw_mutex; std::condition_variable raw_ready;
        uint64_t last_rgb=0,last_depth=0;
        bool stop_latched=false;
        uint64_t stop_after=0;
        // Called with raw_mutex held. Complete the other half of an already
        // received pair when SIGINT falls between the two sensor callbacks.
        auto latch_stop=[&] {
            if(interrupted && !stop_latched) {
                std::lock_guard<std::mutex> qlock(queue.mutex);
                stop_after=std::max(queue.raw_rgb_received,queue.raw_depth_received);
                stop_latched=true;
            }
        };
        std::atomic<int64_t> last_arrival{steadyNs()};
        std::atomic<int> count{0};
        const size_t raw_capacity=std::max<size_t>(16,queue.capacity);
        auto callback=[&](rs2::frame frame) {
            if(queue.failed() || (max_frames>0 && count>=max_frames)) return;
            try {
                auto video=frame.as<rs2::video_frame>();
                if(!video) throw std::runtime_error("non-video frame in RGB-D input");
                Raw raw{video,wallNs(),steadyNs()};
                const bool color=video.get_profile().stream_type()==RS2_STREAM_COLOR;
                if(!color && video.get_profile().stream_type()!=RS2_STREAM_DEPTH)
                    throw std::runtime_error("unexpected camera stream");
                const int bytes=color?3:2;
                if(video.get_width()!=width || video.get_height()!=height || video.get_bytes_per_pixel()!=bytes ||
                   video.get_stride_in_bytes()<width*bytes ||
                   video.get_data_size()<static_cast<size_t>(video.get_stride_in_bytes())*height)
                    throw std::runtime_error("incomplete or mismatched raw image buffer");
                std::lock_guard<std::mutex> lock(raw_mutex);
                latch_stop();
                bool have_previous=false;
                {
                    std::lock_guard<std::mutex> qlock(queue.mutex);
                    auto& received=color?queue.raw_rgb_received:queue.raw_depth_received;
                    if(max_frames>0 && received>=static_cast<uint64_t>(max_frames)) return;
                    if(stop_latched && received>=stop_after) return;
                    have_previous=received>0; // Counter zero is a valid first frame.
                    ++received;
                }
                auto& last=color?last_rgb:last_depth;
                const uint64_t number=video.get_frame_number();
                if(have_previous && number!=last+1) {
                    {
                        std::lock_guard<std::mutex> qlock(queue.mutex);
                        if(number>last+1) (color?queue.rgb_gaps:queue.depth_gaps)+=number-last-1;
                    }
                    throw std::runtime_error("raw camera counter gap, duplicate, or out-of-order frame");
                }
                last=number; last_arrival=raw.arrival_steady;
                auto& frames=color?rgb_raw:depth_raw;
                if(frames.size()>=raw_capacity) throw std::runtime_error("raw camera FIFO capacity exceeded");
                // keep() removes the frame from the SDK's limited published-frame
                // accounting; ownership remains RAII and our FIFOs are bounded.
                raw.frame.keep();
                frames.emplace_back(std::move(raw)); raw_ready.notify_one();
            } catch(const std::exception& e) { queue.fail(e.what()); raw_ready.notify_all(); }
        };
        rs2::align align(RS2_STREAM_COLOR);
        rs2::frame pairing_color;
        rs2::frameset composite;
        rs2::processing_block combine([&](rs2::frame depth,rs2::frame_source& source) {
            source.frame_ready(source.allocate_composite_frame({depth,pairing_color}));
        });
        combine.start([&](rs2::frame frame) { composite=frame.as<rs2::frameset>(); });
        std::vector<rs2::sensor> opened,started;
        auto stop_sensors=[&] {
            for(auto& sensor:started) try { sensor.stop(); } catch(const std::exception& e) { queue.fail(e.what()); }
            started.clear();
            for(auto& sensor:opened) try { sensor.close(); } catch(const std::exception& e) { queue.fail(e.what()); }
            opened.clear();
        };
        try {
            const auto desired=resolved.get_streams();
            for(auto sensor:device.query_sensors()) {
                std::vector<rs2::stream_profile> selection;
                for(auto profile:sensor.get_stream_profiles()) for(auto selected:desired)
                    if(profile.unique_id()==selected.unique_id() && profile.format()==selected.format() &&
                       profile.fps()==selected.fps() && profile.as<rs2::video_stream_profile>().width()==width &&
                       profile.as<rs2::video_stream_profile>().height()==height) {
                        selection.push_back(profile); break;
                    }
                if(!selection.empty()) { sensor.open(selection); opened.push_back(sensor); }
            }
            if(opened.empty()) throw std::runtime_error("unable to open the selected RGB-D sensors");
            for(auto sensor:opened) { sensor.start(callback); started.push_back(sensor); }
            double last_stamp=-1,last_depth_stamp=-1;
            while(true) {
                Raw color,depth;
                {
                    std::unique_lock<std::mutex> lock(raw_mutex);
                    latch_stop();
                    raw_ready.wait_for(lock,std::chrono::milliseconds(10),[&] {return !rgb_raw.empty() && !depth_raw.empty();});
                    if(max_frames>0 && count>=max_frames) break;
                    if(rgb_raw.empty() || depth_raw.empty()) {
                        if(queue.failed()) break;
                        if(stop_latched) {
                            std::lock_guard<std::mutex> qlock(queue.mutex);
                            if(queue.raw_rgb_received==stop_after && queue.raw_depth_received==stop_after) break;
                        }
                        if(!bag.empty() && device.as<rs2::playback>().current_status()==RS2_PLAYBACK_STATUS_STOPPED) break;
                        if(steadyNs()-last_arrival>5000000000LL) { queue.fail("camera input timeout (5 seconds)"); break; }
                        continue;
                    }
                    color=std::move(rgb_raw.front()); rgb_raw.pop_front();
                    depth=std::move(depth_raw.front()); depth_raw.pop_front();
                }
                Pair pair;
                pair.enqueue_ns=color.arrival_ns; pair.enqueue_steady=color.arrival_steady;
                pair.rgb_number=color.frame.get_frame_number(); pair.depth_number=depth.frame.get_frame_number();
                pair.stamp=color.frame.get_timestamp()*0.001; pair.depth_stamp=depth.frame.get_timestamp()*0.001;
                if(!std::isfinite(pair.stamp) || !std::isfinite(pair.depth_stamp) ||
                   color.frame.get_frame_timestamp_domain()!=depth.frame.get_frame_timestamp_domain() ||
                   std::abs(pair.stamp-pair.depth_stamp)*1000>max_skew_ms ||
                   pair.stamp<=last_stamp || pair.depth_stamp<=last_depth_stamp)
                    throw std::runtime_error("RGB/depth timestamp mismatch or clock reversal");
                last_stamp=pair.stamp;
                last_depth_stamp=pair.depth_stamp;
                if(color.frame.get_frame_timestamp_domain()!=RS2_TIMESTAMP_DOMAIN_GLOBAL_TIME)
                    throw std::runtime_error("camera global timestamps unavailable (capture clock not synchronized)");
                pair.capture_ns=static_cast<int64_t>(pair.stamp*1e9);
                pair.pair_ready_steady=steadyNs();
                pairing_color=color.frame; combine.invoke(depth.frame);
                auto aligned=align.process(composite).as<rs2::frameset>();
                auto ac=aligned.get_color_frame(); auto ad=aligned.get_depth_frame();
                pair.rgb=cv::Mat(height,width,CV_8UC3,const_cast<void*>(ac.get_data()),ac.get_stride_in_bytes()).clone();
                pair.depth=cv::Mat(height,width,CV_16UC1,const_cast<void*>(ad.get_data()),ad.get_stride_in_bytes()).clone();
                pairing_color=rs2::frame(); composite=rs2::frameset();
                pair.aligned_steady=steadyNs();
                if(!queue.push(std::move(pair))) break;
                ++count;
            }
            stop_sensors();
            if((max_frames<=0 || count<max_frames) && (!rgb_raw.empty() || !depth_raw.empty()))
                queue.fail("unpaired raw RGB/depth frames at end of capture");
        } catch(...) { stop_sensors(); throw; }
    } catch(const std::exception& e) { queue.fail(e.what()); }
    queue.finish();
}
int main(int argc,char** argv) {
    const auto startup_begin=Clock::now();
    rclcpp::init(argc,argv,rclcpp::InitOptions(),rclcpp::SignalHandlerOptions::None);
    std::signal(SIGINT,onSignal); std::signal(SIGTERM,onSignal);
    auto node=std::make_shared<rclcpp::Node>("orb3_live_publish");
    const auto settings=node->declare_parameter<std::string>("settings","");
    const auto vocabulary=node->declare_parameter<std::string>("vocabulary","");
    const auto output=node->declare_parameter<std::string>("output","");
    const auto dataset=node->declare_parameter<std::string>("dataset","");
    const auto bag=node->declare_parameter<std::string>("realsense_bag","");
    const auto serial=node->declare_parameter<std::string>("serial","");
    const auto map_id=node->declare_parameter<std::string>("map_id","");
    const auto session_id=node->declare_parameter<std::string>("session_id","");
    const auto frame_id=node->declare_parameter<std::string>("frame_id","map");
    const auto camera_frame=node->declare_parameter<std::string>("camera_frame","slam_camera_color_optical_frame");
    const bool mapping=node->declare_parameter<bool>("benchmark_mapping",false);
    const int max_frames=node->declare_parameter<int>("max_frames",0);
    const double rate=node->declare_parameter<double>("replay_rate",1.0);
    const double exposure=node->declare_parameter<double>("color_exposure",0.0);
    const double skew=node->declare_parameter<double>("max_pair_skew_ms",16.7);
    const double max_backlog_ms=node->declare_parameter<double>("max_backlog_ms",250.0);
    const int threads=node->declare_parameter<int>("opencv_threads",1);
    InputQueue queue; queue.capacity=node->declare_parameter<int>("queue_capacity",120);
    if(settings.empty() || vocabulary.empty() || output.empty() || queue.capacity==0 || queue.capacity>4096 || threads<1 || (mapping && dataset.empty())) {
        std::cerr<<"Invalid node parameters; use run_orb3_publish.py\n"; rclcpp::shutdown(); return 2;
    }
    cv::setNumThreads(threads);
    auto qos=rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
    auto pose_pub=node->create_publisher<geometry_msgs::msg::PoseStamped>("/orbslam3/pose",qos);
    auto state_pub=node->create_publisher<distributed_slam_interfaces::msg::State>("/distributed_slam/state",qos);
    auto session_pub=node->create_publisher<distributed_slam_interfaces::msg::Session>("/distributed_slam/session",rclcpp::QoS(1).reliable().transient_local());
    std::unique_ptr<ORB_SLAM3::System> slam;
    try {
        slam.reset(new ORB_SLAM3::System(vocabulary,settings,ORB_SLAM3::System::RGBD,false));
        if(!mapping) {
            if(!slam->HasLoadedAtlas() || slam->GetAtlasKeyFrameCount()==0)
                throw std::runtime_error("Atlas has no usable keyframes; refusing to open camera");
            slam->ActivateLocalizationMode();
        }
    } catch(const std::exception& e) {
        std::cerr<<"Startup failed: "<<e.what()<<std::endl;
        if(slam) slam->Shutdown();
        rclcpp::shutdown(); return 2;
    }
    distributed_slam_interfaces::msg::Session session;
    session.map_id=map_id; session.session_id=session_id; session.serial_a=serial;
    session.frame_a=camera_frame; session.mode=mapping ? "mapping" : "localization";
    session_pub->publish(session);
    std::ofstream csv(output+"/frames.csv"),trajectory(output+"/trajectory.tum");
    if(!csv || !trajectory) { slam->Shutdown(); rclcpp::shutdown(); return 2; }
    csv<<"sequence,rgb_number,depth_number,capture_ns,depth_stamp,enqueue_ns,track_start_ns,track_end_ns,publish_ns,state,tracking,features,map_matches,queue_ms,track_ms,enqueue_to_publish_ms,tx,ty,tz,qx,qy,qz,qw,track_cpu_ms,cpu_start,cpu_end,pair_wait_ms,align_copy_ms,tracking_fifo_ms\n";
    csv<<std::setprecision(17); trajectory<<std::fixed<<std::setprecision(9);
    FrameWriter journal(csv,trajectory,queue);
    const double startup_seconds=std::chrono::duration<double>(Clock::now()-startup_begin).count();
    auto producer=std::thread([&]{
        if(interrupted) queue.finish();
        else if(dataset.empty()) readCamera(serial,settings,bag,rate>0,max_frames,exposure,skew,queue);
        else readDataset(dataset,rate,max_frames,queue);
    });
    uint64_t processed=0,valid=0,epoch=0,exceeded=0; bool previously_valid=false;
    std::vector<double> track_times,end_to_end,normal_latencies;
    Pair pair; const auto run_start=Clock::now(); int64_t last_report=steadyNs();
    try {
        while(queue.pop(pair)) {
            const int64_t begin=wallNs(),begin_steady=steadyNs();
            if((begin_steady-pair.enqueue_steady)*1e-6>max_backlog_ms)
                queue.fail("pending frame exceeded the configured backlog latency limit");
            const int cpu_start=sched_getcpu(); const int64_t cpu_begin=threadCpuNs();
            auto tcw=slam->TrackRGBD(pair.rgb,pair.depth,pair.stamp);
            const int64_t cpu_end=threadCpuNs(); const int cpu_finish=sched_getcpu();
            const int64_t end=wallNs(),end_steady=steadyNs();
            const int state=slam->GetTrackingState();
            auto twc=tcw.inverse();
            const bool tracking=state==2 && twc.matrix().allFinite();
            if(tracking!=previously_valid) { ++epoch; previously_valid=tracking; }
            auto points=slam->GetTrackedMapPoints();
            int matches=0; for(auto* point:points) if(point && !point->isBad() && point->Observations()>0) ++matches;
            geometry_msgs::msg::PoseStamped pose;
            pose.header.stamp=rclcpp::Time(pair.capture_ns); pose.header.frame_id=frame_id;
            if(tracking) {
                auto t=twc.translation(); auto q=twc.unit_quaternion();
                pose.pose.position.x=t.x(); pose.pose.position.y=t.y(); pose.pose.position.z=t.z();
                pose.pose.orientation.x=q.x(); pose.pose.orientation.y=q.y(); pose.pose.orientation.z=q.z(); pose.pose.orientation.w=q.w();
            }
            distributed_slam_interfaces::msg::State status;
            status.header=pose.header; status.sequence=pair.sequence; status.map_id=map_id; status.session_id=session_id;
            status.tracking_epoch=epoch; status.tracking=tracking; status.pose=pose.pose;
            status.enqueue_ns=pair.enqueue_ns; status.track_start_ns=begin; status.track_end_ns=end;
            {
                std::lock_guard<std::mutex> lock(queue.mutex);
                status.dropped_inputs=queue.rejected+std::max(queue.rgb_gaps,queue.depth_gaps);
                if(!queue.camera_serial.empty() && session.serial_a!=queue.camera_serial) {
                    session.serial_a=queue.camera_serial; session_pub->publish(session);
                }
            }
            status.publish_ns=wallNs();
            state_pub->publish(status);
            if(tracking) pose_pub->publish(pose);
            const double total=(steadyNs()-pair.enqueue_steady)*1e-6, track=(end_steady-begin_steady)*1e-6;
            ++processed; track_times.push_back(track); end_to_end.push_back(total);
            if(tracking) { ++valid; normal_latencies.push_back(total); if(total>33) ++exceeded; }
            const auto& p=pose.pose.position; const auto& q=pose.pose.orientation;
            std::ostringstream row,pose_row;
            row<<std::setprecision(17); pose_row<<std::fixed<<std::setprecision(9);
            row<<pair.sequence<<','<<pair.rgb_number<<','<<pair.depth_number<<','<<pair.capture_ns<<','<<pair.depth_stamp<<','<<pair.enqueue_ns<<','<<begin<<','<<end<<','<<status.publish_ns<<','<<state<<','<<tracking<<','<<points.size()<<','<<matches<<','<<(begin_steady-pair.enqueue_steady)*1e-6<<','<<track<<','<<total<<','<<p.x<<','<<p.y<<','<<p.z<<','<<q.x<<','<<q.y<<','<<q.z<<','<<q.w<<','<<(cpu_end-cpu_begin)*1e-6<<','<<cpu_start<<','<<cpu_finish<<','<<(pair.pair_ready_steady-pair.enqueue_steady)*1e-6<<','<<(pair.aligned_steady-pair.pair_ready_steady)*1e-6<<','<<(begin_steady-pair.aligned_steady)*1e-6<<'\n';
            if(tracking) pose_row<<pair.stamp<<' '<<p.x<<' '<<p.y<<' '<<p.z<<' '<<q.x<<' '<<q.y<<' '<<q.z<<' '<<q.w<<'\n';
            journal.append(row.str(),pose_row.str());
            if(steadyNs()-last_report>2000000000LL) {
                std::lock_guard<std::mutex> lock(queue.mutex);
                RCLCPP_INFO(node->get_logger(),"frames=%lu valid=%lu FIFO=%zu track=%.2f ms total=%.2f ms",processed,valid,queue.pairs.size(),track,total);
                last_report=steadyNs();
            }
        }
    } catch(const std::exception& e) { queue.fail(e.what()); }
    // Stop acquisition on exceptions and join before destroying the SDK or SLAM.
    interrupted=true;
    producer.join();
    const double elapsed=std::chrono::duration<double>(Clock::now()-run_start).count();
    // A control event must not overwrite the final frame in KeepLast(1).
    // Reliable session metadata conveys shutdown; frame-state sequence stays 1..N.
    session.mode="stopped";
    session_pub->publish(session);
    const auto shutdown_begin=Clock::now();
    slam->Shutdown();
    const double shutdown_seconds=std::chrono::duration<double>(Clock::now()-shutdown_begin).count();
    if(mapping) slam->SaveKeyFrameTrajectoryTUM(output+"/keyframes.tum");
    journal.finish();
    csv.close(); trajectory.close();
    rusage resource_usage{}; getrusage(RUSAGE_SELF,&resource_usage);
    std::ofstream summary(output+"/native_summary.json");
    // Error strings originate from SDK/files; escape through a minimal JSON writer.
    const std::string escaped=jsonEscape(queue.error);
    const bool raw_complete=!dataset.empty() || (queue.raw_rgb_received==processed && queue.raw_depth_received==processed);
    const bool complete=raw_complete && queue.error.empty() && processed==queue.accepted && queue.rejected==0 && queue.rgb_gaps==0 && queue.depth_gaps==0 && processed>0;
    summary<<std::setprecision(10)<<"{\n  \"source\": \""<<(dataset.empty()?(bag.empty()?"camera":"sdk_replay"):"dataset")<<"\",\n"
      <<"  \"peak_rss_kb\": "<<resource_usage.ru_maxrss<<",\n"
      <<"  \"camera_serial\": \""<<jsonEscape(queue.camera_serial)<<"\",\n"
      <<"  \"startup_seconds\": "<<startup_seconds<<", \"slam_shutdown_seconds\": "<<shutdown_seconds<<",\n"
      <<"  \"raw_rgb_received\": "<<queue.raw_rgb_received<<", \"raw_depth_received\": "<<queue.raw_depth_received<<",\n"
      <<"  \"received_pairs\": "<<queue.received<<", \"accepted_pairs\": "<<queue.accepted<<", \"processed_frames\": "<<processed<<",\n"
      <<"  \"valid_poses\": "<<valid<<", \"tracking_availability\": "<<(processed?double(valid)/processed:0)<<",\n"
      <<"  \"rejected_pairs\": "<<queue.rejected<<", \"rgb_gaps\": "<<queue.rgb_gaps<<", \"depth_gaps\": "<<queue.depth_gaps<<",\n"
      <<"  \"queue_high_water\": "<<queue.high_water<<", \"elapsed_seconds\": "<<elapsed<<",\n"
      <<"  \"track_p50_ms\": "<<quantile(track_times,.5)<<", \"track_p99_ms\": "<<quantile(track_times,.99)<<",\n"
      <<"  \"all_enqueue_to_publish_p99_ms\": "<<quantile(end_to_end,.99)<<",\n"
      <<"  \"valid_enqueue_to_publish_p99_ms\": "<<quantile(normal_latencies,.99)<<",\n"
      <<"  \"valid_enqueue_to_publish_max_ms\": "<<quantile(normal_latencies,1)<<", \"valid_over_33ms\": "<<exceeded<<",\n"
      <<"  \"complete_input_processing\": "<<(complete?"true":"false")<<",\n"
      <<"  \"lan_latency_verified\": false, \"error\": \""<<escaped<<"\"\n}\n";
    summary.close();
    rclcpp::shutdown();
    if(!complete) std::cerr<<"Frame contract failed: "<<queue.error<<std::endl;
    return complete ? 0 : 3;
}
