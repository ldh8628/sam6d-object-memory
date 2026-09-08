// Regression: an unsuccessful candidate must yield after the requested slice,
// and a geometrically consistent candidate must still recover its pose.
#include <MLPnPsolver.h>
#include <Optimizer.h>
#include <CameraModels/Pinhole.h>
#include <memory>
#include <algorithm>
#include <cmath>
#include <random>
#include <iostream>
#include <stdexcept>

int main() {
    using namespace ORB_SLAM3;
    Pinhole camera(std::vector<float>{500,500,320,240});
    Frame frame; frame.mpCamera=&camera; frame.mvLevelSigma2={1.0f};
    std::vector<std::unique_ptr<MapPoint>> owned;
    std::vector<MapPoint*> matches;
    std::mt19937 rng(42); std::uniform_real_distribution<float> xy(-1,1),z(3,6);
    for(int i=0;i<40;++i) {
        owned.emplace_back(new MapPoint());
        owned.back()->SetWorldPos(Eigen::Vector3f(xy(rng),xy(rng),z(rng)));
        matches.push_back(owned.back().get());
        frame.mvKeysUn.emplace_back(cv::Point2f(320+200*xy(rng),240+150*xy(rng)),1);
    }
    frame.mvpMapPoints=matches;
    {
        MLPnPsolver solver(frame,matches);
        solver.SetRansacParameters(.99,20,10,6,.5,1e-12);
        for(int call=1;call<=10;++call) {
            bool done=false; std::vector<bool> inliers; int count=0; Eigen::Matrix4f pose;
            if(solver.iterate(1,done,inliers,count,pose))
                throw std::runtime_error("inconsistent correspondences accepted");
            if(done!=(call==10))
                throw std::runtime_error("per-call or total RANSAC iteration budget violated");
        }
    }
    for(size_t i=0;i<matches.size();++i) {
        const auto pixel=camera.project(matches[i]->GetWorldPos());
        frame.mvKeysUn[i].pt=cv::Point2f(pixel.x(),pixel.y());
    }
    {
        MLPnPsolver solver(frame,matches);
        bool done=false; std::vector<bool> inliers; int count=0; Eigen::Matrix4f pose;
        if(!solver.iterate(5,done,inliers,count,pose) || count!=40 ||
           (pose-Eigen::Matrix4f::Identity()).norm()>1e-3)
            throw std::runtime_error("consistent identity-pose recovery failed");
    }
    std::cout<<"PASS: RANSAC yields after one iteration, stops at ten, and recovers a known pose\n";
    frame.N=40;frame.mpCamera2=nullptr;frame.mvInvLevelSigma2={1.0f};frame.mbf=40;
    Frame::fx=Frame::fy=500;Frame::cx=320;Frame::cy=240;
    const auto true_keys=frame.mvKeysUn;
    for(bool stereo:{false,true}) {
        frame.mvKeysUn=true_keys;frame.mvuRight.resize(40);
        for(int i=0;i<40;++i) {
            if(i>=35) frame.mvKeysUn[i].pt+=cv::Point2f(80,-60);
            frame.mvuRight[i]=stereo?frame.mvKeysUn[i].pt.x-frame.mbf/matches[i]->GetWorldPos().z():-1;
        }
        frame.mvbOutlier.assign(40,false);
        frame.SetPose(Sophus::SE3f(Eigen::Quaternionf(Eigen::AngleAxisf(.08f,Eigen::Vector3f::UnitY())),
                                  Eigen::Vector3f(.05f,-.02f,.03f)));
        const int inliers=Optimizer::PoseOptimization(&frame);
        if(inliers!=35 || (frame.GetPose().matrix()-Eigen::Matrix4f::Identity()).norm()>1e-4)
            throw std::runtime_error("pose solver failed known geometry with five outliers");
    }
    std::cout<<"PASS: mono and RGB-D pose recovery reject five known outliers\n";

    // Compare grid queries with an independent exhaustive geometric search.
    // Dense cells and whole-image queries must return more than 64 results.
    Frame query;query.N=1000;query.Nleft=-1;
    Frame::mnMinX=Frame::mnMinY=0;Frame::mnMaxX=640;Frame::mnMaxY=480;
    Frame::mfGridElementWidthInv=Frame::mfGridElementHeightInv=.1f;
    std::uniform_real_distribution<float> pixel_x(20,620),pixel_y(20,460);
    for(size_t i=0;i<1000;++i) {
        cv::KeyPoint key(i<200?cv::Point2f(320+(i%5)*.5f,240+(i%7)*.5f):cv::Point2f(pixel_x(rng),pixel_y(rng)),1);
        key.octave=i%8;query.mvKeysUn.push_back(key);
        query.mGrid[std::lround(key.pt.x*.1f)][std::lround(key.pt.y*.1f)].push_back(i);
    }
    const std::pair<int,int> level_cases[]={{-1,-1},{0,0},{2,4},{5,-1}};
    for(bool right:{false,true}) {
        if(right) {
            query.Nleft=1000;query.Nright=1000;query.N=2000;query.mvKeys=query.mvKeysUn;
            query.mvKeysRight=query.mvKeysUn;
            for(size_t i=0;i<1000;++i) {
                auto& key=query.mvKeysRight[i];key.pt.x+=2;
                query.mGridRight[std::lround(key.pt.x*.1f)][std::lround(key.pt.y*.1f)].push_back(i);
            }
        }
        const auto& keys=right?query.mvKeysRight:query.mvKeysUn;
        for(const auto center:{cv::Point2f(321,241),cv::Point2f(0,0),cv::Point2f(639,479),cv::Point2f(-100,240)})
        for(float radius:{.1f,3.f,10.f,50.f,1000.f})
        for(const auto& levels:level_cases) {
            auto actual=query.GetFeaturesInArea(center.x,center.y,radius,levels.first,levels.second,right);
            std::vector<size_t> expected;
            for(size_t i=0;i<keys.size();++i) {
                const auto& k=keys[i];
                const bool level_ok=(!(levels.first>0 || levels.second>=0)) ||
                    (k.octave>=levels.first && (levels.second<0 || k.octave<=levels.second));
                if(level_ok && std::abs(k.pt.x-center.x)<radius && std::abs(k.pt.y-center.y)<radius)
                    expected.push_back(i);
            }
            std::sort(actual.begin(),actual.end());
            if(actual!=expected) throw std::runtime_error("grid query omitted or added feature indices");
        }
    }
    std::cout<<"PASS: grid queries retain every feature across dense, boundary, level, and right-camera cases\n";
}
