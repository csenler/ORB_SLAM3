/**
 * This file is part of ORB-SLAM3
 *
 * Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
 * Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
 *
 * ORB-SLAM3 is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even
 * the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with ORB-SLAM3.
 * If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef TRACKING_H
#define TRACKING_H

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "Viewer.h"
#include "FrameDrawer.h"
#include "Atlas.h"
#include "LocalMapping.h"
#include "LoopClosing.h"
#include "Frame.h"
#include "ORBVocabulary.h"
#include "KeyFrameDatabase.h"
#include "ORBextractor.h"
#include "MapDrawer.h"
#include "System.h"
#include "ImuTypes.h"
#include "Settings.h"

#include "GeometricCamera.h"

#include <mutex>
#include <unordered_set>

#include "stack_buffer.h"
#include "AuxiliaryFrameDatabase.h"
namespace ORB_SLAM3
{

    class Viewer;
    class FrameDrawer;
    class Atlas;
    class LocalMapping;
    class LoopClosing;
    class System;
    class Settings;

    class Tracking
    {

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Tracking(System *pSys, ORBVocabulary *pVoc, FrameDrawer *pFrameDrawer, MapDrawer *pMapDrawer, Atlas *pAtlas,
                 KeyFrameDatabase *pKFDB, const string &strSettingPath, const int sensor, Settings *settings, const string &_nameSeq = std::string());

        ~Tracking();

        // Parse the config file
        bool ParseCamParamFile(cv::FileStorage &fSettings);
        bool ParseORBParamFile(cv::FileStorage &fSettings);
        bool ParseIMUParamFile(cv::FileStorage &fSettings);

        // Preprocess the input and call Track(). Extract features and performs stereo matching.
        Sophus::SE3f GrabImageStereo(const cv::Mat &imRectLeft, const cv::Mat &imRectRight, const double &timestamp, string filename);
        Sophus::SE3f GrabImageRGBD(const cv::Mat &imRGB, const cv::Mat &imD, const double &timestamp, string filename);
        Sophus::SE3f GrabImageMonocular(const cv::Mat &im, const double &timestamp, string filename);

        void GrabImuData(const IMU::Point &imuMeasurement);

        void SetLocalMapper(LocalMapping *pLocalMapper);
        void SetLoopClosing(LoopClosing *pLoopClosing);
        void SetViewer(Viewer *pViewer);
        void SetStepByStep(bool bSet);
        bool GetStepByStep();

        // Load new settings
        // The focal lenght should be similar or scale prediction will fail when projecting points
        void ChangeCalibration(const string &strSettingPath);

        // Use this function if you have deactivated local mapping and you only want to localize the camera.
        void InformOnlyTracking(const bool &flag);

        void UpdateFrameIMU(const float s, const IMU::Bias &b, KeyFrame *pCurrentKeyFrame);
        KeyFrame *GetLastKeyFrame()
        {
            return mpLastKeyFrame;
        }

        void CreateMapInAtlas();
        // std::mutex mMutexTracks;

        //--
        void NewDataset();
        int GetNumberDataset();
        int GetMatchesInliers();

        // DEBUG
        void SaveSubTrajectory(string strNameFile_frames, string strNameFile_kf, string strFolder = "");
        void SaveSubTrajectory(string strNameFile_frames, string strNameFile_kf, Map *pMap);

        float GetImageScale();

#ifdef REGISTER_LOOP
        void RequestStop();
        bool isStopped();
        void Release();
        bool stopRequested();
#endif

    public:
        // Tracking states
        enum eTrackingState
        {
            SYSTEM_NOT_READY = -1,
            NO_IMAGES_YET = 0,
            NOT_INITIALIZED = 1,
            OK = 2,
            RECENTLY_LOST = 3,
            LOST = 4,
            OK_KLT = 5
        };

        eTrackingState mState;
        eTrackingState mLastProcessedState;

        // Input sensor
        int mSensor;

        // Current Frame
        Frame mCurrentFrame;
        Frame mLastFrame;

        cv::Mat mImGray;

        // Initialization Variables (Monocular)
        std::vector<int> mvIniLastMatches;
        std::vector<int> mvIniMatches;
        std::vector<cv::Point2f> mvbPrevMatched;
        std::vector<cv::Point3f> mvIniP3D;
        Frame mInitialFrame;

        // Lists used to recover the full camera trajectory at the end of the execution.
        // Basically we store the reference keyframe for each frame and its relative transformation
        list<Sophus::SE3f> mlRelativeFramePoses;
        list<KeyFrame *> mlpReferences;
        list<double> mlFrameTimes;
        list<bool> mlbLost;

        // frames with estimated pose
        int mTrackedFr;
        bool mbStep;

        // True if local mapping is deactivated and we are performing only localization
        bool mbOnlyTracking;

        void Reset(bool bLocMap = false);
        void ResetActiveMap(bool bLocMap = false);

        float mMeanTrack;
        bool mbInitWith3KFs;
        double t0;    // time-stamp of first read frame
        double t0vis; // time-stamp of first inserted keyframe
        double t0IMU; // time-stamp of IMU initialization
        bool mFastInit = false;

        vector<MapPoint *> GetLocalMapMPS();

        bool mbWriteStats;

#ifdef REGISTER_TIMES
        void LocalMapStats2File();
        void TrackStats2File();
        void PrintTimeStats();

        vector<double> vdRectStereo_ms;
        vector<double> vdResizeImage_ms;
        vector<double> vdORBExtract_ms;
        vector<double> vdStereoMatch_ms;
        vector<double> vdIMUInteg_ms;
        vector<double> vdPosePred_ms;
        vector<double> vdLMTrack_ms;
        vector<double> vdNewKF_ms;
        vector<double> vdTrackTotal_ms;
#endif

        void setRelocalizationEntryThreshold(const int &val_);
        void setRelocPnPSolverIterationNum(const int &val_);

        void setORBmatcherMultiplicationFactor(const float factor_)
        {
            std::lock_guard<std::mutex> lock(mMutexORBmatcherFactor);
            iORBmatcherMultiplicationFactor = factor_;
        }

        float getORBmatcherMultiplicationFactor()
        {
            std::lock_guard<std::mutex> lock(mMutexORBmatcherFactor);
            return iORBmatcherMultiplicationFactor;
        }

        // statistics for measuring Track success
        struct TrackStatistics // TODO: update this for new RelocalizationWithExternalBuffer method
        {
            TrackStatistics()
            {
                eCurrentState = SYSTEM_NOT_READY;
                ePreviousState = SYSTEM_NOT_READY;
            }

            void Reset()
            {
                eCurrentState = SYSTEM_NOT_READY;
                ePreviousState = SYSTEM_NOT_READY;
                iNumOfResultantMatches = 0;
                iNumOfWords = 0;
                iNumOfTrackWithMotionModelCalls = 0;
                iNumOfTrackReferenceKeyFrameCalls = 0;
                iNumOfRelocalizationCalls = 0;
                iFrameId = -1;
                iNumOfExtractedORBKeyPoints = 0;
                // iTrackViaMotionMatchResult = 0;
                // iTrackViaReferenceKeyFrameMatchResult = 0;
                bVisualOdomFlag = false;

                vRelocStats.clear();
            }

            eTrackingState eCurrentState, ePreviousState; // will be filled at TrackMonocular after GrabImageMonocular returns
            int iNumOfExtractedORBKeyPoints{0};
            int iNumOfMatchedMapPoints{0};
            int iNumOfVisualOdomPoints{0}; // temporal points that are not matches with map points in map, in localization only mode
            // int iTrackViaMotionMatchResult{0};
            // int iTrackViaReferenceKeyFrameMatchResult{0};
            int iNumOfResultantMatches{0};
            int iNumOfWords{0};
            int iNumOfFeaturesOfVocNodes{0};
            int iNumOfTrackWithMotionModelCalls{0};
            int iNumOfTrackReferenceKeyFrameCalls{0};
            int iNumOfRelocalizationCalls{0};
            int iFrameId{-1};
            bool bVisualOdomFlag{false}; // this is set in TrackWithMotionModel() if there are not enough matches with map points
            bool bVelocityFlag{false};
            int iNumOfKeyFrames{0}; // this may be useful when in mapping(slam) mode
            int iNumOfMapPoints{0}; // this may be useful when in mapping(slam) mode
            bool bIsLoadedMap{false};
            int iCurrentMapID{-1};

            struct RelocStats
            {
                RelocStats()
                {
                    iNumOfKFCandidatesBeforeSearchByBow = 0;
                    iNumOfKFCandidatesAfterSearchByBow = 0;
                    vNumOfBoWMatches.clear();
                    vNumOfInliers.clear();
                    vGoodMatchesAfterPoseOptimization.clear();
                    vAdditionalMatchesAfterPoseOptimization.clear();
                    vNumOfResultantGoodInliers.clear();
                    bRelocSuccess = false;
                }

                void Reset()
                {
                    iNumOfKFCandidatesBeforeSearchByBow = 0;
                    iNumOfKFCandidatesAfterSearchByBow = 0;
                    vNumOfBoWMatches.clear();
                    vNumOfInliers.clear();
                    vGoodMatchesAfterPoseOptimization.clear();
                    vAdditionalMatchesAfterPoseOptimization.clear();
                    vNumOfResultantGoodInliers.clear();
                    bRelocSuccess = false;
                }

                void reserveVectors(const int &size_)
                {
                    // reserve should only allocate in case bigger capacity requested
                    vNumOfBoWMatches.reserve(size_);
                    vNumOfInliers.reserve(size_);
                    vGoodMatchesAfterPoseOptimization.reserve(size_);
                    vAdditionalMatchesAfterPoseOptimization.reserve(size_);
                    vNumOfResultantGoodInliers.reserve(size_);
                }

                void initializeVectors(const int &size_, const int &value_)
                {
                    // initialize vectors with given value
                    vNumOfBoWMatches.assign(size_, value_);
                    vNumOfInliers.assign(size_, value_);
                    vGoodMatchesAfterPoseOptimization.assign(size_, value_);
                    vAdditionalMatchesAfterPoseOptimization.assign(size_, value_);
                    vNumOfResultantGoodInliers.assign(size_, value_);
                }

                int iNumOfKFCandidatesBeforeSearchByBow{0};               // KF candidates before PnP solve
                int iNumOfKFCandidatesAfterSearchByBow{0};                // KF candidates after PnP solve
                std::vector<int> vNumOfBoWMatches;                        // after searchByBoW, for each candidate KF
                std::vector<int> vNumOfInliers;                           // after RANSAC iterations, for each candidate KF that have enough words
                std::vector<int> vGoodMatchesAfterPoseOptimization;       // after pose optimization, for  each candidate KF
                std::vector<int> vAdditionalMatchesAfterPoseOptimization; // after pose optimization, calculated if not enough inliers, for  each candidate KF
                std::vector<int> vNumOfResultantGoodInliers;              // at the end of relocation, for each candidate KF
                bool bRelocSuccess{false};

                // elapsed times
                int iRelocalizaionElapsedTimeMs{0};
                int iRelocWithExtBufElapsedTimeMs{0};
            };

            std::vector<RelocStats> vRelocStats;

        } sTrackStats;

        cv::Mat getCurrentViewerFrame() const;

        void notifyTrackingLoadMode(const bool &bIsLoadMode_)
        {
            bIsLoadMode.store(bIsLoadMode_);

            // auxiliary db probably not initialized yet since Tracker constructor is already called, therefore initialize it here just in case
            if (bIsLoadMode_ && !ptrAuxiliaryFrameStorage && !bDisableAuxiliaryDB.load())
            {
                // orb vocabulary should already be given as constructor argument at this point
                ptrAuxiliaryFrameStorage = std::make_unique<AuxiliaryFrameStorage>(mpORBVocabulary);
            }
        }

        void setDisableAuxiliaryFrameDB(const bool &bDisableAuxiliaryDB_)
        {
            bDisableAuxiliaryDB.store(bDisableAuxiliaryDB_);
        }

    protected:
        // Main tracking function. It is independent of the input sensor.
        void Track();

        // Map initialization for stereo and RGB-D
        void StereoInitialization();

        // Map initialization for monocular
        void MonocularInitialization();
        // void CreateNewMapPoints();
        void CreateInitialMapMonocular();

        void CheckReplacedInLastFrame();
        bool TrackReferenceKeyFrame();
        void UpdateLastFrame();
        bool TrackWithMotionModel();
        bool PredictStateIMU();

        bool Relocalization();
        bool Relocalization2();
        bool RelocalizationViaExternalBuffer();

        void UpdateLocalMap();
        void UpdateLocalPoints();
        void UpdateLocalKeyFrames();

        bool TrackLocalMap();
        void SearchLocalPoints();

        bool NeedNewKeyFrame();
        void CreateNewKeyFrame();

        // Perform preintegration from last frame
        void PreintegrateIMU();

        // Reset IMU biases and compute frame velocity
        void ResetFrameIMU();

        bool mbMapUpdated;

        // Imu preintegration from last frame
        IMU::Preintegrated *mpImuPreintegratedFromLastKF;

        // Queue of IMU measurements between frames
        std::list<IMU::Point> mlQueueImuData;

        // Vector of IMU measurements from previous to current frame (to be filled by PreintegrateIMU)
        std::vector<IMU::Point> mvImuFromLastFrame;
        std::mutex mMutexImuQueue;

        // Imu calibration parameters
        IMU::Calib *mpImuCalib;

        // Last Bias Estimation (at keyframe creation)
        IMU::Bias mLastBias;

        // In case of performing only localization, this flag is true when there are no matches to
        // points in the map. Still tracking will continue if there are enough matches with temporal points.
        // In that case we are doing visual odometry. The system will try to do relocalization to recover
        // "zero-drift" localization to the map.
        bool mbVO;

        // Other Thread Pointers
        LocalMapping *mpLocalMapper;
        LoopClosing *mpLoopClosing;

        // ORB
        ORBextractor *mpORBextractorLeft, *mpORBextractorRight;
        ORBextractor *mpIniORBextractor;

        // BoW
        ORBVocabulary *mpORBVocabulary;
        KeyFrameDatabase *mpKeyFrameDB;

        // Initalization (only for monocular)
        bool mbReadyToInitializate;
        bool mbSetInit;

        // Local Map
        KeyFrame *mpReferenceKF;
        std::vector<KeyFrame *> mvpLocalKeyFrames;
        std::vector<MapPoint *> mvpLocalMapPoints;

        // System
        System *mpSystem;

        // Drawers
        Viewer *mpViewer;
        FrameDrawer *mpFrameDrawer;
        MapDrawer *mpMapDrawer;
        bool bStepByStep;

        // Atlas
        Atlas *mpAtlas;

        // Calibration matrix
        cv::Mat mK;
        Eigen::Matrix3f mK_;
        cv::Mat mDistCoef;
        float mbf;
        float mImageScale;

        float mImuFreq;
        double mImuPer;
        bool mInsertKFsLost;

        // New KeyFrame rules (according to fps)
        int mMinFrames;
        int mMaxFrames;

        int mnFirstImuFrameId;
        int mnFramesToResetIMU;

        // Threshold close/far points
        // Points seen as close by the stereo/RGBD sensor are considered reliable
        // and inserted from just one frame. Far points requiere a match in two keyframes.
        float mThDepth;

        // For RGB-D inputs only. For some datasets (e.g. TUM) the depthmap values are scaled.
        float mDepthMapFactor;

        // Current matches in frame
        int mnMatchesInliers;

        // Last Frame, KeyFrame and Relocalisation Info
        KeyFrame *mpLastKeyFrame;
        unsigned int mnLastKeyFrameId;
        unsigned int mnLastRelocFrameId;
        double mTimeStampLost;
        double time_recently_lost;

        unsigned int mnFirstFrameId;
        unsigned int mnInitialFrameId;
        unsigned int mnLastInitFrameId;

        bool mbCreatedMap;

        // Motion Model
        bool mbVelocity{false};
        Sophus::SE3f mVelocity;

        // Color order (true RGB, false BGR, ignored if grayscale)
        bool mbRGB;

        list<MapPoint *> mlpTemporalPoints;

        // int nMapChangeIndex;

        int mnNumDataset;

        ofstream f_track_stats;

        ofstream f_track_times;
        double mTime_PreIntIMU;
        double mTime_PosePred;
        double mTime_LocalMapTrack;
        double mTime_NewKF_Dec;

        GeometricCamera *mpCamera, *mpCamera2;

        int initID, lastID;

        Sophus::SE3f mTlr;

        void newParameterLoader(Settings *settings);

        Sophus::SE3f getWorldFrameRotation(const int &sensor, const cv::FileStorage &fSettings);
        Sophus::SE3f Tc0w; // world frame rotation matrix

#ifdef REGISTER_LOOP
        bool Stop();

        bool mbStopped;
        bool mbStopRequested;
        bool mbNotStop;
        std::mutex mMutexStop;
#endif

        int iRecentlyLostRelocCounter{0};
        int iRelocEntryThreshold{-1};
        int iRelocPnPSolverIteration{5}; // default 5

        // multiplcation factor for ORBmatchers in TrackMotionModel, TrackReferenceKeyframe and Relocalization methods
        std::mutex mMutexORBmatcherFactor;
        float iORBmatcherMultiplicationFactor{1.0}; // default is 1 so that it does not change anything

        struct AuxiliaryFrameStorage
        {
            AuxiliaryFrameStorage()
            {
                ptrORBVocabulary = nullptr;
            }

            AuxiliaryFrameStorage(ORBVocabulary *ptrVoc)
            {
                ptrORBVocabulary = ptrVoc;
                auxFrameDB.SetORBVocabulary(ptrORBVocabulary);
            }

            void addFrameToStorage(Frame &frame)
            {
                // add to auxiliary storage
                auxFrameDB.add(frame);
            }

            // void addFrameToStorage(Frame *pFrame)
            // {
            //     // create AuxiliaryFrame object and add to storage
            //     AuxiliaryFrame auxFrame(*pFrame);
            //     auxFrameDB.add(&auxFrame);
            // }

            void clearStorage()
            {
                auxFrameDB.clear();
            }

            void setVocabulary(ORBVocabulary *ptrVoc)
            {
                ptrORBVocabulary = ptrVoc;
                auxFrameDB.SetORBVocabulary(ptrORBVocabulary);
            }

            AuxiliaryFrameDatabase *GetAuxFrameDB()
            {
                return &auxFrameDB;
            }

        private:
            // vector to hold latest N tracked frames for relocalization
            // StackBuffer<Frame> vLastNTrackedFrames;

            // BoW
            ORBVocabulary *ptrORBVocabulary;
            AuxiliaryFrameDatabase auxFrameDB;
        };

        std::unique_ptr<AuxiliaryFrameStorage> ptrAuxiliaryFrameStorage{nullptr};

        // flag to indicate if slam_node is running in "load" or "save" mode
        std::atomic<bool> bIsLoadMode{false};

        // auxiliary db diasble flag
        std::atomic<bool> bDisableAuxiliaryDB{false};

    public:
        cv::Mat mImRight;
    };

} // namespace ORB_SLAM

#endif // TRACKING_H
