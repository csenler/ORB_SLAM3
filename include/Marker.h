#pragma once
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

// #include <MapPoint.h>

namespace MARKER_NS
{
    struct ArucoMarker
    {
        ArucoMarker() : id(-1)
        {
            corners.reserve(4);
            undistortedCorners.reserve(4);
            cornerKeyPoints.reserve(4);
            undistortedCornerKeyPoints.reserve(4);
            // vCorrespondingMapPoints.resize(4);

            // // fill with nullptr
            // for (int i = 0; i < 4; i++)
            // {
            //     vCorrespondingMapPoints[i] = static_cast<::ORB_SLAM3::MapPoint *>(nullptr);
            // }
        }

        ArucoMarker(const ArucoMarker &marker_)
        {
            corners.reserve(4);
            undistortedCorners.reserve(4);
            cornerKeyPoints.reserve(4);
            undistortedCornerKeyPoints.reserve(4);

            id = marker_.id;
            corners = marker_.corners;
            undistortedCorners = marker_.undistortedCorners;
            cornerKeyPoints = marker_.cornerKeyPoints;
            undistortedCornerKeyPoints = marker_.undistortedCornerKeyPoints;
            // vCorrespondingMapPoints = marker_.vCorrespondingMapPoints;
        }

        int id{-1};
        std::vector<cv::Point2f> corners;
        std::vector<cv::Point2f> undistortedCorners;
        std::vector<cv::KeyPoint> cornerKeyPoints;
        std::vector<cv::KeyPoint> undistortedCornerKeyPoints;
        // std::vector<::ORB_SLAM3::MapPoint *> vCorrespondingMapPoints;
    };

    class ArucoDetector
    {
    public:
        // ArucoDetector();
        // ~ArucoDetector();

        static void detectArucoMarkers(const cv::Mat &image, std::vector<ArucoMarker> &vMarkers);

        static cv::Ptr<cv::aruco::DetectorParameters> mpDetectorParams;
        static cv::Ptr<cv::aruco::Dictionary> mpDictionary;
    };
}; // namespace MARKER_NS