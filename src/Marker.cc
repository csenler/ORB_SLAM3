#include "Marker.h"

namespace MARKER_NS
{
    cv::Ptr<cv::aruco::DetectorParameters> ArucoDetector::mpDetectorParams = cv::aruco::DetectorParameters::create();
    cv::Ptr<cv::aruco::Dictionary> ArucoDetector::mpDictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_250);

    // ArucoDetector::ArucoDetector()
    // {
    //     mpDetectorParams = cv::aruco::DetectorParameters::create();
    //     mpDictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_250);
    // }

    // ArucoDetector::~ArucoDetector()
    // {
    // }

    void ArucoDetector::detectArucoMarkers(const cv::Mat &image, std::vector<ArucoMarker> &vMarkers)
    {
        std::vector<int> vMarkerIds;
        std::vector<std::vector<cv::Point2f>> vMarkerCorners, vRejectedCandidates;
        cv::aruco::detectMarkers(image, mpDictionary, vMarkerCorners, vMarkerIds, mpDetectorParams, vRejectedCandidates);

        // fill the ArucoMarker struct
        if (vMarkerCorners.size() > 0)
        {
            vMarkers.reserve(vMarkerCorners.size());
            for (size_t i = 0; i < vMarkerCorners.size(); i++)
            {
                vMarkers.insert(vMarkers.begin() + i, ArucoMarker());
                vMarkers[i].id = vMarkerIds[i];
                vMarkers[i].corners = vMarkerCorners[i];

                vMarkers[i].cornerKeyPoints.reserve(4);
                for (int j = 0; j < 4; j++)
                {
                    vMarkers[i].cornerKeyPoints.insert(vMarkers[i].cornerKeyPoints.begin() + j, cv::KeyPoint());
                    vMarkers[i].cornerKeyPoints[j].pt = vMarkerCorners[i][j];
                    vMarkers[i].cornerKeyPoints[j].octave = 0;
                    // calculate angle by using a vector from diagonally opposite corner to current corner (angle should point outwards from corners, where corners are clockwise starting with top-left)
                    cv::Point2f v1 = vMarkerCorners[i][j];
                    cv::Point2f v2 = vMarkerCorners[i][(j + 2) % 4];
                    // calculate angle from v2 to v1
                    double dy = v1.y - v2.y;
                    double dx = v1.x - v2.x;
                    // angle should be between 0-360, so add 360 to negative angles
                    vMarkers[i].cornerKeyPoints[j].angle = atan2(dy, dx) * 180 / M_PI;
                    if (vMarkers[i].cornerKeyPoints[j].angle < 0)
                    {
                        vMarkers[i].cornerKeyPoints[j].angle += 360;
                    }
                }
            }
        }
    }

}