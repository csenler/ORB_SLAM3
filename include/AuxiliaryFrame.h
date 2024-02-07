#ifndef AUXILIARYFRAME_H
#define AUXILIARYFRAME_H

#include <vector>
#include <list>
#include <set>
#include <mutex>

#include "Frame.h"

namespace ORB_SLAM3
{
    class Frame;

    class AuxiliaryFrame
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        AuxiliaryFrame() : pFrame(nullptr) {}

        AuxiliaryFrame(const Frame &frame)
        {
            pFrame = std::make_shared<Frame>(frame);
        }

        Frame *GetFrame() const
        {
            return pFrame.get();
        }

        // Variables used by the keyframe database during relocalization
        long unsigned int mnRelocQuery{0};
        int mnRelocWords{0};
        float mRelocScore{0};

    protected:
        // original frame, shared_ptr so that we can get ownership from reference
        std::shared_ptr<Frame> pFrame{nullptr};
    };

} // namespace ORB_SLAM

#endif