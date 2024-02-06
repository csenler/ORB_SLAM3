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
        AuxiliaryFrame(const Frame &frame)
        {
            pFrame = new Frame(frame);
        }

        Frame *GetFrame() const
        {
            return pFrame;
        }

        // Variables used by the keyframe database during relocalization
        long unsigned int mnRelocQuery{0};
        int mnRelocWords{0};
        float mRelocScore{0};

    protected:
        // copied frame
        Frame *pFrame;
    };

} // namespace ORB_SLAM

#endif