#ifndef AUXILIARYFRAMEDATABASE_H
#define AUXILIARYFRAMEDATABASE_H

#include <vector>
#include <list>
#include <set>
#include <mutex>

#include "Frame.h"
#include "KeyFrame.h"
#include "ORBVocabulary.h"
#include "AuxiliaryFrame.h"
#include <stack_buffer.h>

namespace ORB_SLAM3
{
    class Frame;
    class KeyFrame;
    class AuxiliaryFrame;

    class AuxiliaryFrameDatabase
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        AuxiliaryFrameDatabase() {}
        AuxiliaryFrameDatabase(const ORBVocabulary &voc);

        void add(AuxiliaryFrame *pF);
        void erase(AuxiliaryFrame *pF);
        void clear();
        void SetORBVocabulary(ORBVocabulary *pORBVoc);

        void truncateDatabase();

        // TODO:
        // DetectCandidates by iterating through the list of frames in the inverted file
        std::vector<AuxiliaryFrame *> DetectCandidates(Frame *pF);
        // DetectCandidates by iterating thrgough reference keyframe of each frame in the inverted file
        std::vector<KeyFrame *> DetectCandidatesViaKFs(Frame *pF);

    protected:
        // Associated vocabulary
        const ORBVocabulary *pVoc;

        // Inverted file
        std::vector<list<AuxiliaryFrame *>> vInvertedFile;
        // StackBuffer<list<AuxiliaryFrame *>> bufInvertedFile;

        // Mutex
        std::mutex mMutex;

        inline static int AUX_DB_CAPACITY = 1000;
    };

} // namespace ORB_SLAM

#endif