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

        void add(const AuxiliaryFrame &pF);
        void clear();
        void SetORBVocabulary(ORBVocabulary *pORBVoc);
        int getTotalFrameSize() const;
        void truncateDatabase();
        DBoW2::BowVector computeAuxiliaryBoW(Frame *pF);

        // TODO:
        // DetectCandidates by iterating through the list of frames in the inverted file
        std::vector<AuxiliaryFrame *> DetectCandidates(Frame *pF);
        // Detect a given number of candidates to save time
        std::vector<AuxiliaryFrame *> DetectNCandidates(Frame *pF, int candidatesNum);
        // Detect Best Candidates to save time
        std::vector<AuxiliaryFrame *> DetectNBestCandidates(Frame *pF, int candidatesNum);
        // DetectCandidates by iterating thrgough reference keyframe of each frame in the inverted file
        std::vector<KeyFrame *> DetectCandidatesViaKFs(Frame *pF);

    protected:
        // Associated vocabulary
        std::shared_ptr<ORBVocabulary> pVoc{nullptr};

        // Inverted file
        std::vector<list<std::shared_ptr<AuxiliaryFrame>>> vInvertedFile;
        // StackBuffer<list<AuxiliaryFrame *>> bufInvertedFile;

        // Mutex
        std::mutex mMutex;

        inline static int AUX_DB_CAPACITY_PER_WORD = 300; // if 30 fps, 10 seconds = 300 frames
        inline static int AUX_DB_CAPACITY_TOTAL = 0;
    };

} // namespace ORB_SLAM

#endif