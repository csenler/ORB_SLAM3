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

        // compute similarity score between two frames and decide whether to add the frame to the database
        bool shouldBeAddedToDb(const AuxiliaryFrame &frame); // TODO

        // DetectCandidates by iterating through the list of frames in the inverted file
        std::vector<AuxiliaryFrame *> DetectCandidates(Frame *pF);
        // Detect a given number of candidates to save time
        std::vector<AuxiliaryFrame *> DetectNCandidates(Frame *pF, int candidatesNum);
        // Detect Best Candidates to save time
        std::vector<AuxiliaryFrame *> DetectNBestCandidates(Frame *pF, int candidatesNum); // TODO
        // DetectCandidates by iterating thrgough reference keyframe of each frame in the inverted file
        std::vector<KeyFrame *> DetectCandidatesViaKFs(Frame *pF);

    protected:
        // Associated vocabulary
        std::shared_ptr<ORBVocabulary> pVoc{nullptr};

        // Inverted file
        std::vector<std::deque<std::shared_ptr<AuxiliaryFrame>>> vInvertedFile;
        // StackBuffer<list<AuxiliaryFrame *>> bufInvertedFile;

        // last frame that has been added to the database
        std::shared_ptr<AuxiliaryFrame> ptrLastFrame{nullptr};

        // Mutex
        std::mutex mMutex;

        inline static int AUX_DB_CAPACITY_PER_WORD = 1000; // if 30 fps, 10 seconds = 300 frames
        inline static unsigned long long AUX_DB_CAPACITY_TOTAL = 0;
    };

} // namespace ORB_SLAM

#endif