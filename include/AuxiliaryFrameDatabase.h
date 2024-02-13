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
#include <circular_list.h>

namespace ORB_SLAM3
{
    class Frame;
    class KeyFrame;
    class AuxiliaryFrame;

    class AuxiliaryFrameDatabase
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        AuxiliaryFrameDatabase()
        {
            // lLastIvertedFileIndices.setOverflowCallback(std::bind(&AuxiliaryFrameDatabase::circularListOverflowCallback, this, std::placeholders::_1));
        }
        AuxiliaryFrameDatabase(const ORBVocabulary &voc);

        void add(Frame &pF);
        void clear();
        void SetORBVocabulary(ORBVocabulary *pORBVoc);
        unsigned long long getTotalFrameSize() const;
        void truncateDatabase();
        DBoW2::BowVector computeAuxiliaryBoW(Frame *pF, int levelsup = 4);

        // compute similarity score between two frames and decide whether to add the frame to the database
        bool shouldBeAddedToDb(const Frame &refFrame);

        // DetectCandidates by iterating through the list of frames in the inverted file
        std::vector<AuxiliaryFrame *> DetectCandidates(Frame *pF);
        // Detect a given number of candidates to save time
        std::vector<AuxiliaryFrame *> DetectNCandidates(Frame *pF, int candidatesNum);
        // Detect Best Candidates to save time
        std::vector<AuxiliaryFrame *> DetectNBestCandidates(Frame *pF, int candidatesNum); // TODO
        // DetectCandidates by iterating thrgough reference keyframe of each frame in the inverted file
        std::vector<KeyFrame *> DetectCandidatesViaKFs(Frame *pF);

        // // circular list overflow callbcak
        // void circularListOverflowCallback(AuxiliaryFrame *&refAuxFrame); // ref needs to be constant so compiler knows i wont modify it -_-

    protected:
        inline static int AUX_DB_CAPACITY_PER_WORD = 300; // if 30 fps, 10 seconds = 300 frames
        inline static unsigned long long AUX_DB_CAPACITY_TOTAL = 1800;
        inline static unsigned long long ullTotalFramesInDb = 0;
        inline static int iAuxFrameID = 0;

        // Associated vocabulary
        std::shared_ptr<ORBVocabulary> pVoc{nullptr};

        // Inverted file
        std::vector<std::list<std::shared_ptr<AuxiliaryFrame>>> vInvertedFile;
        // StackBuffer<list<AuxiliaryFrame *>> bufInvertedFile;

        CircularList<std::shared_ptr<AuxiliaryFrame>> lLastIvertedFileIndices{AUX_DB_CAPACITY_TOTAL};
        // CircularList<AuxiliaryFrame *> lLastIvertedFileIndices{AUX_DB_CAPACITY_TOTAL}; // hold just index not pointer?

        // last frame that has been added to the database
        std::shared_ptr<AuxiliaryFrame> ptrLastFrame{nullptr};

        // Mutex
        std::mutex mMutex;

    private:
        // compare score between auxiliary frames for priority queue
        struct CompareScore
        {
            bool operator()(const std::pair<AuxiliaryFrame *, float> &lhs, const std::pair<AuxiliaryFrame *, float> &rhs) const
            {
                return lhs.second > rhs.second; // greater score first
            }
        };

        AuxiliaryFrame *pFrameToTruncate{nullptr};
    };

} // namespace ORB_SLAM

#endif