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
#include <decay_sampler.h>

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
        void computeAuxiliaryFrameBoW(AuxiliaryFrame &refAuxFrame, int levelsup = 4);
        void computeFrameBoW(Frame &refFrame, int levelsup = 4);
        static AuxiliaryFrame::BoWAndFeatureVecs getAuxiliaryBoW(const Frame &refFrame, uint levelsup);

        // compute similarity score between two frames and decide whether to add the frame to the database
        bool shouldBeAddedToDb(const Frame &refFrame);

        // DetectCandidates by iterating through the list of frames in the inverted file
        std::vector<AuxiliaryFrame *> DetectCandidates(Frame &refFrame);
        // Detect a given number of candidates to save time
        std::vector<AuxiliaryFrame *> DetectNCandidates(Frame &refFrame, int candidatesNum);
        // Detect Best Candidates to save time
        std::vector<AuxiliaryFrame *> DetectNBestCandidates(Frame &refFrame, int candidatesNum); // TODO
        // DetectCandidates by iterating thrgough reference keyframe of each frame in the inverted file
        std::vector<KeyFrame *> DetectCandidatesViaKFs(Frame &refFrame);
        // DetectNCandidates, but will use aux buf frames corresponding to weighted indices
        std::vector<AuxiliaryFrame *> DetectCandidatesViaWeightedIndices(Frame &refFrame, int candidatesNum);

        // // circular list overflow callbcak
        // void circularListOverflowCallback(AuxiliaryFrame *&refAuxFrame); // ref needs to be constant so compiler knows i wont modify it -_-

        static std::vector<int> calculateSamplingIndices(const int outputSize = 30, const int inputSize = 300, const double decayFactor = 0.5)
        {
            return ExponentialDecaySampler<int>::sampleIndices(outputSize, inputSize, decayFactor);
        }

        static std::vector<int> getSamplingIndices()
        {
            return vExpDecaySamplingIndices;
        }

        void setVocTreeLevelsUp(const uint &levelsup);

        static ORBVocabulary *getAuxiliaryVocabulary()
        {
            return AuxiliaryFrameDatabase::pVoc.get();
        }

        static uint getVocTreeLevelsUp()
        {
            return AuxiliaryFrameDatabase::iVocTreeLevelsUp;
        }

        bool isAuxiliaryBoWAndFeatVecsInUse() const
        {
            return bUseAuxiliaryFrameBoW;
        }

    protected:
        // inline static int AUX_DB_CAPACITY_PER_WORD = 300; // if 30 fps, 10 seconds = 300 frames
        inline static unsigned long long AUX_DB_CAPACITY_TOTAL = 300;
        inline static unsigned long long ullTotalFramesInDb = 0;
        inline static int iAuxFrameID = 0;
        // Associated vocabulary
        inline static std::shared_ptr<ORBVocabulary> pVoc = nullptr;

        inline static std::vector<int> vExpDecaySamplingIndices = AuxiliaryFrameDatabase::calculateSamplingIndices(30, AUX_DB_CAPACITY_TOTAL, 0.5);

        // Inverted file
        std::vector<std::list<AuxiliaryFrame *>> vInvertedFile;
        // StackBuffer<list<AuxiliaryFrame *>> bufInvertedFile;

        CircularList<std::shared_ptr<AuxiliaryFrame>> lLastIvertedFileIndices{AUX_DB_CAPACITY_TOTAL};
        // CircularList<AuxiliaryFrame *> lLastIvertedFileIndices{AUX_DB_CAPACITY_TOTAL}; // hold just index not pointer?

        // last frame that has been added to the database
        std::shared_ptr<AuxiliaryFrame> ptrLastFrame{nullptr};

        // Mutex
        std::mutex mMutex;

        // vocabulary tree levelsup
        inline static uint iVocTreeLevelsUp = 4;
        bool bUseAuxiliaryFrameBoW{false};

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