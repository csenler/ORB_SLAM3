#include "AuxiliaryFrameDatabase.h"

namespace ORB_SLAM3
{
    AuxiliaryFrameDatabase::AuxiliaryFrameDatabase(const ORBVocabulary &voc)
    {
        pVoc = &voc;
        vInvertedFile.resize(voc.size());
    }

    void AuxiliaryFrameDatabase::SetORBVocabulary(ORBVocabulary *pORBVoc)
    {
        pVoc = pORBVoc;
        vInvertedFile.clear();
        vInvertedFile.resize(pVoc->size());
    }

    void AuxiliaryFrameDatabase::add(AuxiliaryFrame *pAuxFrame)
    {
        const auto pFrame = pAuxFrame->GetFrame();

        if (pFrame->mBowVec.empty())
        {
            pVoc->transform(pFrame->mDescriptors, pFrame->mBowVec, pFrame->mFeatVec, 4);
        }
        for (auto vit = pFrame->mBowVec.begin(), vend = pFrame->mBowVec.end(); vit != vend; vit++)
        {
            vInvertedFile[vit->first].push_back(pAuxFrame);
        }

        // truncate if needed
        truncateDatabase();
    }

    void AuxiliaryFrameDatabase::erase(AuxiliaryFrame *pAuxFrame)
    {
        const auto pFrame = pAuxFrame->GetFrame();

        for (auto vit = pFrame->mBowVec.begin(), vend = pFrame->mBowVec.end(); vit != vend; vit++)
        {
            list<AuxiliaryFrame *> &l = vInvertedFile[vit->first];
            for (list<AuxiliaryFrame *>::iterator lit = l.begin(), lend = l.end(); lit != lend; lit++)
            {
                if (*lit == pAuxFrame)
                {
                    l.erase(lit);
                    break;
                }
            }
        }
    }

    void AuxiliaryFrameDatabase::truncateDatabase() // TODO: this needs to be tested for memory leaks
    {
        // if database size is bigger than a threshold value, then truncate the database by removing oldest elements
        if (vInvertedFile.size() > AUX_DB_CAPACITY)
        {
            vInvertedFile.resize(AUX_DB_CAPACITY);
        }
    }

    void AuxiliaryFrameDatabase::clear()
    {
        vInvertedFile.clear();
        vInvertedFile.resize(pVoc->size());
    }

    std::vector<AuxiliaryFrame *> AuxiliaryFrameDatabase::DetectCandidates(Frame *pF)
    {
        list<AuxiliaryFrame *> lFramesSharingWords;
        // find all frames sharing words with the query
        for (auto vit = pF->mBowVec.begin(), vend = pF->mBowVec.end(); vit != vend; vit++)
        {
            list<AuxiliaryFrame *> &l = vInvertedFile[vit->first];
            for (const auto auxFrame : l)
            {
                if (auxFrame->mnRelocQuery != pF->mnId)
                {
                    auxFrame->mnRelocWords = 0;
                    auxFrame->mnRelocQuery = pF->mnId;
                    lFramesSharingWords.push_back(auxFrame);
                }
                auxFrame->mnRelocWords++;
            }
        }

        if (lFramesSharingWords.empty())
        {
            return std::vector<AuxiliaryFrame *>();
        }

        // Only compare against those keyframes that share enough words
        int maxCommonWords = 0;
        for (const auto auxFrame : lFramesSharingWords)
        {
            if (auxFrame->mnRelocWords > maxCommonWords)
            {
                maxCommonWords = auxFrame->mnRelocWords;
            }
        }

        int minCommonWords = maxCommonWords * 0.8f;

        list<pair<float, AuxiliaryFrame *>> lScoreAndMatch;

        int nscores = 0;

        // Compute similarity and best score
        float bestAccScore = 0;
        for (const auto auxFrame : lFramesSharingWords)
        {
            if (auxFrame->mnRelocWords > minCommonWords)
            {
                nscores++;
                float si = pVoc->score(pF->mBowVec, auxFrame->GetFrame()->mBowVec);
                if (si > bestAccScore)
                {
                    bestAccScore = si;
                }
                lScoreAndMatch.push_back(make_pair(si, auxFrame));
            }
        }

        if (lScoreAndMatch.empty())
            return std::vector<AuxiliaryFrame *>();

        // return those that are within the 80% of the best score
        std::vector<AuxiliaryFrame *> vAuxFrames;
        for (const auto &scoreAndMatch : lScoreAndMatch)
        {
            if (scoreAndMatch.first > 0.8f * bestAccScore)
            {
                vAuxFrames.push_back(scoreAndMatch.second);
            }
        }

        return vAuxFrames;
    }

    std::vector<KeyFrame *> AuxiliaryFrameDatabase::DetectCandidatesViaKFs(Frame *pF)
    {
        // NOTE: this is same as the DetectRelocalizationCandidates in KeyFrameDatabase, except we will use reference KeyFrames of Frames in the auxiliary database
        // in addition, we are assuming KFs are in same map since we should be in localization-only mode when this method is used

        list<KeyFrame *> lKFsSharingWords;

        // Search all keyframes that share a word with current frame
        {
            unique_lock<mutex> lock(mMutex);

            for (DBoW2::BowVector::const_iterator vit = pF->mBowVec.begin(), vend = pF->mBowVec.end(); vit != vend; vit++)
            {
                list<AuxiliaryFrame *> &lAuxFrames = vInvertedFile[vit->first];

                for (const auto auxFrame : lAuxFrames)
                {
                    // get reference KF of the auxiliary frame
                    KeyFrame *pKFi = auxFrame->GetFrame()->mpReferenceKF;

                    if (!pKFi)
                        continue;

                    if (pKFi->mnRelocQuery != pF->mnId)
                    {
                        pKFi->mnRelocWords = 0;
                        pKFi->mnRelocQuery = pF->mnId;
                        lKFsSharingWords.push_back(pKFi);
                    }
                    pKFi->mnRelocWords++;
                }
            }
        }
        if (lKFsSharingWords.empty())
            return std::vector<KeyFrame *>();

        // Only compare against those keyframes that share enough words
        int maxCommonWords = 0;
        for (list<KeyFrame *>::iterator lit = lKFsSharingWords.begin(), lend = lKFsSharingWords.end(); lit != lend; lit++)
        {
            if ((*lit)->mnRelocWords > maxCommonWords)
                maxCommonWords = (*lit)->mnRelocWords;
        }

        int minCommonWords = maxCommonWords * 0.8f;

        list<pair<float, KeyFrame *>> lScoreAndMatch;

        int nscores = 0;

        // Compute similarity score.
        for (list<KeyFrame *>::iterator lit = lKFsSharingWords.begin(), lend = lKFsSharingWords.end(); lit != lend; lit++)
        {
            KeyFrame *pKFi = *lit;

            if (pKFi->mnRelocWords > minCommonWords)
            {
                nscores++;
                float si = pVoc->score(pF->mBowVec, pKFi->mBowVec);
                pKFi->mRelocScore = si;
                lScoreAndMatch.push_back(make_pair(si, pKFi));
            }
        }

        if (lScoreAndMatch.empty())
            return vector<KeyFrame *>();

        list<pair<float, KeyFrame *>> lAccScoreAndMatch;
        float bestAccScore = 0;

        // Lets now accumulate score by covisibility
        for (list<pair<float, KeyFrame *>>::iterator it = lScoreAndMatch.begin(), itend = lScoreAndMatch.end(); it != itend; it++)
        {
            KeyFrame *pKFi = it->second;
            vector<KeyFrame *> vpNeighs = pKFi->GetBestCovisibilityKeyFrames(10);

            float bestScore = it->first;
            float accScore = bestScore;
            KeyFrame *pBestKF = pKFi;
            for (vector<KeyFrame *>::iterator vit = vpNeighs.begin(), vend = vpNeighs.end(); vit != vend; vit++)
            {
                KeyFrame *pKF2 = *vit;
                if (pKF2->mnRelocQuery != pF->mnId)
                    continue;

                accScore += pKF2->mRelocScore;
                if (pKF2->mRelocScore > bestScore)
                {
                    pBestKF = pKF2;
                    bestScore = pKF2->mRelocScore;
                }
            }
            lAccScoreAndMatch.push_back(make_pair(accScore, pBestKF));
            if (accScore > bestAccScore)
                bestAccScore = accScore;
        }

        // Return all those keyframes with a score higher than 0.75*bestScore
        float minScoreToRetain = 0.75f * bestAccScore;
        set<KeyFrame *> spAlreadyAddedKF;
        vector<KeyFrame *> vpRelocCandidates;
        vpRelocCandidates.reserve(lAccScoreAndMatch.size());
        for (list<pair<float, KeyFrame *>>::iterator it = lAccScoreAndMatch.begin(), itend = lAccScoreAndMatch.end(); it != itend; it++)
        {
            const float &si = it->first;
            if (si > minScoreToRetain)
            {
                KeyFrame *pKFi = it->second;

                // NOTE: assuming same map here

                if (!spAlreadyAddedKF.count(pKFi))
                {
                    vpRelocCandidates.push_back(pKFi);
                    spAlreadyAddedKF.insert(pKFi);
                }
            }
        }

        return vpRelocCandidates;
    }
}