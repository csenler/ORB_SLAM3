#include "AuxiliaryFrameDatabase.h"

namespace ORB_SLAM3
{
    AuxiliaryFrameDatabase::AuxiliaryFrameDatabase(const ORBVocabulary &voc)
    {
        pVoc = std::make_shared<ORBVocabulary>(voc);
        vInvertedFile.resize(voc.size());

        AUX_DB_CAPACITY_TOTAL = AUX_DB_CAPACITY_PER_WORD * voc.size();
    }

    void AuxiliaryFrameDatabase::SetORBVocabulary(ORBVocabulary *pORBVoc)
    {
        pVoc = std::make_shared<ORBVocabulary>(*pORBVoc);
        vInvertedFile.clear();
        vInvertedFile.resize(pVoc->size());

        AUX_DB_CAPACITY_TOTAL = AUX_DB_CAPACITY_PER_WORD * pVoc->size();
    }

    int AuxiliaryFrameDatabase::getTotalFrameSize() const
    {
        int nSize = 0;
        for (const auto &l : vInvertedFile)
        {
            nSize += l.size();
        }
        return nSize;
    }

    bool AuxiliaryFrameDatabase::shouldBeAddedToDb(const AuxiliaryFrame &frame)
    {
        if (!ptrLastFrame)
            return true; // initial case

        // compute similarty score between last frame and new frame using vocabulary's score function (returns between [0,1]), if similarity is above a threshold, do not add to DB
        const auto similarityScore = pVoc->score(frame.GetFrame()->mBowVec, ptrLastFrame->GetFrame()->mBowVec);
        std::cout << "AuxiliaryFrameDatabase::shouldBeAddedToDb -> similarity score : " << similarityScore << std::endl;
        if (similarityScore > 0.85f)
        {
            return false;
        }
        else
        {
            return true;
        }
    }

    void AuxiliaryFrameDatabase::add(const AuxiliaryFrame &refFrame)
    {
        const auto pFrame = refFrame.GetFrame();

        if (!pFrame)
        {
            std::cerr << "AuxiliaryFrameDatabase::add -> Frame is nullptr !!!" << std::endl;
            return;
        }

        if (pFrame->mBowVec.empty())
        {
            vector<cv::Mat> vCurrentDesc = Converter::toDescriptorVector(pFrame->mDescriptors);
            pVoc->transform(vCurrentDesc, pFrame->mBowVec, pFrame->mFeatVec, 4);
        }

        if (shouldBeAddedToDb(refFrame)) // TODO: test this
        {
            // save the last frame that has been added to the database
            ptrLastFrame = std::make_shared<AuxiliaryFrame>(refFrame); // need to use std::move here?

            for (auto vit = pFrame->mBowVec.begin(), vend = pFrame->mBowVec.end(); vit != vend; vit++)
            {
                vInvertedFile[vit->first].push_back(ptrLastFrame);

                // check for truncation
                if (vInvertedFile[vit->first].size() > AUX_DB_CAPACITY_PER_WORD)
                {
                    std::cout << "AuxiliaryFrameDatabase::add -> AUX_DB_CAPACITY_PER_WORD is exceeded, truncating the database for word idx : " << vit->first << std::endl;
                    vInvertedFile[vit->first].pop_front();
                }
            }

            // truncate if needed
            // truncateDatabase();
        }
    }

    void AuxiliaryFrameDatabase::truncateDatabase()
    {
        // if database size is bigger than a threshold value, then truncate the database by removing oldest elements
        if (getTotalFrameSize() > AUX_DB_CAPACITY_TOTAL)
        {
            for (auto &l : vInvertedFile)
            {
                while (l.size() > AUX_DB_CAPACITY_PER_WORD)
                {
                    l.pop_front();
                }
            }
        }
    }

    void AuxiliaryFrameDatabase::clear()
    {
        vInvertedFile.clear();
        vInvertedFile.resize(pVoc->size());
    }

    DBoW2::BowVector AuxiliaryFrameDatabase::computeAuxiliaryBoW(Frame *pF)
    {
        DBoW2::BowVector vBow;
        vector<cv::Mat> vCurrentDesc = Converter::toDescriptorVector(pF->mDescriptors);
        pVoc->transform(vCurrentDesc, vBow, pF->mFeatVec, 4);
        return vBow;
    }

    std::vector<AuxiliaryFrame *> AuxiliaryFrameDatabase::DetectCandidates(Frame *pF)
    {
        std::unique_ptr<DBoW2::BowVector> pBowVec = nullptr;
        // TODO: check if vocabulary is same here
        if (pF->mBowVec.empty())
        {
            // first, compurt BoW for auxiliary database (if vocabularies are same, should not be needed, but just in case)
            pBowVec = std::make_unique<DBoW2::BowVector>(computeAuxiliaryBoW(pF));
        }
        else
        {
            pBowVec = std::make_unique<DBoW2::BowVector>(pF->mBowVec);
        }

        list<AuxiliaryFrame *> lFramesSharingWords;
        // find all frames sharing words with the query
        for (auto vit = pBowVec->begin(), vend = pBowVec->end(); vit != vend; vit++)
        {
            for (auto &auxFrame : vInvertedFile[vit->first])
            {
                if (auxFrame->mnRelocQuery != pF->mnId)
                {
                    auxFrame->mnRelocWords = 0;
                    auxFrame->mnRelocQuery = pF->mnId;
                    lFramesSharingWords.push_back(auxFrame.get());
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
                float si = pVoc->score(*(pBowVec.get()), auxFrame->GetFrame()->mBowVec);
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
        vAuxFrames.reserve(lScoreAndMatch.size());
        for (const auto &scoreAndMatch : lScoreAndMatch)
        {
            if (scoreAndMatch.first > 0.8f * bestAccScore)
            {
                vAuxFrames.push_back(scoreAndMatch.second);
            }
        }

        return vAuxFrames;
    }

    // use only candidatesNum number of candidates
    std::vector<AuxiliaryFrame *> AuxiliaryFrameDatabase::DetectNCandidates(Frame *pF, int candidatesNum)
    {
        std::unique_ptr<DBoW2::BowVector> pBowVec = nullptr;
        // TODO: check if vocabulary is same here
        if (pF->mBowVec.empty())
        {
            // first, compurt BoW for auxiliary database (if vocabularies are same, should not be needed, but just in case)
            pBowVec = std::make_unique<DBoW2::BowVector>(computeAuxiliaryBoW(pF));
        }
        else
        {
            pBowVec = std::make_unique<DBoW2::BowVector>(pF->mBowVec);
        }

        list<AuxiliaryFrame *> lFramesSharingWords;
        // find all frames sharing words with the query
        for (auto vit = pBowVec->begin(), vend = pBowVec->end(); vit != vend; vit++)
        {
            // last candidatesNum of inverted file list will be used
            if (vInvertedFile[vit->first].empty())
                continue;

            const auto &tempListRef = vInvertedFile[vit->first];
            int count = 0;
            for (auto rit = tempListRef.rbegin(); rit != tempListRef.rend(); ++rit)
            {
                auto &auxFrame = *rit;
                if (auxFrame) // TODO: without this check, it crashes, figure out real reason, list should not contain nullptr
                {
                    if (auxFrame->mnRelocQuery != pF->mnId)
                    {
                        auxFrame->mnRelocWords = 0;
                        auxFrame->mnRelocQuery = pF->mnId;
                        lFramesSharingWords.push_back(auxFrame.get());
                        count++;

                        if (count == candidatesNum)
                            break;
                    }
                    auxFrame->mnRelocWords++;
                }
            }
            if (count == candidatesNum)
                break;
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
                float si = pVoc->score(*(pBowVec.get()), auxFrame->GetFrame()->mBowVec);
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
        vAuxFrames.reserve(lScoreAndMatch.size());
        for (const auto &scoreAndMatch : lScoreAndMatch)
        {
            if (scoreAndMatch.first > 0.8f * bestAccScore)
            {
                vAuxFrames.push_back(scoreAndMatch.second);
            }
        }

        return vAuxFrames;
    }

    std::vector<AuxiliaryFrame *> AuxiliaryFrameDatabase::DetectNBestCandidates(Frame *pF, int candidatesNum)
    {
        // TODO
    }

    std::vector<KeyFrame *> AuxiliaryFrameDatabase::DetectCandidatesViaKFs(Frame *pF)
    {
        // NOTE: this is same as the DetectRelocalizationCandidates in KeyFrameDatabase, except we will use reference KeyFrames of Frames in the auxiliary database
        // in addition, we are assuming KFs are in same map since we should be in localization-only mode when this method is used
        // also, extra BoW calculation just in case

        // first, compurt BoW for auxiliary database (if vocabularies are same, should not be needed, but just in case)
        auto vBow = computeAuxiliaryBoW(pF);

        list<KeyFrame *> lKFsSharingWords;

        // Search all keyframes that share a word with current frame
        {
            unique_lock<mutex> lock(mMutex);

            for (DBoW2::BowVector::const_iterator vit = vBow.begin(), vend = vBow.end(); vit != vend; vit++)
            {
                for (auto &auxFrame : vInvertedFile[vit->first])
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
                float si = pVoc->score(vBow, pKFi->mBowVec);
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