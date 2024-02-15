#include "AuxiliaryFrameDatabase.h"

namespace ORB_SLAM3
{
    AuxiliaryFrameDatabase::AuxiliaryFrameDatabase(const ORBVocabulary &voc)
    {
        pVoc = std::make_shared<ORBVocabulary>(voc);
        vInvertedFile.resize(voc.size());

        // AUX_DB_CAPACITY_TOTAL = AUX_DB_CAPACITY_PER_WORD * voc.size();

        // lLastIvertedFileIndices.setOverflowCallback(std::bind(&AuxiliaryFrameDatabase::circularListOverflowCallback, this, std::placeholders::_1));
    }

    void AuxiliaryFrameDatabase::SetORBVocabulary(ORBVocabulary *pORBVoc)
    {
        pVoc = std::make_shared<ORBVocabulary>(*pORBVoc);
        vInvertedFile.clear();
        vInvertedFile.resize(pVoc->size());

        // AUX_DB_CAPACITY_TOTAL = AUX_DB_CAPACITY_PER_WORD * pVoc->size();
    }

    void AuxiliaryFrameDatabase::setVocTreeLevelsUp(const uint &levelsup)
    {
        iVocTreeLevelsUp = levelsup;
        // if changed from default, set a flag so that auxiliary BoW vectors are recalculated
        if (iVocTreeLevelsUp != 4)
        {
            bUseAuxiliaryFrameBoW = true;
        }
    }

    unsigned long long AuxiliaryFrameDatabase::getTotalFrameSize() const
    {
        // int nSize = 0;
        // for (const auto &l : vInvertedFile)
        // {
        //     nSize += l.size();
        // }
        // return nSize;
        return ullTotalFramesInDb;
    }

    bool AuxiliaryFrameDatabase::shouldBeAddedToDb(const Frame &refFrame) // TODO: if iVocTreeLevelsUp is used, should use that levelup for comparison ???
    {
        if (!ptrLastFrame || !ptrLastFrame->isInternalFrameValid())
            return true; // initial case or dont check last frame

        // compute similarty score between last frame and new frame using vocabulary's score function (returns between [0,1]), if similarity is above a threshold, do not add to DB
        const auto similarityScore = pVoc->score(refFrame.mBowVec, ptrLastFrame->GetFrame().mBowVec);
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

    void AuxiliaryFrameDatabase::add(Frame &refFrame)
    {
        if (refFrame.mBowVec.empty())
        {
            vector<cv::Mat> vCurrentDesc = Converter::toDescriptorVector(refFrame.mDescriptors);
            pVoc->transform(vCurrentDesc, refFrame.mBowVec, refFrame.mFeatVec, 4);
        }

        if (shouldBeAddedToDb(refFrame))
        {
            // save the last frame that has been added to the database
            ptrLastFrame = std::make_shared<AuxiliaryFrame>(refFrame); // need to use std::move here?
            ptrLastFrame->iFrameID = iAuxFrameID++;
            std::cout << "AuxiliaryFrameDatabase::add -> owned frame with id : " << refFrame.mnId << std::endl;

            DBoW2::BowVector *ptrBowVec = nullptr;
            ptrBowVec = &ptrLastFrame->GetFrame().mBowVec;

            // calculate 5th level up BoW vector
            if (bUseAuxiliaryFrameBoW)
            {
                computeAuxiliaryFrameBoW(*ptrLastFrame, iVocTreeLevelsUp);
                ptrBowVec = &ptrLastFrame->mBoWAndFeatureVecs.mAuxBowVec;
            }

            for (auto vit = ptrBowVec->begin(), vend = ptrBowVec->end(); vit != vend; vit++) // default 4 level up of voc tree
            {
                vInvertedFile[vit->first].push_back(ptrLastFrame.get());
                ullTotalFramesInDb++;
            }
            std::cout << "AuxiliaryFrameDatabase::add -> truncating db..." << std::endl;
            // truncate first if necessary
            truncateDatabase();
            std::cout << "AuxiliaryFrameDatabase::add -> db truncated, adding to list with aux frame id: " << std::to_string(ptrLastFrame->iFrameID) << " frame mnId: " << std::to_string(ptrLastFrame->GetFrame().mnId) << std::endl;
            // add to list
            lLastIvertedFileIndices.push_back(ptrLastFrame);
        }
        std::cout << "AuxiliaryFrameDatabase::add -> FIN." << std::endl;
    }

    // void AuxiliaryFrameDatabase::circularListOverflowCallback(AuxiliaryFrame *&pAuxFrame) // TODO: fix ref-of-pointer bullshit here
    // {
    //     // traverse vector and remove corresponding entries from inverted file list in order to truncate
    //     for (auto vit = pAuxFrame->GetFrame().mBowVec.begin(), vend = pAuxFrame->GetFrame().mBowVec.end(); vit != vend; vit++)
    //     {
    //         vInvertedFile[vit->first].front().reset();
    //         vInvertedFile[vit->first].pop_front();
    //     }
    // }

    void AuxiliaryFrameDatabase::truncateDatabase() // this is not needed since new container class is circular
    {
        const auto timeRef = std::chrono::high_resolution_clock::now();
        // // if database size is bigger than a threshold value, then truncate the database
        // while (lLastIvertedFileIndices.size() > AUX_DB_CAPACITY_TOTAL)
        // {
        //     vInvertedFile[lLastIvertedFileIndices.front()].front().reset(); // release the memory
        //     vInvertedFile[lLastIvertedFileIndices.front()].pop_front();
        //     lLastIvertedFileIndices.pop_front();
        // }

        // truncate using the saved pFrameToTruncate
        if (lLastIvertedFileIndices.isFull() && !lLastIvertedFileIndices.empty())
        {
            std::cout << "AuxiliaryFrameDatabase::truncateDatabase -> total frames BEFORE truncate: " << std::to_string(ullTotalFramesInDb) << std::endl;
            // auto pFrameToTruncate = lLastIvertedFileIndices.front();
            // std::cout << "AuxiliaryFrameDatabase::truncateDatabase -> front ref received!" << std::endl;

            // NOTE (alternative): first, mark auxFrame for deletion, than remove those with mark from inverted list, than actually release them

            // save aux frame id for later check
            const auto iAuxFrameIdToTruncate = lLastIvertedFileIndices.front()->iFrameID;

            // some pointers should be invalid now that shared_ptr has released the memory, therefore remove invalid pointers from inverted file list
            // use erase-remove idiom
            auto &counterRef = ullTotalFramesInDb;
            for (auto &auxFrameList : vInvertedFile)
            {
                if (!auxFrameList.empty())
                {
                    // this would only remove from container, not release memory
                    auxFrameList.remove_if([&counterRef, &iAuxFrameIdToTruncate](AuxiliaryFrame *pAuxFrame)
                                           { 
                                            if (pAuxFrame == nullptr){
                                                counterRef--;
                                                return true;
                                            }
                                            else if (pAuxFrame->iFrameID == iAuxFrameIdToTruncate)
                                            {
                                                counterRef--;
                                                return true;
                                            }
                                            else
                                                return false; });
                }
            }

            // release shared_ptr and free memory
            std::cout << "AuxiliaryFrameDatabase::truncateDatabase -> popping from list with aux frame id: " << std::to_string(lLastIvertedFileIndices.front()->iFrameID) << " Frame mnId: " << std::to_string(lLastIvertedFileIndices.front()->GetFrame().mnId) << std::endl;
            lLastIvertedFileIndices.front().reset();
            lLastIvertedFileIndices.front() = nullptr;
            lLastIvertedFileIndices.pop_front();

            // if (pFrameToTruncate && pFrameToTruncate->GetFrame())
            // {
            //     std::cout << "AuxiliaryFrameDatabase::truncateDatabase -> entering truncation loop, bow vec size: " << std::to_string(pFrameToTruncate->GetFrame().mBowVec.size()) << std::endl;
            //     for (auto vit = pFrameToTruncate->GetFrame().mBowVec.begin(), vend = pFrameToTruncate->GetFrame().mBowVec.end(); vit != vend; vit++)
            //     {
            //         if (!vInvertedFile[vit->first].empty())
            //         {
            //             vInvertedFile[vit->first].pop_front();
            //             ullTotalFramesInDb--;
            //         }
            //     }
            // }
            // else
            // {
            //     std::cout << "AuxiliaryFrameDatabase::truncateDatabase -> pFrameToTruncate is nullptr!" << std::endl;
            // }
            // // release shared_ptr
            // std::cout << "AuxiliaryFrameDatabase::truncateDatabase -> popping from list with aux frame id: " << std::to_string(pFrameToTruncate->iFrameID) << " Frame mnId: " << std::to_string(pFrameToTruncate->GetFrame().mnId) << std::endl;
            // lLastIvertedFileIndices.front().reset();
            // lLastIvertedFileIndices.front() = nullptr;
            // lLastIvertedFileIndices.pop_front();
            std::cout << "AuxiliaryFrameDatabase::truncateDatabase -> total frames AFTER truncate: " << std::to_string(ullTotalFramesInDb) << std::endl;
        }
        const auto timeNow = std::chrono::high_resolution_clock::now();
        const auto elapsed_time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(timeNow - timeRef).count();
        std::cout << "AuxiliaryFrameDatabase::truncateDatabase -> truncate elapsed time (ms) : " << std::to_string(elapsed_time_ms) << std::endl;
        std::cout << "AuxiliaryFrameDatabase::truncateDatabase -> FIN." << std::endl;
    }

    void AuxiliaryFrameDatabase::clear()
    {
        vInvertedFile.clear();
        vInvertedFile.resize(pVoc->size());
    }

    void AuxiliaryFrameDatabase::computeFrameBoW(Frame &refFrame, int levelsup)
    {
        refFrame.mBowVec.clear(); // just in case
        vector<cv::Mat> vCurrentDesc = Converter::toDescriptorVector(refFrame.mDescriptors);
        pVoc->transform(vCurrentDesc, refFrame.mBowVec, refFrame.mFeatVec, levelsup);
    }

    void AuxiliaryFrameDatabase::computeAuxiliaryFrameBoW(AuxiliaryFrame &refAuxFrame, int levelsup)
    {
        vector<cv::Mat> vCurrentDesc = Converter::toDescriptorVector(refAuxFrame.GetFrame().mDescriptors);
        pVoc->transform(vCurrentDesc, refAuxFrame.mBoWAndFeatureVecs.mAuxBowVec, refAuxFrame.mBoWAndFeatureVecs.mAuxFeatVec, levelsup);
    }

    AuxiliaryFrame::BoWAndFeatureVecs AuxiliaryFrameDatabase::getAuxiliaryBoW(const Frame &refFrame, uint levelsup)
    {
        AuxiliaryFrame::BoWAndFeatureVecs outStruct;
        if (AuxiliaryFrameDatabase::pVoc == nullptr)
        {
            std::cerr << "AuxiliaryFrameDatabase::getAuxiliaryBoW -> Vocabulary is not set!" << std::endl;
            return outStruct;
        }
        vector<cv::Mat> vCurrentDesc = Converter::toDescriptorVector(refFrame.mDescriptors);
        pVoc->transform(vCurrentDesc, outStruct.mAuxBowVec, outStruct.mAuxFeatVec, static_cast<int>(levelsup));
        return outStruct;
    }

    std::vector<AuxiliaryFrame *> AuxiliaryFrameDatabase::DetectCandidates(Frame &refFrame)
    {
        // TODO: check if vocabulary is same here
        if (refFrame.mBowVec.empty())
        {
            // first, compurt BoW for auxiliary database (if vocabularies are same, should not be needed, but just in case)
            computeFrameBoW(refFrame);
        }

        auto ptrBowVec = &refFrame.mBowVec;
        AuxiliaryFrame::BoWAndFeatureVecs auxBoWAndFeatStruct;

        if (bUseAuxiliaryFrameBoW)
        {
            auxBoWAndFeatStruct = getAuxiliaryBoW(refFrame, iVocTreeLevelsUp);
            ptrBowVec = &auxBoWAndFeatStruct.mAuxBowVec;
        }

        list<AuxiliaryFrame *> lFramesSharingWords;
        // find all frames sharing words with the query
        for (auto vit = ptrBowVec->begin(), vend = ptrBowVec->end(); vit != vend; vit++) // assuming BoW vec is not empty
        {
            for (auto &auxFrame : vInvertedFile[vit->first])
            {
                if (auxFrame->mnRelocQuery != refFrame.mnId)
                {
                    auxFrame->mnRelocWords = 0;
                    auxFrame->mnRelocQuery = refFrame.mnId;
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
                float si = 0;
                if (bUseAuxiliaryFrameBoW)
                    si = pVoc->score(*ptrBowVec, auxFrame->mBoWAndFeatureVecs.mAuxBowVec);
                else
                    si = pVoc->score(*ptrBowVec, auxFrame->GetFrame().mBowVec);

                nscores++;
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
    std::vector<AuxiliaryFrame *> AuxiliaryFrameDatabase::DetectNCandidates(Frame &refFrame, int candidatesNum)
    {
        // TODO: check if vocabulary is same here
        if (refFrame.mBowVec.empty())
        {
            // first, compurt BoW for auxiliary database (if vocabularies are same, should not be needed, but just in case)
            computeFrameBoW(refFrame);
        }

        auto ptrBowVec = &refFrame.mBowVec;
        AuxiliaryFrame::BoWAndFeatureVecs auxBoWAndFeatStruct;

        if (bUseAuxiliaryFrameBoW)
        {
            auxBoWAndFeatStruct = getAuxiliaryBoW(refFrame, iVocTreeLevelsUp);
            ptrBowVec = &auxBoWAndFeatStruct.mAuxBowVec;
        }

        list<AuxiliaryFrame *> lFramesSharingWords;
        // find all frames sharing words with the query
        for (auto vit = ptrBowVec->begin(), vend = ptrBowVec->end(); vit != vend; vit++)
        {
            // last candidatesNum of inverted file list will be used
            if (vInvertedFile[vit->first].empty())
                continue;

            const auto &tempListRef = vInvertedFile[vit->first];
            int count = 0;
            for (auto rit = tempListRef.rbegin(); rit != tempListRef.rend(); ++rit)
            {
                const auto ptrAuxBuf = *rit;
                if (ptrAuxBuf && ptrAuxBuf->isInternalFrameValid())
                {
                    if (ptrAuxBuf->mnRelocQuery != refFrame.mnId) // assuming BoW vec is not empty
                    {
                        ptrAuxBuf->mnRelocWords = 0;
                        ptrAuxBuf->mnRelocQuery = refFrame.mnId;
                        lFramesSharingWords.push_back(ptrAuxBuf);
                        count++;
                    }
                    ptrAuxBuf->mnRelocWords++;
                }
                if (count == candidatesNum)
                    break;
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

        int minCommonWords = maxCommonWords * 0.7f;

        list<pair<float, AuxiliaryFrame *>> lScoreAndMatch;

        int nscores = 0;

        // Compute similarity and best score
        float bestAccScore = 0;
        for (const auto auxFrame : lFramesSharingWords)
        {
            if (auxFrame->mnRelocWords > minCommonWords)
            {
                float si = 0;
                if (bUseAuxiliaryFrameBoW)
                    si = pVoc->score(*ptrBowVec, auxFrame->mBoWAndFeatureVecs.mAuxBowVec);
                else
                    si = pVoc->score(*ptrBowVec, auxFrame->GetFrame().mBowVec);

                nscores++;
                if (si > bestAccScore)
                {
                    bestAccScore = si;
                }
                lScoreAndMatch.push_back(make_pair(si, auxFrame));
            }
        }

        if (lScoreAndMatch.empty())
            return std::vector<AuxiliaryFrame *>();

        // return those that are within the N% of the best score
        std::vector<AuxiliaryFrame *> vAuxFrames;
        vAuxFrames.reserve(lScoreAndMatch.size());
        for (const auto &scoreAndMatch : lScoreAndMatch)
        {
            if (scoreAndMatch.first > 0.7f * bestAccScore)
            {
                vAuxFrames.push_back(scoreAndMatch.second);
            }
        }

        return vAuxFrames;
    }

    std::vector<AuxiliaryFrame *> AuxiliaryFrameDatabase::DetectNBestCandidates(Frame &refFrame, int candidatesNum)
    {
        // Same as DetectCandidates, except we will be using a comparator to sort candidates with respect to their scores

        // construct std::set using the comparator
        std::set<std::pair<AuxiliaryFrame *, float>, CompareScore> sortedCandidates;

        // TODO: check if vocabulary is same here
        if (refFrame.mBowVec.empty())
        {
            // first, compurt BoW for auxiliary database (if vocabularies are same, should not be needed, but just in case)
            computeFrameBoW(refFrame);
        }

        auto ptrBowVec = &refFrame.mBowVec;
        AuxiliaryFrame::BoWAndFeatureVecs auxBoWAndFeatStruct;

        if (bUseAuxiliaryFrameBoW)
        {
            auxBoWAndFeatStruct = getAuxiliaryBoW(refFrame, iVocTreeLevelsUp);
            ptrBowVec = &auxBoWAndFeatStruct.mAuxBowVec;
        }

        // std::cout << "AuxiliaryFrameDatabase::DetectNBestCandidates -> lLastIvertedFileIndices size : " << lLastIvertedFileIndices.getSize() << std::endl;

        const int iIterLimit = candidatesNum * 2;
        int iIterCount = 0;

        for (const auto &auxFrameIndex : lLastIvertedFileIndices)
        {
            if (iIterCount == iIterLimit)
                break;

            // compute score for each frame in db and add to priority queue
            if (auxFrameIndex && auxFrameIndex->isInternalFrameValid())
            {
                float score = 0;
                if (bUseAuxiliaryFrameBoW)
                    score = pVoc->score(*ptrBowVec, auxFrameIndex->mBoWAndFeatureVecs.mAuxBowVec);
                else
                    score = pVoc->score(*ptrBowVec, auxFrameIndex->GetFrame().mBowVec);

                // std::cout << "AuxiliaryFrameDatabase::DetectNBestCandidates -> score : " << score << " between " << refFrame.mnId << " and " << auxFrameIndex->GetFrame().mnId << std::endl;
                sortedCandidates.insert(std::make_pair(auxFrameIndex.get(), score));
            }
            // else
            // {
            //     std::cout << "AuxiliaryFrameDatabase::DetectNBestCandidates -> auxFrameIndex is nullptr!" << std::endl;
            // }
            iIterCount++;
        }

        if (sortedCandidates.empty())
            return std::vector<AuxiliaryFrame *>();

        // return first N candidates with highest score
        std::vector<AuxiliaryFrame *> vAuxFrames;
        vAuxFrames.reserve(candidatesNum);
        for (const auto &candidate : sortedCandidates)
        {
            vAuxFrames.push_back(candidate.first);
            if (vAuxFrames.size() == candidatesNum)
                break;
        }

        return vAuxFrames;
    }

    std::vector<KeyFrame *> AuxiliaryFrameDatabase::DetectCandidatesViaKFs(Frame &refFrame) // TODO: test
    {
        // NOTE: this is same as the DetectRelocalizationCandidates in KeyFrameDatabase, except we will use reference KeyFrames of Frames in the auxiliary database
        // in addition, we are assuming KFs are in same map since we should be in localization-only mode when this method is used
        // also, extra BoW calculation just in case

        // TODO: first, compurt BoW for auxiliary database (if vocabularies are same, should not be needed, but just in case)
        // auto vBow = computeAuxiliaryBoW(pF);
        const auto &vBow = refFrame.mBowVec;

        list<KeyFrame *> lKFsSharingWords;

        // Search all keyframes that share a word with current frame
        {
            unique_lock<mutex> lock(mMutex);

            for (DBoW2::BowVector::const_iterator vit = vBow.begin(), vend = vBow.end(); vit != vend; vit++)
            {
                for (auto &auxFrame : vInvertedFile[vit->first])
                {
                    if (!auxFrame->isInternalFrameValid())
                        continue;

                    // get reference KF of the auxiliary frame
                    KeyFrame *pKFi = auxFrame->GetFrame().mpReferenceKF;

                    if (!pKFi)
                        continue;

                    if (pKFi->mnRelocQuery != refFrame.mnId)
                    {
                        pKFi->mnRelocWords = 0;
                        pKFi->mnRelocQuery = refFrame.mnId;
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
                if (pKF2->mnRelocQuery != refFrame.mnId)
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