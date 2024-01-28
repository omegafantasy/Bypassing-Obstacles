using UnityEngine;
using System.Collections;
using System.Collections.Generic;


public class SyncARC_Redirector : Redirector
{
    
    public override void InjectRedirection()
    {
        SetRotationGain(1); //TODO
        var rm = redirectionManager;
        var aheadLoss = rm.currPhyAheadDis - rm.currVirAheadDis;
        var leftLoss = rm.currPhyLeftDis - rm.currVirLeftDis;
        var rightLoss = rm.currPhyRightDis - rm.currVirRightDis;
        if (movementManager.avatarId == 0)
        {
            // Debug.Log("phyahead:" + rm.currPhyAheadDis + ";phyleft:" + rm.currPhyLeftDis + ";phyright:" + rm.currPhyRightDis);
            // Debug.Log("virahead:" + rm.currVirAheadDis + ";virleft:" + rm.currVirLeftDis + ";virright:" + rm.currVirRightDis);
            // Debug.Log("aheadLoss:" + aheadLoss + ";leftLoss:" + leftLoss + ";rightLoss:" + rightLoss);
        }
        if (Mathf.Abs(aheadLoss) + Mathf.Abs(leftLoss) + Mathf.Abs(rightLoss) < 0.1)
        {
            SetCurvature(0);
            SetTranslationGain(1);
            return;
        }
        if (leftLoss > rightLoss)
        {
            SetCurvature(1 / globalConfiguration.CURVATURE_RADIUS);
        }
        else if (leftLoss < rightLoss)
        {
            SetCurvature(-1 / globalConfiguration.CURVATURE_RADIUS);
        }
        else
        {
            SetCurvature(0);
        }
        SetTranslationGain(rm.currVirAheadDis / rm.currPhyAheadDis);
    }
}