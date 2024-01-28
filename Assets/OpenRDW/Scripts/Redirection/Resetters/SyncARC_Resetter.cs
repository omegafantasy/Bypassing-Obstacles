using UnityEngine;
using System.Collections;
using System.Collections.Generic;

public class SyncARC_Resetter : Resetter
{
    const float INF = 100000;
    const int DIR_SAMPLES = 30;
    float requiredRotateSteerAngle = 0; // steering angle，rotate the physical plane and avatar together

    float requiredRotateAngle = 0; // normal rotation angle, only rotate avatar

    float rotateDir; // rotation direction, positive if rotate clockwise

    float speedRatio;

    APF_Redirector redirector;


    public override bool IsResetRequired()
    {
        return IfCollisionHappens();
    }

    public override void InitializeReset()
    {
        var rm = redirectionManager;
        var currPosReal = Utilities.FlattenedPos2D(rm.currPosReal);
        targetPos = DecideResetPosition(currPosReal);
        var currDir = Utilities.FlattenedDir2D(rm.currDirReal);
        Vector2 targetDir = Vector2.zero;
        float targetLoss = -INF;
        for (int i = 0; i < DIR_SAMPLES; i++)
        {
            float dirAngle = 2 * Mathf.PI * i / DIR_SAMPLES;
            Vector2 dir = new Vector2(Mathf.Cos(dirAngle), Mathf.Sin(dirAngle));
            if (rm.IsDirSafe(currPosReal, dir))
            {
                // virtual alignment states stay unchanged, so we only calculate physical ones
                var aheadDis = rm.GetPhysicalDistance(currPosReal, dir);
                var leftDis = rm.GetPhysicalDistance(currPosReal, Utilities.RotateVector(dir, -90));
                var rightDis = rm.GetPhysicalDistance(currPosReal, Utilities.RotateVector(dir, 90));
                var totalLoss = aheadDis + leftDis + rightDis - rm.currVirAheadDis - rm.currVirLeftDis - rm.currVirRightDis;
                if ((targetLoss < 0 && totalLoss > targetLoss) || (totalLoss >= 0 && totalLoss < targetLoss))
                {
                    targetLoss = totalLoss;
                    targetDir = dir;
                }
            }
        }

        var targetRealRotation = 360 - Vector2.Angle(targetDir, currDir); // required rotation angle in real world

        rotateDir = -(int)Mathf.Sign(Utilities.GetSignedAngle(rm.currDirReal, Utilities.UnFlatten(targetDir)));

        requiredRotateSteerAngle = 360 - targetRealRotation;

        requiredRotateAngle = targetRealRotation;

        speedRatio = requiredRotateSteerAngle / requiredRotateAngle;

        if (globalConfiguration.useResetPanel)
        {
            SetPanel();
        }
        else
        {
            SetHUD((int)rotateDir);
        }
    }

    public override void InjectResetting()
    {
        if (globalConfiguration.useResetPanel)
        { // use freeze-turn to ensure the virtual postion and direction unchanged
            InjectRotation(-redirectionManager.deltaDir);
            InjectTranslation(-redirectionManager.deltaPos);
            UpdatePanel();
        }
        else
        {
            var steerRotation = speedRatio * redirectionManager.deltaDir;
            if (Mathf.Abs(requiredRotateSteerAngle) <= Mathf.Abs(steerRotation) || requiredRotateAngle == 0)
            { // meet the rotation requirement
                InjectRotation(requiredRotateSteerAngle);

                // reset end
                redirectionManager.OnResetEnd();
                requiredRotateSteerAngle = 0;
            }
            else
            { // rotate the rotation calculated by ratio
                InjectRotation(steerRotation);
                requiredRotateSteerAngle -= Mathf.Abs(steerRotation);
            }
        }
    }

    public override void EndReset()
    {
        if (globalConfiguration.useResetPanel)
        {
            DestroyPanel();
        }
        else
        {
            DestroyHUD();
        }
    }

    public override void SimulatedWalkerUpdate()
    {
        // Act is if there's some dummy target a meter away from you requiring you to rotate        
        var rotateAngle = redirectionManager.GetDeltaTime() * redirectionManager.globalConfiguration.rotationSpeed;
        // finish specified rotation
        if (rotateAngle >= requiredRotateAngle)
        {
            rotateAngle = requiredRotateAngle;
            // Avoid accuracy error
            requiredRotateAngle = 0;
        }
        else
        {
            requiredRotateAngle -= rotateAngle;
        }
        redirectionManager.simulatedWalker.RotateInPlace(rotateAngle * rotateDir);
    }

}