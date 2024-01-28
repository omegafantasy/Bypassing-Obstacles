using UnityEngine;
using System;
using System.Collections;
using System.Collections.Generic;


public class PhysicalARC_Redirector : Redirector
{
    const float INF = 100000;
    const float EPS = 1e-5f;
    const int BUFFERSIZE = 25;
    public float currScore;
    public List<Tuple<float, float, Vector2, Vector2, float>> pathPredictionResults; // gt, curvature, pos, dir, score
    public Tuple<float, float, float> redirectionUnitARC; // gt, curvature, gr
    public bool useRedirectionUnit;

    // public float angle2Waypoint;
    public float aheadStraightDis;
    public float backStraightDis;

    public List<float> speedBuffer;
    public float avgSpeed;
    public int speedRecords;
    public float lastGt;

    private void Awake()
    {
        pathPredictionResults = new List<Tuple<float, float, Vector2, Vector2, float>>();
        useRedirectionUnit = false;
        speedBuffer = new List<float>();
        avgSpeed = speedRecords = 0;
        lastGt = 1;
    }

    private void Update()
    {
        if (redirectionManager != null)
        {
            if (redirectionManager.isWalking)
            {
                float speed = Utilities.FlattenedPos2D(redirectionManager.deltaPos).magnitude * lastGt / globalConfiguration.GetDeltaTime();
                speedBuffer.Add(speed);
                if (speedBuffer.Count > BUFFERSIZE)
                {
                    speedBuffer.RemoveAt(0);
                }
                avgSpeed = (avgSpeed * speedRecords + speed) / (speedRecords + 1);
                speedRecords++;
            }
        }
    }

    private float GetAvgSpeed()
    {
        if (speedBuffer.Count < BUFFERSIZE)
        {
            // Debug.Log("use global");
            return globalConfiguration.translationSpeed;
        }
        else
        {
            float buf = 0;
            foreach (float val in speedBuffer)
            {
                buf += val;
            }
            // Debug.Log(0.5f * avgSpeed + 0.5f * buf / speedBuffer.Count);
            return 0.5f * avgSpeed + 0.5f * buf / speedBuffer.Count;
        }
    }

    public override void InjectRedirection()
    {
        if (useRedirectionUnit)
        {
            // if (movementManager.avatarId == 0)
            // {
            //     Debug.Log("gr:" + redirectionUnitARC.Item3);
            // }
            SetTranslationGain(redirectionUnitARC.Item1);
            SetCurvature(redirectionUnitARC.Item2);
            SetRotationGain(redirectionUnitARC.Item3);
            lastGt = redirectionUnitARC.Item1;
        }
        else
        {
            SetTranslationGain(1);
            SetCurvature(0);
            SetRotationGain(1);
            lastGt = 1;
        }
    }

    public float PhysicalPoseScore(Vector2 pos, Vector2 dir, Vector2 oldpos, bool check)
    {
        if (check)
        {
            if (!CheckValid(oldpos, pos))
            {
                return 0;
            }
        }
        var ahead = redirectionManager.GetPhysicalDistance(pos, dir);
        var left = redirectionManager.GetPhysicalDistance(pos, Utilities.RotateVector(dir, -90));
        var right = redirectionManager.GetPhysicalDistance(pos, Utilities.RotateVector(dir, 90));
        // if (movementManager.avatarId == 0)
        // {
        //     Debug.Log("ahead:" + ahead + "left:" + left + "right:" + right);
        // }
        return Mathf.Pow(ahead, 3f / 4) * Mathf.Pow(left * right, 1f / 8) / Mathf.Sqrt(redirectionManager.area);// TODO
    }

    public Tuple<Vector2, Vector2> CalculateNextPose(Vector2 pos, Vector2 dir, float curvature, float gt, float speed)
    { // curvature>0 means left turn, <0 means right turn
        var timeStep = globalConfiguration.timeStep;
        var dis = timeStep * speed / gt;
        dir = dir.normalized;

        if (Mathf.Abs(curvature) < EPS)
        {
            return new Tuple<Vector2, Vector2>(pos + dir * dis, dir);
        }
        float radius = 1 / curvature;
        float angle = dis * curvature;

        float side = radius * (1 - Mathf.Cos(angle));
        float ahead = radius * Mathf.Sin(angle);
        Vector2 newdir = Utilities.RotateVector(dir, -angle * 180 / Mathf.PI);
        Vector2 newpos = pos + dir * ahead + Utilities.RotateVector(dir, -Mathf.Sign(curvature) * 90) * side;

        return new Tuple<Vector2, Vector2>(newpos, newdir);
    }

    public bool CheckValid(Vector2 pos, Vector2 newpos)
    {
        var space = globalConfiguration.physicalSpaces[movementManager.avatarId];
        var wall = space.trackingSpace;
        var obstacles = space.obstaclePolygons;
        if (Utilities.IfPosInPolygon(wall, newpos))
        {
            foreach (var obstacle in obstacles)
            {
                if (Utilities.IfPosInPolygon(obstacle, newpos))
                {
                    return false;
                }
            }
            for (int i = 0; i < wall.Count; i++)
            {
                if (Utilities.IfTwoLinesIntersect(wall[i], wall[(i + 1) % wall.Count], pos, newpos))
                {
                    return false;
                }
            }
            foreach (var obstacle in obstacles)
            {
                for (int i = 0; i < obstacle.Count; i++)
                {
                    if (Utilities.IfTwoLinesIntersect(obstacle[i], obstacle[(i + 1) % obstacle.Count], pos, newpos))
                    {
                        return false;
                    }
                }
            }
            return true;
        }
        return false;
    }

    public void ShortTermPathPrediction()
    {
        var rm = redirectionManager;
        rm.FixHeadTransform();
        rm.UpdateCurrentUserState();
        var currPosReal = Utilities.FlattenedPos2D(rm.currPosReal);
        var currDirReal = Utilities.FlattenedDir2D(rm.currDirReal);
        // var targetDirReal = Utilities.FlattenedDir2D(rm.GetDirReal(redirectionManager.targetWaypoint.position - redirectionManager.currPos));
        // angle2Waypoint = Mathf.Abs(Vector2.SignedAngle(currDirReal, targetDirReal));
        aheadStraightDis = rm.GetPhysicalDistance(currPosReal, currDirReal);
        backStraightDis = rm.GetPhysicalDistance(currPosReal, -currDirReal);
        // if (movementManager.avatarId == 0)
        // {
        //     Debug.Log(targetDirReal);
        // }
        currScore = PhysicalPoseScore(currPosReal, currDirReal, currPosReal, false);
        pathPredictionResults.Clear();
        float[] gtChoices = { globalConfiguration.MIN_TRANS_GAIN, 1, globalConfiguration.MAX_TRANS_GAIN };
        float[] curvatureChoices = { -1f / globalConfiguration.CURVATURE_RADIUS, -0.5f / globalConfiguration.CURVATURE_RADIUS, 0, 0.5f / globalConfiguration.CURVATURE_RADIUS, 1f / globalConfiguration.CURVATURE_RADIUS };
        foreach (var gt in gtChoices)
        {
            foreach (var curvature in curvatureChoices)
            {
                var pose = CalculateNextPose(currPosReal, currDirReal, curvature, gt, GetAvgSpeed());
                var score = PhysicalPoseScore(pose.Item1, pose.Item2, currPosReal, true);
                pathPredictionResults.Add(new Tuple<float, float, Vector2, Vector2, float>(gt, curvature, pose.Item1, pose.Item2, score));
            }
        }
    }
}