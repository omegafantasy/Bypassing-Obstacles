using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using MathNet.Numerics;

public class Bypassing_Redirector : Redirector
{
    const float EPSILON = 1e-5f;
    const float STABILITYRATIO = 0.5f;
    const float SQUARELENGTH = 0.5f;
    const int BUFFERSIZE = 5;
    public List<Tuple<int, float, float, bool, Vector2>> collisionParams;
    public Tuple<float, float> collisionTimeRange;// the time range with current dir
    public Tuple<float, float> resetTimeRange;// the time range with any dir
    public float timeToWayPoint;// time needed to get to current waypoint
    public Tuple<int, float, float> redirectParams;// straight/left/right, gr(radius), gt
    public Tuple<Vector2, int, float, float> furthestParams;// dir, redirectParams; it can go the furthest distance
    public List<PositionSquare> positionSquares;
    public PositionSquare targetPosition;
    public bool useRedirectParams;
    public bool useTurnPoint;
    public Vector2 turnPoint;
    public float gr;
    public List<float> speedBuffer;
    public float avgSpeed;
    public int speedRecords;
    public float lastGt;

    private void Awake()
    {
        useRedirectParams = false;
        speedBuffer = new List<float>();
        avgSpeed = speedRecords = 0;
        lastGt = 1;
    }

    public override void InjectRedirection()
    {
        if (useRedirectParams)
        {
            float radius = redirectParams.Item1 == 1 ? redirectParams.Item2 : -redirectParams.Item2;
            float gt = redirectParams.Item3;
            lastGt = gt;
            SetTranslationGain(gt);
            SetCurvature(1 / radius);
            SetRotationGain(1);
            //SetRotationGain(gr);
            if (useTurnPoint)
            {
                if ((Utilities.FlattenedPos2D(redirectionManager.currPosReal) - turnPoint).magnitude < 0.03f)
                { // S-turn, change the way
                    // if (movementManager.avatarId == 0)
                    // {
                    //     Debug.Log("S-Turn");
                    // }
                    useTurnPoint = false;
                    redirectParams = new Tuple<int, float, float>(redirectParams.Item1 == 1 ? 2 : 1, redirectParams.Item2, gt);
                }
            }
            // if (movementManager.avatarId == 0)
            // {
            //     Debug.Log("------");
            //     Debug.Log(Utilities.FlattenedPos2D(redirectionManager.currPosReal) + " " + (targetPosition == null ? Vector2.zero : targetPosition.center));
            //     Debug.Log(redirectParams.Item1 + " " + redirectParams.Item2 + " " + redirectParams.Item3);
            //     Debug.Log(useTurnPoint + ":" + turnPoint);
            // }
        }
        else
        {
            lastGt = 1;
        }
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

    public class PositionSquare
    {
        public Bypassing_Redirector redirector;
        public Vector2 from;
        public Vector2 to;
        public Vector2 center;
        public List<Tuple<float, float, Vector2>> list; // mintime, maxtime, dir
        public float safePoint; // value to present the position's safety, smaller means safer
        public float maxDis;
        public PositionSquare(float fromx, float fromy, Bypassing_Redirector redirector)
        {
            this.from = new Vector2(fromx, fromy);
            this.to = this.from + new Vector2(SQUARELENGTH, SQUARELENGTH);
            this.center = (this.from + this.to) / 2;
            this.safePoint = 0.0f;
            this.maxDis = 0.0f;
            this.redirector = redirector;
            this.list = new List<Tuple<float, float, Vector2>>();
        }
        public bool InsidePolygon(List<Vector2> polygon)
        {
            int count = 0;
            for (int i = 0; i < polygon.Count; i++)
            {
                Vector2 p = polygon[i];
                Vector2 q = polygon[(i + 1) % polygon.Count];
                if (((p.x <= center.x && center.x <= q.x) || (q.x <= center.x && center.x <= p.x)) && p.y <= center.y)
                {
                    count++;
                }
            }
            if (count % 2 == 1)
            {
                return true;
            }
            return false;
        }

        public void GetTimeRangeList()
        {
            Vector2 unit = new Vector2(1, 0);
            for (int i = 0; i < 30; i++)
            {
                Vector2 nowDir = new Vector2((float)Mathf.Cos(Mathf.PI / 15 * i), (float)Mathf.Sin(Mathf.PI / 15 * i));
                Tuple<float, float> timeRange = redirector.GetTimeRange(redirector.GetCollisionParams(center, nowDir));
                safePoint += 1 / timeRange.Item2;
                maxDis = timeRange.Item2 > maxDis ? timeRange.Item2 : maxDis;
                list.Add(new Tuple<float, float, Vector2>(timeRange.Item1, timeRange.Item2, nowDir));
            }
        }
    }

    public void InitializePositionSquares()
    {
        positionSquares = new List<PositionSquare>();
        var trackingSpacePoints = globalConfiguration.physicalSpaces[movementManager.physicalSpaceIndex].trackingSpace;
        var obstaclePolygons = globalConfiguration.physicalSpaces[movementManager.physicalSpaceIndex].obstaclePolygons;
        float xmin = 100000f, xmax = -100000f, ymin = 100000f, ymax = -100000f;
        foreach (var c in trackingSpacePoints)
        {
            xmin = c.x < xmin ? c.x : xmin;
            xmax = c.x > xmax ? c.x : xmax;
            ymin = c.y < ymin ? c.y : ymin;
            ymax = c.y > ymax ? c.y : ymax;
        }
        float xbase, ybase, xcount, ycount;

        // TODO: the following config must be modified according to the physical spaces
        // space preprocessing config start
        if (movementManager.avatarId == 0)
        {
            xbase = -2.5f;
            ybase = -2f;
            xcount = (int)(4.5f / SQUARELENGTH);
            ycount = (int)(4f / SQUARELENGTH);
        }
        else if (movementManager.avatarId == 1)
        {
            xbase = -12f;
            ybase = -5f;
            xcount = (int)(5f / SQUARELENGTH);
            ycount = (int)(5f / SQUARELENGTH);
        }
        else if (movementManager.avatarId == 2)
        {
            xbase = 0f;
            ybase = -12f;
            xcount = (int)(5f / SQUARELENGTH);
            ycount = (int)(10f / SQUARELENGTH);
        }
        else if (movementManager.avatarId == 3)
        {
            xbase = 0f;
            ybase = -12f;
            xcount = (int)(10f / SQUARELENGTH);
            ycount = (int)(10f / SQUARELENGTH);
        }
        else if (movementManager.avatarId == 4)
        {
            xbase = 7f;
            ybase = 0f;
            xcount = (int)(5 / SQUARELENGTH);
            ycount = (int)(5 / SQUARELENGTH);
        }
        else if (movementManager.avatarId == 5)
        {
            xbase = 7f;
            ybase = -7f;
            xcount = (int)(5 / SQUARELENGTH);
            ycount = (int)(5 / SQUARELENGTH);
        }
        else
        {
            xbase = 0f;
            ybase = 0f;
            xcount = 0;
            ycount = 0;
        }
        // space preprocessing config end

        // System.Diagnostics.Stopwatch stopWatch = new System.Diagnostics.Stopwatch();
        // stopWatch.Start();
        for (int i = 0; i < xcount; i++)
        {
            for (int j = 0; j < ycount; j++)
            {
                PositionSquare newSquare = new PositionSquare(xbase + i * SQUARELENGTH, ybase + j * SQUARELENGTH, this);
                bool valid = newSquare.InsidePolygon(trackingSpacePoints);
                foreach (var obstaclePolygon in obstaclePolygons)
                {
                    if (newSquare.InsidePolygon(obstaclePolygon))
                    {
                        valid = false;
                        break;
                    }
                }
                if (valid)
                {
                    newSquare.GetTimeRangeList();
                    positionSquares.Add(newSquare);
                }
            }
        }
        // stopWatch.Stop();
        // long times1 = stopWatch.ElapsedMilliseconds;
        // Debug.Log(movementManager.avatarId + ":" + times1 + "ms");
    }

    public void RotateToBestAngle()
    {
        var waypointDir = Utilities.FlattenedPos2D(redirectionManager.targetWaypoint.position - redirectionManager.currPos);
        var angleToWaypoint = Utilities.GetSignedAngle(Utilities.FlattenedDir2D(redirectionManager.currDir), waypointDir) / 180 * Mathf.PI;
        var currPosReal = Utilities.FlattenedPos2D(redirectionManager.currPosReal);
        var currDirReal = Utilities.FlattenedDir2D(redirectionManager.currDirReal);
        var currAngleReal = Mathf.Atan2(currDirReal.y, currDirReal.x);
        var minAngle = currAngleReal - angleToWaypoint / globalConfiguration.MAX_ROT_GAIN;
        var maxAngle = currAngleReal - angleToWaypoint / globalConfiguration.MIN_ROT_GAIN;
        if (minAngle > maxAngle)
        {
            var tmp = minAngle;
            minAngle = maxAngle;
            maxAngle = tmp;
        }
        var SPLIT = 9;
        var delta = (maxAngle - minAngle) / SPLIT;
        float globalMax = -1f;
        float bestAngle = 0;
        for (int i = 0; i <= SPLIT; i++)
        {
            var nowAngle = minAngle + i * delta;
            var nowDir = new Vector2(Mathf.Cos(nowAngle), Mathf.Sin(nowAngle));
            var res = GetCollisionParams(currPosReal, nowDir);
            float maxdis = -1f;
            foreach (var route in res)
            {
                if (route.Item3 > maxdis)
                {
                    maxdis = route.Item3;
                }
            }
            if (maxdis > globalMax)
            {
                globalMax = maxdis;
                bestAngle = nowAngle;
            }
        }
        gr = angleToWaypoint / (currAngleReal - bestAngle);
    }

    public List<Tuple<int, float, float, bool, Vector2>> GetCollisionParams(Vector2 pos, Vector2 dir)
    { //return: int(straight:0/left:1/right:2), float(curvature radius), float(dis), s-turn, turnPoint
        dir = dir.normalized;
        Vector2 ortho = new Vector2(-dir.y, dir.x);

        List<Tuple<int, float, float, bool, Vector2>> list = new List<Tuple<int, float, float, bool, Vector2>>();
        float R = globalConfiguration.CURVATURE_RADIUS;

        //left and right
        var K = 5;
        float leftDis = 0, rightDis = 0;
        for (int i = 0; i < K; i++)
        {
            for (int j = 1; j < 3; j++)
            {
                float mindis = 100000f;
                float radius = R / Mathf.Cos(Mathf.PI / (2 * K) * i); //radius of the circle
                Vector2 rotate_radius = radius * dir;
                if (j == 1)
                {//left
                    rotate_radius = new Vector2(-rotate_radius.y, rotate_radius.x);
                }
                else
                {//right,clockwise
                    rotate_radius = new Vector2(rotate_radius.y, -rotate_radius.x);
                }
                Vector2 center = pos + rotate_radius;//center of the circle
                foreach (var polygon in redirectionManager.polygons)
                {
                    for (int k = 0; k < polygon.Count; k++)
                    {
                        var p = polygon[k];
                        var q = polygon[(k + 1) % polygon.Count];
                        Vector2 ortho_dir = new Vector2((p - q).y, (q - p).x).normalized;// point to line, orthogonal to line
                        if (Mathf.Sign(Cross(pos + ortho_dir * 1000f - q, p - q)) == Mathf.Sign(Cross(pos - q, p - q)))
                        {
                            ortho_dir = -ortho_dir;
                        }
                        float tmpDis = CircleCollisionDis(pos, dir, p, q, radius, center);
                        mindis = tmpDis < mindis ? tmpDis : mindis;
                    }
                }
                if (i == 0)
                {
                    if (j == 1)
                    {
                        leftDis = mindis;
                    }
                    else
                    {
                        rightDis = mindis;
                    }
                }
                list.Add(new Tuple<int, float, float, bool, Vector2>(j, radius, mindis, false, Vector2.zero));//add to the list
            }
        }

        // s-turn
        var L = 8;
        for (int i = 1; i <= Mathf.Min(L, (int)(leftDis / 0.5f)); i++)
        { // left
            float dis11 = i * 0.5f;
            float angle11 = dis11 / R;
            Vector2 p1 = pos + dir * R * Mathf.Sin(angle11) + ortho * R * (1 - Mathf.Cos(angle11));
            Vector2 p1Dir = Utilities.RotateVector(dir, angle11 * 180 / Mathf.PI);
            Vector2 center12 = p1 + Utilities.RotateVector(p1Dir, -90) * R;
            float dis12 = 100000f;
            foreach (var polygon in redirectionManager.polygons)
            {
                for (int k = 0; k < polygon.Count; k++)
                {
                    var p = polygon[k];
                    var q = polygon[(k + 1) % polygon.Count];
                    Vector2 ortho_dir = new Vector2((p - q).y, (q - p).x).normalized;// point to line, orthogonal to line
                    if (Mathf.Sign(Cross(pos + ortho_dir * 1000f - q, p - q)) == Mathf.Sign(Cross(pos - q, p - q)))
                    {
                        ortho_dir = -ortho_dir;
                    }
                    float tmpDis = CircleCollisionDis(p1, p1Dir, p, q, R, center12);
                    dis12 = tmpDis < dis12 ? tmpDis : dis12;
                }
            }
            float dis1 = dis11 + dis12;
            list.Add(new Tuple<int, float, float, bool, Vector2>(1, R, dis1, true, p1));
        }
        for (int i = 1; i <= Mathf.Min(L, (int)(rightDis / 0.5f)); i++)
        { // right
            float dis21 = i * 0.5f;
            float angle21 = dis21 / R;
            Vector2 p2 = pos + dir * R * Mathf.Sin(angle21) - ortho * R * (1 - Mathf.Cos(angle21));
            Vector2 p2Dir = Utilities.RotateVector(dir, -angle21 * 180 / Mathf.PI);
            Vector2 center22 = p2 + Utilities.RotateVector(p2Dir, 90) * R;
            float dis22 = 100000f;
            foreach (var polygon in redirectionManager.polygons)
            {
                for (int k = 0; k < polygon.Count; k++)
                {
                    var p = polygon[k];
                    var q = polygon[(k + 1) % polygon.Count];
                    Vector2 ortho_dir = new Vector2((p - q).y, (q - p).x).normalized;// point to line, orthogonal to line
                    if (Mathf.Sign(Cross(pos + ortho_dir * 1000f - q, p - q)) == Mathf.Sign(Cross(pos - q, p - q)))
                    {
                        ortho_dir = -ortho_dir;
                    }
                    float tmpDis = CircleCollisionDis(p2, p2Dir, p, q, R, center22);
                    dis22 = tmpDis < dis22 ? tmpDis : dis22;
                }
            }
            float dis2 = dis21 + dis22;
            list.Add(new Tuple<int, float, float, bool, Vector2>(2, R, dis2, true, p2));
        }

        //straight line
        float straight_mindis = 100000f;
        foreach (var polygon in redirectionManager.polygons)
        {
            for (int k = 0; k < polygon.Count; k++)
            {
                var p = polygon[k];
                var q = polygon[(k + 1) % polygon.Count];
                Vector2 ortho_dir = new Vector2((p - q).y, (q - p).x).normalized;// point to line, orthogonal to line

                if (Mathf.Sign(Cross(pos + ortho_dir * 1000f - q, p - q)) == Mathf.Sign(Cross(pos - q, p - q)))
                {
                    ortho_dir = -ortho_dir;
                }

                //see a segment as 3 segments
                float tmpDis = LineCollisionDis(pos, dir, p, q);
                straight_mindis = tmpDis < straight_mindis ? tmpDis : straight_mindis;
            }
        }
        straight_mindis = straight_mindis > 0 ? straight_mindis : 0;
        list.Add(new Tuple<int, float, float, bool, Vector2>(0, 100000f, straight_mindis, false, Vector2.zero));//add to the list

        foreach (var tuple in list)
        {
            //Debug.Log(tuple);
        }
        //Debug.Log(redirectionManager.currDirReal.x + ":" + redirectionManager.currDirReal.z + ":" + redirectionManager.currPosReal.x + ":" + redirectionManager.currPosReal.z);
        return list;
    }

    public float LineCollisionDis(Vector2 realPos, Vector2 realDir, Vector2 p, Vector2 q)
    {
        Vector2 ortho_dir = new Vector2((p - q).y, (q - p).x).normalized;// recalculate
        if (Mathf.Sign(Cross(realPos + ortho_dir * 1000f - q, p - q)) == Mathf.Sign(Cross(realPos - q, p - q)))
        {
            ortho_dir = -ortho_dir;
        }
        if (Vector2.Dot(ortho_dir, realDir) <= 0)
        {
            return 100000f;
        }
        return (Dis(realPos, p, q) - globalConfiguration.RESET_TRIGGER_BUFFER) / (float)Mathf.Max(Mathf.Acos(Mathf.Abs(Vector2.Dot(realDir, p - q)) / (p - q).magnitude), 0.001f);
    }

    public float CircleCollisionDis(Vector2 realPos, Vector2 realDir, Vector2 p, Vector2 q, float radius, Vector2 center)
    {
        float center_dis = Dis(center, p, q);
        Vector2 ortho_dir = new Vector2((p - q).y, (q - p).x).normalized;// recalculate
        if (Mathf.Sign(Cross(realPos + ortho_dir * 1000f - q, p - q)) == Mathf.Sign(Cross(realPos - q, p - q)))
        {
            ortho_dir = -ortho_dir;
        }
        float collision_dis = 100000f;
        if (center_dis > radius)
        {//not intersect
            return collision_dis;
        }

        Vector2 center_ortho_dir = ortho_dir;
        if (Mathf.Sign(Cross(center + center_ortho_dir * 1000f - q, p - q)) == Mathf.Sign(Cross(center - q, p - q)))
        {
            center_ortho_dir = -center_ortho_dir;
        }

        float half_string_dis = (float)Mathf.Sqrt(radius * radius - center_dis * center_dis);
        //intersect points
        Vector2 point1 = center + center_ortho_dir * center_dis + half_string_dis * (p - q).normalized;
        Vector2 point2 = center + center_ortho_dir * center_dis - half_string_dis * (p - q).normalized;
        if (Mathf.Sign(Cross(p - center, p - q)) == Mathf.Sign(Cross(p - realPos, p - q)))
        {//realPos and center in one side
            if (InLineSection(point1, p, q))
            {// point1 in section
                if (Vector2.Dot(realDir, point1 - center) >= 0)
                {//near intersect
                    float tmp_dis = radius * (float)(Vector2.Angle(realPos - center, point1 - center) / 180 * Mathf.PI);
                    collision_dis = tmp_dis < collision_dis ? tmp_dis : collision_dis;
                }
                else
                {//far intersect
                    float tmp_dis = radius * (float)(Mathf.PI * 2 - Vector2.Angle(realPos - center, point1 - center) / 180 * Mathf.PI);
                    collision_dis = tmp_dis < collision_dis ? tmp_dis : collision_dis;
                }
            }
            if (InLineSection(point2, p, q))
            {// point2 in section
                if (Vector2.Dot(realDir, point2 - center) >= 0)
                {//near intersect
                    float tmp_dis = radius * (float)(Vector2.Angle(realPos - center, point2 - center) / 180 * Mathf.PI);
                    collision_dis = tmp_dis < collision_dis ? tmp_dis : collision_dis;
                }
                else
                {//far intersect
                    float tmp_dis = radius * (float)(Mathf.PI * 2 - Vector2.Angle(realPos - center, point2 - center) / 180 * Mathf.PI);
                    collision_dis = tmp_dis < collision_dis ? tmp_dis : collision_dis;
                }
            }
        }
        else
        {//realPos and center in two sides
            if (Vector2.Dot(point1 - center, realDir) >= 0)
            {//consider point1
                if (InLineSection(point1, p, q))
                {// point1 in section
                    collision_dis = radius * (float)(Vector2.Angle(realPos - center, point1 - center) / 180 * Mathf.PI);
                }
            }
            else
            {//consider point2
                if (InLineSection(point2, p, q))
                {// point2 in section
                    collision_dis = radius * (float)(Vector2.Angle(realPos - center, point2 - center) / 180 * Mathf.PI);
                }
            }
        }
        collision_dis = Mathf.Max(collision_dis - globalConfiguration.RESET_TRIGGER_BUFFER * 1.5f, 0);
        return collision_dis;
    }

    public Tuple<float, float> GetDisRange(List<Tuple<int, float, float, bool, Vector2>> list)
    {
        float min = 100000f, max = 0.0f;
        foreach (var tuple in list)
        {
            min = tuple.Item3 < min ? tuple.Item3 : min;
            max = tuple.Item3 > max ? tuple.Item3 : max;
        }
        return new Tuple<float, float>(min, max);
    }

    public Tuple<float, float> GetTimeRange(List<Tuple<int, float, float, bool, Vector2>> list)
    {
        Tuple<float, float> dis_range = GetDisRange(list);
        return new Tuple<float, float>(dis_range.Item1 / GetAvgSpeed() * globalConfiguration.MIN_TRANS_GAIN, dis_range.Item2 / GetAvgSpeed() * globalConfiguration.MAX_TRANS_GAIN);
    }

    public Tuple<float, float> GetCircleDisRange(float time)
    {
        float min = time * GetAvgSpeed() / globalConfiguration.MAX_TRANS_GAIN;
        float radius = globalConfiguration.CURVATURE_RADIUS;
        float angle = min / radius;
        if (angle < Mathf.PI)
        {
            min = (float)(Mathf.Sin(angle / 2) * radius * 2);
        }
        else
        {
            Debug.Log("Too Long for a circle");
        }
        float max = time * GetAvgSpeed() / globalConfiguration.MIN_TRANS_GAIN;
        return new Tuple<float, float>(min, max);
    }

    public Tuple<int, float, float, bool, Vector2> GetFurthestParams(List<Tuple<int, float, float, bool, Vector2>> list)
    {
        float max = 0.0f;
        Tuple<int, float, float, bool, Vector2> ret = new Tuple<int, float, float, bool, Vector2>(0, 100000f, 1.0f, false, Vector2.zero);
        foreach (var tuple in list)
        {
            if (tuple.Item3 > max)
            {
                max = tuple.Item3;
                ret = tuple;
            }
        }
        return ret;
    }

    public void ResetAndGuideToSafePos()
    {
        // Debug.Log(movementManager.avatarId + "ResetAndGuideToSafePos");
        float time = timeToWayPoint;
        Tuple<float, float> disRange = GetCircleDisRange(time);
        Vector2 pos = new Vector2(redirectionManager.currPosReal.x, redirectionManager.currPosReal.z);
        float minSafePoint = 100000f;
        targetPosition = null;
        foreach (var position in positionSquares)
        {
            if ((position.center - pos).magnitude >= disRange.Item1 && (position.center - pos).magnitude <= disRange.Item2 && position.safePoint < minSafePoint)
            { // reachable and valueable
                var reachable = ResetReachable(position.center, pos, time);
                if (reachable.Item1)
                {
                    targetPosition = position;
                    minSafePoint = position.safePoint;
                    ((SeparateSpace_Resetter)redirectionManager.resetter).resetDir = reachable.Item2;
                    redirectParams = new Tuple<int, float, float>(reachable.Item3, reachable.Item4, reachable.Item5);
                }
            }
        }
        useRedirectParams = true;
        ((SeparateSpace_Resetter)redirectionManager.resetter).useResetDir = true;
        if (targetPosition == null)
        {
            ResetAndGuideToFurthest();
        }
    }
    public void GuideToSafePos()
    {
        // Debug.Log(movementManager.avatarId + "GuideToSafePos");
        float time = timeToWayPoint;
        Vector2 pos = new Vector2(redirectionManager.currPosReal.x, redirectionManager.currPosReal.z);
        Vector2 dir = Utilities.FlattenedPos2D(Utilities.GetRelativePosition(redirectionManager.targetWaypoint.position, redirectionManager.trackingSpace.transform) - redirectionManager.currPosReal).normalized;
        float minSafePoint = 100000f;
        if (targetPosition != null)
        { // consider the chosen target first
            var reachable = SWayReachable(targetPosition.center, pos, dir, time);
            if (reachable.Item1)
            {
                // if (movementManager.avatarId == 0)
                //     Debug.Log("succ " + pos + " " + targetPosition.center + " " + reachable.Item2 + " " + reachable.Item3 + " " + reachable.Item6);
                redirectParams = new Tuple<int, float, float>(reachable.Item2, reachable.Item3, reachable.Item4);
                useTurnPoint = reachable.Item5;
                turnPoint = reachable.Item6;
                useRedirectParams = true;
                return;
            }
        }
        targetPosition = null;
        foreach (var position in positionSquares)
        {
            var reachable = SWayReachable(position.center, pos, dir, time);
            if (reachable.Item1 && position.safePoint < minSafePoint)
            { // reachable and valuable
                // if (movementManager.avatarId == 0)
                //     Debug.Log("succ " + pos + " " + position.center + " " + reachable.Item2 + " " + reachable.Item3 + " " + reachable.Item6);
                targetPosition = position;
                redirectParams = new Tuple<int, float, float>(reachable.Item2, reachable.Item3, reachable.Item4);
                useTurnPoint = reachable.Item5;
                turnPoint = reachable.Item6;
                minSafePoint = position.safePoint;
            }
        }
        if (targetPosition == null)
        {
            GuideToFurthest();
        }
        useRedirectParams = true;
    }
    public void ResetAndGuideToFurthest()
    {
        // Debug.Log(movementManager.avatarId + "ResetAndGuideToFurthest");
        ((SeparateSpace_Resetter)redirectionManager.resetter).resetDir = furthestParams.Item1;

        redirectParams = new Tuple<int, float, float>(furthestParams.Item2, furthestParams.Item3, globalConfiguration.MAX_TRANS_GAIN);

        useRedirectParams = true;
        ((SeparateSpace_Resetter)redirectionManager.resetter).useResetDir = true;
    }
    public void GuideToFurthest()
    {
        // Debug.Log(movementManager.avatarId + "GuideToFurthest");
        float max = 0.0f;
        useTurnPoint = false;
        foreach (var tuple in collisionParams)
        {
            if (tuple.Item3 > max)
            {
                max = tuple.Item3;
                redirectParams = new Tuple<int, float, float>(tuple.Item1, tuple.Item2, globalConfiguration.MAX_TRANS_GAIN);
                useTurnPoint = tuple.Item4;
                turnPoint = tuple.Item5;
            }
        }
        targetPosition = null;
        useRedirectParams = true;
    }
    public void ResetAndGuideToMaxTimePos(float time)
    {
        time = Mathf.Min(time, timeToWayPoint);
        // Debug.Log(movementManager.avatarId + "ResetAndGuideToMaxTimePos");
        Tuple<float, float> disRange = GetCircleDisRange(time);
        Vector2 pos = new Vector2(redirectionManager.currPosReal.x, redirectionManager.currPosReal.z);
        float maxDis = 0f;
        targetPosition = null;
        foreach (var position in positionSquares)
        {
            if ((position.center - pos).magnitude >= disRange.Item1 && (position.center - pos).magnitude <= disRange.Item2 && position.maxDis > maxDis)
            { // reachable and valuable
                var reachable = ResetReachable(position.center, pos, time);
                if (reachable.Item1)
                {
                    targetPosition = position;
                    maxDis = position.maxDis;
                    ((SeparateSpace_Resetter)redirectionManager.resetter).resetDir = reachable.Item2;
                    redirectParams = new Tuple<int, float, float>(reachable.Item3, reachable.Item4, reachable.Item5);
                }
            }
        }
        useRedirectParams = true;
        ((SeparateSpace_Resetter)redirectionManager.resetter).useResetDir = true;
        if (targetPosition == null)
        {
            ResetAndGuideToFurthest();
        }
    }
    public void GuideToMaxTimePos(float time)
    {
        time = Mathf.Min(time, timeToWayPoint);
        // Debug.Log(movementManager.avatarId + "GuideToMaxTimePos");
        Vector2 pos = new Vector2(redirectionManager.currPosReal.x, redirectionManager.currPosReal.z);
        Vector2 dir = Utilities.FlattenedPos2D(Utilities.GetRelativePosition(redirectionManager.targetWaypoint.position, redirectionManager.trackingSpace.transform) - redirectionManager.currPosReal).normalized;
        float maxDis = 0f;
        if (targetPosition != null)
        { // consider the chosen target first
            var reachable = SWayReachable(targetPosition.center, pos, dir, time);
            if (reachable.Item1)
            {
                // if (movementManager.avatarId == 0)
                //     Debug.Log("succ " + pos + " " + targetPosition.center + " " + reachable.Item2 + " " + reachable.Item3 + " " + reachable.Item6);
                redirectParams = new Tuple<int, float, float>(reachable.Item2, reachable.Item3, reachable.Item4);
                useTurnPoint = reachable.Item5;
                turnPoint = reachable.Item6;
                useRedirectParams = true;
                return;
            }
        }
        targetPosition = null;
        foreach (var position in positionSquares)
        {
            var reachable = SWayReachable(position.center, pos, dir, time);
            if (reachable.Item1 && position.maxDis > maxDis)
            { // reachable and valuable
                // if (movementManager.avatarId == 0)
                //     Debug.Log("succ " + pos + " " + position.center + " " + reachable.Item2 + " " + reachable.Item3 + " " + reachable.Item6);
                targetPosition = position;
                redirectParams = new Tuple<int, float, float>(reachable.Item2, reachable.Item3, reachable.Item4);
                useTurnPoint = reachable.Item5;
                turnPoint = reachable.Item6;
                maxDis = position.maxDis;
            }
        }
        if (targetPosition == null)
        {
            GuideToFurthest();
        }
        useRedirectParams = true;
    }

    public float GetTimeToWayPoint()
    {
        float time = (Utilities.FlattenedPos2D(redirectionManager.currPos - redirectionManager.targetWaypoint.position).magnitude - globalConfiguration.distanceToWaypointThreshold) / GetAvgSpeed();
        time = time > 0 ? time : 0;
        return time;
    }

    public Tuple<Vector2, float> GetOptimalDirection()
    {
        Vector2 curr = new Vector2(redirectionManager.currDirReal.x, redirectionManager.currDirReal.z);
        Vector2 pos = new Vector2(redirectionManager.currPosReal.x, redirectionManager.currPosReal.z);
        Vector2 optimal = curr;
        float max = 0f;
        for (int i = 0; i < 30; i++)
        {
            Vector2 nowDir = new Vector2((float)(Mathf.Cos(Mathf.PI / 15 * i) * curr.x - Mathf.Sin(Mathf.PI / 15 * i) * curr.y), (float)(Mathf.Sin(Mathf.PI / 15 * i) * curr.x + Mathf.Cos(Mathf.PI / 15 * i) * curr.y));//rotation
            if (!IsDirSafe(pos, nowDir))
            {
                continue;
            }
            float nowMax = GetTimeRange(GetCollisionParams(pos, nowDir)).Item2;
            if (nowMax > max)
            {
                optimal = nowDir;
                max = nowMax;
            }
        }
        return new Tuple<Vector2, float>(optimal, max);
    }

    public Tuple<float, float> GetResetTimeRange(Vector2 pos)
    {
        Vector2 unit = new Vector2(1, 0);
        Tuple<int, float, float, bool, Vector2> furParams = new Tuple<int, float, float, bool, Vector2>(0, 100000f, 1.0f, false, Vector2.zero);
        Vector2 furDir = new Vector2(1.0f, 0);
        float min = 100000f, max = 0;
        for (int i = 0; i < 30; i++)
        {
            Vector2 nowDir = new Vector2((float)(Mathf.Cos(Mathf.PI / 15 * i) * unit.x - Mathf.Sin(Mathf.PI / 15 * i) * unit.y), (float)(Mathf.Sin(Mathf.PI / 15 * i) * unit.x + Mathf.Cos(Mathf.PI / 15 * i) * unit.y));
            if (!IsDirSafe(pos, nowDir))
            {
                continue;
            }
            var coParams = GetCollisionParams(pos, nowDir);
            Tuple<float, float> timeRange = GetTimeRange(coParams);
            if (timeRange.Item2 > max)
            {
                furParams = GetFurthestParams(coParams);
                furDir = nowDir;
            }
            min = timeRange.Item1 < min ? timeRange.Item1 : min;
            max = timeRange.Item2 > max ? timeRange.Item2 : max;
        }
        furthestParams = new Tuple<Vector2, int, float, float>(furDir, furParams.Item1, furParams.Item2, furParams.Item3);
        return new Tuple<float, float>(min, max);
    }

    public bool IsResetValuable()
    {
        if (GetOptimalDirection().Item2 > GetDisRange(collisionParams).Item2 * 1.2f)
        {
            return true;
        }
        return false;
    }

    //utils

    private Tuple<bool, Vector2, int, float, float> ResetReachable(Vector2 target, Vector2 pos, float time)
    {
        Tuple<bool, Vector2, int, float, float> wrong = new Tuple<bool, Vector2, int, float, float>(false, new Vector2(0, 0), 0, 0, 0);
        float dis = (target - pos).magnitude;
        if (time * GetAvgSpeed() / globalConfiguration.MAX_TRANS_GAIN <= dis)
        { // use line
            Vector2 dir = (target - pos).normalized;
            if (IsDirSafe(pos, dir))
            {
                float mindis = 100000f;
                foreach (var polygon in redirectionManager.polygons)
                {
                    for (int k = 0; k < polygon.Count; k++)
                    {
                        var p = polygon[k];
                        var q = polygon[(k + 1) % polygon.Count];
                        float tmpDis = LineCollisionDis(pos, dir, p, q);
                        mindis = tmpDis < mindis ? tmpDis : mindis;
                    }
                }
                if (mindis < dis)
                {
                    return wrong;
                }
                else
                {
                    float gt = GetAvgSpeed() / dis * time;
                    return new Tuple<bool, Vector2, int, float, float>(true, dir, 0, 100000, gt);
                }
            }
            else
            {
                return wrong;
            }
        }
        else
        {
            float radius = globalConfiguration.CURVATURE_RADIUS;
            float angle = (float)Mathf.Asin(dis / 2 / radius);
            float curDis = angle * radius * 2;
            Vector2 dir = (target - pos).normalized;
            Vector2 dir1 = Rotate(dir, angle);
            Vector2 dir2 = Rotate(dir, -angle);
            if (IsDirSafe(pos, dir1))
            {
                float mindis = 100000f;
                Vector2 rotate_radius = radius * dir1;
                rotate_radius = new Vector2(-rotate_radius.y, rotate_radius.x);
                Vector2 center = pos + rotate_radius; //center of the circle
                foreach (var polygon in redirectionManager.polygons)
                {
                    for (int k = 0; k < polygon.Count; k++)
                    {
                        var p = polygon[k];
                        var q = polygon[(k + 1) % polygon.Count];
                        float tmpDis = CircleCollisionDis(pos, dir1, p, q, radius, center);
                        mindis = tmpDis < mindis ? tmpDis : mindis;
                    }
                }
                if (mindis < curDis)
                {
                    return wrong;
                }
                else
                {
                    float gt = GetAvgSpeed() / curDis * time;
                    return new Tuple<bool, Vector2, int, float, float>(true, dir1, 1, radius, gt);
                }
            }
            if (IsDirSafe(pos, dir2))
            {
                float mindis = 100000f;
                Vector2 rotate_radius = radius * dir2;
                rotate_radius = new Vector2(rotate_radius.y, -rotate_radius.x);
                Vector2 center = pos + rotate_radius;//center of the circle
                foreach (var polygon in redirectionManager.polygons)
                {
                    for (int k = 0; k < polygon.Count; k++)
                    {
                        var p = polygon[k];
                        var q = polygon[(k + 1) % polygon.Count];
                        float tmpDis = CircleCollisionDis(pos, dir2, p, q, radius, center);
                        mindis = tmpDis < mindis ? tmpDis : mindis;
                    }
                }
                if (mindis < curDis)
                {
                    return wrong;
                }
                else
                {
                    float gt = GetAvgSpeed() / curDis * time;
                    return new Tuple<bool, Vector2, int, float, float>(true, dir2, 2, radius, gt);
                }
            }

            return wrong;
        }
    }

    private Vector2 Rotate(Vector2 ori, float angle)
    { // angle is in radian
        return new Vector2((float)(Mathf.Cos(angle) * ori.x - Mathf.Sin(angle) * ori.y), (float)(Mathf.Sin(angle) * ori.x + Mathf.Cos(angle) * ori.y));
    }

    private Tuple<float, float> second_circle(float x, float y, float OI_y)
    {
        float D = (y - OI_y) * (y - OI_y) + x * x;
        float DELTA = (D - OI_y * OI_y) * (9 * OI_y * OI_y - D);
        if (DELTA < 0)
            return new Tuple<float, float>(float.NaN, float.NaN);
        float OII_x = (1f / 2 + 3 * OI_y * OI_y / (2 * D)) * x + OI_y / Mathf.Abs(OI_y) * (Mathf.Sqrt(DELTA) / (2 * D)) * (y - OI_y);
        float OII_y = OI_y + (1f / 2 + 3 * OI_y * OI_y / (2 * D)) * (y - OI_y) - OI_y / Mathf.Abs(OI_y) * (Mathf.Sqrt(DELTA) /
                                                                                            (2 * D)) * x;

        return new Tuple<float, float>(OII_x, OII_y);
    }

    private Tuple<Complex32, Complex32, Complex32, Complex32> first_circle_standard(float x1, float y1, float x2, float y2)
    {
        float a = 2 * (x1 - x2);
        float b = 2 * (y1 - y2);
        float c = (x2 + x1) * (x2 - x1) + (y2 + y1) * (y2 - y1);

        float A = (1f / 2) * b * b;

        float K2 = b * c + 2 * a * b * x1 - 2 * a * a * y1;
        float B = -K2;

        float K3 = c * c + 4 * a * c * x1 + 2 * a * a * x1 * x1 - 2 * b * b * x1 * x1 + 2 * b * c * y1 + 6 * a * b * x1 * y1 - a * a * y1 * y1 + b * b * y1 * y1;
        float C = (1f / 2) * K3;

        float K4 = b * c * x1 * x1 + a * b * x1 * x1 * x1 - 2 * c * c * y1 - 2 * a * c * x1 * y1 - a * a * x1 * x1 * y1 - b * c * y1 * y1 + a * b * x1 * y1 * y1 - a * a * y1 * y1 * y1;
        float D = (1f / 2) * K4;

        float K5 = 4 * c * c * x1 * x1 + 4 * a * c * x1 * x1 * x1 + a * a * x1 * x1 * x1 * x1 + b * b * x1 * x1 * x1 * x1 + 4 * b * c * x1 * x1 * y1 + 4 * c * c * y1 * y1 + 4 * a * c * x1 * y1 * y1 + 2 * a * a * x1 * x1 * y1 * y1 + 2 * b * b * x1 * x1 * y1 * y1 + 4 * b * c * y1 * y1 * y1 + a * a * y1 * y1 * y1 * y1 + b * b * y1 * y1 * y1 * y1;
        float E = (1f / 8) * K5;

        float delta1 = C * C - 3 * B * D + 12 * A * E;
        float delta2 = 2 * C * C * C - 9 * B * C * D + 27 * A * D * D + 27 * B * B * E - 72 * A * C * E;

        Complex32 delta = (Complex32.Pow(2, (1f / 3)) * delta1) / (3 * A * Complex32.Pow((delta2 + Complex32.Sqrt(-4 * delta1 * delta1 * delta1 + delta2 * delta2)), (1f / 3))) +
                Complex32.Pow((delta2 + Complex32.Sqrt(-4 * delta1 * delta1 * delta1 + delta2 * delta2)), (1f / 3)) / (3 * Complex32.Pow(2, (1f / 3)) * A);

        Complex32 OI_y1 = -B / (4 * A)
                - (1f / 2) * Complex32.Sqrt(B * B / (4 * A * A) - 2 * C / (3 * A) + delta)
                - (1f / 2) * Complex32.Sqrt(B * B / (2 * A * A) - 4 * C / (3 * A) - delta
                - (-(B * B * B) / (A * A * A) + 4 * B * C / (A * A) - 8 * D / A)
                / (4 * Complex32.Sqrt(B * B / (4 * A * A) - 2 * C / (3 * A) + delta))
                );

        Complex32 OI_y2 = -B / (4 * A)
                - (1f / 2) * Complex32.Sqrt(B * B / (4 * A * A) - 2 * C / (3 * A) + delta)
                + (1f / 2) * Complex32.Sqrt(B * B / (2 * A * A) - 4 * C / (3 * A) - delta
                - (-(B * B * B) / (A * A * A) + 4 * B * C / (A * A) - 8 * D / A)
                / (4 * Complex32.Sqrt(B * B / (4 * A * A) - 2 * C / (3 * A) + delta))
                );

        Complex32 OI_y3 = -B / (4 * A)
                + (1f / 2) * Complex32.Sqrt(B * B / (4 * A * A) - 2 * C / (3 * A) + delta)
                - (1f / 2) * Complex32.Sqrt(B * B / (2 * A * A) - 4 * C / (3 * A) - delta
                + (-(B * B * B) / (A * A * A) + 4 * B * C / (A * A) - 8 * D / A)
                / (4 * Complex32.Sqrt(B * B / (4 * A * A) - 2 * C / (3 * A) + delta))
                );

        Complex32 OI_y4 = -B / (4 * A)
                + (1f / 2) * Complex32.Sqrt(B * B / (4 * A * A) - 2 * C / (3 * A) + delta)
                + (1f / 2) * Complex32.Sqrt(B * B / (2 * A * A) - 4 * C / (3 * A) - delta
                + (-(B * B * B) / (A * A * A) + 4 * B * C / (A * A) - 8 * D / A)
                / (4 * Complex32.Sqrt(B * B / (4 * A * A) - 2 * C / (3 * A) + delta))
                );

        return new Tuple<Complex32, Complex32, Complex32, Complex32>(OI_y1, OI_y2, OI_y3, OI_y4);
    }

    private Tuple<float, float, float, float> segment_arc_intersection(float x1, float y1, float x2, float y2, float Ox, float Oy, float R, Vector2 A, Vector2 B, float sign)
    {
        Tuple<float, float, float, float> NONE = new Tuple<float, float, float, float>(float.NaN, float.NaN, float.NaN, float.NaN);
        float a = (y2 - y1);
        float b = (x1 - x2);
        float c = (x2 - x1) * y1 + (y1 - y2) * x1;

        if ((-Mathf.Pow((c + a * Ox + b * Oy), 2) + (a * a + b * b) * R * R) < 0)
            return NONE;

        float delta = Mathf.Sqrt((-Mathf.Pow((c + a * Ox + b * Oy), 2) + (a * a + b * b) * R * R));

        float Jx1 = (b * b * Ox - a * (c + b * Oy) - b * delta) / (a * a + b * b);
        float Jy1 = (-b * (c + a * Ox) + a * a * Oy + a * delta) / (a * a + b * b);

        float Jx2 = (b * b * Ox - a * (c + b * Oy) + b * delta) / (a * a + b * b);
        float Jy2 = (-b * (c + a * Ox) + a * a * Oy - a * delta) / (a * a + b * b);

        float xmin = Mathf.Min(x1, x2);
        float xmax = Mathf.Max(x1, x2);
        float ymin = Mathf.Min(y1, y2);
        float ymax = Mathf.Max(y1, y2);

        if (Jx1 < xmin - 0.01 || Jx1 > xmax + 0.01 || Jy1 < ymin - 0.01 || Jy1 > ymax + 0.01)
            Jx1 = Jy1 = float.NaN;
        if (Jx2 < xmin - 0.01 || Jx2 > xmax + 0.01 || Jy2 < ymin - 0.01 || Jy2 > ymax + 0.01)
            Jx2 = Jy2 = float.NaN;

        if (!float.IsNaN(Jx1) && !float.IsNaN(Jy1))
            if (Vector3.Dot(Vector3.Cross(new Vector3(Jx1 - A[0], Jy1 - A[1], 0), new Vector3(B[0] - Jx1, B[1] - Jy1, 0)),
                    new Vector3(0, 0, sign)) < 0)
                Jx1 = Jy1 = float.NaN;

        if (!float.IsNaN(Jx2) && !float.IsNaN(Jy2))
            if (Vector3.Dot(Vector3.Cross(new Vector3(Jx2 - A[0], Jy2 - A[1], 0), new Vector3(B[0] - Jx2, B[1] - Jy2, 0)),
                    new Vector3(0, 0, sign)) < 0)
                Jx2 = Jy2 = float.NaN;

        return new Tuple<float, float, float, float>(Jx1, Jy1, Jx2, Jy2);
    }

    private float occluded_OI_y_obs(float x, float y, float x_obs, float y_obs)
    {
        float OI_x_obs = 0;

        float OI_y_obs = (x_obs * x_obs + y_obs * y_obs) / (2 * y_obs);

        var OII_p_obs = second_circle(x, y, OI_y_obs);
        float OII_x_obs = OII_p_obs.Item1;
        float OII_y_obs = OII_p_obs.Item2;
        if (!float.IsNaN(OII_x_obs))
        {
            float qie_x_obs = (OII_x_obs + OI_x_obs) / 2;
            float qie_y_obs = (OII_y_obs + OI_y_obs) / 2;

            if (qie_x_obs >= 0 && (!(Vector3.Dot(Vector3.Cross(new Vector3(x_obs, y_obs, 0), new Vector3(qie_x_obs, qie_y_obs, 0)),
                        new Vector3(0, 0, OI_y_obs / Mathf.Abs(OI_y_obs))) < 0)))
                return OI_y_obs;
        }

        var OI_p_obs = first_circle_standard(x, y, x_obs, y_obs);
        Complex32 OI_y1_obs = OI_p_obs.Item1;
        Complex32 OI_y2_obs = OI_p_obs.Item2;
        Complex32 OI_y3_obs = OI_p_obs.Item3;
        Complex32 OI_y4_obs = OI_p_obs.Item4;

        var OI_y_obs_list = new List<Complex32> { OI_y1_obs, OI_y2_obs, OI_y3_obs, OI_y4_obs };

        for (int i = 0; i < 4; i++)
        {
            OI_y_obs = OI_y_obs_list[i].Real;

            OII_p_obs = second_circle(x, y, OI_y_obs);
            OII_x_obs = OII_p_obs.Item1;
            OII_y_obs = OII_p_obs.Item2;
            if (float.IsNaN(OII_x_obs))
                continue;

            if (Mathf.Abs((x_obs - OII_x_obs) * (x_obs - OII_x_obs) + (y_obs - OII_y_obs) *
                (y_obs - OII_y_obs) - OI_y_obs * OI_y_obs) > 0.01)
                continue;

            float qie_x_obs = (OII_x_obs + OI_x_obs) / 2;
            float qie_y_obs = (OII_y_obs + OI_y_obs) / 2;

            if (qie_x_obs < 0)  
                continue;

            if (Vector3.Dot(
                    Vector3.Cross(new Vector3(x_obs - qie_x_obs, y_obs - qie_y_obs, 0), new Vector3(
                        x - qie_x_obs, y - qie_y_obs, 0)), new Vector3(0, 0, -OI_y_obs / Mathf.Abs(OI_y_obs))) < 0)
                continue;

            return OI_y_obs;
        }

        return float.NaN;
    }

    private Tuple<float, float, float, float> radius_avaliable_domain(float x_s, float y_s, float theta_s, float x_e, float y_e)
    {
        float theta_s_radian = theta_s * Mathf.PI / 180;
        float cos_theta = Mathf.Cos(theta_s_radian);
        float sin_theta = Mathf.Sin(theta_s_radian);

        float x = cos_theta * (x_e - x_s) + sin_theta * (y_e - y_s);
        float y = -sin_theta * (x_e - x_s) + cos_theta * (y_e - y_s);

        if (Mathf.Abs(y) < 0.01)
            y = 0.01f;

        float R_max = Mathf.Abs((x * x + y * y) / (2 * y));
        float R_posi_min = (-y + Mathf.Sqrt(8 * x * x + 9 * y * y)) / 8;

        float R_minus_max = (-y - Mathf.Sqrt(8 * x * x + 9 * y * y)) / 8;

        return new Tuple<float, float, float, float>(-R_max, R_minus_max, R_posi_min, R_max);
    }

    class TupleXComparer : IComparer<Vector3>
    {
        public int Compare(Vector3 p1, Vector3 p2)
        {
            return p1[0].CompareTo(p2[0]);
        }
    }
    class TupleYComparer : IComparer<Vector3>
    {
        public int Compare(Vector3 p1, Vector3 p2)
        {
            return p1[1].CompareTo(p2[1]);
        }
    }


    private List<Tuple<float, float>> interval_merge(List<Tuple<float, float>> A, List<Tuple<float, float>> B)
    {
        var res = new List<Tuple<float, float>>();
        int i = 0;
        int j = 0;
        while (i < A.Count && j < B.Count)
        {
            float l = Mathf.Max(A[i].Item1, B[j].Item1);
            float r = Mathf.Min(A[i].Item2, B[j].Item2);
            if (l <= r)
                res.Add(new Tuple<float, float>(l, r));

            if (A[i].Item2 < B[j].Item2)
                i += 1;
            else
                j += 1;
        }

        return res;
    }

    private List<Tuple<float, float>> run(float x_s, float y_s, float theta_s, float x_e, float y_e, List<Tuple<Vector2, Vector2>> obstacle_edges)
    {
        var Total_Feasible_range = new List<Tuple<float, float>>();
        float theta_s_radian = theta_s * Mathf.PI / 180;
        float cos_theta = Mathf.Cos(theta_s_radian);
        float sin_theta = Mathf.Sin(theta_s_radian);

        float x = cos_theta * (x_e - x_s) + sin_theta * (y_e - y_s);
        float y = -sin_theta * (x_e - x_s) + cos_theta * (y_e - y_s);

        if (x <= 0)
        {
            // Debug.Log("Backside Waypoint");
            return Total_Feasible_range;
        }

        var domain = radius_avaliable_domain(
            x_s, y_s, theta_s, x_e, y_e);
        float minus_min_value = domain.Item1;
        float minus_max_value = domain.Item2;
        float posi_min_value = domain.Item3;
        float posi_max_value = domain.Item4;

        if (minus_max_value < minus_min_value && posi_min_value > posi_max_value)
        {
            // Debug.Log("Unreachable");
            return Total_Feasible_range;
        }

        Total_Feasible_range = new List<Tuple<float, float>>() { new Tuple<float, float>(minus_min_value, minus_max_value), new Tuple<float, float>(posi_min_value, posi_max_value) };
        var Threshold_range = new List<Tuple<float, float>>() { new Tuple<float, float>(-10000, -globalConfiguration.CURVATURE_RADIUS), new Tuple<float, float>(globalConfiguration.CURVATURE_RADIUS, 10000) };
        Total_Feasible_range = interval_merge(Total_Feasible_range, Threshold_range);
        if (Total_Feasible_range.Count == 0)
        {
            return Total_Feasible_range;
        }

        for (int i = 0; i < obstacle_edges.Count; i++)
        {
            var Edge_Feasible_range_clean = edge_blocks_range(x_s, y_s, theta_s, x_e, y_e, obstacle_edges[i], minus_min_value, minus_max_value, posi_min_value, posi_max_value);

            Total_Feasible_range = interval_merge(Total_Feasible_range, Edge_Feasible_range_clean);
        }

        var Total_Feasible_range_clean = new List<Tuple<float, float>>();
        for (int i = 0; i < Total_Feasible_range.Count; i++)
        {
            float scale = Total_Feasible_range[i].Item2 / Total_Feasible_range[i].Item1;
            if (scale > 0.98 && scale < 1.02)
                continue;

            Total_Feasible_range_clean.Add(new Tuple<float, float>(Total_Feasible_range[i].Item1, Total_Feasible_range[i].Item2));
        }
        return Total_Feasible_range_clean;
    }

    private List<Tuple<float, float>> edge_blocks_range(float x_s, float y_s, float theta_s, float x_e, float y_e, Tuple<Vector2, Vector2> edge, float minus_min_value, float minus_max_value, float posi_min_value, float posi_max_value)
    {
        var Edge_Feasible_range = new List<Tuple<float, float>>() { new Tuple<float, float>(minus_min_value, minus_max_value), new Tuple<float, float>(posi_min_value, posi_max_value) };
        float theta_s_radian = theta_s * Mathf.PI / 180;
        float cos_theta = Mathf.Cos(theta_s_radian);
        float sin_theta = Mathf.Sin(theta_s_radian);

        float x = cos_theta * (x_e - x_s) + sin_theta * (y_e - y_s);
        float y = -sin_theta * (x_e - x_s) + cos_theta * (y_e - y_s);

        float x_obs1 = cos_theta * (edge.Item1[0] - x_s) + sin_theta * (edge.Item1[1] - y_s);
        float y_obs1 = -sin_theta * (edge.Item1[0] - x_s) + cos_theta * (edge.Item1[1] - y_s);

        float x_obs2 = cos_theta * (edge.Item2[0] - x_s) + sin_theta * (edge.Item2[1] - y_s);
        float y_obs2 = -sin_theta * (edge.Item2[0] - x_s) + cos_theta * (edge.Item2[1] - y_s);

        var intersection_list = new List<Vector3>();
        float OI_x_obs = 0;
        float OI_y_obs = minus_max_value + 0.01f * minus_max_value / Mathf.Abs(minus_max_value);
        var OII_p_obs = second_circle(x, y, OI_y_obs);
        float OII_x_obs = OII_p_obs.Item1;
        float OII_y_obs = OII_p_obs.Item2;
        float pathR = Mathf.Abs(OI_y_obs);
        float Jx1, Jy1, Jx2, Jy2;
        if (!float.IsNaN(OII_x_obs))
        {
            float qie_x_obs = (OII_x_obs + OI_x_obs) / 2;
            float qie_y_obs = (OII_y_obs + OI_y_obs) / 2;
            var res1 = segment_arc_intersection(x_obs1, y_obs1, x_obs2, y_obs2, OI_x_obs, OI_y_obs, pathR, new Vector2(0, 0),
                                                        new Vector2(qie_x_obs, qie_y_obs), OI_y_obs / Mathf.Abs(OI_y_obs));
            Jx1 = res1.Item1;
            Jy1 = res1.Item2;
            Jx2 = res1.Item3;
            Jy2 = res1.Item4;
            if (!float.IsNaN(Jx1) && !float.IsNaN(Jy1))
                intersection_list.Add(new Vector3(Jx1, Jy1, 0));
            if (!float.IsNaN(Jx2) && !float.IsNaN(Jy2))
                intersection_list.Add(new Vector3(Jx2, Jy2, 0));

            var res2 = segment_arc_intersection(x_obs1, y_obs1, x_obs2, y_obs2, OII_x_obs, OII_y_obs, pathR,
                                                        new Vector2(qie_x_obs, qie_y_obs), new Vector2(x, y), -OI_y_obs / Mathf.Abs(OI_y_obs));
            Jx1 = res2.Item1;
            Jy1 = res2.Item2;
            Jx2 = res2.Item3;
            Jy2 = res2.Item4;
            if (!float.IsNaN(Jx1) && !float.IsNaN(Jy1))
                intersection_list.Add(new Vector3(Jx1, Jy1, 0));
            if (!float.IsNaN(Jx2) && !float.IsNaN(Jy2))
                intersection_list.Add(new Vector3(Jx2, Jy2, 0));
        }

        OI_x_obs = 0;
        OI_y_obs = posi_min_value + 0.01f * posi_min_value / Mathf.Abs(posi_min_value);
        OII_p_obs = second_circle(x, y, OI_y_obs);
        OII_x_obs = OII_p_obs.Item1;
        OII_y_obs = OII_p_obs.Item2;
        pathR = Mathf.Abs(OI_y_obs);
        if (!float.IsNaN(OII_x_obs))
        {
            float qie_x_obs = (OII_x_obs + OI_x_obs) / 2;
            float qie_y_obs = (OII_y_obs + OI_y_obs) / 2;

            var res1 = segment_arc_intersection(x_obs1, y_obs1, x_obs2, y_obs2, OI_x_obs, OI_y_obs, pathR, new Vector2(0, 0),
                                                        new Vector2(qie_x_obs, qie_y_obs), OI_y_obs / Mathf.Abs(OI_y_obs));
            Jx1 = res1.Item1;
            Jy1 = res1.Item2;
            Jx2 = res1.Item3;
            Jy2 = res1.Item4;
            if (!float.IsNaN(Jx1) && !float.IsNaN(Jy1))
                intersection_list.Add(new Vector3(Jx1, Jy1, 0));
            if (!float.IsNaN(Jx2) && !float.IsNaN(Jy2))
                intersection_list.Add(new Vector3(Jx2, Jy2, 0));

            var res2 = segment_arc_intersection(x_obs1, y_obs1, x_obs2, y_obs2, OII_x_obs, OII_y_obs, pathR,
                                                        new Vector2(qie_x_obs, qie_y_obs), new Vector2(x, y), -OI_y_obs / Mathf.Abs(OI_y_obs));
            Jx1 = res2.Item1;
            Jy1 = res2.Item2;
            Jx2 = res2.Item3;
            Jy2 = res2.Item4;
            if (!float.IsNaN(Jx1) && !float.IsNaN(Jy1))
                intersection_list.Add(new Vector3(Jx1, Jy1, 0));
            if (!float.IsNaN(Jx2) && !float.IsNaN(Jy2))
                intersection_list.Add(new Vector3(Jx2, Jy2, 0));
        }

        if (Mathf.Abs(y) < 0.01f)
            y = 0.01f;
        float OI_x_mid = 0;
        float OI_y_mid = (x * x + y * y) / (2 * y);
        pathR = Mathf.Abs(OI_y_mid);
        var res3 = segment_arc_intersection(x_obs1, y_obs1, x_obs2, y_obs2, OI_x_mid, OI_y_mid, pathR, new Vector2(0, 0),
                                                    new Vector2(x, y), OI_y_mid / Mathf.Abs(OI_y_mid));
        Jx1 = res3.Item1;
        Jy1 = res3.Item2;
        Jx2 = res3.Item3;
        Jy2 = res3.Item4;
        if (!float.IsNaN(Jx1) && !float.IsNaN(Jy1))
            intersection_list.Add(new Vector3(Jx1, Jy1, 1));
        if (!float.IsNaN(Jx2) && !float.IsNaN(Jy2))
            intersection_list.Add(new Vector3(Jx2, Jy2, 1));

        intersection_list.Add(new Vector3(x_obs1, y_obs1, 0));
        intersection_list.Add(new Vector3(x_obs2, y_obs2, 0));

        if (Mathf.Abs(x_obs2 - x_obs1) > Mathf.Abs(y_obs2 - y_obs1))
            intersection_list.Sort(new TupleXComparer());
        else
            intersection_list.Sort(new TupleYComparer());

        for (int i = 0; i < intersection_list.Count - 1; i++)
        {
            Vector3 ed = intersection_list[i + 1] - intersection_list[i];
            if (ed.x * ed.x + ed.y * ed.y < 1e-4f)
            {
                continue;
            }
            float bad_OI_y_min = 10000;
            float bad_OI_y_max = -10000;

            // 按照1米的长度来采样
            int distance = (int)(Mathf.Sqrt(Mathf.Pow(intersection_list[i + 1][0] - intersection_list[i][0], 2) +
                                Mathf.Pow(intersection_list[i + 1][1] - intersection_list[i][1], 2))) + 1;
            int step = 1000 / distance;
            float x_obs_k, y_obs_k, bad_OI_y_obs_k;
            int k = 1;
            for (; k < 999; k += step)
            {
                x_obs_k = k * (intersection_list[i + 1][0] - intersection_list[i][0]) / 1000 + intersection_list[i][0];
                y_obs_k = k * (intersection_list[i + 1][1] - intersection_list[i][1]) / 1000 + intersection_list[i][1];
                bad_OI_y_obs_k = occluded_OI_y_obs(x, y, x_obs_k, y_obs_k);
                if (float.IsNaN(bad_OI_y_obs_k))
                    continue;
                //障碍物边与中间路径交点上的障碍物点，阻挡的路径，手动修正其bad_OI_y_obs_k为minus_min_value或posi_max_value
                if (k == 1 && intersection_list[i][2] == 1)
                {
                    if (bad_OI_y_obs_k < 0)
                        bad_OI_y_obs_k = minus_min_value;
                    else if (bad_OI_y_obs_k > 0)
                        bad_OI_y_obs_k = posi_max_value;
                }
                if (bad_OI_y_obs_k < bad_OI_y_min)
                    bad_OI_y_min = bad_OI_y_obs_k;
                if (bad_OI_y_obs_k > bad_OI_y_max)
                    bad_OI_y_max = bad_OI_y_obs_k;
            }

            k = 999;
            x_obs_k = k * (intersection_list[i + 1][0] - intersection_list[i][0]) / 1000 + intersection_list[i][0];
            y_obs_k = k * (intersection_list[i + 1][1] - intersection_list[i][1]) / 1000 + intersection_list[i][1];
            bad_OI_y_obs_k = occluded_OI_y_obs(x, y, x_obs_k, y_obs_k);
            if (!float.IsNaN(bad_OI_y_obs_k))
            {//障碍物边与中间路径交点上的障碍物点，阻挡的路径，手动修正其bad_OI_y_obs_k为minus_min_value或posi_max_value
                if (intersection_list[i + 1][2] == 1)
                {
                    if (bad_OI_y_obs_k < 0)
                        bad_OI_y_obs_k = minus_min_value;
                    else if (bad_OI_y_obs_k > 0)
                        bad_OI_y_obs_k = posi_max_value;
                }
                if (bad_OI_y_obs_k < bad_OI_y_min)
                    bad_OI_y_min = bad_OI_y_obs_k;
                if (bad_OI_y_obs_k > bad_OI_y_max)
                    bad_OI_y_max = bad_OI_y_obs_k;
            }

            if (bad_OI_y_min >= bad_OI_y_max)
                continue;

            var occluded_interval = new Tuple<float, float>(bad_OI_y_min, bad_OI_y_max);

            // 分情况判断本段阻挡后的范围
            var occluded_Feasible_range = new List<Tuple<float, float>>() { new Tuple<float, float>(minus_min_value, minus_max_value), new Tuple<float, float>(posi_min_value, posi_max_value) };
            if (bad_OI_y_min < 0 && bad_OI_y_max < 0)
                occluded_Feasible_range = new List<Tuple<float, float>>(){new Tuple<float, float>(minus_min_value, bad_OI_y_min),new Tuple<float, float> (bad_OI_y_max, minus_max_value),
                                        new Tuple<float, float>(posi_min_value, posi_max_value)};
            else if (bad_OI_y_min > 0 && bad_OI_y_max > 0)
                occluded_Feasible_range = new List<Tuple<float, float>>(){new Tuple<float, float>(minus_min_value, minus_max_value),new Tuple<float, float> (posi_min_value, bad_OI_y_min),
                                        new Tuple<float,float>(bad_OI_y_max, posi_max_value)};
            else
            {
                // Debug.Log("【【【Error!!!】】】" + new Tuple<float, float>(bad_OI_y_min, bad_OI_y_max) + " " + x + " " + y + " " +
                //  intersection_list[i + 1][0] + " " + intersection_list[i + 1][1] + " " + intersection_list[i][0] + " " + intersection_list[i][1]);
            }

            // 求交集
            Edge_Feasible_range = interval_merge(Edge_Feasible_range, occluded_Feasible_range);
        }

        var Edge_Feasible_range_clean = new List<Tuple<float, float>>();
        for (int i = 0; i < Edge_Feasible_range.Count; i++)
        {
            float scale = Edge_Feasible_range[i].Item2 / Edge_Feasible_range[i].Item1;
            if (scale > 0.98 && scale < 1.02)
                continue;
            Edge_Feasible_range_clean.Add(new Tuple<float, float>(Edge_Feasible_range[i].Item1, Edge_Feasible_range[i].Item2));
        }

        return Edge_Feasible_range_clean;
    }

    private Tuple<Vector2, Vector2> GetTurnPoints(Vector2 target, Vector2 pos, Vector2 dir, float R)
    { // return: left turnpoint, right turnpoint
        Vector2 diff = target - pos;
        Vector2 orthoDir = new Vector2(-dir.y, dir.x);
        float x = diff.x * dir.x + diff.y * dir.y;
        float y = diff.x * orthoDir.x + diff.y * orthoDir.y;
        float D1s = x * x + (y - R) * (y - R);
        float D2s = x * x + (y + R) * (y + R);
        float delta1 = Mathf.Sqrt((D1s - R * R) * (9 * R * R - D1s));
        float delta2 = Mathf.Sqrt((D2s - R * R) * (9 * R * R - D2s));
        float Ox1 = (0.5f + 1.5f * R * R / D1s) * x - 0.5f * delta1 / D1s * (R - y);
        float Ox2 = (0.5f + 1.5f * R * R / D2s) * x - 0.5f * delta2 / D2s * (R + y);
        float Oy1 = -(0.5f + 1.5f * R * R / D1s) * (R - y) - 0.5f * delta1 / D1s * x + R;
        float Oy2 = (0.5f + 1.5f * R * R / D2s) * (R + y) + 0.5f * delta2 / D2s * x - R;
        float Tx1 = Ox1 / 2;
        float Tx2 = Ox2 / 2;
        float Ty1 = (Oy1 + R) / 2;
        float Ty2 = (Oy2 - R) / 2;
        Vector2 p1 = pos + dir * Tx1 + orthoDir * Ty1;
        Vector2 p2 = pos + dir * Tx2 + orthoDir * Ty2;
        return new Tuple<Vector2, Vector2>(p1, p2);
    }

    private bool SWayCollision(Vector2 pos, Vector2 dir, Vector2 p1, Vector2 p1Dir, Vector2 center1, Vector2 center2, float dis1, float dis2, float R)
    {
        float coldis;
        foreach (var polygon in redirectionManager.polygons)
        {
            for (int k = 0; k < polygon.Count; k++)
            {
                var p = polygon[k];
                var q = polygon[(k + 1) % polygon.Count];
                Vector2 ortho_dir = new Vector2((p - q).y, (q - p).x).normalized;// point to line, orthogonal to line
                if (Mathf.Sign(Cross(pos + ortho_dir * 1000f - q, p - q)) == Mathf.Sign(Cross(pos - q, p - q)))
                {
                    ortho_dir = -ortho_dir;
                }
                coldis = CircleCollisionDis(pos, dir, p, q, R, center1);
                if (coldis < dis1)
                {
                    return false;
                }
                coldis = CircleCollisionDis(p1, p1Dir, p, q, R, center2);
                if (coldis < dis2)
                {
                    return false;
                }
            }
        }
        return true;
    }

    private Tuple<bool, float> ChooseDis(float minGtDis, float midGtDis, float maxGtDis, float minGcDis, float maxGcDis)
    {
        if (minGcDis > maxGtDis || maxGcDis < minGtDis)
        {
            return new Tuple<bool, float>(false, 0);
        }
        if (minGcDis > midGtDis)
        {
            return new Tuple<bool, float>(true, minGcDis);
        }
        else if (maxGcDis < midGtDis)
        {
            return new Tuple<bool, float>(true, maxGcDis);
        }
        else
        {
            return new Tuple<bool, float>(true, midGtDis);
            // if (Mathf.Abs(minGcDis - midGtDis) < Mathf.Abs(maxGcDis - minGtDis))
            // {
            //     return new Tuple<bool, float>(true, minGcDis);
            // }
            // return new Tuple<bool, float>(true, maxGcDis);
        }
    }

    private float SwayPathLength(float x, float y, float R, int way)
    {
        if (way == 1)
        {
            float D1s = x * x + (y - R) * (y - R);
            float delta1 = Mathf.Sqrt(Mathf.Max(D1s - R * R, 0) * (9 * R * R - D1s));
            float Ox1 = (0.5f + 1.5f * R * R / D1s) * x - 0.5f * delta1 / D1s * (R - y);
            float Oy1 = -(0.5f + 1.5f * R * R / D1s) * (R - y) - 0.5f * delta1 / D1s * x + R;
            float Tx1 = Ox1 / 2;
            float Ty1 = (Oy1 + R) / 2;
            return 2 * R * (Mathf.Asin(Mathf.Sqrt(Tx1 * Tx1 + Ty1 * Ty1) / (2 * R)) +
                Mathf.Asin(Mathf.Sqrt((x - Tx1) * (x - Tx1) + (y - Ty1) * (y - Ty1)) / (2 * R)));
        }
        else
        {
            float D2s = x * x + (y + R) * (y + R);
            float delta2 = Mathf.Sqrt(Mathf.Max(D2s - R * R, 0) * (9 * R * R - D2s));
            float Ox2 = (0.5f + 1.5f * R * R / D2s) * x - 0.5f * delta2 / D2s * (R + y);
            float Oy2 = (0.5f + 1.5f * R * R / D2s) * (R + y) + 0.5f * delta2 / D2s * x - R;
            float Tx2 = Ox2 / 2;
            float Ty2 = (Oy2 - R) / 2;
            return 2 * R * (Mathf.Asin(Mathf.Sqrt(Tx2 * Tx2 + Ty2 * Ty2) / (2 * R)) +
                Mathf.Asin(Mathf.Sqrt((x - Tx2) * (x - Tx2) + (y - Ty2) * (y - Ty2)) / (2 * R)));
        }
    }

    private Tuple<bool, int, float, float, bool, Vector2> SWayReachable(Vector2 target, Vector2 pos, Vector2 dir, float time)
    {// return: reachable, way(left1/right2), radius, gt, s-turn, turnPoint
        dir = dir.normalized;
        float thetaDir = Mathf.Atan2(dir.y, dir.x) * 180 / Mathf.PI;
        var REJECT = new Tuple<bool, int, float, float, bool, Vector2>(false, 0, 0, 0, false, Vector2.zero);
        float speed = GetAvgSpeed();
        float minGtDis = speed * time / globalConfiguration.MAX_TRANS_GAIN;
        float maxGtDis = speed * time / globalConfiguration.MIN_TRANS_GAIN;
        float midGtDis = (minGtDis + maxGtDis) / 2;
        minGtDis = minGtDis * STABILITYRATIO + midGtDis * (1 - STABILITYRATIO); // stablility smoothing
        maxGtDis = maxGtDis * STABILITYRATIO + midGtDis * (1 - STABILITYRATIO); // stablility smoothing
        float R = globalConfiguration.CURVATURE_RADIUS;

        float cos_theta = Mathf.Cos(thetaDir * Mathf.PI / 180);
        float sin_theta = Mathf.Sin(thetaDir * Mathf.PI / 180);
        float x = cos_theta * (target.x - pos.x) + sin_theta * (target.y - pos.y);
        float y = -sin_theta * (target.x - pos.x) + cos_theta * (target.y - pos.y);

        var runRes = run(pos.x, pos.y, thetaDir, target.x, target.y, redirectionManager.polygonEdges);
        if (runRes.Count == 0)
        {
            return REJECT;
        }
        // Debug.Log("judge");
        float targetR = 0;
        float targetGtDis = maxGtDis;

        for (int i = 0; i < runRes.Count; i++)
        {
            runRes[i] = new Tuple<float, float>(-runRes[i].Item2, -runRes[i].Item1);
        }

        foreach (var range in runRes)
        {
            float r1 = Mathf.Min(Mathf.Abs(range.Item1), Mathf.Abs(range.Item2));
            float r2 = Mathf.Max(Mathf.Abs(range.Item1), Mathf.Abs(range.Item2));
            float sign = Mathf.Sign(range.Item1);
            int way = range.Item1 < 0 ? 1 : 2;
            float l1 = SwayPathLength(x, y, r1, way);
            float l2 = SwayPathLength(x, y, r2, way);
            float lmin = Mathf.Min(l1, l2);
            float lmax = Mathf.Max(l1, l2);
            // Debug.Log(x + " " + y + " " + lmin + " " + lmax);
            if (lmin < maxGtDis && lmax > minGtDis)
            {
                if (lmin <= midGtDis && lmax >= midGtDis)
                {
                    targetGtDis = midGtDis;
                    float a = r1;
                    float b = r2;
                    float a_val = l1;
                    float b_val = l2;
                    float c = 0, c_val;
                    for (int k = 0; k < 10; k++)
                    {
                        c = (a + b) / 2;
                        c_val = SwayPathLength(x, y, c, way);
                        if (Mathf.Abs(c_val - midGtDis) < EPSILON)
                        {
                            break;
                        }
                        if ((c_val - midGtDis) * (a_val - midGtDis) < 0)
                        {
                            b = c;
                            b_val = c_val;
                        }
                        else
                        {
                            a = c;
                            a_val = c_val;
                        }
                    }
                    targetR = c * sign;
                    // Debug.Log("bisection");
                    break;
                }
                else if (lmin > midGtDis && (lmin - midGtDis < Mathf.Abs(targetGtDis - midGtDis)))
                {
                    targetGtDis = lmin;
                    targetR = (l1 == lmin ? r1 * sign : r2 * sign);
                }
                else if (lmax < midGtDis && (midGtDis - lmax < Mathf.Abs(targetGtDis - midGtDis)))
                {
                    targetGtDis = lmax;
                    targetR = (l1 == lmax ? r1 * sign : r2 * sign);
                }
            }
        }

        if (targetR == 0)
        {
            return REJECT;
        }

        var turnPoints = GetTurnPoints(target, pos, dir, Mathf.Abs(R));
        Vector2 turnPoint = targetR < 0 ? turnPoints.Item1 : turnPoints.Item2;
        int targetWay = targetR < 0 ? 1 : 2;
        float gt = speed * time / targetGtDis;

        return new Tuple<bool, int, float, float, bool, Vector2>(true, targetWay, Mathf.Abs(targetR), gt, true, turnPoint);

    }


    private class TupleDisComparer : IComparer<Tuple<int, float, float>>
    {
        public int Compare(Tuple<int, float, float> p1, Tuple<int, float, float> p2)
        {
            if (p1.Item3 > p2.Item3)
                return -1;
            if (p1.Item3 < p2.Item3)
            {
                return 1;
            }
            return 0;
        }
    }
    private bool InLineSection(Vector2 point, Vector2 a, Vector2 b)
    {// if point in section pq
        float buffer = globalConfiguration.RESET_TRIGGER_BUFFER;
        Vector2 p = a + (a - b).normalized * (buffer);
        Vector2 q = b + (b - a).normalized * (buffer);
        if (Mathf.Abs(p.x - q.x) > Mathf.Abs(p.y - q.y))
        {
            return (point.x <= p.x && point.x >= q.x) || (point.x <= q.x && point.x >= p.x);
        }
        return (point.y <= p.y && point.y >= q.y) || (point.y <= q.y && point.y >= p.y);
    }

    private bool IsDirSafe(Vector2 realPos, Vector2 dir)
    {// if this direction is away from obstacles
        float mindis = 100000f;
        Vector2 nearestPoint = new Vector2(0f, 0f);
        foreach (var polygon in redirectionManager.polygons)
        {
            for (int k = 0; k < polygon.Count; k++)
            {
                var p = polygon[k];
                var q = polygon[(k + 1) % polygon.Count];
                Vector2 nearestPos = Utilities.GetNearestPos(realPos, new List<Vector2> { p, q });
                if ((nearestPos - realPos).magnitude < mindis)
                {
                    mindis = (nearestPos - realPos).magnitude;
                    nearestPoint = nearestPos;
                }
            }
        }

        if (mindis > 0.1f)
        {
            return true;
        }
        else
        {
            if (Vector2.Dot(realPos - nearestPoint, dir) <= 0 || Vector2.Angle(realPos - nearestPoint, dir) >= 80)
            {
                return false;
            }
            return true;
        }
    }
    private float Dis(Vector2 ori, Vector2 a, Vector2 b) //dis from point ori to line ab
    {
        return Mathf.Abs(Cross(a - b, ori - a)) / (a - b).magnitude;
    }
    private float Cross(Vector2 a, Vector2 b)
    {
        return a.x * b.y - a.y * b.x;
    }
}
