#include "map_Maintain.hpp"

namespace lo_space
{
  MapMaintain::MapMaintain(double InCubeLength,
                           double InfilterSizeMapMin,
                           float IndetRange,
                           int InNumMatchPoint,
                           double InMaxMatchDist)
      : cubeLength(InCubeLength),
        filterSizeMapMin(InfilterSizeMapMin),
        DET_RANGE(IndetRange),
        NUM_MATCH_POINT(InNumMatchPoint),
        MAX_MATCH_DIST(InMaxMatchDist)
  {
    ikdTreePtr = std::make_shared<KD_TREE<PointType>>();
    localMapInitialized = false;
    treeInitialized = false;
    AddPointSizeLate = 0;

    // would't change
    MOV_THRESHOLD = 1.2f;
  }

  void MapMaintain::MapFovSegment(V3D &LidarPose)
  {
    cubeNeedRemove.clear();
    treeDeleteCounter = 0;
    V3D pos_LiD = LidarPose;

    // initiallize the Local map Box
    if (!localMapInitialized)
    {
      for (int i = 0; i < 3; i++)
      {
        localMapBox.vertex_min[i] = pos_LiD(i) - cubeLength / 2.0;
        localMapBox.vertex_max[i] = pos_LiD(i) + cubeLength / 2.0;
      }
      // change tab
      localMapInitialized = true;
      return;
    }

    // check whether need Move Box
    float distToMapEdge[3][2];
    bool needMove(false);
    for (int i = 0; i < 3; i++)
    {
      distToMapEdge[i][0] = fabs(pos_LiD(i) - localMapBox.vertex_min[i]);
      distToMapEdge[i][1] = fabs(pos_LiD(i) - localMapBox.vertex_max[i]);
      if (distToMapEdge[i][0] <= MOV_THRESHOLD * DET_RANGE ||
          distToMapEdge[i][1] <= MOV_THRESHOLD * DET_RANGE)
        needMove = true;
    }

    // when don't need Move
    if (!needMove)
      return;

    // when need Move
    BoxPointType newLocalMapBox, tmpBox;
    newLocalMapBox = localMapBox;
    float mov_dist = max((cubeLength - 2.0 * MOV_THRESHOLD * DET_RANGE) * 0.5 * 0.9,
                         double(DET_RANGE * (MOV_THRESHOLD - 1)));
    for (int i = 0; i < 3; i++)
    {
      tmpBox = localMapBox;
      if (distToMapEdge[i][0] <= MOV_THRESHOLD * DET_RANGE)
      {
        newLocalMapBox.vertex_max[i] -= mov_dist;
        newLocalMapBox.vertex_min[i] -= mov_dist;
        tmpBox.vertex_min[i] = localMapBox.vertex_max[i] - mov_dist;
        cubeNeedRemove.push_back(tmpBox);
      }
      else if (distToMapEdge[i][1] <= MOV_THRESHOLD * DET_RANGE)
      {
        newLocalMapBox.vertex_max[i] += mov_dist;
        newLocalMapBox.vertex_min[i] += mov_dist;
        tmpBox.vertex_min[i] = localMapBox.vertex_max[i] + mov_dist;
        cubeNeedRemove.push_back(tmpBox);
      }

      // update Map Box
      localMapBox = newLocalMapBox;

      // update ikd-tree
      PointVector points_history;
      ikdTreePtr->acquire_removed_points(points_history);
      if (cubeNeedRemove.size() > 0)
        treeDeleteCounter = ikdTreePtr->Delete_Point_Boxes(cubeNeedRemove);
    }
  }

  void MapMaintain::IkdTreeInitial(PointCloudPtr pointCloudInWorld)
  {
    // check whether tree need initialize
    if (ikdTreePtr->Root_Node != nullptr)
      return;

    // Initialize tree
    int pointSize = pointCloudInWorld->points.size();
    if (pointSize > 5)
    {
      ikdTreePtr->set_downsample_param(filterSizeMapMin);
      ikdTreePtr->Build(pointCloudInWorld->points);
    }

    // change tab
    treeInitialized = true;
  }

  bool MapMaintain::SearchPointInTree(PointType &pointInWorld,
                                      PointVector &NearPoints,
                                      DistVector &DistPoints)
  {
    ikdTreePtr->Nearest_Search(pointInWorld, NUM_MATCH_POINT, NearPoints, DistPoints);
    if (NearPoints.size() < NUM_MATCH_POINT)
      return false;
    if (DistPoints[NUM_MATCH_POINT - 1] > MAX_MATCH_DIST)
      return false;
    return true;
  }

  void MapMaintain::SearchPointCloudInTree(PointCloudPtr pointCloudInWorld)
  {
    PointCloudTemp = pointCloudInWorld;
    int pointNum = PointCloudTemp->size();
    NeartPointsVector.clear();
    NeartPointsVector.resize(pointNum);
    DistVectorVector.clear();
    DistVectorVector.resize(pointNum);
    for (int i = 0; i < pointNum; i++)
    {
      auto &pointNear = NeartPointsVector[i];
      auto &distPoints = DistVectorVector[i];
      if (SearchPointInTree(PointCloudTemp->points[i], pointNear, distPoints))
        continue;
      pointNear.clear();
      distPoints.clear();
    }
  }

  void MapMaintain::MapIncrement()
  {
    // organize container
    int pointNum = PointCloudTemp->size();
    PointVector PointToAdd;
    PointVector PointNoNeedDownsample;
    PointToAdd.reserve(pointNum);
    PointNoNeedDownsample.reserve(pointNum);

    // check which points need add
    for (int i = 0; i < pointNum; i++)
    {
      if (!NeartPointsVector[i].empty())
      {
        const PointVector &pointsNear = NeartPointsVector[i];
        bool needAdd = true;
        BoxPointType Box_of_Point;
        PointType downsample_result, mid_point;
        mid_point.x = floor(PointCloudTemp->points[i].x / filterSizeMapMin) * filterSizeMapMin + 0.5 * filterSizeMapMin;
        mid_point.y = floor(PointCloudTemp->points[i].y / filterSizeMapMin) * filterSizeMapMin + 0.5 * filterSizeMapMin;
        mid_point.z = floor(PointCloudTemp->points[i].z / filterSizeMapMin) * filterSizeMapMin + 0.5 * filterSizeMapMin;
        float dist = computDist(PointCloudTemp->points[i], mid_point);

        if (fabs(pointsNear[0].x - mid_point.x) > 0.5 * filterSizeMapMin &&
            fabs(pointsNear[0].y - mid_point.y) > 0.5 * filterSizeMapMin &&
            fabs(pointsNear[0].z - mid_point.z) > 0.5 * filterSizeMapMin)
        {
          PointNoNeedDownsample.push_back(PointCloudTemp->points[i]);
          continue;
        }

        for (int readd_i = 0; readd_i < NUM_MATCH_POINT; readd_i++)
        {
          if (pointsNear.size() < NUM_MATCH_POINT)
            break;
          if (computDist(pointsNear[readd_i], mid_point) <= dist)
          {
            needAdd = false;
            break;
          }
        }
        if (needAdd)
          PointToAdd.push_back(PointCloudTemp->points[i]);
      }
      else
      {
        PointToAdd.push_back(PointCloudTemp->points[i]);
      }
    }

    // add points
    AddPointSizeLate = ikdTreePtr->Add_Points(PointToAdd, true);
    AddPointSizeLate += ikdTreePtr->Add_Points(PointNoNeedDownsample, false);
  }
}