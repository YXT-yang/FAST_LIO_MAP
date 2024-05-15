#ifndef MAP_MAINTAIN_HPP_
#define MAP_MAINTAIN_HPP_

#include "ikd-Tree/ikd_Tree.h"
#include "utils/data_type.hpp"

namespace lo_space
{
  using IKDtreeType = KD_TREE<PointType>;
  using IKDtreePtr = IKDtreeType::Ptr;
  using PointVector = IKDtreeType::PointVector;
  using PointVVector = vector<PointVector>;
  using DistVector = vector<float>;
  using DistVVector = vector<DistVector>;

  class MapMaintain
  {
  public:
    using Ptr = std::shared_ptr<MapMaintain>;

  public:
    IKDtreePtr ikdTreePtr; /// container of ikd-tree

  public:
    MapMaintain(double InCubeLength = 200,
                double InfilterSizeMapMin = 0.5,
                float IndetRange = 80.0f,
                int InNumMatchPoint = 5,
                double InMaxMatchDist = 2.0);
    ~MapMaintain(){};

    void setCubeLength(const double &input) { cubeLength = input; }
    void setFilterSize(const double &input) { filterSizeMapMin = input; }
    void setDetRange(const float &input) { DET_RANGE = input; }
    void setNumMatchPoint(const int &input) { NUM_MATCH_POINT = input; }
    void setMaxMatchDist(const double &input) { MAX_MATCH_DIST = input; }

  public:
    /**
     * @brief modified Map Box according to pose
     *
     * @param LidarPose
     */
    void MapFovSegment(V3D &LidarPose);

    /**
     * @brief Initialize tree with Point Cloud
     *
     * @param pointCloudInWorld
     */
    void IkdTreeInitial(PointCloudPtr pointCloudInWorld);

    bool IfTreeInitialized() { return treeInitialized; };

    /**
     * @brief compute distance between two points
     *
     * @param p1
     * @param p2
     * @return float
     */
    float computDist(PointType p1, PointType p2)
    {
      return (p1.x - p2.x) * (p1.x - p2.x) +
             (p1.y - p2.y) * (p1.y - p2.y) +
             (p1.z - p2.z) * (p1.z - p2.z);
    }

    /**
     * @brief acquire nearest points (NUM_MATCH_POINT) and correspond distance
     * of input point (In world)
     *
     * @param pointInWorld
     * @param NearPoints
     * @param DistPoints
     * @return true
     * @return false
     */
    bool SearchPointInTree(PointType &pointInWorld, PointVector &NearPoints, DistVector &DistPoints);

    /**
     * @brief acquire nearest point (NUM_MATCH_POINT)
     * of input pointCloud
     *
     * @param pointCloudInWorld
     */
    void SearchPointCloudInTree(PointCloudPtr pointCloudInWorld);

    /**
     * @brief Increase Map after solve by PointCloudTemp & NeartPointsVector
     *
     */
    void MapIncrement();

    int getAddPointsNumLate() { return AddPointSizeLate; }

  private:
    vector<BoxPointType> cubeNeedRemove; /// record Cub - Need removed
    BoxPointType localMapBox;            /// record Box of local map
    int treeDeleteCounter;               /// record Cub number
    bool localMapInitialized;            /// record if map has been initialized
    bool treeInitialized;                /// record if tree has been initialized

  public:
    PointVVector NeartPointsVector; /// record the correspond Nearest Points Vector of PointCloud
    DistVVector DistVectorVector;   /// record the dist
    PointCloudPtr PointCloudTemp;   /// record the PointCloud temporary deal

  private:
    double cubeLength;       /// record Cube Size (Need Input)
    double filterSizeMapMin; /// record the size of Filter
    float MOV_THRESHOLD;     /// record the threshold of move Box (const)
    float DET_RANGE;         /// record the measure farthest distance of radar
    int NUM_MATCH_POINT;     /// Point number when match
    double MAX_MATCH_DIST;   /// max distance when match
    int AddPointSizeLate;    /// record added points number lately
  };
}

#endif /// MAP_MAINTAIN_HPP_