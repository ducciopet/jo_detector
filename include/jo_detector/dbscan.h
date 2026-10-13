#ifndef DBSCAN_H
#define DBSCAN_H

#include <vector>
#include <cmath>
#include <map>
#include <limits>
#include <algorithm>
#include <Eigen/Dense>

#define UNCLASSIFIED -1
#define CORE_POINT 1
#define BORDER_POINT 2
#define NOISE -2
#define DBSCAN_SUCCESS 0
#define DBSCAN_FAILURE -3

namespace joDetector {

    typedef struct Point_
    {
        float x, y, z;   // X, Y, Z position
        int clusterID;   // clustered ID
    } Point;

    typedef struct AABB_
    {
        Eigen::Vector3d min_pt;
        Eigen::Vector3d max_pt;
    } AABB;

    typedef struct ClusterRefined_
    {
        std::vector<Eigen::Vector3d> points;
        AABB box;
        double density;
        bool was_split;
    } ClusterRefined;

    class DBSCAN {
    public:
        DBSCAN(unsigned int minPts, float eps, const std::vector<Point>& points)
        {
            m_minPoints = minPts;
            m_epsilon = eps;
            m_points = points;
            m_pointSize = points.size();

            // default refinement params
            m_enableRefinement = false;
            m_refineMaxDiagonal = 2.0;
            m_refineMinDensity = 80.0;
            m_refineSplitMinPts = 8;
            m_refineSplitEps = 0.08;
            m_refineMinSubclusterPts = 10;
            m_refineAxisSliceWidth = 0.40;
            m_refineMaxDepth = 2;
            m_refineRecursive = true;
            m_refineMinBoxVolume = 1e-4;
        }

        ~DBSCAN() {}

        int run();
        std::vector<int> calculateCluster(const Point& point);
        int expandCluster(const Point& point, int clusterID);
        inline double calculateDistanceSquared(const Point& pointCore, const Point& pointTarget) const;

        int getTotalPointSize() const { return static_cast<int>(m_pointSize); }
        int getMinimumClusterSize() const { return static_cast<int>(m_minPoints); }
        float getEpsilonSize() const { return m_epsilon; }

        // =========================
        // REFINEMENT PARAMS
        // =========================
        void setRefinementParams(
            bool enableRefinement,
            double refineMaxDiagonal,
            double refineMinDensity,
            int refineSplitMinPts,
            double refineSplitEps,
            int refineMinSubclusterPts,
            double refineAxisSliceWidth,
            int refineMaxDepth,
            bool refineRecursive,
            double refineMinBoxVolume);

        // =========================
        // OUTPUT CLUSTERS
        // =========================
        void getRawClusters(std::vector<std::vector<Eigen::Vector3d>>& clusters) const;

        void getRefinedClusters(std::vector<std::vector<Eigen::Vector3d>>& clusters) const;

        void getRefinedClustersWithAABB(std::vector<ClusterRefined>& refinedClusters) const;

        // public as in your original code
    public:
        std::vector<Point> m_points;

    private:
        unsigned int m_pointSize;
        unsigned int m_minPoints;
        float m_epsilon;

        // refinement params
        bool m_enableRefinement;
        double m_refineMaxDiagonal;
        double m_refineMinDensity;
        int m_refineSplitMinPts;
        double m_refineSplitEps;
        int m_refineMinSubclusterPts;
        double m_refineAxisSliceWidth;
        int m_refineMaxDepth;
        bool m_refineRecursive;
        double m_refineMinBoxVolume;

        // helpers
        void computeAABB(const std::vector<Eigen::Vector3d>& pts, AABB& box) const;
        double computeDensity(const std::vector<Eigen::Vector3d>& pts, const AABB& box) const;
        double computeDiagonal(const AABB& box) const;
        bool shouldSplit(const std::vector<Eigen::Vector3d>& pts,
                         const AABB& box,
                         double density,
                         int depth) const;

        void refineClusterRecursive(const std::vector<Eigen::Vector3d>& inputCluster,
                                    std::vector<ClusterRefined>& outputClusters,
                                    int depth) const;

        bool splitClusterDBSCAN(const std::vector<Eigen::Vector3d>& pts,
                                std::vector<std::vector<Eigen::Vector3d>>& subclusters) const;

        bool splitClusterAxisSlicing(const std::vector<Eigen::Vector3d>& pts,
                                     const AABB& box,
                                     std::vector<std::vector<Eigen::Vector3d>>& subclusters) const;
    };

} // namespace joDetector

#endif // DBSCAN_H