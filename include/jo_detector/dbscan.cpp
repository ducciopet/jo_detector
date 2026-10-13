#include <jo_detector/dbscan.h>
#include <iostream>

namespace joDetector {

    int DBSCAN::run()
    {
        int clusterID = 1;

        for (auto iter = m_points.begin(); iter != m_points.end(); ++iter)
        {
            if (iter->clusterID == UNCLASSIFIED)
            {
                if (expandCluster(*iter, clusterID) != DBSCAN_FAILURE)
                {
                    clusterID += 1;
                }
            }
        }

        return DBSCAN_SUCCESS;
    }

    int DBSCAN::expandCluster(const Point& point, int clusterID)
    {
        std::vector<int> clusterSeeds = calculateCluster(point);

        if (clusterSeeds.size() < m_minPoints)
        {
            for (auto& p : m_points)
            {
                if (p.x == point.x && p.y == point.y && p.z == point.z)
                {
                    p.clusterID = NOISE;
                    break;
                }
            }
            return DBSCAN_FAILURE;
        }
        else
        {
            int index = 0;
            int indexCorePoint = 0;

            for (auto iterSeeds = clusterSeeds.begin(); iterSeeds != clusterSeeds.end(); ++iterSeeds)
            {
                m_points.at(*iterSeeds).clusterID = clusterID;

                if (m_points.at(*iterSeeds).x == point.x &&
                    m_points.at(*iterSeeds).y == point.y &&
                    m_points.at(*iterSeeds).z == point.z)
                {
                    indexCorePoint = index;
                }
                ++index;
            }

            clusterSeeds.erase(clusterSeeds.begin() + indexCorePoint);

            for (std::vector<int>::size_type i = 0; i < clusterSeeds.size(); ++i)
            {
                std::vector<int> clusterNeighbors = calculateCluster(m_points.at(clusterSeeds[i]));

                if (clusterNeighbors.size() >= m_minPoints)
                {
                    for (auto iterNeighbors = clusterNeighbors.begin();
                         iterNeighbors != clusterNeighbors.end();
                         ++iterNeighbors)
                    {
                        if (m_points.at(*iterNeighbors).clusterID == UNCLASSIFIED ||
                            m_points.at(*iterNeighbors).clusterID == NOISE)
                        {
                            if (m_points.at(*iterNeighbors).clusterID == UNCLASSIFIED)
                            {
                                clusterSeeds.push_back(*iterNeighbors);
                            }
                            m_points.at(*iterNeighbors).clusterID = clusterID;
                        }
                    }
                }
            }

            return DBSCAN_SUCCESS;
        }
    }

    std::vector<int> DBSCAN::calculateCluster(const Point& point)
    {
        int index = 0;
        std::vector<int> clusterIndex;

        for (auto iter = m_points.begin(); iter != m_points.end(); ++iter)
        {
            if (calculateDistanceSquared(point, *iter) <= m_epsilon)
            {
                clusterIndex.push_back(index);
            }
            ++index;
        }

        return clusterIndex;
    }

    inline double DBSCAN::calculateDistanceSquared(const Point& pointCore, const Point& pointTarget) const
    {
        return std::pow(pointCore.x - pointTarget.x, 2) +
               std::pow(pointCore.y - pointTarget.y, 2) +
               std::pow(pointCore.z - pointTarget.z, 2);
    }

    void DBSCAN::setRefinementParams(
        bool enableRefinement,
        double refineMaxDiagonal,
        double refineMinDensity,
        int refineSplitMinPts,
        double refineSplitEps,
        int refineMinSubclusterPts,
        double refineAxisSliceWidth,
        int refineMaxDepth,
        bool refineRecursive,
        double refineMinBoxVolume)
    {
        m_enableRefinement = enableRefinement;
        m_refineMaxDiagonal = refineMaxDiagonal;
        m_refineMinDensity = refineMinDensity;
        m_refineSplitMinPts = refineSplitMinPts;
        m_refineSplitEps = refineSplitEps;
        m_refineMinSubclusterPts = refineMinSubclusterPts;
        m_refineAxisSliceWidth = refineAxisSliceWidth;
        m_refineMaxDepth = refineMaxDepth;
        m_refineRecursive = refineRecursive;
        m_refineMinBoxVolume = refineMinBoxVolume;
    }

    void DBSCAN::getRawClusters(std::vector<std::vector<Eigen::Vector3d>>& clusters) const
    {
        clusters.clear();

        int clusterNum = 0;
        for (size_t i = 0; i < m_points.size(); ++i)
        {
            if (m_points[i].clusterID > clusterNum)
                clusterNum = m_points[i].clusterID;
        }

        if (clusterNum <= 0) return;

        clusters.resize(clusterNum);

        for (size_t i = 0; i < m_points.size(); ++i)
        {
            const Point& p = m_points[i];
            if (p.clusterID > 0)
            {
                clusters[p.clusterID - 1].push_back(Eigen::Vector3d(p.x, p.y, p.z));
            }
        }

        std::vector<std::vector<Eigen::Vector3d>> filtered;
        for (size_t i = 0; i < clusters.size(); ++i)
        {
            if (!clusters[i].empty())
                filtered.push_back(clusters[i]);
        }

        clusters.swap(filtered);
    }

    void DBSCAN::getRefinedClusters(std::vector<std::vector<Eigen::Vector3d>>& clusters) const
    {
        clusters.clear();

        std::vector<ClusterRefined> refined;
        getRefinedClustersWithAABB(refined);

        for (size_t i = 0; i < refined.size(); ++i)
            clusters.push_back(refined[i].points);
    }

    void DBSCAN::getRefinedClustersWithAABB(std::vector<ClusterRefined>& refinedClusters) const
    {
        refinedClusters.clear();

        std::vector<std::vector<Eigen::Vector3d>> rawClusters;
        getRawClusters(rawClusters);

        if (!m_enableRefinement)
        {
            for (size_t i = 0; i < rawClusters.size(); ++i)
            {
                ClusterRefined out;
                out.points = rawClusters[i];
                computeAABB(out.points, out.box);
                out.density = computeDensity(out.points, out.box);
                out.was_split = false;
                refinedClusters.push_back(out);
            }
            return;
        }

        for (size_t i = 0; i < rawClusters.size(); ++i)
        {
            refineClusterRecursive(rawClusters[i], refinedClusters, 0);
        }
    }

    void DBSCAN::computeAABB(const std::vector<Eigen::Vector3d>& pts, AABB& box) const
    {
        box.min_pt = Eigen::Vector3d(
            std::numeric_limits<double>::max(),
            std::numeric_limits<double>::max(),
            std::numeric_limits<double>::max());

        box.max_pt = Eigen::Vector3d(
            -std::numeric_limits<double>::max(),
            -std::numeric_limits<double>::max(),
            -std::numeric_limits<double>::max());

        for (size_t i = 0; i < pts.size(); ++i)
        {
            box.min_pt = box.min_pt.cwiseMin(pts[i]);
            box.max_pt = box.max_pt.cwiseMax(pts[i]);
        }
    }

    double DBSCAN::computeDensity(const std::vector<Eigen::Vector3d>& pts, const AABB& box) const
    {
        Eigen::Vector3d size = box.max_pt - box.min_pt;
        double volume = size.x() * size.y() * size.z();

        if (volume < m_refineMinBoxVolume)
            volume = m_refineMinBoxVolume;

        return static_cast<double>(pts.size()) / volume;
    }

    double DBSCAN::computeDiagonal(const AABB& box) const
    {
        // Only consider x and y dimensions for diagonal
        Eigen::Vector2d diff_xy = (box.max_pt - box.min_pt).head<2>();
        return diff_xy.norm();
    }

    bool DBSCAN::shouldSplit(const std::vector<Eigen::Vector3d>& pts,
                             const AABB& box,
                             double density,
                             int depth) const
    {
        if (pts.size() < static_cast<size_t>(m_refineMinSubclusterPts))
            return false;

        if (depth >= m_refineMaxDepth)
            return false;

        const double diagonal = computeDiagonal(box);

        if (diagonal <= m_refineMaxDiagonal)
            return false;

        if (density >= m_refineMinDensity)
            return false;

        return true;
    }

    void DBSCAN::refineClusterRecursive(const std::vector<Eigen::Vector3d>& inputCluster,
                                        std::vector<ClusterRefined>& outputClusters,
                                        int depth) const
    {
        if (inputCluster.empty()) return;

        AABB box;
        computeAABB(inputCluster, box);

        double density = computeDensity(inputCluster, box);

        if (!shouldSplit(inputCluster, box, density, depth))
        {
            ClusterRefined out;
            out.points = inputCluster;
            out.box = box;
            out.density = density;
            out.was_split = (depth > 0);
            outputClusters.push_back(out);
            return;
        }

        std::vector<std::vector<Eigen::Vector3d>> subclusters;
        bool split_ok = splitClusterDBSCAN(inputCluster, subclusters);

        if (!split_ok)
        {
            subclusters.clear();
            split_ok = splitClusterAxisSlicing(inputCluster, box, subclusters);
        }

        if (!split_ok || subclusters.size() < 2)
        {
            ClusterRefined out;
            out.points = inputCluster;
            out.box = box;
            out.density = density;
            out.was_split = false;
            outputClusters.push_back(out);
            return;
        }

        bool added_any = false;

        for (size_t i = 0; i < subclusters.size(); ++i)
        {
            if (static_cast<int>(subclusters[i].size()) < m_refineMinSubclusterPts)
                continue;

            added_any = true;

            if (m_refineRecursive)
                refineClusterRecursive(subclusters[i], outputClusters, depth + 1);
            else
            {
                ClusterRefined out;
                out.points = subclusters[i];
                computeAABB(out.points, out.box);
                out.density = computeDensity(out.points, out.box);
                out.was_split = true;
                outputClusters.push_back(out);
            }
        }

        if (!added_any)
        {
            ClusterRefined out;
            out.points = inputCluster;
            out.box = box;
            out.density = density;
            out.was_split = false;
            outputClusters.push_back(out);
        }
    }

    bool DBSCAN::splitClusterDBSCAN(const std::vector<Eigen::Vector3d>& pts,
                                    std::vector<std::vector<Eigen::Vector3d>>& subclusters) const
    {
        subclusters.clear();

        std::vector<Point> dbPts;
        dbPts.reserve(pts.size());

        for (size_t i = 0; i < pts.size(); ++i)
        {
            Point pt;
            pt.x = static_cast<float>(pts[i].x());
            pt.y = static_cast<float>(pts[i].y());
            pt.z = static_cast<float>(pts[i].z());
            pt.clusterID = UNCLASSIFIED;
            dbPts.push_back(pt);
        }

        DBSCAN subDb(m_refineSplitMinPts, static_cast<float>(m_refineSplitEps), dbPts);
        subDb.run();

        std::vector<std::vector<Eigen::Vector3d>> rawSubclusters;
        subDb.getRawClusters(rawSubclusters);

        for (size_t i = 0; i < rawSubclusters.size(); ++i)
        {
            if (static_cast<int>(rawSubclusters[i].size()) >= m_refineMinSubclusterPts)
                subclusters.push_back(rawSubclusters[i]);
        }

        return subclusters.size() >= 2;
    }

    bool DBSCAN::splitClusterAxisSlicing(const std::vector<Eigen::Vector3d>& pts,
                                         const AABB& box,
                                         std::vector<std::vector<Eigen::Vector3d>>& subclusters) const
    {
        subclusters.clear();
        if (pts.empty()) return false;

        Eigen::Vector3d size = box.max_pt - box.min_pt;

        int axis = 0;
        if (size.y() > size.x() && size.y() >= size.z())
            axis = 1;
        else if (size.z() > size.x() && size.z() > size.y())
            axis = 2;

        double axisLength = size[axis];
        if (axisLength <= m_refineAxisSliceWidth)
            return false;

        int numSlices = std::max(2, static_cast<int>(std::ceil(axisLength / m_refineAxisSliceWidth)));
        std::vector<std::vector<Eigen::Vector3d>> bins(numSlices);

        double minAxis = box.min_pt[axis];
        double denom = std::max(m_refineAxisSliceWidth, 1e-6);

        for (size_t i = 0; i < pts.size(); ++i)
        {
            int idx = static_cast<int>((pts[i][axis] - minAxis) / denom);
            idx = std::max(0, std::min(idx, numSlices - 1));
            bins[idx].push_back(pts[i]);
        }

        std::vector<Eigen::Vector3d> current;
        for (int i = 0; i < numSlices; ++i)
        {
            if (bins[i].empty())
            {
                if (static_cast<int>(current.size()) >= m_refineMinSubclusterPts)
                    subclusters.push_back(current);
                current.clear();
                continue;
            }

            current.insert(current.end(), bins[i].begin(), bins[i].end());

            if (static_cast<int>(current.size()) >= m_refineMinSubclusterPts)
            {
                subclusters.push_back(current);
                current.clear();
            }
        }

        if (!current.empty())
        {
            if (static_cast<int>(current.size()) >= m_refineMinSubclusterPts)
                subclusters.push_back(current);
            else if (!subclusters.empty())
                subclusters.back().insert(subclusters.back().end(), current.begin(), current.end());
        }

        return subclusters.size() >= 2;
    }

} // namespace joDetector