//
// Created by caochao on 6/5/19.
//

#include "../include/misc_utils/misc_utils.h"

namespace misc_utils_ns{
    /// Function for converting a PointType to a geometry_msgs::Point
    /// \param pnt A PointType
    /// \return A geometry_msgs::Point
    geometry_msgs::Point PCL2GeoMsgPnt(const PCLPointType &pnt) {
        return GeoMsgPoint(pnt.x, pnt.y, pnt.z);
    }

    /// Function for converting a geometry_msgs::Point to a PointType
    /// \param pnt A geometry_msgs::Point
    /// \return A PointType
    PCLPointType GeoMsgPnt2PCL(const geometry_msgs::Point &pnt) {
        PCLPointType point_o;
        point_o.x = (float) pnt.x;
        point_o.y = (float) pnt.y;
        point_o.z = (float) pnt.z;
        return point_o;
    }

    geometry_msgs::Point GeoMsgPoint(double x, double y, double z) {
        geometry_msgs::Point p;
        p.x = x;
        p.y = y;
        p.z = z;
        return p;
    }

    PCLPointType PCLPoint(float x, float y, float z) {
        PCLPointType p;
        p.x = x;
        p.y = y;
        p.z = z;
        return p;
    }

    void LeftRotatePoint(PCLPointType &pnt) {
        float tmp_z = pnt.z;
        pnt.z = pnt.y;
        pnt.y = pnt.x;
        pnt.x = tmp_z;
    }

    void RightRotatePoint(PCLPointType &pnt) {
        float tmp_x = pnt.x;
        pnt.x = pnt.y;
        pnt.y = pnt.z;
        pnt.z = tmp_x;
    }

    void LeftRotatePoint(geometry_msgs::Point &pnt) {
        double tmp_z = pnt.z;
        pnt.z = pnt.y;
        pnt.y = pnt.x;
        pnt.x = tmp_z;
    }

    void RightRotatePoint(geometry_msgs::Point &pnt) {
        double tmp_x = pnt.x;
        pnt.x = pnt.y;
        pnt.y = pnt.z;
        pnt.z = tmp_x;
    }

    template <class CloudType> void KeyposeToMap(CloudType & cloud, const nav_msgs::Odometry::ConstPtr & keypose)
    {
        float tx = (float)keypose->pose.pose.position.x;
        float ty = (float)keypose->pose.pose.position.y;
        float tz = (float)keypose->pose.pose.position.z;

        tf::Quaternion tf_q(keypose->pose.pose.orientation.x,
                            keypose->pose.pose.orientation.y,
                            keypose->pose.pose.orientation.z,
                            keypose->pose.pose.orientation.w);
        tf::Matrix3x3 tf_m(tf_q);
        double roll, pitch, yaw;
        tf_m.getRPY(roll, pitch, yaw);

        float sin_roll = (float)sin(roll);
        float cos_roll = (float)cos(roll);
        float sin_pitch = (float)sin(pitch);
        float cos_pitch = (float)cos(pitch);
        float sin_yaw = (float)sin(yaw);
        float cos_yaw = (float)cos(yaw);

        for(auto & point : cloud->points)
        {
            //To map_rot frame
            float x1 = point.x;
            float y1 = point.y;
            float z1 = point.z;

            float x2 = x1;
            float y2 = y1 * cos_roll - z1 * sin_roll;
            float z2 = y1 * sin_roll + z1 * cos_roll;

            float x3 = x2 * cos_pitch + z2 * sin_pitch;
            float y3 = y2;
            float z3 = -x2 * sin_pitch + z2 * cos_pitch;

            float x4 = x3 * cos_yaw - y3 * sin_yaw;
            float y4 = x3 * sin_yaw + y3 * cos_yaw;
            float z4 = z3;

            float x5 = x4 + tx;
            float y5 = y4 + ty;
            float z5 = z4 + tz;

            //To map frame
            point.x = z5;
            point.y = x5;
            point.z = y5;
        }
    }

    /// Function to compute the distance between two geometry_msgs::Point
    /// \param pnt1 The first point
    /// \param pnt2 The second point
    /// \return Distance between the two points
    double PointXYDist(const geometry_msgs::Point &pnt1, const geometry_msgs::Point &pnt2) {
        return sqrt(pow((pnt1.x - pnt2.x), 2) + pow((pnt1.y - pnt2.y), 2));
    }

    /// Function to compute the distance between two PointType
    /// \param pnt1 The first point
    /// \param pnt2 The second point
    /// \return Distance between the two points
    double PointXYDist(const PCLPointType &pnt1, const PCLPointType &pnt2) {
        return sqrt(pow((pnt1.x - pnt2.x), 2) + pow((pnt1.y - pnt2.y), 2));
    }

    /// Function to compute the direction (angle) of a geometry_msgs::Point
    /// \param pnt Input point
    /// \param robot_pos Robot position
    /// \return Direction (angle)
    double PointAngle(const geometry_msgs::Point &pnt, const geometry_msgs::Point &robot_pos) {
        return atan2((pnt.y - robot_pos.y), (pnt.x - robot_pos.x));
    }

    /// Function to compute the direction (angle) of a PointType
    /// \param pnt Intput point
    /// \param robot_pos Robot position
    /// \return Direction (angle)
    double PointAngle(const PCLPointType &pnt, const geometry_msgs::Point &robot_pos) {
        return atan2((pnt.y - robot_pos.y), (pnt.x - robot_pos.x));
    }

    /// Function to calculate the overlap of two angle intervals [s1, e1] and [s2, e2]. s1(2)->e1(2) counter-clockwise and all angles shoud be in [-pi, pi]
    /// \param s1 starting angle of the first interval
    /// \param e1 end angle of the second interval
    /// \param s2 starting angle of the first interval
    /// \param e2 end angle of the second interval
    /// \return overlap the overlap of the two intervals. > 0 if overlapped, < 0 otherwise.
    double AngleOverlap(double s1, double e1, double s2, double e2) {
        double overlap = 0.0;
        //TODO: normalize angles to [-pi, pi]
        //The first interval crosses the evil branch
        if (e1 < s1) {
            //Recursively compute the overlaps
            double sub_overlap1 = AngleOverlap(s1, PI, s2, e2);
            double sub_overlap2 = AngleOverlap(-PI, e1, s2, e2);
            //If both sub-overlaps are negative (no overlap) or there is only one positive sub-overlap
            if ((sub_overlap1 < 0 && sub_overlap2 < 0) || sub_overlap1 * sub_overlap2 < 0) {
                overlap = std::max(sub_overlap1, sub_overlap2);
            } else {
                overlap = sub_overlap1 + sub_overlap2;
            }
        } else if (e2 < s2) {
            //Similar to the case above
            double sub_overlap1 = AngleOverlap(s1, e1, s2, PI);
            double sub_overlap2 = AngleOverlap(s1, e1, -PI, e2);
            if ((sub_overlap1 < 0 && sub_overlap2 < 0) || sub_overlap1 * sub_overlap2 < 0) {
                overlap = std::max(sub_overlap1, sub_overlap2);
            } else {
                overlap = sub_overlap1 + sub_overlap2;
            }
        } else {
            if (e1 > e2) {
                if (s1 > e2) {
                    //No overlap
                    overlap = e2 - s1;
                } else if (s1 > s2) {
                    overlap = e2 - s1;
                } else {
                    overlap = e2 - s2;
                }
            } else {
                if (s2 > e1) {
                    //No overlap
                    overlap = e1 - s2;
                } else if (s2 > s1) {
                    overlap = e1 - s2;
                } else {
                    overlap = e1 - s1;
                }
            }
        }
        return overlap;
    }

    double AngleDiff(double source_angle, double target_angle) {
        double angle_diff = target_angle - source_angle;
        if (angle_diff > PI) {
            angle_diff -= 2 * PI;
        }
        if (angle_diff < -PI) {
            angle_diff += 2 * PI;
        }
        return angle_diff;
    }

    /// Function to determine if a point is on a line segment.
    /// Given three colinear points p, q, r, the function checks if point q lies on line segment 'pr'
    /// \param p End point of line segment pr
    /// \param q Point to be examined
    /// \param r End point of line segment pr
    /// \return If q is on pr
    bool PointOnLineSeg(const geometry_msgs::Point &p, const geometry_msgs::Point &q,
                        const geometry_msgs::Point &r) {
        if (q.x <= std::max(p.x, r.x) && q.x >= std::min(p.x, r.x) &&
            q.y <= std::max(p.y, r.y) && q.y >= std::min(p.y, r.y)) {
            return true;
        } else {
            return false;
        }

    }

    /// Function to find orientation of ordered triplet (p, q, r).
    /// \param p The first point
    /// \param q The second point
    /// \param r The third point
    /// \return 0 --> p, q and r are colinear, 1 --> Clockwise, 2 --> Counterclockwise
    int ThreePointOrientation(const geometry_msgs::Point &p,
                              const geometry_msgs::Point &q,
                              const geometry_msgs::Point &r) {
        // See https://www.geeksforgeeks.org/orientation-3-ordered-points/
        // for details of below formula.
        double val = (q.y - p.y) * (r.x - q.x) -
                     (q.x - p.x) * (r.y - q.y);

        if (fabs(val) < 0.1) return 0;  // colinear

        return (val > 0) ? 1 : 2; // clock or counterclock wise
    }

    /// Function to check if two line segments intersect, returns
    /// \param p1 end point of 'p1q1'
    /// \param q1 end point of 'p1q1'
    /// \param p2 end point of 'p2q2'
    /// \param q2 end point of 'p2q2'
    /// \return true if line segment 'p1q1' and 'p2q2' intersect, false otherwise
    bool LineSegIntersect(const geometry_msgs::Point &p1, const geometry_msgs::Point &q1,
                          const geometry_msgs::Point &p2, const geometry_msgs::Point &q2) {
        // Find the four orientations needed for general and
        // special cases
        int o1 = ThreePointOrientation(p1, q1, p2);
        int o2 = ThreePointOrientation(p1, q1, q2);
        int o3 = ThreePointOrientation(p2, q2, p1);
        int o4 = ThreePointOrientation(p2, q2, q1);

        // General case
        if (o1 != o2 && o3 != o4)
            return true;

        // Special Cases
        // p1, q1 and p2 are colinear and p2 lies on segment p1q1
        if (o1 == 0 && PointOnLineSeg(p1, p2, q1)) return true;

        // p1, q1 and q2 are colinear and q2 lies on segment p1q1
        if (o2 == 0 && PointOnLineSeg(p1, q2, q1)) return true;

        // p2, q2 and p1 are colinear and p1 lies on segment p2q2
        if (o3 == 0 && PointOnLineSeg(p2, p1, q2)) return true;

        // p2, q2 and q1 are colinear and q1 lies on segment p2q2
        if (o4 == 0 && PointOnLineSeg(p2, q1, q2)) return true;

        return false; // Doesn't fall in any of the above cases
    }

    /// Similar to LineSegIntersect(), except adds a "tolerance"
    /// such that each line segment is extended at both ends by this distance
    /// i.e. this will return true more often than LineSegIntersect()
    /// \param p1 end point of 'p1q1'
    /// \param q1 end point of 'p1q1'
    /// \param p2 end point of 'p2q2'
    /// \param q2 end point of 'p2q2'
    /// \param tolerance distance to be added at both ends of both lines
    /// \return true if line segment 'p1q1' and 'p2q2' intersect, false otherwise
    bool LineSegIntersectWithTolerance(const geometry_msgs::Point &p1, const geometry_msgs::Point &q1,
                          const geometry_msgs::Point &p2, const geometry_msgs::Point &q2, const double tolerance) {

        // Make a copy
        geometry_msgs::Point p1_extend = p1;
        geometry_msgs::Point q1_extend = q1;
        geometry_msgs::Point p2_extend = p2;
        geometry_msgs::Point q2_extend = q2;

        // Extend line segment 1
        double dist1 = PointXYDist(p1,q1);
        if(dist1==0) 
            return false; //trivial
        double dir1_x = (q1.x-p1.x)/dist1;
        double dir1_y = (q1.y-p1.y)/dist1;
        p1_extend.x -= tolerance*dir1_x;
        p1_extend.y -= tolerance*dir1_y;
        q1_extend.x += tolerance*dir1_x;
        q1_extend.y += tolerance*dir1_y;

        // Extend line segment 2
        double dist2 = PointXYDist(p2,q2);
        if(dist2==0) 
            return false; //trivial
        double dir2_x = (q2.x-p2.x)/dist2;
        double dir2_y = (q2.y-p2.y)/dist2;
        p2_extend.x -= tolerance*dir2_x;
        p2_extend.y -= tolerance*dir2_y;
        q2_extend.x += tolerance*dir2_x;
        q2_extend.y += tolerance*dir2_y;

        // Call the standard function with these extended line segments
        return LineSegIntersect(p1_extend, q1_extend, p2_extend, q2_extend);
    }


    /// Function to check if a point is inside a polygon
    /// \param p point
    /// \param polygon polygon
    /// \return true if the point is inside the polygon
    bool PointInPolygon(const geometry_msgs::Point & point, const geometry_msgs::Polygon & polygon)
    {
        int polygon_pnt_num = polygon.points.size();
        if(polygon_pnt_num < 3) return false;

        geometry_msgs::Point inf_point;
        inf_point.x = std::numeric_limits<float>::max();
        inf_point.y = point.y;
        int count = 0;
        int cur_idx = 0;
        do{
            int next_idx = (cur_idx + 1)%polygon_pnt_num;
            // Check if the line segment from 'point' to 'inf_point' intersects
            // with the line segment from 'polygon[cur_idx]' to 'polygon[next_idx]'
            geometry_msgs::Point polygon_cur_pnt = GeoMsgPoint(polygon.points[cur_idx].x, polygon.points[cur_idx].y, polygon.points[cur_idx].z);
            geometry_msgs::Point polygon_next_pnt = GeoMsgPoint(polygon.points[next_idx].x, polygon.points[next_idx].y, polygon.points[next_idx].z);
            if(LineSegIntersect(polygon_cur_pnt, polygon_next_pnt, point, inf_point))
            {
                // If the point 'point' is colinear with line segment 'cur-next',
                // then check if it lies on segment. If it lies, return true,
                // otherwise false
                if(ThreePointOrientation(polygon_cur_pnt, point, polygon_next_pnt) == 0)
                {
                    return PointOnLineSeg(polygon_cur_pnt, point, polygon_next_pnt);
                }
                count++;
            }
            cur_idx = next_idx;
        }
        while(cur_idx != 0);

        //return true if count is odd, false othterwise
        return count%2 == 1;
    }

    /// Function to get distance from a point to a line segment
    /// assumes z components is 0!
    /// \param p point
    /// \param line_segment_start
    /// \param line_segment_end
    /// \return distance
    double LineSegDistance2D(const geometry_msgs::Point & point, const geometry_msgs::Point & line_segment_start, const geometry_msgs::Point & line_segment_end) {
        // code adapted from http://geomalgorithms.com/a02-_lines.html

        // vector end to start
        double v_x = line_segment_end.x - line_segment_start.x;
        double v_y = line_segment_end.y - line_segment_start.y;

        // vector start to point
        double w_x = point.x - line_segment_start.x;
        double w_y = point.y - line_segment_start.y;

        // if outside one boundary, get point to point distance
        double c1 = v_x*w_x + v_y*w_y; // dot product
        
        if ( c1 <= 0 )
          return PointXYDist(point, line_segment_start);

        // if outside other boundary, get point to point distance
        double c2 = v_x*v_x + v_y*v_y; // dot product
        if ( c2 <= c1 )
          return PointXYDist(point, line_segment_end);

        // otherwise project point and get distance (seems inefficient?)
        double b = c1 / c2;
        geometry_msgs::Point point_projected;
        point_projected.x = line_segment_start.x + b * v_x;
        point_projected.y = line_segment_start.y + b * v_y;
        return PointXYDist(point, point_projected);
    }

    /// Function to get distance from a point to the closest point on boundary of a polygon
    /// \param p point
    /// \param polygon polygon
    /// \return distance
    double DistancePoint2DToPolygon(const geometry_msgs::Point & point, const geometry_msgs::Polygon & polygon) {
        int polygon_pnt_num = polygon.points.size();
        if(polygon_pnt_num < 1) return 0;
        if(polygon_pnt_num == 1) {
        	geometry_msgs::Point poly_point = GeoMsgPoint(polygon.points[0].x, polygon.points[0].y, 0);
        	return PointXYDist(point, poly_point);
        }

        double distance_return = INFINITY;
        int cur_idx = 0;

        // iterate through points in polygon
        do{
            int next_idx = (cur_idx + 1)%polygon_pnt_num;

            // get point to line segment distance
            geometry_msgs::Point polygon_cur_pnt = GeoMsgPoint(polygon.points[cur_idx].x, polygon.points[cur_idx].y, 0);
            geometry_msgs::Point polygon_next_pnt = GeoMsgPoint(polygon.points[next_idx].x, polygon.points[next_idx].y, 0);
            double distance = LineSegDistance2D(point, polygon_cur_pnt, polygon_next_pnt);
            if (distance < distance_return) {
                distance_return = distance;
            }

            cur_idx = next_idx;
        }
        while(cur_idx != 0);

        return distance_return;
    }

    double DegreeToRadian(double degree){return degree / 180.0 * M_PI;}
    double RadianToDegree(double radian){return radian * 180.0 / M_PI;}

}

template void misc_utils_ns::KeyposeToMap<pcl::PointCloud<pcl::PointXYZI>::Ptr>(pcl::PointCloud<pcl::PointXYZI>::Ptr & cloud, const nav_msgs::Odometry::ConstPtr & keypose);
template void misc_utils_ns::KeyposeToMap<pcl::PointCloud<pcl::PointXYZINormal>::Ptr>(pcl::PointCloud<pcl::PointXYZINormal>::Ptr & cloud, const nav_msgs::Odometry::ConstPtr & keypose);
template void misc_utils_ns::KeyposeToMap<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr>(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr & cloud, const nav_msgs::Odometry::ConstPtr & keypose);

