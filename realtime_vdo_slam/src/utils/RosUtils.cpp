#include "utils/RosUtils.hpp"
#include "utils/GeoNavConstants.h"

#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/Transform.h>

namespace VDO_SLAM {

    namespace utils {

        void mat_to_image_msg(sensor_msgs::Image& img_msg, const cv::Mat& img, const std::string& encoding, const std_msgs::Header& header) {
            cv_bridge::CvImage img_ptr = cv_bridge::CvImage(header, encoding, img);
            img_ptr.toImageMsg(img_msg);
        }

        void image_msg_to_mat(cv::Mat& img, const sensor_msgs::Image& image_msg, const std::string& encoding) {
                cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(image_msg, encoding);
                img = cv_ptr->image;
        }

        void publish_static_tf(const nav_msgs::Odometry& odom,
                                const std::string& parent_frame_id,
                                const std::string& child_frame_id, const ros::Time& time) {
                                    
            static tf2_ros::StaticTransformBroadcaster static_broadcaster;
            geometry_msgs::TransformStamped static_transform_stamped;
            // TODO(Toni): Warning: using ros::Time::now(), will that bring issues?
            static_transform_stamped.header.stamp = ros::Time::now();
            static_transform_stamped.header.frame_id = parent_frame_id;
            static_transform_stamped.child_frame_id = child_frame_id;

            
            odom_to_tf(odom, &static_transform_stamped.transform);
            static_broadcaster.sendTransform(static_transform_stamped);
            ros::spinOnce();
        }

        void odom_to_tf(const nav_msgs::Odometry& odom, geometry_msgs::Transform* transform) {
            transform->translation.x = odom.pose.pose.position.x;
            transform->translation.y = odom.pose.pose.position.y;
            transform->translation.z = odom.pose.pose.position.z;

            transform->rotation.x = odom.pose.pose.orientation.x;
            transform->rotation.y = odom.pose.pose.orientation.y;
            transform->rotation.z = odom.pose.pose.orientation.z;
            transform->rotation.w = odom.pose.pose.orientation.w;

        }

        void geometry_to_odom(const geometry_msgs::Pose& pose, const geometry_msgs::Twist& twist, const ros::Time& time, nav_msgs::Odometry& odom) {
            odom.header.stamp = time;
            odom.pose.pose = pose;
            odom.twist.twist = twist;
        }

        void geometry_to_odom(const geometry_msgs::Pose& pose, const geometry_msgs::Twist& twist, const ros::Time& time, 
                                        const std::string& frame_id, const std::string& child_frame_id, nav_msgs::Odometry& odom) {
            odom.header.stamp = time;
            odom.header.frame_id = frame_id;
            odom.child_frame_id = child_frame_id;
            odom.pose.pose = pose;
            odom.twist.twist = twist;
                                    
        }

        void LLtoUTM(const double Lat, const double Long,
                           double &UTMNorthing, double &UTMEasting,
                           std::string &UTMZone) {
            double a = WGS84_A;
            double eccSquared = UTM_E2;
            double k0 = UTM_K0;

            double LongOrigin;
            double eccPrimeSquared;
            double N, T, C, A, M;

            // Make sure the longitude is between -180.00 .. 179.9
            double LongTemp = (Long+180)-static_cast<int>((Long+180)/360)*360-180;

            double LatRad = Lat*RADIANS_PER_DEGREE;
            double LongRad = LongTemp*RADIANS_PER_DEGREE;
            double LongOriginRad;
            int    ZoneNumber;

            ZoneNumber = static_cast<int>((LongTemp + 180)/6) + 1;

            if ( Lat >= 56.0 && Lat < 64.0 && LongTemp >= 3.0 && LongTemp < 12.0 )
                ZoneNumber = 32;

                    // Special zones for Svalbard
            if ( Lat >= 72.0 && Lat < 84.0 )
            {
                if (      LongTemp >= 0.0  && LongTemp <  9.0 ) ZoneNumber = 31;
                else if ( LongTemp >= 9.0  && LongTemp < 21.0 ) ZoneNumber = 33;
                else if ( LongTemp >= 21.0 && LongTemp < 33.0 ) ZoneNumber = 35;
                else if ( LongTemp >= 33.0 && LongTemp < 42.0 ) ZoneNumber = 37;
            }
                    // +3 puts origin in middle of zone
            LongOrigin = (ZoneNumber - 1)*6 - 180 + 3;
            LongOriginRad = LongOrigin * RADIANS_PER_DEGREE;

            // Compute the UTM Zone from the latitude and longitude
            char zone_buf[] = {0, 0, 0, 0};
            snprintf(zone_buf, sizeof(zone_buf), "%d%c", ZoneNumber, UTMLetterDesignator(Lat));
            UTMZone = std::string(zone_buf);

            eccPrimeSquared = (eccSquared)/(1-eccSquared);

            N = a/sqrt(1-eccSquared*sin(LatRad)*sin(LatRad));
            T = tan(LatRad)*tan(LatRad);
            C = eccPrimeSquared*cos(LatRad)*cos(LatRad);
            A = cos(LatRad)*(LongRad-LongOriginRad);

            M = a*((1 - eccSquared/4 - 3*eccSquared*eccSquared/64
                            - 5*eccSquared*eccSquared*eccSquared/256) * LatRad
                        - (3*eccSquared/8 + 3*eccSquared*eccSquared/32
                            + 45*eccSquared*eccSquared*eccSquared/1024)*sin(2*LatRad)
                        + (15*eccSquared*eccSquared/256
                            + 45*eccSquared*eccSquared*eccSquared/1024)*sin(4*LatRad)
                        - (35*eccSquared*eccSquared*eccSquared/3072)*sin(6*LatRad));

            UTMEasting = static_cast<double>
                    (k0*N*(A+(1-T+C)*A*A*A/6
                            + (5-18*T+T*T+72*C-58*eccPrimeSquared)*A*A*A*A*A/120)
                    + 500000.0);

            UTMNorthing = static_cast<double>
                    (k0*(M+N*tan(LatRad)
                        *(A*A/2+(5-T+9*C+4*C*C)*A*A*A*A/24
                            + (61-58*T+T*T+600*C-330*eccPrimeSquared)*A*A*A*A*A*A/720)));

            if (Lat < 0) {
                // 10000000 meter offset for southern hemisphere
                UTMNorthing += 10000000.0;
            }
        }

        char UTMLetterDesignator(double Lat) {
            char LetterDesignator;
            if     ((84 >= Lat) && (Lat >= 72))  LetterDesignator = 'X';
            else if ((72 > Lat) && (Lat >= 64))  LetterDesignator = 'W';
            else if ((64 > Lat) && (Lat >= 56))  LetterDesignator = 'V';
            else if ((56 > Lat) && (Lat >= 48))  LetterDesignator = 'U';
            else if ((48 > Lat) && (Lat >= 40))  LetterDesignator = 'T';
            else if ((40 > Lat) && (Lat >= 32))  LetterDesignator = 'S';
            else if ((32 > Lat) && (Lat >= 24))  LetterDesignator = 'R';
            else if ((24 > Lat) && (Lat >= 16))  LetterDesignator = 'Q';
            else if ((16 > Lat) && (Lat >= 8))   LetterDesignator = 'P';
            else if (( 8 > Lat) && (Lat >= 0))   LetterDesignator = 'N';
            else if (( 0 > Lat) && (Lat >= -8))  LetterDesignator = 'M';
            else if ((-8 > Lat) && (Lat >= -16)) LetterDesignator = 'L';
            else if ((-16 > Lat) && (Lat >= -24)) LetterDesignator = 'K';
            else if ((-24 > Lat) && (Lat >= -32)) LetterDesignator = 'J';
            else if ((-32 > Lat) && (Lat >= -40)) LetterDesignator = 'H';
            else if ((-40 > Lat) && (Lat >= -48)) LetterDesignator = 'G';
            else if ((-48 > Lat) && (Lat >= -56)) LetterDesignator = 'F';
            else if ((-56 > Lat) && (Lat >= -64)) LetterDesignator = 'E';
            else if ((-64 > Lat) && (Lat >= -72)) LetterDesignator = 'D';
            else if ((-72 > Lat) && (Lat >= -80)) LetterDesignator = 'C';
                    // 'Z' is an error flag, the Latitude is outside the UTM limits
            else LetterDesignator = 'Z';
            return LetterDesignator;
        }
        namespace geometry_converter {

          
            Eigen::Quaterniond quat_from_orientation(const geometry_msgs::Quaternion& quat) {
                return Eigen::Quaterniond(quat.w, quat.x, quat.y, quat.z);
            }
        
            Eigen::Vector3d vector_from_translation(const geometry_msgs::Point& point) {
                return Eigen::Vector3d(point.x, point.y, point.z);
            }
            Eigen::Vector3d vector_from_translation(const geometry_msgs::Vector3& point) {
                return Eigen::Vector3d(point.x, point.y, point.z);
            }


            geometry_msgs::Quaternion orienation_from_quat(const Eigen::Quaterniond& orientation) {
                geometry_msgs::Quaternion quat;
                quat.x = orientation.x();
                quat.y = orientation.y();
                quat.z = orientation.z();
                quat.w = orientation.w();
                return quat;
            }
   
            geometry_msgs::Point translation_from_vector(const Eigen::Vector3d& vector) {
                geometry_msgs::Point point;
                point.x = vector.x();
                point.y = vector.y();
                point.z = vector.z();
                return point;
            }


        }


        namespace g2o_converter {
            
            g2o::SE3Quat from_pose_msg(const geometry_msgs::Pose& pose_msg) {
                Eigen::Quaterniond quat = geometry_converter::quat_from_orientation(pose_msg.orientation);
                Eigen::Vector3d vector = geometry_converter::vector_from_translation(pose_msg.position);
                Eigen::Matrix<double,3,3> rotation_m =  quat.toRotationMatrix();
                return g2o::SE3Quat(rotation_m,vector);

            }

            geometry_msgs::Pose to_pose_msg(const g2o::SE3Quat& pose) {
                geometry_msgs::Quaternion orientation = geometry_converter::orienation_from_quat(pose.rotation());
                geometry_msgs::Point point = geometry_converter::translation_from_vector(pose.translation());

                geometry_msgs::Pose pose_msg;
                pose_msg.orientation = orientation;
                pose_msg.position = point;

                return pose_msg;

            }

            g2o::SE3Quat from_twist_msg(const geometry_msgs::Twist& twist_msg) {
                Eigen::Quaterniond quat(1, 0, 0, 0);
                //we only get the linear component from VDO
                Eigen::Vector3d vector = geometry_converter::vector_from_translation(twist_msg.linear);
                Eigen::Matrix<double,3,3> rotation_m =  quat.toRotationMatrix();
                return g2o::SE3Quat(rotation_m,vector);
            }


            geometry_msgs::Twist to_twist_msg(const g2o::SE3Quat& twist) {
                geometry_msgs::Vector3 angular;
                angular.x = 0;
                angular.y = 0;
                angular.z = 0;
                geometry_msgs::Point point = geometry_converter::translation_from_vector(twist.translation());
                geometry_msgs::Vector3 linear;
                linear.x = point.x;
                linear.y = point.y;
                linear.z = point.z;

                geometry_msgs::Twist twist_msg;
                twist_msg.linear = linear;
                twist_msg.angular = angular;

                return twist_msg;

            }


       
        } //namespace g2o_converter

    } //namespace utils
}