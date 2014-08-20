/*********************************************************************
*
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*
* Author: Eitan Marder-Eppstein
*********************************************************************/
#ifndef NAVFN_NAVFN_ROS_H_
#define NAVFN_NAVFN_ROS_H_

#include <ros/ros.h>
#include <navfn/navfn.h>
#include <costmap_2d/costmap_2d.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Path.h>
#include <tf/transform_datatypes.h>
#include <vector>
#include <nav_core/base_global_planner.h>
#include <nav_msgs/GetPlan.h>
#include <navfn/potarr_point.h>
#include <pcl_ros/publisher.h>
#include <pcl_conversions/pcl_conversions.h>

#include <iostream>
#include <pcl/console/parse.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_sphere.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <pcl/sample_consensus/sac_model_line.h>

namespace navfn {
  /**
   * @class NavfnROS
   * @brief Provides a ROS wrapper for the navfn planner which runs a fast, interpolated navigation function on a costmap.
   */
  class NavfnROS : public nav_core::BaseGlobalPlanner {
    public:
      /**
       * @brief  Default constructor for the NavFnROS object
       */
      NavfnROS();

      /**
       * @brief  Constructor for the NavFnROS object
       * @param  name The name of this planner
       * @param  costmap A pointer to the ROS wrapper of the costmap to use
       */
      NavfnROS(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

      /**
       * @brief  Initialization function for the NavFnROS object
       * @param  name The name of this planner
       * @param  costmap A pointer to the ROS wrapper of the costmap to use for planning
       */
      void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

      bool makePlanSmall(const geometry_msgs::PoseStamped& start,
          const geometry_msgs::PoseStamped& goal, double tolerance, std::vector<geometry_msgs::PoseStamped>& plan);
      /**
       * @brief Given a goal pose in the world, compute a plan
       * @param start The start pose 
       * @param goal The goal pose 
       * @param plan The plan... filled by the planner
       * @return True if a valid plan was found, false otherwise
       */
      bool makePlan(const geometry_msgs::PoseStamped& start, 
          const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);

      /**
       * @brief Given a goal pose in the world, compute a plan
       * @param start The start pose 
       * @param goal The goal pose 
       * @param tolerance The tolerance on the goal point for the planner
       * @param plan The plan... filled by the planner
       * @return True if a valid plan was found, false otherwise
       */
      bool makePlan(const geometry_msgs::PoseStamped& start, 
          const geometry_msgs::PoseStamped& goal, double tolerance, std::vector<geometry_msgs::PoseStamped>& plan);

      /**
       * @brief  Computes the full navigation function for the map given a point in the world to start from
       * @param world_point The point to use for seeding the navigation function 
       * @return True if the navigation function was computed successfully, false otherwise
       */
      bool computePotential(const geometry_msgs::Point& world_point);

      /**
       * @brief Compute a plan to a goal after the potential for a start point has already been computed (Note: You should call computePotential first)
       * @param goal The goal pose to create a plan to
       * @param plan The plan... filled by the planner
       * @return True if a valid plan was found, false otherwise
       */
      bool getPlanFromPotential(const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan);

      /**
       * @brief Get the potential, or naviagation cost, at a given point in the world (Note: You should call computePotential first)
       * @param world_point The point to get the potential for 
       * @return The navigation function's value at that point in the world
       */
      double getPointPotential(const geometry_msgs::Point& world_point);

      /**
       * @brief Check for a valid potential value at a given point in the world (Note: You should call computePotential first)
       * @param world_point The point to get the potential for 
       * @return True if the navigation function is valid at that point in the world, false otherwise
       */
      bool validPointPotential(const geometry_msgs::Point& world_point);

      /**
       * @brief Check for a valid potential value at a given point in the world (Note: You should call computePotential first)
       * @param world_point The point to get the potential for 
       * @param tolerance The tolerance on searching around the world_point specified
       * @return True if the navigation function is valid at that point in the world, false otherwise
       */
      bool validPointPotential(const geometry_msgs::Point& world_point, double tolerance);

      /**
       * @brief  Publish a path for visualization purposes
       */
      void publishPlan(const std::vector<geometry_msgs::PoseStamped>& path, double r, double g, double b, double a);

      ~NavfnROS(){}

      bool makePlanService(nav_msgs::GetPlan::Request& req, nav_msgs::GetPlan::Response& resp);

    protected:

      /**
       * @brief Store a copy of the current costmap in \a costmap.  Called by makePlan.
       */
      costmap_2d::Costmap2DROS* costmap_ros_;
      boost::shared_ptr<NavFn> planner_;
      ros::Publisher plan_pub_;
      pcl_ros::Publisher<PotarrPoint> potarr_pub_;
      bool initialized_, allow_unknown_, visualize_potential_;


    private:
      inline double sq_distance(const geometry_msgs::PoseStamped& p1, const geometry_msgs::PoseStamped& p2){
        double dx = p1.pose.position.x - p2.pose.position.x;
        double dy = p1.pose.position.y - p2.pose.position.y;
        return dx*dx +dy*dy;
      }
      inline double FindDistanceToSegment(double x1, double y1, double x2, double y2, double pointX, double pointY)
      {
          double diffX = x2 - x1;
          float diffY = y2 - y1;
          if ((diffX == 0) && (diffY == 0))
          {
              diffX = pointX - x1;
              diffY = pointY - y1;
              return sqrt(diffX * diffX + diffY * diffY);
          }

          float t = ((pointX - x1) * diffX + (pointY - y1) * diffY) / (diffX * diffX + diffY * diffY);

          if (t < 0)
          {
              //point is nearest to the first point i.e x1 and y1
              diffX = pointX - x1;
              diffY = pointY - y1;
          }
          else if (t > 1)
          {
              //point is nearest to the end point i.e x2 and y2
              diffX = pointX - x2;
              diffY = pointY - y2;
          }
          else
          {
              //if perpendicular line intersect the line segment.
              diffX = pointX - (x1 + t * diffX);
              diffY = pointY - (y1 + t * diffY);
          }

          //returning shortest distance
          return sqrt(diffX * diffX + diffY * diffY);
      }
      inline bool clear_path(geometry_msgs::PoseStamped& point1,geometry_msgs::PoseStamped& point2 ){
    	  bool clear_path = true;
    	  costmap_2d::Costmap2D* costmap = costmap_ros_->getCostmap();
    	  unsigned int p1y,p1x,p2x, p2y;
    	  double beforex1 =  point1.pose.position.x;
    	  double beforey1 =  point1.pose.position.y;
    	  costmap->worldToMap( beforex1 , beforey1, p1x , p1y);
    	  double beforex2 =  point2.pose.position.x;
    	  double beforey2 =  point2.pose.position.y;
    	  costmap->worldToMap( beforex2 , beforey2, p2x , p2y);

    	  int lowxpoint;
    	  int lowxabspoint = abs (p1x - p2x);
    	  if(p1x < p2x){
    		  lowxpoint = round(p1x);
    	  }else{
    		  lowxpoint = round(p2x);
    	  }

		  int lowypoint;
		  int lowyabspoint = abs (p1y - p2y);
		  if(p1y < p2y){
			lowypoint = round(p1y);
		  }else{
			lowypoint = round(p2y);
		  }
		  ROS_INFO_STREAM("lowx"<<lowxpoint<<"   lowy" << lowypoint);
		  ROS_INFO_STREAM("lowxabs"<<lowxabspoint<<"   lowyabs" << lowyabspoint);
		  for(int i = lowxpoint ; i < lowxpoint+lowxabspoint ; i++){
			for(int j = lowypoint ; j < lowypoint +lowyabspoint ; j++){
				if (   FindDistanceToSegment(p1x, p1y, p2x, p2y, i,j) <= 2.0){
//					ROS_INFO_STREAM("x,y""COST"<<(int)costmap->getCost(i,j));
//					if ( (int)costmap->getCost(i,j) >= 253) {
					if ( (int)costmap->getCost(i,j) >= 128) {
						clear_path = false;
					}
				}
			}
		  }
		  return clear_path;
      }

    inline bool isPointInRange(int x1,int y1,int x2,int y2,int start_deg,int end_deg){
        double xdiff = abs(x2-x1);
        double ydiff = abs(y2-y1);
        double theta;
            theta = atan(ydiff/xdiff);
        if( xdiff > 0   &&  ydiff > 0){ //QUAD 1
            theta = atan(ydiff/xdiff);
        }
        else if( xdiff < 0   &&  ydiff > 0){ //QUAD 2
            theta = M_PI - atan(ydiff/xdiff);
        }
        else if( xdiff < 0   &&  ydiff < 0){ //QUAD 3
            theta = atan(ydiff/xdiff) + M_PI;
        }
        else if( xdiff > 0   &&  ydiff < 0){ //QUAD 4
            theta =2*M_PI - atan(ydiff/xdiff);
        }

        if( theta > start_deg && theta < end_deg){
            return true;
        }
        return false;
    }
    inline double degToPoint(int x1,int y1,int x2,int y2){
        double xdiff = abs(x2-x1);
        double ydiff = abs(y2-y1);
        double theta;
            theta = atan(ydiff/xdiff);
        if( xdiff > 0   &&  ydiff > 0){ //QUAD 1
            theta = atan(ydiff/xdiff);
        }
        else if( xdiff < 0   &&  ydiff > 0){ //QUAD 2
            theta = M_PI - atan(ydiff/xdiff);
        }
        else if( xdiff < 0   &&  ydiff < 0){ //QUAD 3
            theta = atan(ydiff/xdiff) + M_PI;
        }
        else if( xdiff > 0   &&  ydiff < 0){ //QUAD 4
            theta =2*M_PI - atan(ydiff/xdiff);
        }

        return theta;
    }

    pcl::PointCloud<pcl::PointXYZ> findLongestLineInCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double dist_treshold){

        // initialize PointClouds
        pcl::PointCloud<pcl::PointXYZ>::Ptr final (new pcl::PointCloud<pcl::PointXYZ>);
        std::vector<int> inliers;

        pcl::SampleConsensusModelLine<pcl::PointXYZ>::Ptr model_l (new pcl::SampleConsensusModelLine<pcl::PointXYZ> (cloud));

        ROS_INFO("Calling Ransac");
        pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model_l);
        ransac.setDistanceThreshold (dist_treshold);
        ransac.computeModel();
        ransac.getInliers(inliers);
        pcl::copyPointCloud<pcl::PointXYZ>(*cloud, inliers, *final);
        
        //for(int i = 0; i < final->points.size(); i++){
            //ROS_INFO_STREAM("Point"<< i<<" " <<final->points[i].x  <<  final->points[i].y  <<  final->points[i].z  <<std::endl);
        //}
        return *final;
    }

    inline pcl::PointCloud<pcl::PointXYZ> findInliers (int robotPoseX, int robotPoseY, double start_deg,double end_deg){
        //unsigned int robotPoseX = mx;
        //unsigned int robotPoseY = my;
        //double start_deg = 0;
        //double end_deg = M_PI;
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr final (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
        costmap_2d::Costmap2D* costmap = costmap_ros_->getCostmap();
        std::string global_frame = costmap_ros_->getGlobalFrameID();
        int mapx = costmap->getSizeInCellsX();
        int mapy = costmap->getSizeInCellsY();

        int iter = 0;
        for( int i = 0; i < mapx; i++){
            for( int j = 0; j < mapy; j++){
                if(costmap->getIndex(i,j) >= 254){
                    geometry_msgs::PoseStamped tempRobotPose;
                    tempRobotPose.header.frame_id = "map";
                    //tempRobotPose.pose.position.x = wx;
                    //tempRobotPose.pose.position.y = wy;
                    costmap->mapToWorld(robotPoseX,robotPoseY,tempRobotPose.pose.position.x, tempRobotPose.pose.position.y);

                    geometry_msgs::PoseStamped tempGoToLoc;
                    tempGoToLoc.header.frame_id = "map";
                    costmap->mapToWorld(i,j,tempGoToLoc.pose.position.x, tempGoToLoc.pose.position.y);
                    
                    //if(clear_path(tempRobotPose, tempGoToLoc)  && isPointInRange(robotPoseX,robotPoseY,i,j,start_deg,end_deg)){
                    if (iter%10000 == 0) {
                        ROS_INFO_STREAM("Calling isPointInRange"<<iter);
                    }
                    iter++;
                    if(isPointInRange(robotPoseX,robotPoseY,i,j,start_deg,end_deg)){
                        ROS_INFO_STREAM("Adding Point to cloud");
                        pcl::PointXYZ tempPoint;
                        tempPoint.x = i;
                        tempPoint.y = j;
                        tempPoint.z = 0;
                        cloud->push_back(tempPoint);
                    }
                }
            }
        }
        ROS_INFO("Calling findLongestLineInCloud");
        pcl::copyPointCloud<pcl::PointXYZ>( findLongestLineInCloud(cloud,4), *final);
        return *final;
    }


      void mapToWorld(double mx, double my, double& wx, double& wy);
      void clearRobotCell(const tf::Stamped<tf::Pose>& global_pose, unsigned int mx, unsigned int my);
      double planner_window_x_, planner_window_y_, default_tolerance_;
      std::string tf_prefix_;
      boost::mutex mutex_;
      ros::ServiceServer make_plan_srv_;
  };
};

#endif
