#ifndef PLANNING_ALGORITHM_SRC_GRAPH_SEARCH_PLANNER_INCLUDE_HYBRID_A_STAR_HPP_
#define PLANNING_ALGORITHM_SRC_GRAPH_SEARCH_PLANNER_INCLUDE_HYBRID_A_STAR_HPP_

#include <ros/ros.h>
#include <utility>
#include <vector>
#include <algorithm>
#include <unordered_map>
#include <memory>
#include <string>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/Path.h>
#include <queue>
#include <boost/heap/binomial_heap.hpp>
#include "a_star.hpp"
namespace planning {

struct HybridAStarResult {
  std::vector<double> x;
  std::vector<double> y;
  std::vector<double> phi;
  std::vector<double> v;
  std::vector<double> a;
  std::vector<double> steer;
  std::vector<double> accumulated_s;
};

class Node3d {
 public:

  Node3d(double x, double y, double phi);
  Node3d(double x, double y, double phi,
         nav_msgs::OccupancyGrid::Ptr grid_map,
         double phi_resolution);
  Node3d(const std::vector<double> &traversed_x,
         const std::vector<double> &traversed_y,
         const std::vector<double> &traversed_phi,
         nav_msgs::OccupancyGrid::Ptr grid_map,
         double phi_resolution);
  ~Node3d() = default;
  bool operator==(const Node3d &other) const;
  ///////////////////////// getter ////////////////////////
  double GetX() const { return x_; }
  double GetY() const { return y_; }
  double GetPhi() const { return phi_; }
  double GetGCost() const { return gcost_; }
  double GetFCost() const { return fcost_; }
  double GetHCost() const { return hcost_; }
  bool GetDirection() const { return direction_; }
  double GetSteering() const { return steering_; }
  std::string GetIndex() const { return index_; }
  int GetIndexX() const { return index_x_; }
  int GetIndexY() const { return index_y_; }
  int GetIndexPhi() const { return index_phi_; }
  std::vector<double> GetXs() const { return traversed_x_; }
  std::vector<double> GetYs() const { return traversed_y_; }
  std::vector<double> GetPhis() const { return traversed_phi_; }
  size_t GetStepSize() const { return step_size_; }
  std::shared_ptr<Node3d> GetPreNode() const { return pre_node_; }
  //////////////////// setter ///////////////////////
  void SetGCost(double g) {
    gcost_ = g;
    fcost_ = gcost_ + hcost_;
  }
  void SetHCost(double h) {
    hcost_ = h;
    fcost_ = hcost_ + gcost_;
  }
  void SetDirection(bool dir) { this->direction_ = dir; }
  void SetSteering(double steering) { this->steering_ = steering; }
  void SetPreNode(std::shared_ptr<Node3d> pre_node) { this->pre_node_ = std::move(pre_node); }
  bool IsInRange(const std::shared_ptr<Node3d> &goal, double analytic_expand_range) const {
    double dis = std::hypot(goal->GetY() - y_, goal->GetX() - x_);
    return dis <= analytic_expand_range;
  }

 private:
  double x_ = 0.0;
  double y_ = 0.0;
  double phi_ = 0.0;
  int index_x_ = 0;
  int index_y_ = 0;
  int index_phi_ = 0;
  std::string index_;
  std::vector<double> traversed_x_;
  std::vector<double> traversed_y_;
  std::vector<double> traversed_phi_;
  size_t step_size_ = 1;
  double hcost_{};
  double gcost_{};
  double fcost_{};
  std::shared_ptr<Node3d> pre_node_ = nullptr;
  // true move forward, false, move backward
  bool direction_ = true;
  double steering_ = 0.0;

};

class HybridAStar {
 public:
  explicit HybridAStar(const ros::NodeHandle &nh);
  ~HybridAStar() = default;
  void RunOnce();

 protected:
  std::shared_ptr<Node3d> ReedsSheppShot(const std::shared_ptr<Node3d> &node, double *len);
  std::shared_ptr<Node3d> DubinShot(const std::shared_ptr<Node3d> &node, double *len);
  bool Search(double sx, double sy, double sphi, double ex,
              double ey, double ephi, HybridAStarResult *result);
  bool TrajectoryPartition(const HybridAStarResult *result,
                           std::vector<HybridAStarResult> *parition_results) const;
  // anlytic expansion node
  bool AnalyticExpansion(const std::shared_ptr<Node3d> &current_node);
  // check the 3d node is valid
  bool CheckNode3d(const std::shared_ptr<Node3d> &node);
  // generate next node according to index
  std::shared_ptr<Node3d> NextNodeGenerator(
      const std::shared_ptr<Node3d> &current_node, size_t index);
  // heuristic with obstacle
  double HoloWithObstacleHeuristic(const std::shared_ptr<Node3d> &next_node);
  // non holonmoic heuristic without obstacle
  double NonHoloWithoutObstacleHeuristic(const std::shared_ptr<Node3d> &next_node) const;
  // calculate edge cost
  double EdgeCost(const std::shared_ptr<Node3d> &current_node,
                  const std::shared_ptr<Node3d> &next_node) const;

  struct cmp {
    bool operator()(const std::shared_ptr<Node3d> &left,
                    const std::shared_ptr<Node3d> &right) const;
  };

  void CalcNodCost(const std::shared_ptr<Node3d> &current_node, const std::shared_ptr<Node3d> &next_node);
  void SetGoal(const geometry_msgs::PoseStamped::Ptr &goal);
  void SetStart(const geometry_msgs::PoseWithCovarianceStamped::Ptr &start);
  void SetMap(const nav_msgs::OccupancyGrid::Ptr &grid_map);
  bool LoadResult(HybridAStarResult *result);
  static nav_msgs::Path HybridAStarResult2NavPath(const HybridAStarResult &result);
  static nav_msgs::Path AStarResult2NavPath(const AStarResult &astar_result);
  void VisualizedPath(const nav_msgs::Path &path);

 private:
  bool has_grid_map_ = false;
  bool valid_start_ = false;
  bool valid_goal_ = false;
  bool enable_backward_ = false;
  bool has_dp_map_ = false;
  geometry_msgs::PoseWithCovarianceStamped start_pose_;
  geometry_msgs::PoseStamped goal_pose_;
  ros::Publisher ha_star_path_publisher_;
  ros::Publisher visulized_path_publisher_;
  ros::Publisher visualized_vehicle_publisher_;
  ros::Subscriber grid_map_subscriber_;
  ros::Subscriber goal_pose_subscriber_;
  ros::Subscriber init_pose_subscriber_;
  int next_node_num_ = 0;
  double step_size_ = 0.0;
  double max_steer_angle_ = 0.0;
  double adc_width_ = 0.0;
  double adc_length_ = 0.0;
  double axle_ref_x_ = 0.0;
  double wheel_base_ = 0.0;
  double xy_resolution_ = 0.0;
  double phi_resolution_ = 0.0;
  double delta_steer_ = 0.0;
  double traj_forward_penalty_ = 0.0;
  double traj_backward_penalty_ = 0.0;
  double traj_gear_switch_penalty_ = 0.0;
  double traj_steer_penalty_ = 0.0;
  double traj_steer_change_penalty_ = 0.0;
  double delta_t_ = 0.1;
  double traj_step_length_penalty_ = 0.0;
//    double max_cost_ = 1000.0;
  double max_cost_ = 1.0;
  double anlytic_expand_distance_ = 0.0;
  std::shared_ptr<Node3d> start_node_ = nullptr;
  std::shared_ptr<Node3d> end_node_ = nullptr;
  std::shared_ptr<Node3d> final_node_ = nullptr;
  nav_msgs::OccupancyGrid::Ptr grid_map_;
  std::unique_ptr<AStar> heuristic_generator_;
//    std::priority_queue<std::shared_ptr<Node3d>, std::vector<std::shared_ptr<Node3d>>, cmp> open_pq_;
  boost::heap::binomial_heap<std::shared_ptr<Node3d>, boost::heap::compare<cmp>> priority_queue_;
  std::unordered_map<
      std::string, boost::heap::binomial_heap<std::shared_ptr<Node3d>, boost::heap::compare<cmp>>::handle_type>
      open_set_handles_;
  std::unordered_map<std::string, std::shared_ptr<Node3d>> open_set_;
  std::unordered_map<std::string, std::shared_ptr<Node3d>> closed_set_;
  ros::NodeHandle nh_;
};

}

#endif //PLANNING_ALGORITHM_SRC_GRAPH_SEARCH_PLANNER_INCLUDE_HYBRID_A_STAR_HPP_
