#ifndef MOVEIT_COLLISION_DETECTION_SAFE_FCL_COLLISION_WORLD_FCL_
#define MOVEIT_COLLISION_DETECTION_SAFE_FCL_COLLISION_WORLD_FCL_

#include <moveit/collision_detection_fcl/collision_world_fcl.h>

//STa
#include "fcl/octree.h"
#include "fcl/collision_object.h"


namespace collision_detection
{

  class SafeCollisionWorldFCL : public CollisionWorldFCL
  {
  public:

	SafeCollisionWorldFCL();
    explicit SafeCollisionWorldFCL(const WorldPtr& world);
    SafeCollisionWorldFCL(const CollisionWorldFCL &other, const WorldPtr& world);
    virtual ~SafeCollisionWorldFCL();


    virtual void checkRobotCollision(const CollisionRequest &req, CollisionResult &res, const CollisionRobot &robot, const robot_state::RobotState &state) const;
    virtual void checkRobotCollision(const CollisionRequest &req, CollisionResult &res, const CollisionRobot &robot, const robot_state::RobotState &state, const AllowedCollisionMatrix &acm) const;
    virtual void checkRobotCollision(const CollisionRequest &req, CollisionResult &res, const CollisionRobot &robot, const robot_state::RobotState &state1, const robot_state::RobotState &state2) const;
    virtual void checkRobotCollision(const CollisionRequest &req, CollisionResult &res, const CollisionRobot &robot, const robot_state::RobotState &state1, const robot_state::RobotState &state2, const AllowedCollisionMatrix &acm) const;
    virtual void checkWorldCollision(const CollisionRequest &req, CollisionResult &res, const CollisionWorld &other_world) const;
    virtual void checkWorldCollision(const CollisionRequest &req, CollisionResult &res, const CollisionWorld &other_world, const AllowedCollisionMatrix &acm) const;

    virtual double distanceRobot(const CollisionRobot &robot, const robot_state::RobotState &state) const;
    virtual double distanceRobot(const CollisionRobot &robot, const robot_state::RobotState &state, const AllowedCollisionMatrix &acm) const;
    virtual double distanceWorld(const CollisionWorld &world) const;
    virtual double distanceWorld(const CollisionWorld &world, const AllowedCollisionMatrix &acm) const;

    //STa
    std::vector<boost::shared_ptr<fcl::CollisionObject> > getCollisionObjects() const;
    std::vector<boost::shared_ptr<fcl::CollisionObject> > getCollisionObjects(std::string co_name) const;
    int getCollisionObjectIndex(std::string co_name) const;
    std::vector<std::string> getCollisionObjectNames() const;
    void checkRobotCollision(const CollisionRequest &req, CollisionResult &res, const CollisionRobot &robot, const robot_state::RobotState &state, const AllowedCollisionMatrix *acm, std::vector<std::string> current_link_names, std::size_t object_index) const;
    double distanceRobot(const CollisionRobot* robot, const robot_state::RobotState &state, const AllowedCollisionMatrix *acm, std::vector<std::string> current_link_names, std::size_t object_index) const;
    double distanceRobot(const CollisionRobot* robot, const robot_state::RobotState &state, const AllowedCollisionMatrix *acm, std::vector<std::string> current_link_names, std::size_t object_index, fcl::DistanceResult& result) const;
    double distanceRobot(const CollisionRobot &robot, const robot_state::RobotState &state, const AllowedCollisionMatrix &acm, std::string group_name) const;
    void generateBoxesFromOctomap(std::vector<fcl::CollisionObject*>& boxes, const fcl::OcTree* tree) const;


  protected:

    //STa
    void allocCollisionBroadPhase(FCLManager &manager) const;
    double distanceRobotHelper(const CollisionRobot &robot, const robot_state::RobotState &state, const AllowedCollisionMatrix *acm, std::string group_name = "") const;

  private:
//    void initialize();
//    void notifyObjectChange(const ObjectConstPtr& obj, World::Action action);
//    World::ObserverHandle observer_handle_;
  };

}

#endif
