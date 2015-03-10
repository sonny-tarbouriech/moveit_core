#include <moveit/collision_detection_fcl/safe_collision_world_fcl.h>
#include <moveit/collision_detection_fcl/safe_collision_robot_fcl.h>

//STa temp
#include <fstream>

collision_detection::SafeCollisionWorldFCL::SafeCollisionWorldFCL() :
	CollisionWorldFCL()
{

}

collision_detection::SafeCollisionWorldFCL::SafeCollisionWorldFCL(const WorldPtr& world) :
	CollisionWorldFCL(world)
{

}

collision_detection::SafeCollisionWorldFCL::SafeCollisionWorldFCL(const CollisionWorldFCL &other, const WorldPtr& world) :
		CollisionWorldFCL(other, world)
{

}

collision_detection::SafeCollisionWorldFCL::~SafeCollisionWorldFCL()
{
}

void collision_detection::SafeCollisionWorldFCL::checkRobotCollision(const CollisionRequest &req, CollisionResult &res, const CollisionRobot &robot, const robot_state::RobotState &state) const
{
  checkRobotCollisionHelper(req, res, robot, state, NULL);
}

void collision_detection::SafeCollisionWorldFCL::checkRobotCollision(const CollisionRequest &req, CollisionResult &res, const CollisionRobot &robot, const robot_state::RobotState &state, const AllowedCollisionMatrix &acm) const
{
  checkRobotCollisionHelper(req, res, robot, state, &acm);
}

void collision_detection::SafeCollisionWorldFCL::checkRobotCollision(const CollisionRequest &req, CollisionResult &res, const CollisionRobot &robot, const robot_state::RobotState &state1, const robot_state::RobotState &state2) const
{
  logError("FCL continuous collision checking not yet implemented");
}

void collision_detection::SafeCollisionWorldFCL::checkRobotCollision(const CollisionRequest &req, CollisionResult &res, const CollisionRobot &robot, const robot_state::RobotState &state1, const robot_state::RobotState &state2, const AllowedCollisionMatrix &acm) const
{
  logError("FCL continuous collision checking not yet implemented");
}

void collision_detection::SafeCollisionWorldFCL::allocCollisionBroadPhase(FCLManager &manager) const
{
  fcl::DynamicAABBTreeCollisionManager* m = new fcl::DynamicAABBTreeCollisionManager();
  manager.manager_.reset(m);
}

void collision_detection::SafeCollisionWorldFCL::checkRobotCollision(const CollisionRequest &req, CollisionResult &res, const CollisionRobot &robot, const robot_state::RobotState &state, const AllowedCollisionMatrix *acm,
		std::vector<std::string> current_link_names, std::size_t object_index) const
{
	FCLManager link_manager;
	allocCollisionBroadPhase(link_manager);

	const SafeCollisionRobotFCL &safe_robot_fcl = dynamic_cast<const SafeCollisionRobotFCL&>(robot);

	safe_robot_fcl.constructFCLObject(state, link_manager.object_, current_link_names);

	CollisionData cd(&req, &res, acm);
	cd.enableGroup(robot.getRobotModel());

	std::vector<fcl::CollisionObject*> fcl_collision_obj;
	manager_->getObjects(fcl_collision_obj);

	link_manager.manager_->collide(fcl_collision_obj[object_index], &cd, &collisionCallback);

	if (req.distance)
		link_manager.manager_->distance(fcl_collision_obj[object_index], &cd, &distanceCallback);
}

void collision_detection::SafeCollisionWorldFCL::checkWorldCollision(const CollisionRequest &req, CollisionResult &res, const CollisionWorld &other_world) const
{
  checkWorldCollisionHelper(req, res, other_world, NULL);
}

void collision_detection::SafeCollisionWorldFCL::checkWorldCollision(const CollisionRequest &req, CollisionResult &res, const CollisionWorld &other_world, const AllowedCollisionMatrix &acm) const
{
  checkWorldCollisionHelper(req, res, other_world, &acm);
}

double collision_detection::SafeCollisionWorldFCL::distanceRobot(const CollisionRobot* robot, const robot_state::RobotState &state, const AllowedCollisionMatrix *acm,
		std::vector<std::string> current_link_names, std::size_t object_index) const
{
//	ros::WallTime start = ros::WallTime::now();

	FCLManager link_manager;
	allocCollisionBroadPhase(link_manager);

	const SafeCollisionRobotFCL* safe_robot_fcl = static_cast<const SafeCollisionRobotFCL*>(robot);

//	ros::WallTime flag1 = ros::WallTime::now();

	safe_robot_fcl->constructFCLObject(state, link_manager.object_, current_link_names);
	link_manager.object_.registerTo(link_manager.manager_.get());

//	ros::WallTime flag2 = ros::WallTime::now();

	CollisionRequest req;
	CollisionResult res;
	CollisionData cd(&req, &res, acm);
	cd.enableGroup(robot->getRobotModel());

//	ros::WallTime flag3 = ros::WallTime::now();

	std::vector<fcl::CollisionObject*> fcl_collision_obj;
	manager_->getObjects(fcl_collision_obj);

//	ros::WallTime flag4 = ros::WallTime::now();

	link_manager.manager_->distance(fcl_collision_obj[object_index], &cd, &distanceCallback);

//	ros::WallTime flag5 = ros::WallTime::now();

//	//STa test temp
//	std::string homepath = getenv("HOME");
//	std::ofstream output_file_((homepath + "/time2.txt").c_str(), std::ios::out | std::ios::app);
//	if (output_file_)
//	{
//		output_file_
//		<< "flag1 " << flag1 - start <<  "\n"
//		<< "flag2 " << flag2 - start <<  "\n"
//		<< "flag3 " << flag3 - start <<  "\n"
//		<< "flag4 " << flag4 - start <<  "\n"
//		<< "flag5 " << flag5 - start <<  "\n \n";
//		output_file_.close();
//	}

	return res.distance;
}

double collision_detection::SafeCollisionWorldFCL::distanceRobot(const CollisionRobot &robot, const robot_state::RobotState &state) const
{
  return distanceRobotHelper(robot, state, NULL, "");
}

double collision_detection::SafeCollisionWorldFCL::distanceRobot(const CollisionRobot &robot, const robot_state::RobotState &state, const AllowedCollisionMatrix &acm) const
{
	return distanceRobotHelper(robot, state, &acm, "");
}

double collision_detection::SafeCollisionWorldFCL::distanceRobot(const CollisionRobot &robot, const robot_state::RobotState &state, const AllowedCollisionMatrix &acm, std::string group_name) const
{
	return distanceRobotHelper(robot, state, &acm, group_name);
}


double collision_detection::SafeCollisionWorldFCL::distanceRobotHelper(const CollisionRobot &robot, const robot_state::RobotState &state, const AllowedCollisionMatrix *acm, std::string group_name) const
{
  const SafeCollisionRobotFCL& robot_fcl = dynamic_cast<const SafeCollisionRobotFCL&>(robot);
  FCLObject fcl_obj;
  robot_fcl.CollisionRobotFCL::constructFCLObject(state, fcl_obj);

  CollisionRequest req;

  if (!group_name.empty())
	  req.group_name = "group_name";

  CollisionResult res;
  CollisionData cd(&req, &res, acm);
  cd.enableGroup(robot.getRobotModel());

  for(std::size_t i = 0; !cd.done_ && i < fcl_obj.collision_objects_.size(); ++i)
    manager_->distance(fcl_obj.collision_objects_[i].get(), &cd, &distanceCallback);


  return res.distance;
}

//STa
std::vector<fcl::CollisionObject*> collision_detection::SafeCollisionWorldFCL::getCollisionObjects() const
{
	std::vector<fcl::CollisionObject*> fcl_collision_obj;
	manager_->getObjects(fcl_collision_obj);
	return fcl_collision_obj;
}

double collision_detection::SafeCollisionWorldFCL::distanceWorld(const CollisionWorld &world) const
{
  return distanceWorldHelper(world, NULL);
}

double collision_detection::SafeCollisionWorldFCL::distanceWorld(const CollisionWorld &world, const AllowedCollisionMatrix &acm) const
{
  return distanceWorldHelper(world, &acm);
}

#include <moveit/collision_detection_fcl/safe_collision_detector_allocator_fcl.h>
const std::string collision_detection::SafeCollisionDetectorAllocatorFCL::NAME_("SAFE_FCL");

