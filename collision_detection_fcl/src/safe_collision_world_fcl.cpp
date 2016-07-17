#include <moveit/collision_detection_fcl/safe_collision_world_fcl.h>
#include <moveit/collision_detection_fcl/safe_collision_robot_fcl.h>

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

void collision_detection::SafeCollisionWorldFCL::generateBoxesFromOctomap(std::vector<fcl::CollisionObject*>& boxes, const fcl::OcTree* tree) const
{
  std::vector<boost::array<fcl::FCL_REAL, 6> > boxes_ = tree->toBoxes();

  for(std::size_t i = 0; i < boxes_.size(); ++i)
  {
	  fcl::FCL_REAL x = boxes_[i][0];
	  fcl::FCL_REAL y = boxes_[i][1];
	  fcl::FCL_REAL z = boxes_[i][2];
	  fcl::FCL_REAL size = boxes_[i][3];
	  fcl::FCL_REAL cost = boxes_[i][4];
	  fcl::FCL_REAL threshold = boxes_[i][5];

	  fcl::Box* box = new fcl::Box(size, size, size);
    box->cost_density = cost;
    box->threshold_occupied = threshold;
    fcl::CollisionObject* obj = new fcl::CollisionObject(boost::shared_ptr<fcl::CollisionGeometry>(box), fcl::Transform3f(fcl::Vec3f(x, y, z)));
    boxes.push_back(obj);
  }

  std::cout << "boxes size: " << boxes.size() << std::endl;

}

double collision_detection::SafeCollisionWorldFCL::distanceRobot(const CollisionRobot* robot, const robot_state::RobotState &state, const AllowedCollisionMatrix *acm,
		std::vector<std::string> current_link_names, std::size_t object_index) const
{
	FCLManager link_manager;
	allocCollisionBroadPhase(link_manager);

	const SafeCollisionRobotFCL* safe_robot_fcl = static_cast<const SafeCollisionRobotFCL*>(robot);

	safe_robot_fcl->constructFCLObject(state, link_manager.object_, current_link_names);
	link_manager.object_.registerTo(link_manager.manager_.get());

	CollisionRequest req;
	CollisionResult res;
	CollisionData cd(&req, &res, acm);
	cd.enableGroup(robot->getRobotModel());

	boost::shared_ptr<fcl::CollisionObject> co = getCollisionObjects()[object_index];

	if (co->getObjectType() == 3)
	{
		std::vector<fcl::CollisionObject*> boxes;
		const fcl::OcTree* tree = static_cast<const fcl::OcTree*>(co->collisionGeometry().get());
		generateBoxesFromOctomap(boxes, tree);
		for(std::size_t i = 0; !cd.done_ && i < boxes.size(); ++i)
		{
			  link_manager.manager_->distance(boxes[i], &cd, &distanceCallback);
		}
	}
	else
		link_manager.manager_->distance(getCollisionObjects()[object_index].get(), &cd, &distanceCallback);

	return res.distance;
}

double collision_detection::SafeCollisionWorldFCL::distanceRobot(const CollisionRobot* robot, const robot_state::RobotState &state, const AllowedCollisionMatrix *acm,
        std::vector<std::string> current_link_names, std::size_t object_index, fcl::DistanceResult& result) const
{
    FCLManager link_manager;
    allocCollisionBroadPhase(link_manager);

    const SafeCollisionRobotFCL* safe_robot_fcl = static_cast<const SafeCollisionRobotFCL*>(robot);

    safe_robot_fcl->constructFCLObject(state, link_manager.object_, current_link_names);
    link_manager.object_.registerTo(link_manager.manager_.get());

    result.min_distance = std::numeric_limits<double>::infinity();
    fcl::DistanceResult result_temp;
    for (size_t i=0; i < link_manager.object_.collision_objects_.size(); ++i)
    {
        result_temp.clear();
        fcl::DistanceRequest request;
        request.enable_nearest_points = true;
        fcl::distance(getCollisionObjects()[object_index].get(), link_manager.object_.collision_objects_[i].get(), request, result_temp);

        if (result_temp.min_distance < result.min_distance)
            result = result_temp;
    }

    return result.min_distance;
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
std::vector<boost::shared_ptr<fcl::CollisionObject> > collision_detection::SafeCollisionWorldFCL::getCollisionObjects() const
{
	std::vector<boost::shared_ptr<fcl::CollisionObject> > fcl_collision_obj;
	for(std::map<std::string, FCLObject >::const_iterator it = fcl_objs_.begin(); it != fcl_objs_.end(); ++it)
	{
    	for (size_t i=0; i < it->second.collision_objects_.size(); ++i)
    		fcl_collision_obj.push_back(it->second.collision_objects_[i]);
	}
	return fcl_collision_obj;
}

std::vector<boost::shared_ptr<fcl::CollisionObject> > collision_detection::SafeCollisionWorldFCL::getCollisionObjects(std::string co_name) const
{
	std::vector<boost::shared_ptr<fcl::CollisionObject> > fcl_collision_obj;

    if(fcl_objs_.find(co_name) != fcl_objs_.end())
    {

        for (size_t i=0; i < fcl_objs_.find(co_name)->second.collision_objects_.size(); ++i)
            fcl_collision_obj.push_back(fcl_objs_.find(co_name)->second.collision_objects_[i]);
    }

    return fcl_collision_obj;
}

int collision_detection::SafeCollisionWorldFCL::getCollisionObjectIndex(std::string co_name) const
{
    size_t index = 0;
    for(std::map<std::string, FCLObject >::const_iterator it = fcl_objs_.begin(); it != fcl_objs_.end(); ++it)
    {
        if (co_name.compare(it->first) == 0)
            return index;
        index++;
    }
    return -1;
}

std::vector<std::string> collision_detection::SafeCollisionWorldFCL::getCollisionObjectNames() const
{

	std::vector<std::string> s;
	for(std::map<std::string, FCLObject >::const_iterator it = fcl_objs_.begin(); it != fcl_objs_.end(); ++it)
	{
		for (size_t i=0; i < it->second.collision_objects_.size(); ++i)
			s.push_back(it->first);

	}
	return s;
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

