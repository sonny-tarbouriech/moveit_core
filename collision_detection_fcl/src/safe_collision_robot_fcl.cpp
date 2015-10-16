/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
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
 *   * Neither the name of Willow Garage nor the names of its
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
 *********************************************************************/


#include <moveit/collision_detection_fcl/safe_collision_robot_fcl.h>

//STa test
#include <geometric_shapes/shape_operations.h>


collision_detection::SafeCollisionRobotFCL::SafeCollisionRobotFCL(const robot_model::RobotModelConstPtr &model, double padding, double scale)
  : CollisionRobotFCL(model, padding, scale)
{
  const std::vector<const robot_model::LinkModel*>& links = robot_model_->getLinkModelsWithCollisionGeometry();
  geoms_.resize(robot_model_->getLinkGeometryCount());

  //STa
  link_names_.resize(robot_model_->getLinkGeometryCount());

  // we keep the same order of objects as what RobotState *::getLinkState() returns
  for (std::size_t i = 0 ; i < links.size() ; ++i)
      for (std::size_t j = 0 ; j < links[i]->getShapes().size() ; ++j)
      {
          FCLGeometryConstPtr g;
          if (links[i]->getShapes()[j]->type != shapes::MESH)
          {
              const shapes::Cylinder* cyl = static_cast<const shapes::Cylinder*>(links[i]->getShapes()[j].get());

              shapes::Mesh* mesh = createMeshFromShape(links[i]->getShapes()[j].get());
              shapes::ShapeConstPtr shape_cptr(mesh);
              g = createCollisionGeometry(shape_cptr, getLinkScale(links[i]->getName()), getLinkPadding(links[i]->getName()), links[i], j);
          }
          else
              g = createCollisionGeometry(links[i]->getShapes()[j], getLinkScale(links[i]->getName()), getLinkPadding(links[i]->getName()), links[i], j);


          //      FCLGeometryConstPtr g = createCollisionGeometry(links[i]->getShapes()[j], getLinkScale(links[i]->getName()), getLinkPadding(links[i]->getName()), links[i], j);
          if (g)
          {
              geoms_[links[i]->getFirstCollisionBodyTransformIndex() + j] = g;

              //STa
              link_names_[links[i]->getFirstCollisionBodyTransformIndex() + j] = links[i]->getName();
          }
          else
              logError("Unable to construct collision geometry for link '%s'", links[i]->getName().c_str());

      }
}

collision_detection::SafeCollisionRobotFCL::SafeCollisionRobotFCL(const CollisionRobotFCL &other) : CollisionRobotFCL(other)
{
  const std::vector<const robot_model::LinkModel*>& links = robot_model_->getLinkModelsWithCollisionGeometry();
  link_names_.resize(robot_model_->getLinkGeometryCount());
  for (std::size_t i = 0 ; i < links.size() ; ++i)
      for (std::size_t j = 0 ; j < links[i]->getShapes().size() ; ++j)
      {
    	  if (geoms_[links[i]->getFirstCollisionBodyTransformIndex() + j])
    		  link_names_[links[i]->getFirstCollisionBodyTransformIndex() + j] = links[i]->getName();
      }
}

void collision_detection::SafeCollisionRobotFCL::allocSelfCollisionBroadPhase(FCLManager &manager) const
{
  fcl::DynamicAABBTreeCollisionManager* m = new fcl::DynamicAABBTreeCollisionManager();
  manager.manager_.reset(m);
}

void collision_detection::SafeCollisionRobotFCL::constructFCLObject(const robot_state::RobotState &state, FCLObject &fcl_obj, std::vector<std::string> link_names) const
{
	for (std::size_t i = 0 ; i < geoms_.size() ; ++i)
		if (std::find(link_names.begin(), link_names.end(), link_names_[i]) != link_names.end())
		{
			if (geoms_[i] && geoms_[i]->collision_geometry_)
			{
				fcl::CollisionObject *collObj = new fcl::CollisionObject
						(geoms_[i]->collision_geometry_, transform2fcl(state.getCollisionBodyTransform(geoms_[i]->collision_geometry_data_->ptr.link, geoms_[i]->collision_geometry_data_->shape_index)));
				fcl_obj.collision_objects_.push_back(boost::shared_ptr<fcl::CollisionObject>(collObj));
				// the CollisionGeometryData is already stored in the class member geoms_, so we need not copy it
			}
		}
}

void collision_detection::SafeCollisionRobotFCL::checkSelfCollision(const CollisionRequest &req, CollisionResult &res, const robot_state::RobotState &state,
		const AllowedCollisionMatrix *acm, std::vector<std::string> current_link_names, std::vector<std::string> other_link_names) const
{
	FCLManager manager, other_manager;

	allocSelfCollisionBroadPhase(manager);
	allocSelfCollisionBroadPhase(other_manager);

	constructFCLObject(state, manager.object_, current_link_names);
	manager.object_.registerTo(manager.manager_.get());

	constructFCLObject(state, other_manager.object_, other_link_names);
	other_manager.object_.registerTo(other_manager.manager_.get());

	CollisionData cd(&req, &res, acm);
	cd.enableGroup(getRobotModel());

	manager.manager_->collide(other_manager.manager_.get(), &cd, &collisionCallback);
	if (req.distance)
		manager.manager_->distance(other_manager.manager_.get(), &cd, &distanceCallback);
}

void collision_detection::SafeCollisionRobotFCL::checkSelfCollision(const CollisionRequest &req, CollisionResult &res, const robot_state::RobotState &state, const AllowedCollisionMatrix *acm, std::vector<std::string> current_link_names, std::string other_link_name) const
{
	FCLManager manager;

	allocSelfCollisionBroadPhase(manager);

	constructFCLObject(state, manager.object_, current_link_names);
	manager.object_.registerTo(manager.manager_.get());

	FCLObject other_obj;
	std::vector<std::string> other_link_name_vec;
	other_link_name_vec.push_back(other_link_name);
	constructFCLObject(state, other_obj, other_link_name_vec);

	CollisionData cd(&req, &res, acm);
	cd.enableGroup(getRobotModel());

	manager.manager_->collide(other_obj.collision_objects_[0].get(), &cd, &collisionCallback);

	if (req.distance)
		manager.manager_->distance(other_obj.collision_objects_[0].get(), &cd, &distanceCallback);
}

double collision_detection::SafeCollisionRobotFCL::distanceSelf(const robot_state::RobotState &state,
                                                                  const AllowedCollisionMatrix *acm, std::vector<std::string> current_link_names, std::vector<std::string> other_link_names) const
{
	FCLManager manager, other_manager;

	allocSelfCollisionBroadPhase(manager);
	allocSelfCollisionBroadPhase(other_manager);

	constructFCLObject(state, manager.object_, current_link_names);
	manager.object_.registerTo(manager.manager_.get());

	constructFCLObject(state, other_manager.object_, other_link_names);
	other_manager.object_.registerTo(other_manager.manager_.get());

	CollisionRequest req;
	CollisionResult res;
	CollisionData cd(&req, &res, acm);
	cd.enableGroup(getRobotModel());

	manager.manager_->distance(other_manager.manager_.get(), &cd, &distanceCallback);

	return res.distance;
}

double collision_detection::SafeCollisionRobotFCL::distanceSelf(const robot_state::RobotState &state, const AllowedCollisionMatrix *acm, std::vector<std::string> current_link_names, std::string other_link_name) const
{
	FCLManager manager;

	allocSelfCollisionBroadPhase(manager);

	constructFCLObject(state, manager.object_, current_link_names);
	manager.object_.registerTo(manager.manager_.get());

	FCLObject other_obj;
	std::vector<std::string> other_link_name_vec;
	other_link_name_vec.push_back(other_link_name);
	constructFCLObject(state, other_obj, other_link_name_vec);

	CollisionRequest req;
	CollisionResult res;
	CollisionData cd(&req, &res, acm);
	cd.enableGroup(getRobotModel());

	manager.manager_->distance(other_obj.collision_objects_[0].get(), &cd, &distanceCallback);

	return res.distance;
}


void collision_detection::SafeCollisionRobotFCL::checkSelfCollision(const CollisionRequest &req, CollisionResult &res, const robot_state::RobotState &state) const
{
	CollisionRobotFCL::checkSelfCollisionHelper(req, res, state, NULL);
}

void collision_detection::SafeCollisionRobotFCL::checkSelfCollision(const CollisionRequest &req, CollisionResult &res, const robot_state::RobotState &state,
                                                                const AllowedCollisionMatrix &acm) const
{
	CollisionRobotFCL::checkSelfCollisionHelper(req, res, state, &acm);
}

void collision_detection::SafeCollisionRobotFCL::checkSelfCollision(const CollisionRequest &req, CollisionResult &res, const robot_state::RobotState &state1, const robot_state::RobotState &state2) const
{
  logError("FCL continuous collision checking not yet implemented");
}

void collision_detection::SafeCollisionRobotFCL::checkSelfCollision(const CollisionRequest &req, CollisionResult &res, const robot_state::RobotState &state1, const robot_state::RobotState &state2, const AllowedCollisionMatrix &acm) const
{
  logError("FCL continuous collision checking not yet implemented");
}

void collision_detection::SafeCollisionRobotFCL::checkOtherCollision(const CollisionRequest &req, CollisionResult &res, const robot_state::RobotState &state,
                                                                 const CollisionRobot &other_robot, const robot_state::RobotState &other_state) const
{
  checkOtherCollisionHelper(req, res, state, other_robot, other_state, NULL);
}

void collision_detection::SafeCollisionRobotFCL::checkOtherCollision(const CollisionRequest &req, CollisionResult &res, const robot_state::RobotState &state,
                                                                 const CollisionRobot &other_robot, const robot_state::RobotState &other_state,
                                                                 const AllowedCollisionMatrix &acm) const
{
  checkOtherCollisionHelper(req, res, state, other_robot, other_state, &acm);
}

void collision_detection::SafeCollisionRobotFCL::checkOtherCollision(const CollisionRequest &req, CollisionResult &res, const robot_state::RobotState &state1, const robot_state::RobotState &state2,
                                                                 const CollisionRobot &other_robot, const robot_state::RobotState &other_state1, const robot_state::RobotState &other_state2) const
{
  logError("FCL continuous collision checking not yet implemented");
}

void collision_detection::SafeCollisionRobotFCL::checkOtherCollision(const CollisionRequest &req, CollisionResult &res, const robot_state::RobotState &state1, const robot_state::RobotState &state2,
                                                                 const CollisionRobot &other_robot, const robot_state::RobotState &other_state1, const robot_state::RobotState &other_state2,
                                                                 const AllowedCollisionMatrix &acm) const
{
  logError("FCL continuous collision checking not yet implemented");
}

double collision_detection::SafeCollisionRobotFCL::distanceSelf(const robot_state::RobotState &state) const
{
  return CollisionRobotFCL::distanceSelfHelper(state, NULL);
}

double collision_detection::SafeCollisionRobotFCL::distanceSelf(const robot_state::RobotState &state,
                                                            const AllowedCollisionMatrix &acm) const
{
  return CollisionRobotFCL::distanceSelfHelper(state, &acm);
}

double collision_detection::SafeCollisionRobotFCL::distanceOther(const robot_state::RobotState &state,
                                                             const CollisionRobot &other_robot,
                                                             const robot_state::RobotState &other_state) const
{
  return distanceOtherHelper(state, other_robot, other_state, NULL);
}

double collision_detection::SafeCollisionRobotFCL::distanceOther(const robot_state::RobotState &state,
                                                             const CollisionRobot &other_robot,
                                                             const robot_state::RobotState &other_state,
                                                             const AllowedCollisionMatrix &acm) const
{
  return distanceOtherHelper(state, other_robot, other_state, &acm);
}
