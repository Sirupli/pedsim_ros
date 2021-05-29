/*
 * @name	 	scene_services.cpp
 * @brief	 	Provides services to spawn and remove pedestrians dynamically and add static obstacle spawning. 
 *          The spawned agents are forwarded to flatland
 * @author 	Ronja Gueldenring
 * @date 		2019/04/05
 * @author Junhui Li
 * @date 24.3.2021
 **/

#include <pedsim_simulator/element/agent.h>
#include <pedsim_simulator/element/agentcluster.h>
#include <pedsim_simulator/element/areawaypoint.h>
#include <pedsim_simulator/element/obstacle.h>
#include <pedsim_simulator/scene_services.h>
#include <geometry_msgs/Point.h>
#include <flatland_msgs/SpawnModels.h>
#include <flatland_msgs/DeleteModels.h>
#include <flatland_msgs/RespawnModels.h>
#include <iostream>
#include <ros/package.h>




SceneServices::SceneServices(){
  //Pedsim service
  spawn_ped_service_ =
      nh.advertiseService("pedsim_simulator/spawn_ped", &SceneServices::spawnPed, this);
  respawn_peds_service_ =
      nh.advertiseService("pedsim_simulator/respawn_peds", &SceneServices::respawnPeds, this);
  remove_all_peds_service_ = nh.advertiseService("pedsim_simulator/remove_all_peds", &SceneServices::removeAllPeds, this);
  remove_all_polygons_service_ = nh.advertiseService("pedsim_simulator/remove_all_polygons", &SceneServices::removeAllPolygons, this);
  add_obstacle_service_= nh.advertiseService("pedsim_simulator/add_obstacle", &SceneServices::addStaticObstacles, this);
  move_peds_service_= nh.advertiseService("pedsim_simulator/move_peds", &SceneServices::moveAgentClustersInPedsim, this);
  add_polygon_service_= nh.advertiseService("pedsim_simulator/add_polygon", &SceneServices::spawnPolygons, this);



  
  //flatland service clients
  spawn_model_topic = ros::this_node::getNamespace() + "/spawn_models";
  spawn_agents_ = nh.serviceClient<flatland_msgs::SpawnModels>(spawn_model_topic, true);
  respawn_model_topic = ros::this_node::getNamespace() + "/respawn_models";
  respawn_agents_ = nh.serviceClient<flatland_msgs::RespawnModels>(respawn_model_topic, true);
  delete_model_topic = ros::this_node::getNamespace() + "/delete_models";
  delete_agents_ = nh.serviceClient<flatland_msgs::DeleteModels>(delete_model_topic, true);
  flatland_path_ = ros::package::getPath("simulator_setup");
  // initialize values
  last_id_ = 0;
  last_id_polygon_=0;  
}


bool SceneServices::spawnPed(pedsim_srvs::SpawnPeds::Request &request,
                                pedsim_srvs::SpawnPeds::Response &response) {
      flatland_msgs::SpawnModels srv;
      ros::WallTime start = ros::WallTime::now();
      for (int ped_i = 0; ped_i < (int)request.peds.size(); ped_i++){
        pedsim_msgs::Ped ped = request.peds[ped_i];
             std::vector<flatland_msgs::Model> new_models = addAgentClusterToPedsim(ped);
        srv.request.models.insert(srv.request.models.end(), new_models.begin(), new_models.end());
      }
      start = ros::WallTime::now();
      while(!spawn_agents_.isValid()){
          ROS_WARN("Reconnecting spawn_agents_-server....");
          spawn_agents_.waitForExistence(ros::Duration(5.0));
          spawn_agents_ = nh.serviceClient<flatland_msgs::SpawnModels>(spawn_model_topic, true);
      }
      spawn_agents_.call(srv);
      if (!srv.response.success)
      {
          response.finished = false;
          ROS_ERROR("Failed to spawn all %d agents", int(request.peds.size()));
      }
      response.finished = true;
      return true;
}

bool SceneServices::respawnPeds(pedsim_srvs::SpawnPeds::Request &request,
                                pedsim_srvs::SpawnPeds::Response &response){
  
  ros::Time begin = ros::Time::now();
  flatland_msgs::RespawnModels srv;
  srv.request.old_model_names = removePedsInPedsim();
  for (int ped_i = 0; ped_i < (int)request.peds.size(); ped_i++){
    pedsim_msgs::Ped ped = request.peds[ped_i];
    std::vector<flatland_msgs::Model> new_models = addAgentClusterToPedsim(ped);
    srv.request.new_models.insert(srv.request.new_models.end(), new_models.begin(), new_models.end());
  }
  response.finished=false;
  int count=0;

  begin = ros::Time::now();
  while(!respawn_agents_.isValid()){
    ROS_DEBUG("Reconnecting spawn_agents_-server....");
    respawn_agents_.waitForExistence(ros::Duration(5.0));
    respawn_agents_ = nh.serviceClient<flatland_msgs::RespawnModels>(respawn_model_topic, true);
  } 
  while(!response.finished&&count<10){
  respawn_agents_.call(srv);
  if (!srv.response.success)
  {
      ROS_ERROR("Failed to respawn all %d humans", int(request.peds.size()));
      response.finished=false;
      count++;
  }else{
    response.finished = true;
    break;
  } 

  }

  return true;
}

bool SceneServices::removeAllPeds(std_srvs::SetBool::Request &request,
                                std_srvs::SetBool::Response &response){
    flatland_msgs::DeleteModels srv;

    srv.request.name = removePedsInPedsim();
    // Deleting pedestrian in flatland
    while(!delete_agents_.isValid()){
        ROS_WARN("Reconnecting delete_agents_-server....");
        delete_agents_.waitForExistence(ros::Duration(5.0));
        delete_agents_ = nh.serviceClient<flatland_msgs::DeleteModels>(delete_model_topic, true);
    }
    delete_agents_.call(srv);
    if (!srv.response.success)
    {
        ROS_ERROR("Failed to delete all %d agents. Maybe a few were deleted.", int(srv.request.name.size()));
    }

    response.success = true;
    return true;
}


std::vector<std::string> SceneServices::removePedsInPedsim(){
    //Remove all agents
    QList<Agent*> agents = SCENE.getAgents();
    int num_agents = agents.size();

    std::vector<std::string> flatland_ids;
    int count = 1;
    for(Agent* a:agents){
      int i = a->getId();
      //We don't want to delete the robot agent
      if(i == 0){
        continue;
      }
      // Deleting pedestrian and waypoints in SCENE
      for(Waypoint* w: a->getWaypoints()){
        SCENE.removeWaypoint(w);
      }
      SCENE.removeAgent(a);
      if(a->getType()!=Ped::Tagent::ZEROER){
        flatland_ids.push_back("person_" + std::to_string(count));
        count++;
      }
    }
    return flatland_ids;
}

bool SceneServices::removeAllPolygons(std_srvs::SetBool::Request &request,
                                std_srvs::SetBool::Response &response){
    flatland_msgs::DeleteModels srv;

    srv.request.name = removePolygonsInPedsim();
    // Deleting pedestrian in flatland
    while(!delete_agents_.isValid()){
        ROS_WARN("Reconnecting delete_agents_-server....");
        delete_agents_.waitForExistence(ros::Duration(5.0));
        delete_agents_ = nh.serviceClient<flatland_msgs::DeleteModels>(delete_model_topic, true);
    }
    delete_agents_.call(srv);
    if (!srv.response.success)
    {
        ROS_ERROR("Failed to delete all %d agents. Maybe a few were deleted.", int(srv.request.name.size()));
    }

    response.success = true;
    return true;
}

std::vector<std::string> SceneServices::removePolygonsInPedsim(){
    //Remove all agents
    QList<Agent*> agents = SCENE.getAgents();
    int num_agents = agents.size();

    std::vector<std::string> flatland_ids;
    int count = 1;
    for(Agent* a:agents){
      int i = a->getId();
      //We don't want to delete the robot agent
      if(i == 0){
        continue;
      }
      // Deleting pedestrian and waypoints in SCENE
      for(Waypoint* w: a->getWaypoints()){
        SCENE.removeWaypoint(w);
      }
      SCENE.removeAgent(a);
      if(a->getType()==Ped::Tagent::ZEROER){
        flatland_ids.push_back("polygon_" + std::to_string(count));
        count++;
      }
    }
    return flatland_ids;
}

std::vector<flatland_msgs::Model> SceneServices::addAgentClusterToPedsim(pedsim_msgs::Ped ped){
  const double x = ped.pos.x;
  const double y = ped.pos.y;
  const int n = ped.number_of_peds;
  const int type = ped.type;
  double dx, dy;
  if(ped.type!=4){
      dx = 2;
      dy = 2;
  }else{
      dx = 4;
      dy = 4;
  }
  AgentCluster* agentCluster = new AgentCluster(x, y, n);
  agentCluster->setDistribution(dx, dy);

  agentCluster->setType(static_cast<Ped::Tagent::AgentType>(type));
  for(int i = 0; i < (int)ped.waypoints.size(); i++){
    QString id;
    id.sprintf("%d_%d", ped.id, i);
    const double x = ped.waypoints[i].x;
    const double y = ped.waypoints[i].y;
    const double r = ped.waypoints[i].z;
    // Waypoint behaviour always set to SIMPLE --> ToDo: Parametrize
    AreaWaypoint* w = new AreaWaypoint(id, x, y, r);
    w->setBehavior(static_cast<Ped::Twaypoint::Behavior>(0));
    SCENE.addWaypoint(w);
    agentCluster->addWaypoint(w);
  }
  SCENE.addAgentCluster(agentCluster);
  std::vector<flatland_msgs::Model> flatland_msg;
  if(ped.type!=4){
  for (int i = last_id_+1; i <= last_id_+n; i++){
    flatland_msgs::Model model;
    std::string name = "person_" + std::to_string(ped.id);
    std::string ns = "pedsim_agent_" +  std::to_string(i);
    model.yaml_path =ped.yaml_file;
    model.name = name;
    model.ns = ns;
    model.pose.x = x;
    model.pose.y = y;
    flatland_msg.push_back(model);
  }  
  last_id_+=n;
  }else{
  for (int i = last_id_polygon_+1; i <= last_id_polygon_+n; i++){
    flatland_msgs::Model model;
    std::string name = "polygon_" + std::to_string(i);
    std::string ns = "pedsim_polygon_" +  std::to_string(i);
    model.yaml_path =ped.yaml_file;
    model.name = name;
    model.ns = ns;
    model.pose.x = x;
    model.pose.y = y;
    flatland_msg.push_back(model);
  } 
  last_id_polygon_+=n;
  }
  // last_id_+=n;
  return flatland_msg;
}

bool SceneServices::addStaticObstacles(pedsim_srvs::SpawnObstacle::Request &request,
                                pedsim_srvs::SpawnObstacle::Response &response){
  int k = (int)request.staticObstacles.obstacles.size();  
  for (int i = 0; i <= k; i++){
    pedsim_msgs::LineObstacle obstacle=request.staticObstacles.obstacles[i];
    // Obstacle obstacle=request.staticObstacles.obstacles[i];
    Obstacle *o = new Obstacle(obstacle.start.x, obstacle.start.y, obstacle.end.x, obstacle.end.y);
    SCENE.addObstacle(o);
  }

  response.finished=true;
  return true;
}

bool SceneServices::spawnPolygons(pedsim_srvs::SpawnZeroAgents::Request &request,
                                pedsim_srvs::SpawnZeroAgents::Response &response){
  
  ros::Time begin = ros::Time::now();
  flatland_msgs::RespawnModels srv;
  srv.request.old_model_names = removePolygonsInPedsim();
  for (int polygon_i = 0; polygon_i < (int)request.polygons.size(); polygon_i++){
    pedsim_msgs::ZeroAgent po = request.polygons[polygon_i];
    std::vector<flatland_msgs::Model> new_models = addPolygonsToPedsim(po);
    srv.request.new_models.insert(srv.request.new_models.end(), new_models.begin(), new_models.end());
  }
  response.finished=false;
  int count=0;

  begin = ros::Time::now();
  while(!respawn_agents_.isValid()){
    ROS_DEBUG("Reconnecting spawn_agents_-server....");
    respawn_agents_.waitForExistence(ros::Duration(5.0));
    respawn_agents_ = nh.serviceClient<flatland_msgs::RespawnModels>(respawn_model_topic, true);
  } 
  while(!response.finished&&count<10){
  respawn_agents_.call(srv);
  if (!srv.response.success)
  {
      ROS_ERROR("Failed to respawn all %d polygons", int(request.polygons.size()));
      response.finished=false;
      count++;
  }else{
    response.finished = true;
    break;
  } 

  }

  return true;
}

std::vector<flatland_msgs::Model>  SceneServices::addPolygonsToPedsim(pedsim_msgs::ZeroAgent polygon){
  const double x = polygon.pos.x;
  const double y = polygon.pos.y;
  const int n = polygon.number_of_agents;
  const double dx = 2;
  const double dy = 2;
  const int type = polygon.type;
  AgentCluster* agentClusterP = new AgentCluster(x, y, n);
  agentClusterP->setDistribution(dx, dy);

  agentClusterP->setType(static_cast<Ped::Tagent::AgentType>(type));
  for(int i = 0; i < (int)polygon.waypoints.size(); i++){
    QString id;
    id.sprintf("%d_%d", polygon.id, i);
    const double x = polygon.waypoints[i].x;
    const double y = polygon.waypoints[i].y;
    const double r = polygon.waypoints[i].z;
    // Waypoint behaviour always set to SIMPLE --> ToDo: Parametrize
    AreaWaypoint* w = new AreaWaypoint(id, x, y, r);
    w->setBehavior(static_cast<Ped::Twaypoint::Behavior>(0));
    SCENE.addWaypoint(w);
    agentClusterP->addWaypoint(w);
  }
  SCENE.addAgentCluster(agentClusterP);
  std::vector<flatland_msgs::Model> flatland_msg;
  for (int i = last_id_polygon_+1; i <= last_id_polygon_+n; i++){
    flatland_msgs::Model model;
    std::string name = "polygon_" + std::to_string(polygon.id);
    std::string ns = "pedsim_polyg_" +  std::to_string(i);
    model.yaml_path =polygon.yaml_file;
    model.name = name;
    model.ns = ns;
    model.pose.x = x;
    model.pose.y = y;
    flatland_msg.push_back(model);
  }  
  last_id_polygon_+=n;
  return flatland_msg;
}

bool SceneServices::moveAgentClustersInPedsim(pedsim_srvs::MovePeds::Request &request,
                                pedsim_srvs::MovePeds::Response &response){
  // if((int)request.pattern_waypoints.size()==0){
  //   response.waypoints=SCENE.moveClusters(request.episode);
  // }else{
    // ROS_INFO("move clusters");
  response.waypoints=SCENE.moveClusters(request.pattern_waypoints, request.episode);
  // }  
  // static obstacle infos are updated every episode too
  SCENE.removeAllObstacles();
  response.finished=true;
  return true;
}