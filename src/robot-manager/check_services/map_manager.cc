/**
 * @copyright Copyright <JT-Innovation> (c) 2021
 */
#include "map_manager.h"
#include "common/MapImageTrans.hpp"
#include <thread>
#include "common/OssUploadHelper.hpp"

void MapManager::Init(ros::NodeHandle *nh_ptr) {
  nh_ptr_ = nh_ptr;
  initMapIdToMapName();  

  save_map_action_client_ptr_.reset(new actionlib::SimpleActionClient<
	  mobile_platform_msgs::SavingAllMapAction>(
	  *nh_ptr, "savingAllMap", true));

  ui_mapping_save_ = nh_ptr_->advertiseService(
	  "/ui/mapping/save", &MapManager::HandleMappingSave, this);
  ui_mapping_delete_ = nh_ptr_->advertiseService(
	  "/ui/mapping/delete", &MapManager::HandleMappingDelete, this);
  ui_mapping_select_ = nh_ptr_->advertiseService(
	  "/ui/mapping/select", &MapManager::HandleMappingSelect, this);
  ui_mapping_selected_ = nh_ptr_->advertiseService(
	  "/ui/mapping/selected", &MapManager::HandleMappingSelected, this);
  ui_mapping_revert_ = nh_ptr_->advertiseService(
	  "/ui/mapping/revert", &MapManager::HandleMappingRevert, this);
  ui_mapping_edit_ = nh_ptr_->advertiseService(
	  "/ui/mapping/edit", &MapManager::HandleMappingEdit, this);
  ui_mapping_list_ = nh_ptr_->advertiseService("/ui/mapping/list",
											   &MapManager::GetValidMaps, this);
  ui_mapping_upload_ = nh_ptr_->advertiseService(
	  "/ui/mapping/upload", &MapManager::HandleMappingUpload, this);

  ui_mapping_division_create_main_ = nh_ptr_->advertiseService(
	  "/ui/mapping_division/main", &MapManager::HandleMappingDivisionCreateMain,
	  this);
  ui_mapping_division_create_sub_ = nh_ptr_->advertiseService(
	  "/ui/mapping_division/sub", &MapManager::HandleMappingDivisionCreateSub,
	  this);
  ui_mapping_division_save_custom_name_ =
	  nh_ptr_->advertiseService("/ui/mapping_division/rename",
								&MapManager::HandleMappingDivisionRename, this);
  ui_mapping_division_delete_ =
	  nh_ptr_->advertiseService("/ui/mapping_division/delete",
								&MapManager::HandleMappingDivisionDelete, this);
  ui_boundary_recording_save_ =
	  nh_ptr_->advertiseService("/ui/boundary_recording/save",
								&MapManager::HandleBoundaryRecordingSave, this);

  systemd_service_ =
	  nh_ptr_->serviceClient<SystemdService>("/ui/systemdservice");
  save_layer_client_ =
	  nh_ptr_->serviceClient<OssUpload>("/ui/semanticlayer/save");
  divide_map_main_ = nh_ptr_->serviceClient<MappingSave>("/ui/divide_map/main");
  divide_map_sub_ = nh_ptr_->serviceClient<MappingSave>("/ui/divide_map/sub");

  check_mapping_timer_ = nh_ptr_->createTimer(
	  ros::Duration(1.0), &MapManager::checkMappingStatus, this);

  MLG->info("init map manager");
}

/**
 * @brief init map.json
 * 
 */
void MapManager::initMapIdToMapName()
{
  std::string map_json_fn = MAP_DIRECTORY + "map.json";
  auto files = getAllMaps();

  if(!BF::exists(BF::path(map_json_fn))){
    std::ofstream out(map_json_fn);
    json map_json;
    for(const auto& f : files)  map_json[f] = f;
    out << map_json;
    out.close();
    return;
  }

  // if exist, then deal with map not in json
  std::ifstream inf(map_json_fn);
  auto map_json = json::parse(inf);
  inf.close();
  for(const auto& f : files){
    if(!map_json.contains(f)) map_json[f] = f;
  }
  std::ofstream out(map_json_fn);
  out << map_json;
  out.close();
}

void uploadMappingStatus(int code) {
  mobile_platform_msgs::MappingStart srv;
  srv.request.msg = code;
  ros::service::call("/ui/mapping/status", srv);
}


void MapManager::setJobmanagerPtr(JobManager *ptr) { jobPtr = ptr; }


/**
 * @brief parse map.json is enough !!
 * 
 * @param req 
 * @param res 
 * @return true 
 * @return false 
 */
bool MapManager::GetValidMaps(MappingList::Request &req,
                              MappingList::Response &res) {

  std::vector<std::string> legit_files;
  std::vector<std::string> map_id_vec;
  std::vector<std::string> files = getFileName(MAP_DIRECTORY);

  std::ifstream inf(MAP_DIRECTORY + "map.json");
  auto j = json::parse(inf);

  for(json::iterator it=j.begin(); it!=j.end(); it++){
    std::string map_id = it.key();
    std::string map_name = it.value();
    std::string f = MAP_DIRECTORY + map_id + "/" + map_id;
    if(BF::exists(BF::path(f + ".png")) && BF::exists(f + ".yaml")){
        map_id_vec.push_back(map_id);
      legit_files.push_back(map_name + " " + map_id);
    }
  }

  last_legit_files = std::move(map_id_vec);
  res.message = "OK";
  res.status = 0;
  res.names = legit_files;
  return true;
}

/**
 * @brief 保存的时候自动生成map_id，同时关联map_name
 * 
 * @param req 
 * @param res 
 * @return true 
 * @return false 
 */
bool MapManager::HandleMappingSave(MappingSave::Request &req,
								   MappingSave::Response &res) {
  // 0. check whether is mapping or not
  if (!getEgoStatus("mapping_lio_sam")) {
    responseHelper(res, -1, "HandleMappingSave: is not mapping now!!");
    return true;
  }

  // generate map_id
  auto now = std::time(0);
  tm *ltm = localtime(&now);
  char cr[64];
  std::sprintf(cr, "jt%02d%02d%02dx%02d%02d%02d", -100 + ltm->tm_year, 1 + ltm->tm_mon,
              ltm->tm_mday, ltm->tm_hour, ltm->tm_min, ltm->tm_sec);

  auto map_id = std::string(cr);
  auto map_name = req.filename;

  // 1. check file name is valid or exist
  std::cerr << "map id: " << map_id << std::endl;
  bool map_expansion = false;
  nh_ptr_->getParam("/lio_sam/map_expansion", map_expansion);
  if (!map_expansion)
  {
    BF::create_directories(BF::path(MAP_DIRECTORY + map_id + "/map"));
    // 增加path文件夹
    BF::create_directories(
        BF::path(MAP_DIRECTORY + map_id + "/path"));
    // 增加虚拟墙空白文件virtualpoints.yaml
    std::string command =
        "touch " MAP_DIRECTORY + map_id + "/virtualpoints.yaml";
    open_system(command);
    MLG->trace("HandleMappingSave: Directory {} Created", MAP_DIRECTORY + map_id);
    
  }

  // 2. --------------- action to save liosam map ---------------
  if (!save_map_action_client_ptr_->waitForServer(ros::Duration(1.0))) {
    MLG->error("HandleMappingSave: save map action server is not existed!!");
    responseHelper(res, -1, "save map action server is not existed!!");
    uploadMappingStatus(-1);
    return true;
  }

  mobile_platform_msgs::SavingAllMapGoal samg;
  samg.selected_map = map_id;

  save_map_action_client_ptr_->sendGoal(samg,
										std::bind(&MapManager::MappingSaveDoneHelper,
												  this, map_id, std::placeholders::_1, std::placeholders::_2));

  // 3. check yaml and pcd and png is saved!!!
  char c_path[256];
  std::string check_path_form =
	  MAP_DIRECTORY + map_id + "/" + map_id + "%s";
  std::vector<std::string> check_ext{".yaml", ".png", ".pcd"};

  // wait for map is save until timeout !!!
  ros::Duration r_dur(2.0);
  int time_cnt = 0;
  bool checkFlag = false;
  while (time_cnt++ < 15) {
    int good_cnt = 0;
    for (const auto &e : check_ext) {
      std::sprintf(c_path, check_path_form.c_str(), e.c_str());
      if (access(c_path, F_OK) == 0) ++good_cnt;
    }
    if (good_cnt == check_ext.size()) {
      checkFlag = true;
      break;
    }
    r_dur.sleep();
  }

  if (!checkFlag) {
    MLG->error("HandleMappingSave: pcd yaml or png is not saved!!");
    responseHelper(res, -1,
            "HandleMappingSave: pcd yaml or png is not saved!!");
    uploadMappingStatus(-1);
    return true;
  }

  // 5. save path_record path to path/map_name.csv then terminated boundary_recording.service

  std::string source_path = MAP_DIRECTORY + "current/";

  std::string dist_path = MAP_DIRECTORY + map_id + "/";

  std::string cmd =
	  "cp " + source_path + "current_path_record.yaml " + dist_path + "path/" + map_id + ".csv";
  open_popen(cmd);
  open_popen("cp " + source_path + "current_path_record.yaml " + dist_path + "path/" + map_id + ".yaml");

  // terminated boundary_recording process !!

  SystemdService::Request request;
  request.service_name = "boundary_recording";
  request.turn_on = false;
  SystemdService::Response resp;
  if (jobPtr)
	  jobPtr->HandleSystemdService(request, resp);
  else
	  MLG->error("HandleMappingSave: nullptr");

  MLG->trace("HandleMappingSave: terminated boundary_recording");

  // 4.1 write map.json
  std::ifstream inf(MAP_DIRECTORY + "map.json");
  auto j = json::parse(inf);
  inf.close();
  j[map_id] = map_name;

  std::ofstream out(MAP_DIRECTORY + "map.json");
  out << j;
  out.close();

  // 5. upload oss !!
  auto& oss = OssUploadHelper::getInstance();
  oss.setMapId(map_id);
  if (oss.uploadOssFile(map_id + ".png", map_id + ".png") &&
	  oss.uploadOssFile(map_id + ".yaml", map_id + ".yaml")) {
    res.message += " oss file upload also success!!";
    MLG->trace("HandleMappingSave: oss file upload also success!!");
  } else {
    res.message += " oss file upload fail!!";
    MLG->error("oss file upload fail!!");
    res.status = -1;
  }

  if (oss.uploadMapMetaInfo(map_id, map_id)) {
    res.message += " oss meta info upload also success!!";
    MLG->trace("HandleMappingSave: oss meta info upload also success!!");
  } else {
    res.message += " oss meta info upload fail!!";
    MLG->error("oss meta info upload fail!!");
    res.status = -1;
  }

  return true;
}

bool MapManager::HelpDelete(std::string &filename) {
  if (rm_dir(MAP_DIRECTORY + filename))
	return true;
  else
	return false;
}

bool MapManager::HandleMappingDelete(MappingDelete::Request &req,
									 MappingDelete::Response &res) {
  MappingList msg;
  msg.request.msg = 0;
  GetValidMaps(msg.request, msg.response);
  for (auto f : last_legit_files) {
	if (req.filename == f) {
	  if (!getEgoStatus("navigation")) {
		if (HelpDelete(req.filename)) {
		  res.message = "OK";
		  res.status = 0;
		  return true;
		} else {
		  res.message = "map delete may be failed, os returned error";
		  res.status = -9;
		  return true;
		}
	  } else {
		res.message = "navigation is on!!";
		res.status = -5;
		return true;
	  }
	}
  }
  res.message = "map delete failed, no such name";
  res.status = -5;
  return true;
}

bool MapManager::HandleMappingSelect(MappingSelect::Request &req,
									 MappingSelect::Response &res) {
  MappingList msg;
  msg.request.msg = 0;
  GetValidMaps(msg.request, msg.response);
  MLG->info("select map name: {}", req.filename);
  for (auto f : last_legit_files) {
	if (req.filename == f) {
	  if (!getEgoStatus("navigation")) {
		nh_ptr_->setParam("selected_map", req.filename);
		open_popen("sudo -u " + USER_NAME +
			" bash -c \"source /opt/ros/$ROS_DISTRO/setup.bash;rosparam "
			"dump ~/.robot/config/selected_map.yaml /selected_map\"");

		nav_msgs::LoadMap srv;
		srv.request.map_url =
			MAP_DIRECTORY + "/" + req.filename + "/" + req.filename + ".yaml";
		if (ros::service::exists("change_map", true)) {
		  if (ros::service::call("change_map", srv)) {
			if (srv.response.result != srv.response.RESULT_SUCCESS) {
			  MLG->error("switchMap: change map in map_server failed, code({})",
						(int)srv.response.result);
			}
		  } else {
			MLG->error("switchMap: change map in map_server failed!!");
		  }
		} else {
		  MLG->error(
			  "switchMap: change_map is not exist!! check map_server node is "
			  "exist!!");
		}
		res.message = "OK";
		res.status = 0;
		return true;
	  } else {
		res.status = -8;
		res.message = "map select failed, navigation is on going";
		return true;
	  }
	  break;
	}
  }
  res.message = "map select failed, no such name";
  res.status = -6;
  return true;
}

bool MapManager::HandleMappingSelected(MappingSelected::Request &req,
									   MappingSelected::Response &res) {
  MappingList msg;
  msg.request.msg = 0;
  GetValidMaps(msg.request, msg.response);
  if (!nh_ptr_->hasParam("selected_map")) {
	res.status = -1;
	res.map = "";
	return true;
  }
  std::string map_name;
  nh_ptr_->getParam("selected_map", map_name);
  res.status = 0;
  res.map = map_name;
  return true;
}

bool MapManager::HandleMappingRevert(MappingRevert::Request &req,
									 MappingRevert::Response &res) {
  MappingList msg;
  msg.request.msg = 0;
  GetValidMaps(msg.request, msg.response);
  for (auto f : last_legit_files) {
	if (f == req.filename) {
	  std::string src =
		  MAP_DIRECTORY + req.filename + "/" + req.filename + ".png.orig";
	  std::string dst =
		  MAP_DIRECTORY + req.filename + "/" + req.filename + ".png";
	  struct stat flag;
	  if (stat((src).c_str(), &flag) == 0) {
		if ((flag.st_mode & S_IFREG)) {
		  try {
			copy_file(src, dst);
			res.message = "OK";
			res.status = 0;
			return true;
		  } catch (const std::exception &e) {
			res.message = "map revert probably IO error";
			res.status = -27;
			return true;
		  }
		} else {
		  res.message = "map revert finds no original file";
		  res.status = -26;
		  return true;
		}
	  } else {
		perror("stat");
		return true;
	  }
	}
  }
  res.message = "map revert failed, no such name";
  res.status = -22;
  return true;
}

bool MapManager::HandleMappingEdit(MappingEdit::Request &req,
								   MappingEdit::Response &res) {
  MappingList msg;
  msg.request.msg = 0;
  GetValidMaps(msg.request, msg.response);
  for (auto f : last_legit_files) {
	if (req.filename == f) {
	  std::string data;
	  try {
      MLG->info("start to b64decode");
      data = base64_decode(req.data);
      MLG->trace("b64: {}", data);
	  } catch (const std::exception &e) {
      res.message = "map edit data corrupt";
      res.status = -24;
      return true;
	  }
	  try {
		std::ofstream out(
			MAP_DIRECTORY + req.filename + "/" + req.filename + ".png",
			std::ios::binary | std::ios::out);
		if (out.fail()) {
		  MLG->error("Fail to open the source file");
		  out.close();
		} else {
		  out << data;
		  out.close();
		}
	  } catch (const std::exception &e) {
		res.message = "map edit saving failed";
		res.status = -25;
		return true;
	  }
	  res.message = "OK";
	  res.status = 0;
	  return true;
	}
  }
  res.message = "map edit failed, no such name";
  res.status = -23;
  return true;
}

bool MapManager::HandleBoundaryRecordingSave(MappingSave::Request &req,
											 MappingSave::Response &res) {
  // 1.check .robot/maps/current/current_path.csv
  // 2.copy current_path.yaml to
  // ./robot/maps/map_name(example:jt_demo_1th)/zone/innerboder_i.csv 3.create
  // border.json for web searching innerborder_i.csv
  // 4. stop system boundary_recording.service
  if (!nh_ptr_->hasParam("/selected_map")) {
    MLG->error("no param /selected_map");
    return false;
  }
  std::string map_name;
  nh_ptr_->getParam("/selected_map", map_name);

  std::string source_path = MAP_DIRECTORY + "current/";
  std::string dist_path = MAP_DIRECTORY + map_name + "/zone/";
  if (IsFileExistent(BF::path(source_path + "current_path_record.yaml"))) {
	if (!BF::exists(BF::path(dist_path))) {
	  BF::create_directories(dist_path);
	}

	std::string cmd =
		"cp " + source_path + "current_path_record.yaml " + dist_path + req.filename + ".csv";
	open_popen(cmd);
	open_popen("cp " + source_path + "current_path_record.yaml " + dist_path + req.filename + ".yaml");

	json border_json;

	if (IsFileExistent(BF::path(dist_path + "/border.json"))) {
	  std::ifstream input(dist_path + "/border.json");
	  input >> border_json;

	  input.close();
	}
	if (req.filename.find("inBorder", 0) != req.filename.npos) {
	  border_json["inBorder"].push_back(
		  json(std::map<std::string, std::string>{{"name", req.filename}}));
	}

	std::ofstream out(dist_path + "/border.json");
	if (!out) MLG->warn("output to border.json wrong!");
	out << std::setw(4) << border_json;
	out.close();

	SystemdService srv;
	srv.request.service_name = "boundary_recording";
	srv.request.turn_on = false;
	if (jobPtr)
	  jobPtr->HandleSystemdService(srv.request, srv.response);
	else
	  MLG->error("HandleBoundaryRecordingSave: nullptr");

	responseHelper(
		res, 0,
		"save recoding path ok and shutdown boundary_recording.service!");
  } else {
	responseHelper(res, -1, "current_path_record.yaml not exists!");
  }

  return true;
}

bool MapManager::HandleMappingDivisionCreateMain(MappingSave::Request &req,
												 MappingSave::Response &res) {
  
  // clear all timing task!!
  std_srvs::Empty em;
  if(!ros::service::call("/timing_task/reset", em)){
    MLG->error("timing task offline!!");
  }

  auto json_base = json::parse(req.filename.c_str());
  std::string map_name = json_base["map_name"];

    // reset zone mate info   
    MLG->warn("start to reset zone mate info");
    auto& oss = OssUploadHelper::getInstance();
    oss.setMapId(map_name);
    if(!oss.resetZoneMetaInfo()){
        MLG->error("reset zone meta info fail!!");
        return true;
    }

  /// create map main partition
  std::string dist_path = MAP_DIRECTORY + map_name + "/zone/";

  if (!BF::exists(BF::path(dist_path)))
	BF::create_directory(BF::path(dist_path));

  /***   save divide map info into layer.json ***/

  std::ofstream layer_out(dist_path + "layer.json");

  if (!layer_out)
	MLG->error("main divide map lay_out open layer.json error");

  int index = 0;
  for (auto &item : json_base["lines"]) {
	item["id"] = index++;
  }

  layer_out << std::setw(4) << json_base;

  /***   call  divide map main service    ***/

  MappingSave srv;
  srv.request = req;

  if (divide_map_main_.call(srv.request, srv.response)) {
	auto receive_json = json::parse(srv.response.message);

	std::ofstream res_out(dist_path + "partition.json");

	if (!res_out)
	  MLG->error("main divide map res_out open partition.json error");

	res_out << std::setw(4) << receive_json;

	res_out.close();

	res.message = "mapping division main success!";
	res.status = 0;

  } else {
	  MLG->error("divide_map_main call failed !");

	res.message = srv.response.message;
	res.status = -1;
  }

  return true;
}

bool MapManager::HandleMappingDivisionCreateSub(MappingSave::Request &req,
												MappingSave::Response &res) {
  /// create map sub partition
  auto json_base = json::parse(req.filename.c_str());

  std::string map_name = json_base["map_name"];

  std::string zone_name = json_base["zone_name"];

  std::string dist_path = MAP_DIRECTORY + map_name + "/zone/" + zone_name + "/";

  if (!BF::exists(BF::path(dist_path)))
	BF::create_directory(BF::path(dist_path));

  /***   save sub divide map info into layer.json ***/

  std::ofstream layer_out(dist_path + "layer.json");

  if (!layer_out)
	MLG->error("sub divide map lay_out open layer.json error");

  layer_out << std::setw(4) << json_base;

  /***    call sub map divide service          ***/
  MappingSave srv;
  srv.request = req;

  if (divide_map_sub_.call(srv.request, srv.response)) {
	auto receive_json = json::parse(srv.response.message);

	std::ofstream res_out(dist_path + "partition.json");

	if (!res_out)
	  MLG->error("sub divide map res_out open partition.json error");

	res_out << std::setw(4) << receive_json;

	res_out.close();

	res.message = "mapping division sub success!";
	res.status = 0;

  } else {
	  MLG->error("divide_map_sub call failed !");

	res.message = srv.response.message;
	res.status = -1;
  }

  return true;
}

bool MapManager::HandleMappingDivisionRename(MappingSave::Request &req,
											 MappingSave::Response &res) {
  /// rename map partition

  auto receive_json = json::parse(req.filename);

  std::string map_name = receive_json["map_name"];
  std::string zone_name = receive_json["zone_name"];
  std::string dist_path = MAP_DIRECTORY + map_name + "/zone/";

  if (!BF::exists(BF::path(dist_path)))
	  MLG->error("zone path doesnt exist ,rename fail!");

  std::ifstream input(dist_path + "partition.json");
  if (!input)
	MLG->error("rename input open partition.json error");

  json res_json;

  input >> res_json;

  auto json_base = res_json["zone_info"];

  for (auto &item : json_base) {
	if (item["dir_name"] == zone_name) {
	  item["name"] = receive_json["rename"];
	  break;
	}
  }

  std::ofstream out(dist_path + "partition.json");

  if (!out)
	MLG->error("rename output open partition.json error");

  res.message = "rename zone success!";
  res.status = 0;

  return true;
}

bool MapManager::HandleMappingDivisionDelete(MappingSave::Request &req,
											 MappingSave::Response &res) {
  return true;
}

bool MapManager::LoadRosParamFromFile(std::string &param_name,
									  std::string &file_name) {
  if (!nh_ptr_->hasParam("/selected_map")) {
    MLG->error("no param /selected_map");
    return false;
  }
  std::string map_name;
  nh_ptr_->getParam("/selected_map", map_name);
  std::string map_dir = MAP_DIRECTORY + map_name + "/";
  std::string rosparam_name = " " + param_name + "\"";
  open_system(
	  "sudo -u " + USER_NAME +
		  " bash -c \"source /opt/ros/$ROS_DISTRO/setup.bash;rosparam load " +
		  map_dir + file_name + rosparam_name);
  return true;
}

int MapManager::SaveMapFromTopic(std::string &filename) {
  if (filename.length() > 0) {
    MLG->trace("saving {}", filename);
    std::string path_dir = MAP_DIRECTORY + filename;
    struct stat flag;
    if (access(path_dir.c_str(), F_OK)) {
      // mkdir((path_dir).c_str(), S_IRWXU | S_IRGRP | S_IXGRP);
      BF::create_directory(BF::path(path_dir));
      MLG->trace("Directory {} Created ", path_dir);
    } else {
      MLG->warn("Directory {} already exists", path_dir);
    }
    std::vector<std::string> output;
    open_popen("rosrun robot_manager mapping_save.sh " + filename, output);
    for (auto str : output) {
      MLG->trace("{}", str);
    }
    return 0;
  }
  return -1;
}

bool MapManager::CheckIfMapIsSelected() {
  if (!nh_ptr_->hasParam("selected_map")) {
	  MLG->error("no param /selected_map");
	  return false;
  }

  int selected_map;
  nh_ptr_->getParam("selected_map", selected_map);
  if (selected_map <= 0) return false;
  return true;
}

// upload map to oss
bool MapManager::HandleMappingUpload(OssUpload::Request &req,
									 OssUpload::Response &res) {
	
	auto& oss = OssUploadHelper::getInstance();
	std::string map_id{req.filename};
	oss.setMapId(map_id);
  if (oss.uploadOssFile(req.filename + ".png", req.filename + ".png") &&
	  oss.uploadOssFile(req.filename + ".yaml", req.filename + ".yaml")) {
    res.message = " oss file upload success!!";
    res.status = 0;
    MLG->trace("HandleMappingSave: oss file upload also success!!");
  } else {
    res.message = " oss file upload fail!!";
    res.status = -1;
    MLG->error("oss file upload fail!!");
    return true;
  }

  if (oss.uploadMapMetaInfo(map_id, map_id)) {
    res.message += " oss meta info upload also success!!";
    MLG->trace("HandleMappingSave: oss meta info upload also success!!");
  } else {
    res.message += " oss meta info upload fail!!";
    res.status = -1;
    MLG->error("oss meta info upload fail!!");
  }

	MLG->trace("start to upload zone !!");
	std::string zone_dir = MAP_DIRECTORY + map_id + "/zone/";
	if(!OssUploadHelper::checkPathExist(zone_dir)){
		res.message += " zone dir is not exist!!";
		res.status = -1;
		return true;
	}

	std::string zone_prefix = "zone_";
    for (int i = 0; i < 10; i++)
    {
        std::string zone_id = zone_prefix + std::to_string(i);
        auto s_zone_dir = zone_dir + zone_id + "/";

        if(!OssUploadHelper::checkPathExist(s_zone_dir))    break;

        // upload mask
        std::string mask_name = "zone/" + zone_id + "/mask.png";
        std::string mask_show_name = "zone/" + zone_id + "/mask_show.png";
        std::string edge_name = "zone/" + zone_id + "/edge.csv";
        std::string coverage_name = "zone/" + zone_id + "/coverage.csv";
        std::string both_name = "zone/" + zone_id + "/both.csv";
        MLG->trace("try to upload mask: {}", mask_name);

        std::string cmd = "touch ";
        if(!OssUploadHelper::checkPathExist(s_zone_dir + "coverage.csv"))    open_popen(cmd + s_zone_dir + "coverage.csv");    
        if(!OssUploadHelper::checkPathExist(s_zone_dir + "both.csv"))    open_popen(cmd + s_zone_dir + "both.csv");    

        oss.uploadOssFile(mask_show_name, mask_name);
        oss.uploadOssFile(edge_name, edge_name);
        oss.uploadOssFile(coverage_name, coverage_name);
        oss.uploadOssFile(both_name, both_name);
    }

    if(oss.uploadZoneMetaInfo()){
		MLG->trace("upload zone meta info success!!");
		res.message += " upload zone meta info success!!";
		res.status = 0;
	}else{
		MLG->error("upload zone meta info fail!!");
		res.message += " upload zone meta info fail!!";
		res.status = -1;
	}

  	return true;
}

void MapManager::checkMappingStatus(const ros::TimerEvent &e) {
  XmlRpc::XmlRpcValue status;
  nh_ptr_->getParam("process_status", status);

  if (status["mapping_lio_sam"]) {
	std::vector<std::string> out;
	open_popen("rosnode info /lio_sam_mapOptmization 2>&1", out);

	auto it = std::find_if(out.begin(), out.end(), [](const std::string &e) {
	  return e.find("ERROR") != e.npos;
	});
	// if find the node is dead
	if (it != out.end()) {
	  // info mqtt
	  uploadMappingStatus(-1);
	  SystemdService::Request request;
	  request.service_name = "mapping";
	  request.turn_on = false;
	  SystemdService::Response resp;
	  jobPtr->HandleSystemdService(request, resp);
	}
  }
}

void MapManager::MappingSaveDoneHelper(std::string map_name, const actionlib::SimpleClientGoalState &state,
									   const mobile_platform_msgs::SavingAllMapResultConstPtr &result) {

  // save the map reference points
  if (result->status == 0) {
	/// save map param start/end points to layer.json
	std::string message = save_map_action_client_ptr_->getResult()->message;

	MLG->trace("********* {} **********", message);

	std::string field;

	std::istringstream sin(message);

	std::vector<float> data;

	while (getline(sin, field, ' ')) {
	  data.push_back(std::stof(field));
	}

	// ---------- trans to img first: for ui/semantic need image coordinate -------------
	MapImageTrans mit;
	mit.setYaml(MAP_DIRECTORY + map_name + "/" + map_name + ".yaml");
	std::vector<float> tv(2, 0);
	tv = mit.mapToImage(std::vector<float>{data[0], data[1]});
	for (int i = 0; i < 2; i++) data[i] = tv[i];
  data[2] = MapImageTrans::toAng(M_PI - data[2]);
	tv = mit.mapToImage(std::vector<float>{data[3], data[4]});
	for (int i = 3; i < 5; i++) data[i] = tv[i - 3];
  data[5] = MapImageTrans::toAng(M_PI - data[5]);

	json req_json_start;
	req_json_start["property"] = "ref_point";
	req_json_start["type"] = "point";
	req_json_start["map_name"] = map_name;
	json start_point;
	start_point["id"] = "0";
	start_point["name"] = "start";
	start_point["data"].push_back(data[0]);
	start_point["data"].push_back(data[1]);
	start_point["data"].push_back(data[2]);
	req_json_start["message"] = start_point;

	OssUpload srv_start;
	srv_start.request.filename = req_json_start.dump();
	if (save_layer_client_.call(srv_start)) {
	  MLG->trace("****** {} ******", srv_start.response.message);
	} else {
	  MLG->error("save_layer_client call fail !");
	}

	sleep(1);
	json req_json_end;

	req_json_end["property"] = "ref_point";
	req_json_end["type"] = "point";
	req_json_end["map_name"] = map_name;
	json end_point;
	end_point["id"] = "0";
	end_point["name"] = "end";
	end_point["data"].push_back(data[3]);
	end_point["data"].push_back(data[4]);
	end_point["data"].push_back(data[5]);
	req_json_end["message"] = end_point;

	OssUpload srv_end;

	srv_end.request.filename = req_json_end.dump();
	if (save_layer_client_.call(srv_end)) {
	  MLG->trace("****** {} ******", srv_end.response.message);
	} else {
	  MLG->error("save_layer_client call fail !");
	}
  } else {
	  MLG->error("save_map_action_client_ptr_ get result fail!");
  }

  // 4. terminated mapping process !!
  MLG->trace("HandleMappingSave: terminating mapping");
  SystemdService::Request request;
  request.service_name = "mapping_lio_sam";
  request.turn_on = false;
  SystemdService::Response resp;
  if (jobPtr)
	  jobPtr->HandleSystemdService(request, resp);
  else
	  MLG->error("HandleMappingSave: nullptr");

  MLG->trace("HandleMappingSave: terminated mapping");

}
