/**
 * @copyright Copyright <JT-Innovation> (c) 2021
 */
#include "job_manager.h"

#include "common/common.h"

void JobManager::Init(ros::NodeHandle *nh_ptr) {
  navigation_started_ = false;
  mapping_started_ = false;
  nh_ptr_ = nh_ptr;

  // check service is in dir: /etc/systemd/system or not!!
  checkSystemInDir();

  timer_ =
	  nh_ptr_->createTimer(ros::Duration(1.), &JobManager::CheckStatus, this);
  ui_systemdservice_ = nh_ptr_->advertiseService(
	  "/ui/systemdservice", &JobManager::HandleSystemdService, this);
  ui_process_status_ = nh_ptr_->advertiseService(
	  "/ui/process_status", &JobManager::HandleProcessStatusService, this);
  ui_shutdown_IPC_ = nh_ptr_->advertiseService(
	  "/ui/shutdown_IPC", &JobManager::HandleShutdownIPCService, this);
  service_load_virtual_line_ =
	  nh_ptr_->serviceClient<VirtualLineLoad>("/ui/virtualline/load");

  pub_cpu_temp_ = nh_ptr_->advertise<std_msgs::String>("/cpu_temprature", 1);

	locate_node_.setCallbackQueue(&locate_queue_);
	locate_thd_ptr_ = std::unique_ptr<std::thread>(new std::thread(&JobManager::locateThreadHandler, this));
	ui_locate_server_ = locate_node_.advertiseService("/ui/localization", &JobManager::locateHandler, this);

	JLG->info("job manager init!!");
}

void JobManager::checkSystemInDir() {
  std::set<std::string> systemdFileSet;

  BF::path systemdDir("/etc/systemd/system");
  BF::directory_iterator it(systemdDir);
  for (; it != BF::directory_iterator(); it++) {
	systemdFileSet.insert(it->path().filename().string());
  }

  service_valid_list_.resize(service_list_.size());

  for (size_t i = 0; i < service_list_.size(); i++) {
	service_valid_list_[i] =
		(systemdFileSet.find(service_list_[i].GetServiceName() + ".service") !=
			systemdFileSet.end());
	if(!service_valid_list_[i]){
		JLG->warn("{}.service could not be found under /etc/systemd/system, please check!!",
				   service_list_[i].GetServiceName());
	}
  }
}

void JobManager::CheckCpuTemprature() {
  std::string cpu_temp_path = "/sys/class/thermal/thermal_zone";
  json res_json;

  for (int i = 0; i < 8; i++) {
	cpu_temp_path += std::to_string(i);

	std::vector<std::string> out;

	if (BF::exists(BF::path(cpu_temp_path))) {
	  open_popen("cat " + cpu_temp_path + "/temp", out);
	  std::string cpu_id = "CPU_TEMP_ZONE" + std::to_string(i);
	  res_json[cpu_id] = std::stoi(out[0]) / 1000.0;
	} else
	  break;
  }

  std_msgs::String message;
  message.data = res_json.dump();
  pub_cpu_temp_.publish(message);
}

void JobManager::CheckStatus(const ros::TimerEvent &e) {
  XmlRpc::XmlRpcValue srv;
  std::vector<std::string> output;
  // for (auto service : service_list_)
  for (int i = 0; i < service_valid_list_.size(); i++) {
	if (!service_valid_list_[i]) continue;

	auto service = service_list_[i];
	int returncode = open_popen(
		"systemctl status " + service.GetServiceName() + " | grep Active",
		output);
	if (!returncode) {
	  bool flag = false;
	  for (auto str : output) {
		if (str.find("active (running)") != std::string::npos) {
		  flag = true;
		  break;
		}
	  }
	  srv[service.GetServiceName()] = flag;

	} else {
	  JLG->error("unknown error, bug!");
	  srv[service.GetServiceName()] = false;
	}
  }
  nh_ptr_->setParam("process_status", srv);
}

bool JobManager::HandleSystemdService(SystemdService::Request &req,
									  SystemdService::Response &res) {
  JLG->info("get systemd service: {}", req.service_name);
  if (req.service_name == "") {
	res.status = -1;
	res.message = "There is no such service";
	return true;
  }
  if (req.turn_on == true) {
	if (req.service_name == "navigation") {
	  // check mapping!!
	  if (getEgoStatus("mapping") || getEgoStatus("mapping_lio_sam")) {
		res.status = -1;
		res.message = "mapping service is runnning";
		return true;
	  }
	  VirtualLineLoad msg;
	  service_load_virtual_line_.call(msg);
	}

	if (req.service_name == "mapping_expansion") {
	  if (!getEgoStatus("navigation")) {
		res.status = -1;
		res.message = "navigation service is  not runnning";
		return true;
	  }
	  open_system("sudo systemctl stop navigation.service");
	  sleep(5);
	  nh_ptr_->setParam("/lio_sam/map_expansion", true);
	  open_system("sudo systemctl restart mapping_lio_sam.service");
	  res.message = req.service_name + " is turned on";
	  res.status = 0;
	  return true;
	} else
	  nh_ptr_->setParam("/lio_sam/map_expansion", false);

	if ((req.service_name == "mapping" ||
		req.service_name == "mapping_lio_sam")) {
	  // check navigation is on ??

	  if (getEgoStatus("navigation")) {
		res.message = "navigation service is runnning";
		res.status = -1;
		return true;
	  } else {
		open_system("sudo systemctl start boundary_recording.service");
	  }
	}

	/// start navigation.service for giving tf message to boundary_recording
	if (req.service_name == "boundary_recording") {
	  if (!getEgoStatus("navigation"))
		open_system("sudo systemctl start navigation.service");
	}
	open_system("sudo systemctl restart " + req.service_name + ".service");
	res.message = req.service_name + " is turned on";
	res.status = 0;
	return true;
  } else {
	// if (req.service_name == "navigation") {
	// }

	if (req.service_name == "boundary_recording") {
	  if (getEgoStatus("navigation"))
		open_system("sudo systemctl stop navigation.service");
	}
	open_system("sudo systemctl stop " + req.service_name + ".service");
	res.status = 0;
	res.message = "service is turned off";
	return true;
  }
  return true;
}

bool JobManager::HandleProcessStatusService(ProcessStatus::Request &req,
											ProcessStatus::Response &res) {
  std::vector<Process> processes_list;
  Process navigation_status;
  Process mapping_status;
  try {
	std::map<std::string, int> param;
	nh_ptr_->getParam("process_status", param);
	if (param["navigation"] > 0)
	  navigation_started_ = true;
	else
	  navigation_started_ = false;
	if ((param["mapping"] > 0) || (param["mapping_lio_sam"] > 0))
	  mapping_started_ = true;
	else
	  mapping_started_ = false;

	navigation_status.name = "navigation";
	navigation_status.active = navigation_started_;
	processes_list.push_back(navigation_status);
	mapping_status.name = "mapping";
	mapping_status.active = mapping_started_;
	processes_list.push_back(mapping_status);
	res.status = 0;
	res.processes = processes_list;
  } catch (const std::exception &e) {
	res.status = 1;
	res.processes = processes_list;
  }
  return true;
}

bool JobManager::HandleShutdownIPCService(SystemdService::Request &req,
										  SystemdService::Response &res) {
  if (req.turn_on == true) {
	system("shutdown -r +1");
	JLG->warn("IPC will shutdown after 60 seconeds");
	res.message = "shutdown success";
	res.status = 0;
	return true;
  } else if (req.turn_on == false) {
	system("shundown -c");
	JLG->warn("cancel shuntdown IPC success");
	res.message = "cancel success";
	res.status = 0;
	return true;
  }

  return true;
}

void JobManager::locateThreadHandler()
{
	while(locate_node_.ok()){
		locate_queue_.callOne();
		ros::Duration(0.1).sleep();
	}
}

bool JobManager::locateHandler(AgentRequest& req, AgentResponse& res)
{
	res.trace_id = req.trace_id;
	if(req.service_id == "locate"){
		SystemdService ss;
		ss.request.service_name = "navigation";
		json j = json::parse(req.data);
		JLG->info("locate: {}", req.data);
		ss.request.turn_on = j["turn_on"];
		HandleSystemdService(ss.request, ss.response);
		agentHelper(jlw_for_job_manager->logger, res, ss.response.message, ss.response.status);
	}else if(req.service_id == "relocate"){
		std_srvs::Empty em;
		if(!ros::service::call("/relocalize", em)){
			agentHelper(jlw_for_job_manager->logger, res, "reloc offline!!", -1);
			return true;
		}
		agentHelper(jlw_for_job_manager->logger, res, "ok", 0);
	}else{
		agentHelper(jlw_for_job_manager->logger, res, "invalid service id", -1);
	}
	return true;
}

JobManager::~JobManager(){
	locate_node_.shutdown();
	if(locate_thd_ptr_->joinable())	locate_thd_ptr_->join();
	JLG->info("exit job manager!!");
}