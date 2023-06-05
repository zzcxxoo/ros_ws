#include "common.h"

static const std::string base64_chars =
	"ABCDEFGHIJKLMNOPQRSTUVWXYZ"
	"abcdefghijklmnopqrstuvwxyz"
	"0123456789+/";

static inline bool is_base64(unsigned char c) {
  return (isalnum(c) || (c == '+') || (c == '/'));
}

std::string getUserName() {
  uid_t userid;
  struct passwd *pwd;
  userid = getuid();
  pwd = getpwuid(userid);
  return pwd->pw_name;
}

std::vector<std::string> getFileName(const std::string &path) {
  DIR *dir;
  struct dirent *ent;
  std::vector<std::string> res;
  if ((dir = opendir(path.c_str())) != NULL) {
	while ((ent = readdir(dir)) != NULL) {
	  if (strcmp(ent->d_name, ".") && strcmp(ent->d_name, ".."))
		res.push_back(ent->d_name);
	}
	closedir(dir);
  } else {
	perror("");
  }
  return res;
}

std::vector<std::string> getFileName(const std::string &path,
									 const std::string &temp,
									 const std::string con) {
  DIR *dir;
  struct dirent *ent;
  std::vector<std::string> res;
  if ((dir = opendir(path.c_str())) != NULL) {
	while ((ent = readdir(dir)) != NULL) {
	  std::string name(ent->d_name);
	  int pos = name.rfind(".");
	  if (pos == -1) continue;
	  if (name.substr(pos) == temp) {
		res.push_back(con + name.substr(0, pos));
	  }
	}
	closedir(dir);
  } else {
	perror("");
  }
  return res;
}

void open_system(const std::string &cmd) {
  int status = 0;
  status = system(cmd.c_str());
  if (status == 1) {
	perror("open_system()");
	exit(1);
  }
  if (WIFEXITED(status) != 0) {
	if (WEXITSTATUS(status) == 0) {
	  printf("run command success\n");
	} else {
	  printf("run command fail and exit code is %d", WEXITSTATUS(status));
	}
  } else {
	printf("exit code is %d\n", WEXITSTATUS(status));
  }
}

int open_popen(const std::string &cmd) {
  FILE *fp;
  fp = popen(cmd.c_str(), "r");
  if (!fp) std::cerr << "Couldn't start command" << std::endl;
  return pclose(fp);
}

int open_popen(const std::string &cmd, std::vector<std::string> &out) {
  FILE *fp;
  const int sizebuf = 2048;
  char buff[sizebuf];
  out = std::vector<std::string>();
  fp = popen(cmd.c_str(), "r");
  if (!fp) std::cerr << "Couldn't start command" << std::endl;

  while (fgets(buff, sizeof(buff), fp)) {
	std::string cur_string = "";
	cur_string += buff;
	out.push_back(cur_string.substr(0, cur_string.size()));
  }
  return pclose(fp);
}

bool rm_dir(const std::string &path) {
  DIR *dir;
  struct dirent *dirp;
  struct stat buf;
  if ((dir = opendir(path.c_str())) == NULL) {
	perror("Opendir");
	return false;
  }
  while ((dirp = readdir(dir)) != NULL) {
	if ((strcmp(dirp->d_name, ".") == 0) || (strcmp(dirp->d_name, "..") == 0))
	  continue;
	if (stat((path + "/" + dirp->d_name).c_str(), &buf) == -1) {
	  perror("stat");
	  return false;
	}
	if (S_ISDIR(buf.st_mode)) {
	  rm_dir(path + "/" + dirp->d_name);
	  continue;
	}
	if (remove((path + "/" + dirp->d_name).c_str()) == -1) {
	  perror("remove");
	  return false;
	}
	printf("rm %s successed..\n", dirp->d_name);
  }
  closedir(dir);
  if (rmdir(path.c_str()) == -1) {
	perror("rmdir");
	return false;
  }
  printf("rm %s Successed . . .\n", path.c_str());
  return true;
}

int copy_file(const std::string &sourcefile, const std::string &distfile) {
  std::ifstream in;
  std::ofstream out;

  try {
	in.open(sourcefile, std::ios::binary);
	if (in.fail()) {
	  std::cout << "Error 1: Fail to open the source file." << std::endl;
	  in.close();
	  out.close();
	  return 0;
	}
	out.open(distfile, std::ios::binary);
	if (out.fail()) {
	  std::cout << "Error 2: Fail to create the new file." << std::endl;
	  out.close();
	  in.close();
	  return 0;
	} else {
	  out << in.rdbuf();
	  out.close();
	  in.close();
	  return 1;
	}
  } catch (std::exception e) {
  }
  printf("copyfile success!\n");
  return 0;
}

std::string base64_encode(unsigned char const *bytes_to_encode,
						  unsigned int in_len) {
  std::string ret;
  int i = 0;
  int j = 0;
  unsigned char char_array_3[3];
  unsigned char char_array_4[4];

  while (in_len--) {
	char_array_3[i++] = *(bytes_to_encode++);
	if (i == 3) {
	  char_array_4[0] = (char_array_3[0] & 0xfc) >> 2;
	  char_array_4[1] =
		  ((char_array_3[0] & 0x03) << 4) + ((char_array_3[1] & 0xf0) >> 4);
	  char_array_4[2] =
		  ((char_array_3[1] & 0x0f) << 2) + ((char_array_3[2] & 0xc0) >> 6);
	  char_array_4[3] = char_array_3[2] & 0x3f;

	  for (i = 0; (i < 4); i++) ret += base64_chars[char_array_4[i]];
	  i = 0;
	}
  }

  if (i) {
	for (j = i; j < 3; j++) char_array_3[j] = '\0';

	char_array_4[0] = (char_array_3[0] & 0xfc) >> 2;
	char_array_4[1] =
		((char_array_3[0] & 0x03) << 4) + ((char_array_3[1] & 0xf0) >> 4);
	char_array_4[2] =
		((char_array_3[1] & 0x0f) << 2) + ((char_array_3[2] & 0xc0) >> 6);
	char_array_4[3] = char_array_3[2] & 0x3f;

	for (j = 0; (j < i + 1); j++) ret += base64_chars[char_array_4[j]];

	while ((i++ < 3)) ret += '=';
  }

  return ret;
}

std::string base64_decode(std::string const &encoded_string) {
  int in_len = encoded_string.size();
  int i = 0;
  int j = 0;
  int in_ = 0;
  unsigned char char_array_4[4], char_array_3[3];
  std::string ret;

  while (in_len-- && (encoded_string[in_] != '=') &&
	  is_base64(encoded_string[in_])) {
	char_array_4[i++] = encoded_string[in_];
	in_++;
	if (i == 4) {
	  for (i = 0; i < 4; i++)
		char_array_4[i] = base64_chars.find(char_array_4[i]);

	  char_array_3[0] =
		  (char_array_4[0] << 2) + ((char_array_4[1] & 0x30) >> 4);
	  char_array_3[1] =
		  ((char_array_4[1] & 0xf) << 4) + ((char_array_4[2] & 0x3c) >> 2);
	  char_array_3[2] = ((char_array_4[2] & 0x3) << 6) + char_array_4[3];

	  for (i = 0; (i < 3); i++) ret += char_array_3[i];
	  i = 0;
	}
  }

  if (i) {
	for (j = i; j < 4; j++) char_array_4[j] = 0;

	for (j = 0; j < 4; j++)
	  char_array_4[j] = base64_chars.find(char_array_4[j]);

	char_array_3[0] = (char_array_4[0] << 2) + ((char_array_4[1] & 0x30) >> 4);
	char_array_3[1] =
		((char_array_4[1] & 0xf) << 4) + ((char_array_4[2] & 0x3c) >> 2);
	char_array_3[2] = ((char_array_4[2] & 0x3) << 6) + char_array_4[3];

	for (j = 0; (j < i - 1); j++) ret += char_array_3[j];
  }

  return ret;
}

std::string md5sum(const std::string &str) {
  std::string md5;
  MD5_CTX ctx;
  MD5_Init(&ctx);
  MD5_Update(&ctx, str.c_str(), str.size());
  unsigned char digest[MD5_DIGEST_LENGTH];
  MD5_Final(digest, &ctx);
  char hex[35];
  memset(hex, 0, sizeof(hex));
  for (int i = 0; i < MD5_DIGEST_LENGTH; ++i) {
	sprintf(hex + i * 2, "%02x", digest[i]);
  }
  md5 = std::string(hex);
  return md5;
}

bool is_number(const std::string &s) {
  if (s.empty()) return false;

  std::string::const_iterator it = s.begin();
  if (*it == '-' || *it == '+') ++it;
  while (it != s.end() && (std::isdigit(*it) || (*it == '.'))) ++it;
  return !s.empty() && it == s.end();
}

bool YamlToJson(const YAML::Node &ynode, Json::Value &jnode) {
  try {
	if (ynode.IsScalar()) {
	  Json::Value v(ynode.Scalar());
	  if (is_number(ynode.Scalar())) {
		v = std::atof(ynode.Scalar().c_str());
	  }
	  jnode.swapPayload(v);
	  return true;
	}
	if (ynode.IsSequence()) {
	  for (size_t i = 0; i < ynode.size(); ++i) {
		Json::Value v;
		if (YamlToJson(ynode[i], v)) {
		  jnode.append(v);
		} else {
		  return false;
		}
	  }
	} else if (ynode.IsMap()) {
	  for (auto it = ynode.begin(); it != ynode.end(); ++it) {
		Json::Value v;
		if (YamlToJson(it->second, v)) {
		  jnode[it->first.Scalar()] = v;
		} else {
		  return false;
		}
	  }
	}
  } catch (...) {
	return false;
  }
  return true;
}

std::string getSelectedMap() {
  std::string mapName;
  ros::param::get("/selected_map", mapName);
  return mapName;
}

std::string getEtherMac() {
  std::vector<std::string> res;
  std::string mac_file = "/usr/local/agent/macaddress";
  std::string cmd = "cat " + mac_file;
  if (open_popen(cmd, res) == 0) {
	std::string r;
	auto res0 = res.at(0);
	// 如果有换行符，那就截掉
	if (res0.back() == '\n') res0 = res0.substr(0, res0.size() - 1);
	return res0;
  }

  return "";
  // return std::string(std::getenv("DEVICE_MAC"));
}

bool getEgoStatus(std::string name) {
  XmlRpc::XmlRpcValue srv;
  if (!ros::param::get("process_status", srv)) return false;

  try {
	return srv[name];
  }
  catch (const XmlRpc::XmlRpcException &e) {
	ROS_ERROR("getEgoStatus: %s", e.getMessage().c_str());
  }

  return false;

}

bool IsFileExistent(const BF::path &path) {
  return BF::is_regular_file(path);
}


std::vector<std::string> getAllMaps()
{
    std::vector<std::string> res;
    BF::directory_iterator it(MAP_DIRECTORY);
    for(; it!=BF::directory_iterator(); it++)
    {
        auto name = it->path().stem().string();
        if(BF::is_directory(it->path()) && name != "current"){
            res.emplace_back(std::move(name));
        }
    }
    
    return res;
}

std::string genUUID() noexcept
{
    uuid_t uuid;
    char str[36];

    uuid_generate(uuid);
    uuid_unparse(uuid, str);

    return std::string(str);
}

geometry_msgs::Pose getPoseFromTwoPoint(const geometry_msgs::Pose& p1, const geometry_msgs::Pose& p2)
{
	auto x = p2.position.x - p1.position.x;
	auto y = p2.position.y - p1.position.y;

	geometry_msgs::Quaternion q;
	tf::quaternionTFToMsg(tf::createQuaternionFromYaw(std::atan2(y, x)), q);

	geometry_msgs::Pose po;
	po = p1;
	po.orientation = q;

	return po;
}
