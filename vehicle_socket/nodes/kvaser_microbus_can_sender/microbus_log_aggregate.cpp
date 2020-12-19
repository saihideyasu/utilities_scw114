#include <ros/ros.h>
#include <fstream>
#include <unordered_map>

class MicrobusLogAggregate
{
private:
	ros::NodeHandle nh_, p_nh_;
	std::vector<std::string> use_field_list_;

	std::vector<std::string> split(const std::string &string, const char sep)
	{
		std::vector<std::string> str_vec_ptr;
		std::string token;
		std::stringstream ss(string);

		while (getline(ss, token, sep))
			str_vec_ptr.push_back(token);

		return str_vec_ptr;
	}

	void parseColumns(const std::string& line, std::vector<std::string>* columns)
	{
		std::istringstream ss(line);
		std::string column;
		while (std::getline(ss, column, '|'))
		{
			/*while (1)
			{
			auto res = std::find(column.begin(), column.end(), ' ');
			if (res == column.end())
			{
				break;
			}
			column.erase(res);
			}
			if (!column.empty())
			{
			columns->emplace_back(column);
			}:*/
			columns->push_back(column);
		}
	}
public:
	MicrobusLogAggregate(ros::NodeHandle nh, ros::NodeHandle p_nh)
		: nh_(nh)
		, p_nh_(p_nh)
	{
		int first_way_id, last_way_id;
		if(!p_nh_.getParam("first_way_id", first_way_id)) {std::cout << "read error : first_way_id"; return;}
		if(!p_nh_.getParam("last_way_id", last_way_id)) {std::cout << "read error : last_way_id"; return;}
		std::string file_path_list;
		if(!p_nh_.getParam("file_list", file_path_list)) {std::cout << "read error : file_list"; return;}
		std::vector<std::string> file_list = split(file_path_list, ',');

		for(std::string file_path : file_list)
		{
			std::ifstream ifs(file_path, std::ios::in);
			if(ifs.is_open()) std::cout << file_path << " processing" << std::endl;

			std::string line;
			std::getline(ifs, line); std::getline(ifs, line);  // get field line
			std::vector<std::string> field_name;
			parseColumns(line, &field_name);
			//for(int i=0; i< field_name.size(); i++) std::cout << field_name[i] << std::endl;

			for(int cou=0; !ifs.eof(); cou++)
			{
				std::getline(ifs, line);
				std::vector<std::string> fields;
				parseColumns(line, &fields);
				/*if(fields.size() != field_name.size())
				{
					std::cout << "field size error : " << field_name.size() << "," << fields.size() <<std::endl;
					continue;
				}
				else std::cout << "processing index : " << cou << std::endl;
				std::cout << fields[0] << std::endl;*/
				std::unordered_map<std::string, std::string> map;
				for (size_t i = 0; i < field_name.size(); i++)
				{
					map[field_name.at(i)] = fields.at(i);
				}
				std::cout << map["can_receive_502_.angle_actual"] << std::endl;
			}

		}
	}
};

int main(int argc, char** argv)
{
   	ros::init(argc, argv, "microbus_log_aggregate");
	ros::NodeHandle nh;
	ros::NodeHandle private_nh("~");

	MicrobusLogAggregate mla(nh, private_nh);
	ros::spin();
	return 0;
}