#include "common.h"

void read_parameters_from_file(const std::string filename, std::map<std::string, std::string> &parameter_list)
{
    ifstream parameters_file;
    parameters_file.open(filename, ios::in);
    if (!parameters_file.is_open())
    {
        std::cout << "ERROR open failed with " << filename << std::endl;
        return;
    }
    std::string key, value;
    std::string line;
    while (std::getline(parameters_file, line))
    {
        if (line[0] == '#')
            continue;

        std::istringstream sline(line);
        // std::cout << line << std::endl;
        if (sline >> key >> value)
        {
            parameter_list[key] = value;
            // std::cout << key << "and" << line << std::endl;
        }
    }
    std::cout << parameter_list.size() << " parameters are read from " << filename << std::endl;
    parameters_file.close();
}

void add_point_cloud_visualization(pcl::visualization::PCLVisualizer::Ptr viewer_ptr, pcl::PointCloud<PCFORMAT>::Ptr m_cloud_ptr)
{
	if (!m_cloud_ptr->points.empty())
	{
		// Keep the same name as shown in main function
		pcl::visualization::PointCloudColorHandlerGenericField<PCFORMAT> fildColor(m_cloud_ptr, "z");
		viewer_ptr->updatePointCloud<PCFORMAT>(m_cloud_ptr, fildColor, "cloud");
	}
}

void write_point_cloud_binary(pcl::PointCloud<PCFORMAT>::Ptr m_cloud_ptr, const std::string filename)
{
	struct my_pointxyzi
	{
		float x, y, z, i;
	};
	std::vector<my_pointxyzi> my_points;
	my_points.resize(m_cloud_ptr->points.size());
	//my_pointxyzi mp;
	for (size_t i = 0; i < m_cloud_ptr->points.size(); i++)
	{
		my_points[i].x = m_cloud_ptr->points[i].x;
		my_points[i].y = m_cloud_ptr->points[i].y;
		my_points[i].z = m_cloud_ptr->points[i].z;
		my_points[i].i = m_cloud_ptr->points[i].intensity;
	}

	ofstream fout(filename, ios::out | ios::binary);
	size_t PCFORMAT_size = sizeof(my_pointxyzi);
	fout.write(reinterpret_cast<char *>(my_points.data()), PCFORMAT_size * my_points.size());
	fout.close();
}

int write_point_cloud_acsii(pcl::PointCloud<PCFORMAT>::Ptr m_cloud_ptr, const std::string filename)
{
	//if (!m_cloud_ptr->points.empty())
	//	pcl::io::savePCDFile(filename, *m_cloud_ptr);
	std::ofstream f(filename, std::ios::out);
	if (!f.is_open()) return -1;

	for (auto &p : m_cloud_ptr->points)
		f << p.x << " " << p.y << " " << p.z << " " << p.intensity << "\n";

	f.close();

	return 0;
}

int write_point_cloud_acsii(pcl::PointCloud<pcl::PointXYZ>::Ptr m_cloud_ptr, const std::string filename)
{
	std::ofstream f(filename, std::ios::out);
	if (!f.is_open()) return -1;

	for (auto &p : m_cloud_ptr->points)
		//f << p.x << " " << p.y << " " << p.z << " " << (int)p.r << " " << (int)p.g << " " << (int)p.b << "\n";
		f << p.x << " " << p.y << " " << p.z << "\n";

	f.close();

	return 0;
}

std::string get_current_date(std::vector<int> & current_date)
{
	time_t tt = time(NULL);
	struct tm *stm = localtime(&tt);

	char tmp[32];
	sprintf(tmp, "%04d-%02d-%02d-%02d-%02d-%02d",
		1900 + stm->tm_year, 1 + stm->tm_mon, stm->tm_mday,
		stm->tm_hour, stm->tm_min, stm->tm_sec);

	current_date = std::vector<int>{ 1900 + stm->tm_year, 1 + stm->tm_mon, stm->tm_mday, stm->tm_hour, stm->tm_min, stm->tm_sec };
	current_date.resize(6);

	return std::string(tmp);
}