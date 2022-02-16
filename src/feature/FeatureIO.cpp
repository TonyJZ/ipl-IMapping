#include "feature/FeatureIO.h"
#include <fstream>

const char IPF_FILE_FLAG[] = "intersection file V1.0";

int  
ipl::load_IntersectionPoint2D(const std::string filename,
							std::vector<std::string> &model_names,
							double min_pt[3], double max_pt[3],
							std::vector<ipl::feature::IntersectionPoint> &ipt2ds)
{
	std::ifstream  ifs;
	ifs.open(filename);

	if (!ifs.is_open())
	{
		return (-1);
	}

	//	std::string file_desc;
	//	file_desc.resize(100);
	char file_desc[1024];
	ifs.getline(file_desc, 1024);

	std::string flag = file_desc;
	if (flag.compare(IPF_FILE_FLAG) != 0)
		return (-1);

	//	ifs >> file_desc;

	int n_pts; //number of point cloud 
	ifs.getline(file_desc, 1024);
	n_pts = atoi(file_desc);
	//	ifs >> n_pts;

	model_names.resize(n_pts);
	for (int i = 0; i < n_pts; i++)
	{
		ifs.getline(file_desc, 1024);
		model_names[i] = file_desc;
		//ifs >> pointnames[i];
	}

	std::string strDesc;
	ifs >> strDesc;
	ifs >> min_pt[0] >> min_pt[1] >> min_pt[2];
	ifs >> max_pt[0] >> max_pt[1] >> max_pt[2];

	ifs >> strDesc;
	int n_ipt; //number of ipt
	ifs >> n_ipt;

	ipt2ds.resize(n_ipt);
	for (int i = 0; i < n_ipt; i++)
	{
		int type;
		ifs >> type;
		ipt2ds[i].type = static_cast<ipl::feature::FeatureType>(type);

		ifs >> ipt2ds[i].p[0] >> ipt2ds[i].p[1] >> ipt2ds[i].p[2] >> ipt2ds[i].p[3];

		int n_conn;
		ifs >> n_conn;

		ipt2ds[i].connIndices.resize(n_conn);
		for (int j = 0; j < n_conn; j++)
		{
			ifs >> ipt2ds[i].connIndices[j];
		}

		ipt2ds[i].connAtts.resize(n_conn);
		for (int j = 0; j < n_conn; j++)
		{
			ifs >> ipt2ds[i].connAtts[j].ptS[0]
				>> ipt2ds[i].connAtts[j].ptS[1]
				>> ipt2ds[i].connAtts[j].ptS[2]
				>> ipt2ds[i].connAtts[j].ptE[0]
				>> ipt2ds[i].connAtts[j].ptE[1]
				>> ipt2ds[i].connAtts[j].ptE[2];
		}

		ifs >> ipt2ds[i].weight;

		if (ipt2ds[i].type == ipl::feature::FT_Intersection2D)
		{
			ipt2ds[i].coefVar.resize(2);
			ifs >> ipt2ds[i].coefVar[0] >> ipt2ds[i].coefVar[1];
		}
	}
	ifs.close();

	return (0);
}

int  
ipl::save_IntersectionPoint2D(const std::string filename,
							const std::vector<std::string> model_names,
							const std::vector<ipl::feature::IntersectionPoint> &ipt2ds)
{
	double min_pt[3], max_pt[3];
	getBBox(ipt2ds, min_pt, max_pt);

	std::ofstream  ofs;
	ofs.open(filename);

	if (!ofs.is_open())
	{
		return (-1);
	}

	ofs << IPF_FILE_FLAG << std::endl;

	int n_pts = model_names.size(); //number of point cloud 
	ofs << n_pts << std::endl;

	for (int i = 0; i < n_pts; i++)
	{
		ofs << model_names[i] << std::endl;
	}

	ofs << "BoundingBox:" << std::endl;
	ofs << min_pt[0] << " " << min_pt[1] << " " << min_pt[2] << std::endl;
	ofs << max_pt[0] << " " << max_pt[1] << " " << max_pt[2] << std::endl;

	ofs << "intersections:" << std::endl;
	int n_ipt = ipt2ds.size(); //number of ipt
	ofs << n_ipt << std::endl;

	for (int i = 0; i < n_ipt; i++)
	{
		ofs << ipt2ds[i].type << std::endl;
		ofs << ipt2ds[i].p[0] << " " << ipt2ds[i].p[1] << " " << ipt2ds[i].p[2] << " " << ipt2ds[i].p[3] << std::endl;

		int n_conn = ipt2ds[i].connIndices.size();
		ofs << n_conn << std::endl;

		for (int j = 0; j < n_conn; j++)
		{
			ofs << ipt2ds[i].connIndices[j] << " ";
		}
		ofs << std::endl;

		for (int j = 0; j < n_conn; j++)
		{
			ofs << ipt2ds[i].connAtts[j].ptS[0] << " "
				<< ipt2ds[i].connAtts[j].ptS[1] << " "
				<< ipt2ds[i].connAtts[j].ptS[2] << " "
				<< ipt2ds[i].connAtts[j].ptE[0] << " "
				<< ipt2ds[i].connAtts[j].ptE[1] << " "
				<< ipt2ds[i].connAtts[j].ptE[2] << std::endl;
		}

		ofs << ipt2ds[i].weight << std::endl;

		ofs << ipt2ds[i].coefVar[0] << " " << ipt2ds[i].coefVar[1] << std::endl;
	}
	ofs.close();

	return (0);
}

int 
ipl::load_PlaneAttribute(const std::string filename, ipl::feature::ProjectedLineAttribute &att)
{

	return (0);
}

int 
ipl::save_PlaneAttribute(const std::string filename, const ipl::feature::ProjectedLineAttribute &att)
{

	return (0);
}
