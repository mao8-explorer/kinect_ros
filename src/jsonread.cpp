#include <iostream>
#include <json/json.h>
#include <fstream>


#include <vector>

using namespace std;
vector<double> readFileJson()
{
	Json::Reader reader;
	Json::Value root;
	vector<double> tfmat;

 
	//从文件中读取，保证当前文件有demo.json文件  
	ifstream in(
    "/home/zm/RobotVector/tr/multicamera/kinect_ros/src/Azure_Kinect_ROS_Driver/json/test.json", 
    ios::binary);

	if (!in.is_open())
	{
		cout << "Error opening file\n";
		return tfmat; 
	}
	if (reader.parse(in, root))
	{

		tfmat.push_back(root["value0"]["translation"]["m00"].asDouble());
		tfmat.push_back(root["value0"]["translation"]["m10"].asDouble());
		tfmat.push_back(root["value0"]["translation"]["m20"].asDouble());
		tfmat.push_back(root["value0"]["rotation"]["x"].asDouble());
		tfmat.push_back(root["value0"]["rotation"]["y"].asDouble());
		tfmat.push_back(root["value0"]["rotation"]["z"].asDouble());
		tfmat.push_back(root["value0"]["rotation"]["w"].asDouble());

		for(vector<double>::iterator it = tfmat.begin();it!=tfmat.end();it++){
			cout<< *it<< " ";
		}
		cout << endl;

		cout << "Reading Complete!" << endl;
	}
	else
	{
		cout << "parse error\n" << endl;
	}
 
	in.close();
	return tfmat;
}

int main()
{

	vector<double> tfmat;
	tfmat = readFileJson();

	cout<<tfmat.size()<< " "<< tfmat.at(6) <<endl; 
    return 0;
}

