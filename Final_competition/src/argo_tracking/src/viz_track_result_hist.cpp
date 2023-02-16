// visualize track result from output 
#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <json/json.h>
#include <ros/ros.h>
#include <fstream>
#include <vector>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Time.h>
#include <std_msgs/Header.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h> 
#include <pcl/point_types.h> 
#include <pcl/io/ply_io.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sstream>
#include <ros/package.h>
//read all .ply in files
#include <unistd.h>
#include <dirent.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <algorithm>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/LinearMath/Vector3.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/image_encodings.h>
#include <signal.h>

using namespace std;
string log_path, label_path, det_path, pose_path;
int play_rate = 10;
vector <string> id_record;
typedef pcl::PointXYZI PointT;
ros::Publisher pub_id, pub_lidar, pub_label, pub_det, pub_hist;
visualization_msgs::MarkerArray M_id, M_label, M_det, M_hist;
int det_max_size = 0, label_max_size = 0;
int frame_count = 0;
string id_type;
double trajectory_points[500][21][3] = {0}; // object_id, history, point
tf::Transform trans_world;


tf::Transform broadcast_worldToBasicLink(Json::Value pose)
{
	// broadcast TF from world to basic link
	tf::Transform tf_transform;
	tf::Vector3 origin;
	origin.setValue(pose["translation"][0].asDouble(), 
					pose["translation"][1].asDouble(), 
					pose["translation"][2].asDouble());
	tf::Quaternion tfqt;
	double pose_x = pose["rotation"][3].asDouble();
	double pose_y = pose["rotation"][0].asDouble();
	double pose_z = pose["rotation"][1].asDouble();
	double pose_w = pose["rotation"][2].asDouble();
	// printf("pose_x: %lf\n", pose_x);
	// printf("pose_y: %lf\n", pose_y);
	// printf("pose_z: %lf\n", pose_z);
	// printf("pose_w: %lf\n", pose_w);

	tfqt.setX(pose["rotation"][3].asDouble());
	tfqt.setY(pose["rotation"][0].asDouble());
	tfqt.setZ(pose["rotation"][1].asDouble());
	tfqt.setW(pose["rotation"][2].asDouble());
	
	tf::Matrix3x3 m(tfqt);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);
	printf("roll : %lf\n", roll);
	printf("pitch: %lf\n", pitch);
	printf("yaw  : %lf\n", yaw);
	// tfqt.setRPY(0, 0, yaw);

	tf_transform.setOrigin(origin);
	tf_transform.setRotation(tfqt);
	// tf_broadcaster.sendTransform(tf::StampedTransform(tf_transform, ros::Time::now(),"world","scan"));
	printf("pose x, y, z: %lf %lf %lf\n", pose["translation"][0].asDouble(), pose["translation"][1].asDouble(), pose["translation"][2].asDouble());
	return tf_transform;
}

int add_point(int id, double point[3])
{
	tf::Vector3 point_temp;
	// tf::Transform trans_temp;
	// tf::Quaternion tfqt;
	// tfqt.setW(1.0);
	// trans_temp.setOrigin(trans_world.getOrigin());
	// trans_temp.setRotation(tfqt);
	
	point_temp.setValue(point[0], point[1], point[2]);
	point_temp = trans_world * point_temp;	// Save points on world frame
	point[0] = point_temp.getX();
	point[1] = point_temp.getY();
	point[2] = point_temp.getZ();
	int hist_len = 0;
	for (int i = 0; i < 20; i++) {
		if (trajectory_points[id][i][0] == 0) {
			break;
		}
		else {
			hist_len++;
		}
	}
	if (hist_len == 0) {
		trajectory_points[id][hist_len][0] = point[0];
		trajectory_points[id][hist_len][1] = point[1];
		trajectory_points[id][hist_len][2] = point[2];
	}
	else {
		trajectory_points[id][hist_len][0] = point[0];
		trajectory_points[id][hist_len][1] = point[1];
		trajectory_points[id][hist_len][2] = point[2];

		// for (int hist = 0; hist < hist_len; hist++) {
		// 	update_points(id, hist, displacement, yaw_change);
		// }
		if (hist_len == 20) {
			for (int hist = 0; hist < hist_len; hist++) {
				trajectory_points[id][hist][0] = trajectory_points[id][hist+1][0];
				trajectory_points[id][hist][1] = trajectory_points[id][hist+1][1];
				trajectory_points[id][hist][2] = trajectory_points[id][hist+1][2];
			}
		}
		hist_len--;
	}
	int temp = 29;
	if (id == temp) {
		printf("trajectory points:\n0: %lf, %lf, %lf\n", trajectory_points[temp][0][0], trajectory_points[temp][0][1], trajectory_points[temp][0][2]);
		printf("1: %lf, %lf, %lf\n", trajectory_points[temp][1][0], trajectory_points[temp][1][1], trajectory_points[temp][1][2]);
		printf("2: %lf, %lf, %lf\n", trajectory_points[temp][2][0], trajectory_points[temp][2][1], trajectory_points[temp][2][2]);
		printf("3: %lf, %lf, %lf\n", trajectory_points[temp][3][0], trajectory_points[temp][3][1], trajectory_points[temp][3][2]);
		printf("4: %lf, %lf, %lf\n", trajectory_points[temp][4][0], trajectory_points[temp][4][1], trajectory_points[temp][4][2]);
		printf("Trans point: %lf %lf %lf\n", point[0], point[1], point[2]);
	}
	return hist_len + 1;
}

void showAllFiles( const char * dir_name, int shit, vector<string>& filenames)
{
	// check the parameter !
	if( NULL == dir_name )
	{
		ROS_WARN("Argo Data directory path is null !");
		return;
	}
 
	// check if dir_name is a valid dir
	struct stat s;
	lstat( dir_name , &s );
	if( ! S_ISDIR( s.st_mode ) )
	{
		ROS_WARN("Argo Data directory path is not a valid directory !");
		return;
	}
	
	struct dirent * filename;    // return value for readdir()
 	DIR * dir;                   // return value for opendir()
	dir = opendir( dir_name );
	if( NULL == dir )
	{
		ROS_WARN("Can not open dir %s",dir_name);
		return;
	}
	ROS_INFO("Successfully opened the dir !");
	
	/* read all the files in the dir ~ */
	while( ( filename = readdir(dir) ) != NULL )
	{
		// get rid of "." and ".."
		if( strcmp( filename->d_name , "." ) == 0 || 
			strcmp( filename->d_name , "..") == 0 ||
            strcmp( filename->d_name , ".ipynb_checkpoints") == 0 )
			continue;
		// cout<<filename ->d_name <<endl;
        
        if( shit==0 )
            filenames.push_back(filename->d_name);
        // else if ( shit==1 )
        //     stereos_l.push_back(filename->d_name);
        // else
        //     stereos_r.push_back(filename->d_name);
        
    }   
}

void sort_timestamp(vector<string> &filenames, vector<unsigned long int> &sorted_names){
	int i=0;

    //sorting lidar
	for(i; i<filenames.size(); i++){
		string str = filenames.at(i).substr(3, 18);//star at 3th index and read 18 char(timestamp in file name)
		istringstream is(str);
		unsigned long int ts;
		is >> ts;
		sorted_names.push_back(ts);
		//cout << sorted_plys.at(i) <<endl;
	}

	sort(sorted_names.begin(), sorted_names.end());
    // cout<<"\033[1;33mAfter sorting:\n\033[0m";
	// for(i=0;i<sorted_names.size();i++)
    //     cout<< i << " " << sorted_names[i]<<"\n";
    // cout << "get out of function" << endl;
    return; 
}

template<class T>
string convertToString(T value){
	stringstream ss;
	ss << value;
	return ss.str();
}


void initialize_marker(Json::Value label_array, Json::Value det_array){
	visualization_msgs::Marker marker, marker_id, marker_det;
	double timestamp = det_array[0]["timestamp"].asDouble();
	//checking id
	if ( ! id_type.compare("uuid") ){
		if (id_record.size() == 0){
			for (int k =0; k<label_array.size(); k++){
				string track_label_uuid = label_array[k]["track_label_uuid"].asString();
				id_record.push_back(track_label_uuid);
			}
		}
		else{
			for (int k=0; k<label_array.size(); k++){
				string track_label_uuid = label_array[k]["track_label_uuid"].asString();
				vector<string>::iterator find_id = find(id_record.begin(), id_record.end(),track_label_uuid);
				if(find_id == id_record.end())
				id_record.push_back(track_label_uuid);
			}
		}	
	}

	// detection
	int counter = 0;
	for (int m=0; m<det_array.size(); m++){
		float score = det_array[m]["score"].asDouble();
		if (score < 0.3)
			continue;
		counter ++;
		marker_det.header.frame_id = "scan";
		marker_det.header.stamp = ros::Time::now();
		marker_det.action = visualization_msgs::Marker::ADD;
        marker_det.lifetime = ros::Duration(1/(play_rate+10));
		marker_det.ns = "detection";
        marker_det.type = visualization_msgs::Marker::CUBE;
        
        // marker_det.id = m + 200;
        marker_det.id = counter - 1;
        
        marker_det.pose.position.x = det_array[m]["center"]["x"].asDouble();
        marker_det.pose.position.y = det_array[m]["center"]["y"].asDouble();
        marker_det.pose.position.z = det_array[m]["center"]["z"].asDouble();

        marker_det.pose.orientation.x = det_array[m]["rotation"]["x"].asDouble();
        marker_det.pose.orientation.y = det_array[m]["rotation"]["y"].asDouble();
        marker_det.pose.orientation.z = det_array[m]["rotation"]["z"].asDouble();
        marker_det.pose.orientation.w = det_array[m]["rotation"]["w"].asDouble();
        
        marker_det.scale.x = det_array[m]["length"].asDouble();
        marker_det.scale.y = det_array[m]["width"].asDouble();
        marker_det.scale.z = det_array[m]["height"].asDouble();
        
        marker_det.color.b = 0.0f;
        marker_det.color.g = 0.0f; //1.0f
        marker_det.color.r = 0.0f;
	    marker_det.color.a = 0.7f;

		string det_class = det_array[m]["label_class"].asString();
		if(  det_class == "VEHICLE"){
            marker_det.color.b = 1.0f;
		}
		else if ( det_class == "PEDESTRIAN"){	
			marker_det.color.g = 1.0f;
			marker_det.color.b = 1.0f;
		}
		else
			ROS_INFO("We have undefined class %s", det_class.c_str());
		
		M_det.markers.push_back(marker_det);		
	}
	cout << "We have " << counter << " detections \tat " << timestamp << endl;
	

	for (int i=0; i<label_array.size(); i++){
		std::pair<int, string> track_label_uuid;
		if (! id_type.compare("uuid")){
			track_label_uuid.second = label_array[i]["track_label_uuid"].asString();
		}
		else{
			track_label_uuid.first = label_array[i]["track_label_uuid"].asInt();
		}

		marker.header.frame_id = "scan";
		marker.header.stamp = ros::Time::now();
		marker.action = visualization_msgs::Marker::ADD;
        marker.lifetime = ros::Duration(1/(play_rate+10));
		marker.ns = "result";
        marker.type = visualization_msgs::Marker::CUBE;
        
        marker.id = i;
        
        marker.pose.position.x = label_array[i]["center"]["x"].asDouble();
        marker.pose.position.y = label_array[i]["center"]["y"].asDouble();
        marker.pose.position.z = label_array[i]["center"]["z"].asDouble();

        marker.pose.orientation.x = label_array[i]["rotation"]["x"].asDouble();
        marker.pose.orientation.y = label_array[i]["rotation"]["y"].asDouble();
        marker.pose.orientation.z = label_array[i]["rotation"]["z"].asDouble();
        marker.pose.orientation.w = label_array[i]["rotation"]["w"].asDouble();
        
        marker.scale.x = label_array[i]["length"].asDouble();
        marker.scale.y = label_array[i]["width"].asDouble();
        marker.scale.z = label_array[i]["height"].asDouble();
        
        marker.color.b = 0.0f;
        marker.color.g = 0.0f; //1.0f
        marker.color.r = 0.0f;
	    marker.color.a = 0.7f;

		string track_class = label_array[i]["label_class"].asString();
		if(  track_class == "VEHICLE")
            marker.color.r = 1.0f;
		else if ( track_class == "PEDESTRIAN")
			marker.color.g = 1.0f;
		else
			ROS_INFO("We have undefined class %s", track_class.c_str());
	
		
		M_label.markers.push_back(marker);

		
		// id
		marker_id.header.frame_id="scan";
        marker_id.header.stamp = ros::Time::now();//to show every tag  (ros::Time::now()for latest tag)
        marker_id.action = visualization_msgs::Marker::ADD;
		marker_id.lifetime = ros::Duration(1/(play_rate+10));
		marker_id.ns = "tracking_id";
		marker_id.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        marker_id.pose.orientation.w = 1.0;
        // marker_id.id = i + 100;
        marker_id.id = i;

        marker_id.scale.z = 1.5f;
        marker_id.color.b = 0.0f;
        marker_id.color.g = 0.9f;
        marker_id.color.r = 0.9f;
        marker_id.color.a = 1;

        marker_id.pose.position.x = label_array[i]["center"]["x"].asDouble();
        marker_id.pose.position.y = label_array[i]["center"]["y"].asDouble();
        marker_id.pose.position.z = label_array[i]["center"]["z"].asDouble() + 1.0f;

		if(! id_type.compare("uuid")){
			for(int m=0; m<id_record.size(); m++){
				if ( !( (track_label_uuid.second).compare(id_record.at(m))) ) {
					marker_id.text = convertToString(m);
				}
			}
		}
		else{
			marker_id.text = convertToString(track_label_uuid.first);
		}

		M_id.markers.push_back(marker_id);
	}
	cout << "We have " << M_id.markers.size() << " trackers \tat " << timestamp << endl;


	if (M_label.markers.size() > label_max_size)
		label_max_size = M_label.markers.size();
	for (int a = label_array.size(); a < label_max_size; a++)
	{
		marker.id = a;
		marker.color.a = 0;
		marker.pose.position.x = 0;
		marker.pose.position.y = 0;
		marker.pose.position.z = 0;
		marker.scale.z = 0;
		M_label.markers.push_back(marker);

		// marker_id.id = a+100;
		marker_id.id = a;
		marker_id.color.a = 0;
		marker_id.pose.position.x = 0;
		marker_id.pose.position.y = 0;
		marker_id.pose.position.z = 0;
		marker_id.scale.z = 0;
		M_id.markers.push_back(marker_id);
	}

	if (M_det.markers.size() > det_max_size)
		det_max_size = M_det.markers.size();
	for (int a = M_det.markers.size(); a < det_max_size; a++)
	{
		// marker_det.id = a + 200;
		marker_det.id = a;
		marker_det.color.a = 0;
		marker_det.pose.position.x = 0;
		marker_det.pose.position.y = 0;
		marker_det.pose.position.z = 0;
		marker_det.scale.z = 0;
		M_det.markers.push_back(marker_det);
	}
	
	
	// Tracking history
	for (int i=0; i<label_array.size(); i++){
		std::pair<int, string> track_label_uuid;
		if (! id_type.compare("uuid")){
			track_label_uuid.second = label_array[i]["track_label_uuid"].asString();
		}
		else{
			track_label_uuid.first = label_array[i]["track_label_uuid"].asInt();
		}

		visualization_msgs::Marker marker_hist;
		marker_hist.header.frame_id = "world";
		marker_hist.header.stamp = ros::Time::now();
		marker_hist.action = visualization_msgs::Marker::ADD;
        marker_hist.lifetime = ros::Duration(1);
		marker_hist.ns = "History";
        marker_hist.type = visualization_msgs::Marker::LINE_STRIP;

		marker_hist.scale.x = 0.5;
        
        marker_hist.color.b = 0.0f;
        marker_hist.color.g = 0.0f; 
        marker_hist.color.r = 0.0f;
	    marker_hist.color.a = 0.7f;

		marker_hist.pose.orientation.w = 1.0;

		double point[3] = {label_array[i]["center"]["x"].asDouble(),
						   label_array[i]["center"]["y"].asDouble(),
						   label_array[i]["center"]["z"].asDouble()};
						   

		string track_class = label_array[i]["label_class"].asString();
		if(  track_class == "VEHICLE")
            marker_hist.color.r = 1.0f;
		else if ( track_class == "PEDESTRIAN")
			marker_hist.color.g = 1.0f;
		else
			ROS_INFO("We have undefined class %s", track_class.c_str());

		// If saw recorded id
		if(! id_type.compare("uuid")){
			for(int m=0; m<id_record.size(); m++){
				if ( !( (track_label_uuid.second).compare(id_record.at(m))) ) {
					// printf("Track id: %d\n", m);
					int id = m;
					if (id == 29) printf("Add Point: %lf %lf %lf\n", point[0], point[1], point[2]);
					int hist_len = add_point(id, point);
					// printf("Hist len: %d\n", hist_len);
					for (int j = 0; j < hist_len; j++) {
						marker_hist.id = i;
						// printf("i: %d\n", i);
						geometry_msgs::Point p;
						p.x = trajectory_points[id][j][0];
						p.y = trajectory_points[id][j][1];
						p.z = trajectory_points[id][j][2];
						// tf::Vector3 point_temp;
						// point_temp[0] = trajectory_points[id][j][0];
						// point_temp[1] = trajectory_points[id][j][1];
						// point_temp[2] = trajectory_points[id][j][2];
						// point_temp = trans_world * point_temp;
						// p.x = point_temp[0];
						// p.y = point_temp[1];
						// p.z = point_temp[2];
						// printf("Point:\n%lf %lf %lf\n", p.x, p.y, p.z);
						marker_hist.points.push_back(p);
					}	
				}
			}
		}
		else{
			// marker_id.text = convertToString(track_label_uuid.first);
		}
		M_hist.markers.push_back(marker_hist);
	}

	pub_id.publish(M_id);
	pub_label.publish(M_label);
	pub_det.publish(M_det);
	pub_hist.publish(M_hist);
	return;
}


void MySigintHandler(int sig)
{
	//这里主要进行退出前的数据保存、内存清理、告知其他节点等工作
	ROS_INFO("shutting down!");
	ros::shutdown();
}


int main(int argc, char** argv){
    ros::init(argc, argv, "viz_track_result");
    ros::NodeHandle nh("~");
	tf::TransformBroadcaster tf_broadcaster;
    
	pub_lidar = nh.advertise<sensor_msgs::PointCloud2>("/scan", 1000);
	pub_label = nh.advertise<visualization_msgs::MarkerArray>("/tracking_result", 1000);
	pub_id	  = nh.advertise<visualization_msgs::MarkerArray>("/marker_id", 1000);
	pub_det	  = nh.advertise<visualization_msgs::MarkerArray>("/detection_result", 1000);
	pub_hist  = nh.advertise<visualization_msgs::MarkerArray>("/tracking_hist", 10);

	signal(SIGINT, MySigintHandler);

    if ( !nh.getParam("log_path", log_path) ){
        ROS_INFO("Please provide the log path.");
        return -1;
    }
	if ( !nh.getParam("label_path", label_path) ){
        ROS_INFO("Please provide the label path.");
        return -1;
    }
	if ( !nh.getParam("det_path", det_path) ){
        ROS_INFO("Please provide the detection path.");
        return -1;
    }
	// if ( !nh.getParam("pose_path", pose_path) ){
	// 	ROS_INFO("Please provide the pose path");
	// 	return -1;
	// }
	
	nh.getParam("play_rate", play_rate);
	nh.getParam("id_type", id_type);

	// wait secs to open the Rviz
    double wait_rviz_sec;
	nh.getParam("rviz_wating_time", wait_rviz_sec);
	ROS_INFO("Wait %f sec for opening Rviz",wait_rviz_sec);
	ros::Duration(wait_rviz_sec).sleep();

	string lidar_string = log_path + "/lidar";
	string pose_string = log_path + "/poses";
    char dir_lidar[200], dir_pose[200];
	strcpy(dir_lidar, lidar_string.c_str());	// .c_str():將String物件轉換成C語言形式的字串常數
	strcpy(dir_pose, pose_string.c_str());	// .c_str():將String物件轉換成C語言形式的字串常數
	cout<<"dir_pose: "<< dir_pose << endl;

    vector <string> filenames, posefiles; 
	vector <unsigned long int> sorted_filenames, sorted_posefiles;
    showAllFiles( dir_lidar, 0, filenames);
	showAllFiles( dir_pose, 0, posefiles);
    sort_timestamp(filenames, sorted_filenames);
	sort_timestamp(posefiles, sorted_posefiles);
	
	std::cout << std::fixed; std::cout << std::setprecision(0);

	ros::Rate r(play_rate);
	for (int i=0; i<sorted_filenames.size(); i++){
        string lidar_time = convertToString(sorted_filenames.at(i));
		string pose_time = convertToString(sorted_posefiles.at(i));

		string ply_path = log_path + "/lidar/PC_" + lidar_time + ".ply";
		string pose_path = log_path + "/poses/city_SE3_egovehicle_" + lidar_time + ".json";
		string label_log_path = label_path + "/per_sweep_annotations_amodal/tracked_object_labels_" + lidar_time + ".json";
		string detection_path = det_path + "/per_sweep_annotations_amodal/tracked_object_labels_" + lidar_time + ".json";

		pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
		if (pcl::io::loadPLYFile<PointT>(ply_path, *cloud) == -1){
            PCL_ERROR("Couldn't read ",lidar_time ," file.\n");
            cout << i << endl;
            continue;
        }
		
		Json::Reader reader_label, reader_det, reader_pose;
		Json::Value label_array, det_array, pose;
		ifstream ifs1(label_log_path, ios::binary);
		ifstream ifs2(detection_path, ios::binary);
		ifstream ifs3(pose_path, ios::binary);
		if( !reader_label.parse(ifs1, label_array) ){
			cout<<"\033[1;34mFailed to parse "<< lidar_time <<" label.\033[0m"<<endl;
            continue;
		}
		if( !reader_det.parse(ifs2, det_array) ){
			cout<<"\033[1;34mFailed to parse "<< lidar_time <<" detection.\033[0m"<<endl;
            continue;
		}
		if( !reader_pose.parse(ifs3, pose) ){
			cout<<"\033[1;34mFailed to parse "<< lidar_time <<" pose.\033[0m"<<endl;
			cout<<"Pose path:" << pose_path <<endl;
            continue;
		}
		
		M_id.markers.clear();
		M_label.markers.clear();
		M_label.markers.shrink_to_fit();
		M_det.markers.clear();
		M_det.markers.shrink_to_fit();
		M_hist.markers.clear();
		M_hist.markers.shrink_to_fit();
		cout << "At frame "<<frame_count++<<endl;		
		
		trans_world = broadcast_worldToBasicLink(pose);
		tf_broadcaster.sendTransform(tf::StampedTransform(trans_world, ros::Time::now(),"world","scan"));
		initialize_marker(label_array, det_array);
		
		sensor_msgs::PointCloud2 sensor_scan;
		pcl::toROSMsg(*cloud, sensor_scan);
		sensor_scan.header.frame_id = "scan";
		// cout << cloud->points.size() << endl;
		// cout<<"Now pose at: "<< pose_time << endl;
		cout<<"We now at "<< lidar_time << "\nHave " << setw(5) << label_array.size() << " labels" << endl;
		cout << "---------------------------------------------------" << endl;
		pub_lidar.publish(sensor_scan);
		r.sleep();
	
	}

    return 0;
}