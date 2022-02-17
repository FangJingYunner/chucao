#include <ros/ros.h>
#include <pcl/common/common.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/crop_box.h>
#include <visualization_msgs/MarkerArray.h>
#include <Eigen/Dense>
#include <nav_msgs/GridCells.h>
#include <set>
#include <chrono>
#include <tf/LinearMath/Quaternion.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl/filters/voxel_grid.h>


using namespace std;
string camera_input_topic;
string input_Lidar = "/zvision_lidar_points";


// todo 点云不存在GridNode中，会占用内存
struct GridCloudNode{
    pcl::PointCloud<pcl::PointXYZ> Cloud;
    float ElevGrassHigh;

    GridCloudNode(){
        ElevGrassHigh = -100;
    }

    void ClearCloud(){
        Cloud.clear();
        ElevGrassHigh = -100;
    }
};
typedef GridCloudNode* GridCloudNodePtr;
GridCloudNodePtr ** GridCloudNodeMap;

pcl::PointCloud<pcl::PointXYZ>::Ptr laserCloudCrop(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZI>::Ptr ObjPointCloud(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZ>::Ptr CameraDeepCloud(new pcl::PointCloud<pcl::PointXYZ>());

//pcl::CropBox<pcl::PointXYZ> boundary_box_filter_;
//pcl::CropBox<pcl::PointXYZ> nearby_box_filter_;
//
//pcl::CropBox<pcl::PointXYZ> DeepCamera_bbox_filter_;

ros::Publisher GridCell_pub_;
ros::Publisher ObjPointCloudPub_;
ros::Publisher pub_MarkerArray_;
ros::Publisher full_cloud_pub_;
ros::Subscriber camera_sub_;

double dcx = 425.70074462890625;
double dcy = 234.9811248779297;
double dfx = 429.1876220703125;
double dfy = 429.1876220703125;
float kScaleFactor = 1000;
float DeepCameraHigh = 0.25;

float fx = 1264.252,fy = 1264.52,cx = 650.92,cy = 536.64;

float LidarHigh = 0.408;
float DCameraHigh = 0.25;

float quantileZ = 0.1;

bool newlaserCloud = false;

float GridSize = 0.5;
int GridSizeInverse = 1.0/GridSize;

const int GridCloudWidth = 81;
int GridCloudHalfWidth = (GridCloudWidth - 1) / 2;
int minGridCloudNum = 20;
float maxGrassHigh = 0.15;
float AbsObjElevThr = 0.4;
bool newCameraCloud = false;

const int GridNum = GridCloudWidth*GridCloudWidth;
int cluster_flag[GridNum] = {0}; // todo 这里需要修改一下，不然总是用const变量
int high_flag[GridNum] = {0}; // todo 这里需要修改一下，不然总是用const变量

//lidar->camera
Eigen::Matrix3d lidar2camera_R;
Eigen::Vector3d lidar2camera_T;

float dcamera2lidar_R[9] = {1,0,0,0,-1,0,0,0,1};
float dcamera2lidar_T[3] = {0,0.2,0};

float LidarxAxislimit = 10.0,LidaryAxislimit = 10.0;
float LidarzAxislimit1 = -LidarHigh - 0.5 ,LidarzAxislimit2 = 0.5;
float Lidarminlimit = 0.1;

float DCameraxAxislimit = 5,DCamerayAxislimit = 5;
float DCamerazAxislimit1 = -DCameraHigh - 0.5 ,DCamerazAxislimit2 = 0.5;
int DCameraSampleRate = 5;

// registered laser scan callback function
void laserCloudHandler(const sensor_msgs::PointCloud2ConstPtr &laserCloud2) {

    laserCloudCrop->clear();

    pcl::fromROSMsg(*laserCloud2, *laserCloudCrop);

    newlaserCloud = true;

}

void Deep2PointCloud(const sensor_msgs::ImageConstPtr & Depth_row_image) {

    auto cv_ptr =  cv_bridge::toCvCopy(*Depth_row_image, sensor_msgs::image_encodings::TYPE_16UC1);
    cv::Mat Depth_img = cv_ptr->image;

    int rows = Depth_img.rows, cols = Depth_img.cols;
    CameraDeepCloud->clear();
    pcl::PointXYZ thisPoint;
    //recognize top rows cause no use
    // todo 对camera cloud 随机降采样
    for (int i = 50; i < rows; i+=DCameraSampleRate)
    {
        for (int j = 0; j < cols; j+=DCameraSampleRate)
        {
            thisPoint.y = (double)(Depth_img.at<unsigned short>(i, j)) / kScaleFactor;
            if (isnan(thisPoint.y) || thisPoint.y <= 0 || thisPoint.y >= DCamerayAxislimit)
                continue;

            thisPoint.x = -((double)j - dcx) * thisPoint.y / dfx;
//            if (thisPoint.y <= -DCameraxAxislimit || thisPoint.y >= DCameraxAxislimit)
//                continue;

            thisPoint.z = -((double)i - dcy) * thisPoint.y / dfy;
//            if (thisPoint.y <= DCamerazAxislimit1 || thisPoint.y >= DCamerazAxislimit2)
//                continue;

            CameraDeepCloud->push_back(thisPoint);
        }
    }

}

void CameraHandler(const sensor_msgs::ImageConstPtr & Depth_row_image) {

    Deep2PointCloud(Depth_row_image);

//    std::vector<int> boxIndices_boundary;
//    DeepCamera_bbox_filter_.setInputCloud(CameraDeepCloud);
//    DeepCamera_bbox_filter_.filter(boxIndices_boundary);//
//    DeepCamera_bbox_filter_.filter(*CameraDeepCloud);
//
//    pcl::VoxelGrid<pcl::PointXYZ> sor;
//    sor.setInputCloud(CameraDeepCloud);
//    sor.setLeafSize(0.05f, 0.05f, 0.05f);
//    sor.filter(*CameraDeepCloud);

    pcl::PointXYZ temppoint;
    int cloudsize = CameraDeepCloud->size();
    for(int i = 0 ;i <cloudsize;i++) {
        temppoint.x = dcamera2lidar_R[0]*CameraDeepCloud->points[i].x +
                        dcamera2lidar_R[1]*CameraDeepCloud->points[i].y+
                        dcamera2lidar_R[2]*CameraDeepCloud->points[i].z + dcamera2lidar_R[0];
        temppoint.y = dcamera2lidar_R[3]*CameraDeepCloud->points[i].x +
                       dcamera2lidar_R[4]*CameraDeepCloud->points[i].y+
                       dcamera2lidar_R[5]*CameraDeepCloud->points[i].z + dcamera2lidar_R[1];
        temppoint.z = dcamera2lidar_R[6]*CameraDeepCloud->points[i].x +
                      dcamera2lidar_R[7]*CameraDeepCloud->points[i].y+
                      dcamera2lidar_R[8]*CameraDeepCloud->points[i].z + dcamera2lidar_R[2];
        CameraDeepCloud->points[i] = temppoint;
    }


//        Eigen::Vector3d v3d;
//
//    //dcamera->lidar
//    for(int i = 0 ;i <cloudsize;i++){
//        v3d<<CameraDeepCloud->points[i].x,CameraDeepCloud->points[i].y,CameraDeepCloud->points[i].z;
//        v3d = dcamera2lidar_R.transpose()*v3d + lidar2camera_T;
//        CameraDeepCloud->points[i].x = v3d.x();
//        CameraDeepCloud->points[i].y = v3d.y();
//        CameraDeepCloud->points[i].z = v3d.z();
//    }

    newCameraCloud = true;

}


void ProjectCloudToGrid(){
    std::vector<std::pair<int,int>> GridIdx;

    int CloudSize = laserCloudCrop->size();
    int indX,indY;

    // todo debug 模式下这个操作要0.75ms,之后可以考虑用到那一部分，那一部分clear一下,但是这样又要执行多个判断
    for(int i = 0;i<GridCloudWidth;i++)
        for(int j = 0;j<GridCloudWidth;j++)
            GridCloudNodeMap[i][j]->ClearCloud();

    for(int i = 0;i<CloudSize;i++){
        indX = floor(GridSizeInverse*(laserCloudCrop->points[i].x)) +GridCloudHalfWidth;
        indY = floor(GridSizeInverse*(laserCloudCrop->points[i].y )) +GridCloudHalfWidth;
        if(indX < 0 || indX > GridCloudWidth - 1 || indY < 0 || indY > GridCloudWidth - 1)
            continue;
        GridCloudNodeMap[indX][indY]->Cloud.push_back(laserCloudCrop->points[i]);
    }
}

inline static bool compare(const pcl::PointXYZ& p1,const pcl::PointXYZ& p2){
    return p1.z > p2.z;
}

void ComputeGridFeature(){

    for (int i = 0; i < GridNum; i++) {
        cluster_flag[i] = 0;
        high_flag[i] = 0;
    }

    set<pair<int,int>> ObjectInd;

    for (int indX = 0; indX < GridCloudWidth; indX++) {
        for (int indY = 0; indY < GridCloudWidth; indY++) {
            if(GridCloudNodeMap[indX][indY]->Cloud.size() > minGridCloudNum){

                int CloudSize = GridCloudNodeMap[indX][indY]->Cloud.size();

                int quantileID = int(quantileZ * CloudSize);
                sort(GridCloudNodeMap[indX][indY]->Cloud.points.begin(),
                     GridCloudNodeMap[indX][indY]->Cloud.points.end(),
                     compare);

                if (GridCloudNodeMap[indX][indY]->Cloud.points[quantileID].z >
                    GridCloudNodeMap[indX][indY]->Cloud.points[CloudSize - 1].z + maxGrassHigh) {
                    high_flag[indX * GridCloudWidth + indY] = 1; //1表示这个点可能是障碍物
                    GridCloudNodeMap[indX][indY]->ElevGrassHigh =
                            GridCloudNodeMap[indX][indY]->Cloud.points[quantileID].z
                            - GridCloudNodeMap[indX][indY]->Cloud.points[CloudSize - 1].z;

                }else{
                    //记录草地高度
                    GridCloudNodeMap[indX][indY]->ElevGrassHigh =
                            GridCloudNodeMap[indX][indY]->Cloud.points[quantileID].z
                            - GridCloudNodeMap[indX][indY]->Cloud.points[CloudSize - 1].z;
                }
            }
        }
    }

    for (int indX = 0; indX < GridCloudWidth; indX++) {
        for (int indY = 0; indY < GridCloudWidth; indY++) {
            if(high_flag[indX * GridCloudWidth + indY] == 1) {
                if (GridCloudNodeMap[indX][indY]->ElevGrassHigh > AbsObjElevThr) {
                    cluster_flag[indX * GridCloudWidth + indY] = 1;
                    continue;
                }
                else{
                    int count = 0;// 暂时采用计数方式，后面可以考虑用周围八个点的平均值的方式
                    for(int dx = -1;dx <= 1;dx++){
                        for(int dy = -1;dy <=1;dy++){
                            if((dx == 0 && dy == 0) || indX+dx<0||indX+dx>GridCloudWidth-1||indY+dy<0||indY+dy>GridCloudWidth-1)
                                continue;
                            if(GridCloudNodeMap[indX][indY]->ElevGrassHigh
                               > GridCloudNodeMap[indX+dx][indY+dy]->ElevGrassHigh)
                                count++;
                        }
                    }
                    if(count >6){
                        cluster_flag[indX * GridCloudWidth + indY] = 1;
                    }
                }
            }
        }
    }
}

void PublishGridCells(){

    nav_msgs::GridCells cells;

    cells.header.frame_id = "camera_init";
    cells.cell_height= GridSize;
    cells.cell_width = GridSize;


    geometry_msgs::Point obstacle;
    for(int indX = 0;indX<GridCloudWidth;indX++){
        for(int indY = 0;indY<GridCloudWidth;indY++) {
            if(cluster_flag[indX * GridCloudWidth + indY] > 1){
                obstacle.x = GridSize*(indX - GridCloudHalfWidth);
                obstacle.y = GridSize*(indY - GridCloudHalfWidth);

                // todo 可以改为该栅格最高点的值
                obstacle.z = 0;
                cells.cells.push_back(obstacle);
            }
        }
    }

    GridCell_pub_.publish(cells);
}

void ClusterAndPubObjectGrid(){

    ObjPointCloud->clear();
    //栅格BFS聚类
    int label = 2;
    pcl::PointXYZI thispoint;
    int aa = 1;
    for(int indX = 0 ; indX <GridCloudWidth;indX++) {
        for (int indY = 0; indY < GridCloudWidth; indY++) {
            if (cluster_flag[indX * GridCloudWidth + indY] == 1) {
                vector<pair<int, int>> neighbour;

                neighbour.push_back(pair(indX, indY));
                while (!neighbour.empty()) {

                    pair<int, int> thisgrid = neighbour.back();
                    neighbour.pop_back();
                    int indx = thisgrid.first;
                    int indy = thisgrid.second;

                    cluster_flag[indx * GridCloudWidth + indy] = label;

                    int CloudSize = GridCloudNodeMap[indx][indy]->Cloud.size();
                    for (int i = 0; i < CloudSize; i++) {
                        thispoint.x = GridCloudNodeMap[indx][indy]->Cloud.points[i].x;
                        thispoint.y = GridCloudNodeMap[indx][indy]->Cloud.points[i].y;
                        thispoint.z = GridCloudNodeMap[indx][indy]->Cloud.points[i].z;
                        thispoint.intensity = label;
                        ObjPointCloud->push_back(thispoint);
                    }

                    for (int dx = -1; dx <= 1; dx++) {
                        for (int dy = -1; dy <= 1; dy++) {
                            if (cluster_flag[(indx+dx) * GridCloudWidth + (indy + dy)] != 1 || (dx == 0 && dy == 0) ||
                                    indx + dx < 0 || indx + dx > GridCloudWidth - 1 || indy + dy < 0 ||
                                    indy + dy > GridCloudWidth - 1)
                                continue;
                            neighbour.push_back(pair(indx + dx, indy + dy));
                        }
                    }
                }
                label++;
            }
        }
    }

}

void PubBoundingBox(){
    if(ObjPointCloud->empty())
        return;

    visualization_msgs::Marker bbox_marker;
    bbox_marker.header.frame_id = "camera_init";
    bbox_marker.ns = "";
    bbox_marker.color.r = 0.0f;
    bbox_marker.color.g = 1.0f;
    bbox_marker.color.b = 0.0f;
    bbox_marker.color.a = 0.5;
    bbox_marker.lifetime = ros::Duration();
    bbox_marker.frame_locked = true;
    bbox_marker.type = visualization_msgs::Marker::CUBE;
    bbox_marker.action = visualization_msgs::Marker::ADD;

    visualization_msgs::MarkerArray marker_array;
    pcl::PointCloud<pcl::PointXYZ> TempCloud;
    pcl::PointXYZ SumPoint,maxpoint,minpoint,temppoint;
    maxpoint.x = -100;maxpoint.y = -100;maxpoint.z = -100;
    minpoint.x = 100;minpoint.y = 100;minpoint.z = 100;
    size_t CntNum = 0;
    size_t marker_id = 0;
    pcl::PointXYZ min;	//xyz的最小值
    pcl::PointXYZ max;	//xyz的最大值

    static size_t max_marker_size_ = 0;
    int flag_intensity = ObjPointCloud->begin()->intensity;
    size_t cntcloudsize = 0;
    for(auto iter = ObjPointCloud->begin();iter != ObjPointCloud->end();iter++){
        if(iter->intensity == flag_intensity){
            SumPoint.x += iter->x;
            SumPoint.y += iter->y;
            SumPoint.z += iter->z;

            temppoint.x = iter->x;
            temppoint.y = iter->y;
            temppoint.z = iter->z;
            TempCloud.push_back(temppoint);

            CntNum++;
            cntcloudsize++;
            continue;
        }
        pcl::getMinMax3D(TempCloud,min,max);	//在聚类的坐标系中计算最大最小值

        bbox_marker.id = marker_id;
        bbox_marker.color.a = 0.5;
        bbox_marker.pose.position.x = SumPoint.x/CntNum;
        bbox_marker.pose.position.y = SumPoint.y/CntNum;
        bbox_marker.pose.position.z = SumPoint.z/CntNum;
        bbox_marker.scale.x = max.x - min.x +0.25;
        bbox_marker.scale.y = max.y - min.y +0.25;
        bbox_marker.scale.z = max.z - min.z +0.25;
        bbox_marker.header.stamp = ros::Time::now();
        marker_array.markers.push_back(bbox_marker);


        flag_intensity = iter->intensity;
        iter--;
        marker_id++;
        CntNum = 0;
        maxpoint.x = -100;maxpoint.y = -100;maxpoint.z = -100;
        minpoint.x = 100;minpoint.y = 100;minpoint.z = 100;
        SumPoint.x = 0;SumPoint.y = 0;SumPoint.z = 0;
        TempCloud.clear();
    }

    pcl::getMinMax3D(TempCloud,min,max);	//在聚类的坐标系中计算最大最小值
    bbox_marker.id = marker_id;
    bbox_marker.color.a = 0.5;
    bbox_marker.pose.position.x = SumPoint.x/CntNum;
    bbox_marker.pose.position.y = SumPoint.y/CntNum;
    bbox_marker.pose.position.z = SumPoint.z/CntNum;
    bbox_marker.scale.x = max.x - min.x;
    bbox_marker.scale.y = max.y - min.y;
    bbox_marker.scale.z = max.z - min.z;
    bbox_marker.header.stamp = ros::Time::now();
    marker_array.markers.push_back(bbox_marker);

    if (marker_array.markers.size() > max_marker_size_){
        max_marker_size_ = marker_array.markers.size();

    }
    else{
        for (size_t i = marker_id+1; i < max_marker_size_; ++i)
        {
            bbox_marker.id = i;
            bbox_marker.color.a = 0;
            bbox_marker.pose.position.x = 0;
            bbox_marker.pose.position.y = 0;
            bbox_marker.pose.position.z = 0;
            bbox_marker.scale.x = 0;
            bbox_marker.scale.y = 0;
            bbox_marker.scale.z = 0;
            marker_array.markers.push_back(bbox_marker);
            ++marker_id;
        }
    }

    pub_MarkerArray_.publish(marker_array);
}

void PublishCloud(){

    sensor_msgs::PointCloud2 laserCloudTemp;
    if (ObjPointCloudPub_.getNumSubscribers() != 0) {
        pcl::toROSMsg(*ObjPointCloud, laserCloudTemp);
        laserCloudTemp.header.stamp = ros::Time::now();
        laserCloudTemp.header.frame_id = "camera_init";
        ObjPointCloudPub_.publish(laserCloudTemp);
    }

    if (full_cloud_pub_.getNumSubscribers() != 0) {
        pcl::toROSMsg(*laserCloudCrop, laserCloudTemp);
        laserCloudTemp.header.stamp = ros::Time::now();
        laserCloudTemp.header.frame_id = "camera_init";
        full_cloud_pub_.publish(laserCloudTemp);
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "ObjectSegVoxelGrid");

    ros::NodeHandle nh("~");

    // Do parameter stuff.
    {
/*        lidar2camera_R << 0.9994, -0.006152, 0.034226, -0.034151, 0.011869, 0.99935, -0.0065542, -0.99991, 0.01165;
        lidar2camera_T << -0.00305, 0.42377, -0.09468;
        vector<double> l2c_R, l2c_T;
        nh.param<vector<double>>("lidar2camera_T", l2c_T, l2c_T);
        nh.param<vector<double>>("lidar2camera_R", l2c_R, l2c_R);

        if (!l2c_R.empty()) {
            lidar2camera_R(0, 0) = l2c_R[0];
            lidar2camera_R(0, 1) = l2c_R[1];
            lidar2camera_R(0, 2) = l2c_R[2];
            lidar2camera_R(1, 0) = l2c_R[3];
            lidar2camera_R(1, 1) = l2c_R[4];
            lidar2camera_R(1, 2) = l2c_R[5];
            lidar2camera_R(2, 0) = l2c_R[6];
            lidar2camera_R(2, 1) = l2c_R[7];
            lidar2camera_R(2, 2) = l2c_R[8];
        }

        if (!l2c_T.empty()) {
            lidar2camera_T(0) = l2c_T[0];
            lidar2camera_T(1) = l2c_T[1];
            lidar2camera_T(2) = l2c_T[2];
        }*/

/*        nh.param<float>("VoxelGridSize", GridSize);
        nh.param<int>("GridCloudWidth", GridCloudWidth);

        nh.param<float>("LidarHigh", LidarHigh);
        nh.param<float>("minGridCloudNum", minGridCloudNum);
        nh.param<float>("maxGrassHigh", maxGrassHigh);
        nh.param<float>("AbsObjElevThr", AbsObjElevThr);

        nh.param<string>("camera_input_topic", camera_input_topic, "/camera/depth/image_rect_raw");
        nh.param<double>("dcx", dcx);
        nh.param<double>("dcy", dcy);
        nh.param<double>("dfx", dfx);
        nh.param<double>("dfy", dfy);
        nh.param<double>("kScaleFactor", kScaleFactor);
        nh.param<float>("DeepCameraHigh", DeepCameraHigh);
        nh.param<float>("cx", cx);
        nh.param<float>("cy", cy);
        nh.param<float>("fx", fx);
        nh.param<float>("fy", fy);
        nh.param<string>("input_Lidar", input_Lidar);*/
        GridCloudHalfWidth = (GridCloudWidth - 1) / 2;
        GridSizeInverse = 1 / GridSize;
    }

    ros::Subscriber subLaserCloud =
            nh.subscribe<sensor_msgs::PointCloud2>(input_Lidar, 5, laserCloudHandler);

    GridCell_pub_ = nh.advertise<nav_msgs::GridCells>("/gridCell", 1);
    ObjPointCloudPub_ = nh.advertise<sensor_msgs::PointCloud2>("/ObjPointCloud", 1);
    pub_MarkerArray_ = nh.advertise<visualization_msgs::MarkerArray>("/LidarObjBox",1);
    full_cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/fullPointCloud", 1);

    camera_sub_ = nh.subscribe(camera_input_topic, 1, CameraHandler);

    GridCloudNodeMap = new GridCloudNodePtr * [GridCloudWidth];
    for(int i = 0;i<GridCloudWidth;i++){
        GridCloudNodeMap[i] = new GridCloudNodePtr  [GridCloudWidth];
        for(int j = 0;j<GridCloudWidth;j++){
            GridCloudNodeMap[i][j]= new GridCloudNode();
        }
    }

    ros::Rate rate(10);
    bool status = ros::ok();
    while (status) {

        ros::spinOnce();

        static float avg_ms = 0;
        static int loop = 1;
        std::chrono::high_resolution_clock::time_point start = std::chrono::high_resolution_clock::now();

        if(newCameraCloud || newlaserCloud){
            if(newCameraCloud)
                newCameraCloud = false;
            if(newlaserCloud)
                newlaserCloud = false;
        }
        else{
            continue;
        }

        *laserCloudCrop += *CameraDeepCloud;

        ProjectCloudToGrid();

        ComputeGridFeature();

        ClusterAndPubObjectGrid();

        PubBoundingBox();

        std::chrono::high_resolution_clock::time_point end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double, std::milli> fp_ms = end - start;
        avg_ms += fp_ms.count();
        std::cout<<"avg time consume:"<<avg_ms/loop<<endl;
        loop++;

        PublishGridCells();

        PublishCloud();
    }
}


