#include "utility.h"
#include "lio_sam/cloud_info.h"

// 定义曲率结构体，包含曲率值和点云索引
struct smoothness_t{ 
    float value;        // 曲率值
    size_t ind;         // 点云中的索引
};

// 用于曲率排序的比较器
struct by_value{ 
    bool operator()(smoothness_t const &left, smoothness_t const &right) { 
        return left.value < right.value;    // 升序排序
    }
};

class FeatureExtraction : public ParamServer
{
public:

    // ROS通信相关成员变量
    ros::Subscriber subLaserCloudInfo;                  // 订阅去畸变后的点云信息
    ros::Publisher pubLaserCloudInfo;                   // 发布特征点云信息
    ros::Publisher pubCornerPoints;                     // 发布角点云
    ros::Publisher pubSurfacePoints;                    // 发布平面点云

    // 点云数据容器
    pcl::PointCloud<PointType>::Ptr extractedCloud;     // 原始输入点云
    pcl::PointCloud<PointType>::Ptr cornerCloud;        // 提取的角点/边缘点
    pcl::PointCloud<PointType>::Ptr surfaceCloud;       // 提取的平面点

    pcl::VoxelGrid<PointType> downSizeFilter;           // 体素滤波器用于下采样平面点

    lio_sam::cloud_info cloudInfo;                      // 自定义消息类型，存储处理后的点云信息
    std_msgs::Header cloudHeader;                       // 点云消息头

    // 特征提取中间数据
    std::vector<smoothness_t> cloudSmoothness;          // 存储各点曲率
    float *cloudCurvature;                              // 曲率数组
    int *cloudNeighborPicked;                           // 标记相邻点是否被选中
    int *cloudLabel;                                    // 点类型标签（角点/平面点）

    FeatureExtraction()
    {
        // 初始化订阅与发布
        // 输入
        // 在ROS中，ros::TransportHints().tcpNoDelay() 的作用是 ​​启用TCP_NODELAY选项​​，其含义是​​禁用Nagle算法​​
        // 默认情况下，TCP协议会启用Nagle算法，它会将多个小数据包合并为一个更大的包发送，以减少网络负载。但合并过程会引入延迟（等待更多数据）。​禁用Nagle​​：设置TCP_NODELAY后，数据包会​​立即发送​​，不等待合并，从而降低传输延迟。
        subLaserCloudInfo = nh.subscribe<lio_sam::cloud_info>("lio_sam/deskew/cloud_info", 1, &FeatureExtraction::laserCloudInfoHandler, this, ros::TransportHints().tcpNoDelay());
        pubLaserCloudInfo = nh.advertise<lio_sam::cloud_info> ("lio_sam/feature/cloud_info", 1);
        // 发布角点云
        pubCornerPoints = nh.advertise<sensor_msgs::PointCloud2>("lio_sam/feature/cloud_corner", 1);
        // 发布平面点云
        pubSurfacePoints = nh.advertise<sensor_msgs::PointCloud2>("lio_sam/feature/cloud_surface", 1);
        
        // 初始化参数
        initializationValue();
    }

    void initializationValue()
    {
        // 按雷达线数分配空间
        cloudSmoothness.resize(N_SCAN*Horizon_SCAN);

        // 配置体素滤波器参数（平面点下采样）
        downSizeFilter.setLeafSize(odometrySurfLeafSize, odometrySurfLeafSize, odometrySurfLeafSize);

        // 初始化点云容器
        extractedCloud.reset(new pcl::PointCloud<PointType>());
        cornerCloud.reset(new pcl::PointCloud<PointType>());
        surfaceCloud.reset(new pcl::PointCloud<PointType>());

        // 动态分配数组内存
        cloudCurvature = new float[N_SCAN*Horizon_SCAN];
        cloudNeighborPicked = new int[N_SCAN*Horizon_SCAN];
        cloudLabel = new int[N_SCAN*Horizon_SCAN];
    }

    // 主回调函数, 在接收到
    void laserCloudInfoHandler(const lio_sam::cloud_infoConstPtr& msgIn)
    {
        cloudInfo = *msgIn; // new cloud info               // 获取消息实体
        cloudHeader = msgIn->header; // new cloud header    // 获取消息头
        pcl::fromROSMsg(msgIn->cloud_deskewed, *extractedCloud); // new cloud for extraction    // 转换ROS消息为PCL格式

        // 特征提取流水线
        calculateSmoothness();      // 计算曲率

        // 过滤掉因为是不通物体远距离重叠导致可能出现计算误差的点
        markOccludedPoints();       // 标记遮挡点

        // 提取 平面点和边缘点
        extractFeatures();          // 提取特征

        publishFeatureCloud();      // 发布结果
    }

    // 曲率计算 calculateSmoothness​，计算曲率
    void calculateSmoothness()
    {
        int cloudSize = extractedCloud->points.size();
        for (int i = 5; i < cloudSize - 5; i++)
        {
            // 计算当前点前后5个点的距离差异（类似二阶导数）
            // pointRange 是激光雷达点云中每个点的​​测距值​​（即该点到雷达传感器的欧氏距离），用于区分​​边缘点​​和​​平面点​​
            // 前五个点和后五个点与当前点的差，如果是平面点，差值会很小，如果是边缘点，差值会很大
            float diffRange = cloudInfo.pointRange[i-5] + cloudInfo.pointRange[i-4]
                            + cloudInfo.pointRange[i-3] + cloudInfo.pointRange[i-2]
                            + cloudInfo.pointRange[i-1] - cloudInfo.pointRange[i] * 10
                            + cloudInfo.pointRange[i+1] + cloudInfo.pointRange[i+2]
                            + cloudInfo.pointRange[i+3] + cloudInfo.pointRange[i+4]
                            + cloudInfo.pointRange[i+5];            

            // 曲率平方，归一化，记录当前点的点云曲率
            cloudCurvature[i] = diffRange*diffRange;

            // 初始化标记数组，用来防止
            cloudNeighborPicked[i] = 0;
            cloudLabel[i] = 0;

            // cloudSmoothness for sorting
            cloudSmoothness[i].value = cloudCurvature[i];   // 存储曲率与索引
            cloudSmoothness[i].ind = i;
        }
    }

    // ​​遮挡点标记 markOccludedPoints​
    // 剔除被遮挡点​​
    // 消除因物体间遮挡导致的测量异常点（如物体边缘后的虚假点）
    // 避免遮挡点被误判为边缘特征（Corner Points）
    // ​​抑制平行光束噪声​​
    // 处理激光束与物体表面近平行时产生的测量不稳定点
    // 减少因表面反射特性引起的噪点误分类为平面特征（Surface Points）

    // 边缘点和平面点是针对一个物体而言的，对于物体之间的遮挡产生的特征，没有特殊性
     void markOccludedPoints()
    {
        int cloudSize = extractedCloud->points.size();
        // mark occluded points and parallel beam points
        for (int i = 5; i < cloudSize - 6; ++i)
        {
            // occluded points
            // 遍历所有点，判断当前点和后一个点的深度差
            float depth1 = cloudInfo.pointRange[i];
            float depth2 = cloudInfo.pointRange[i+1];
            int columnDiff = std::abs(int(cloudInfo.pointColInd[i+1] - cloudInfo.pointColInd[i]));
            
            // 处理深度突变（遮挡情况）
            // 如果深度差很小
            if (columnDiff < 10){
                // 10 pixel diff in range image
                // 深度差>0.3m
                // 如果前一个远，后一个点近，则标记前5个点
                if (depth1 - depth2 > 0.3){
                    // 标记前5个点
                    cloudNeighborPicked[i - 5] = 1;
                    cloudNeighborPicked[i - 4] = 1;
                    cloudNeighborPicked[i - 3] = 1;
                    cloudNeighborPicked[i - 2] = 1;
                    cloudNeighborPicked[i - 1] = 1;
                    cloudNeighborPicked[i] = 1;
                }else if (depth2 - depth1 > 0.3){       // 深度差<-0.3m
                    // 标记后6个点
                    // 如果后一个点远，前一个点近，则标记后五个点
                    cloudNeighborPicked[i + 1] = 1;
                    cloudNeighborPicked[i + 2] = 1;
                    cloudNeighborPicked[i + 3] = 1;
                    cloudNeighborPicked[i + 4] = 1;
                    cloudNeighborPicked[i + 5] = 1;
                    cloudNeighborPicked[i + 6] = 1;
                }
            }
            // parallel beam
            // 平行光束处理（距离突变超过2%）
            float diff1 = std::abs(float(cloudInfo.pointRange[i-1] - cloudInfo.pointRange[i]));
            float diff2 = std::abs(float(cloudInfo.pointRange[i+1] - cloudInfo.pointRange[i]));
            // if (相邻点距离变化>2%): 标记当前点
            // 如果当前点的深度变化太大，则标记当前点
            if (diff1 > 0.02 * cloudInfo.pointRange[i] && diff2 > 0.02 * cloudInfo.pointRange[i])
                cloudNeighborPicked[i] = 1;
        }
    }

    // 特征提取
    void extractFeatures()
    {
        cornerCloud->clear();           // 清除边缘点缓存
        surfaceCloud->clear();          // 清除平面点缓存

        pcl::PointCloud<PointType>::Ptr surfaceCloudScan(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr surfaceCloudScanDS(new pcl::PointCloud<PointType>());

        // 遍历每一个扫描线
        for (int i = 0; i < N_SCAN; i++)
        {
            surfaceCloudScan->clear();
            // 每根扫描线分为6段处理
            for (int j = 0; j < 6; j++)
            {

                // sp = 计算分段起点
                int sp = (cloudInfo.startRingIndex[i] * (6 - j) + cloudInfo.endRingIndex[i] * j) / 6;
                // ep = 计算分段终点
                int ep = (cloudInfo.startRingIndex[i] * (5 - j) + cloudInfo.endRingIndex[i] * (j + 1)) / 6 - 1;

                if (sp >= ep)
                    continue;

                // 按曲率排序（升序）
                std::sort(cloudSmoothness.begin()+sp, cloudSmoothness.begin()+ep, by_value());

                // 提取角点（曲率大的点）
                int largestPickedNum = 0;
                // 从 end 到 start 遍历
                for (int k = ep; k >= sp; k--)
                {
                    int ind = cloudSmoothness[k].ind;
                    // 未被标记且曲率>阈值
                    if (cloudNeighborPicked[ind] == 0 && cloudCurvature[ind] > edgeThreshold)
                    {
                        // 标记为角点;
                        // 如果数量超过20个就停止
                        largestPickedNum++;
                        if (largestPickedNum <= 20){
                            cloudLabel[ind] = 1;
                            cornerCloud->push_back(extractedCloud->points[ind]);
                        } else {
                            break;
                        }

                        // 标记邻居点，阻止相邻点被选中;
                        cloudNeighborPicked[ind] = 1;
                        for (int l = 1; l <= 5; l++)
                        {
                            int columnDiff = std::abs(int(cloudInfo.pointColInd[ind + l] - cloudInfo.pointColInd[ind + l - 1]));
                            if (columnDiff > 10)
                                break;
                            cloudNeighborPicked[ind + l] = 1;
                        }
                        for (int l = -1; l >= -5; l--)
                        {
                            int columnDiff = std::abs(int(cloudInfo.pointColInd[ind + l] - cloudInfo.pointColInd[ind + l + 1]));
                            if (columnDiff > 10)
                                break;
                            cloudNeighborPicked[ind + l] = 1;
                        }
                    }
                }

                // 提取平面点（曲率小的点）
                // 从sp到ep顺序遍历
                for (int k = sp; k <= ep; k++)
                {
                    int ind = cloudSmoothness[k].ind;
                    // 未被标记且曲率<阈值
                    if (cloudNeighborPicked[ind] == 0 && cloudCurvature[ind] < surfThreshold)
                    {

                        cloudLabel[ind] = -1;
                        cloudNeighborPicked[ind] = 1;

                        for (int l = 1; l <= 5; l++) {

                            int columnDiff = std::abs(int(cloudInfo.pointColInd[ind + l] - cloudInfo.pointColInd[ind + l - 1]));
                            if (columnDiff > 10)
                                break;

                            cloudNeighborPicked[ind + l] = 1;
                        }
                        for (int l = -1; l >= -5; l--) {

                            int columnDiff = std::abs(int(cloudInfo.pointColInd[ind + l] - cloudInfo.pointColInd[ind + l + 1]));
                            if (columnDiff > 10)
                                break;

                            cloudNeighborPicked[ind + l] = 1;
                        }
                    }
                }

                for (int k = sp; k <= ep; k++)
                {
                    if (cloudLabel[k] <= 0){
                        surfaceCloudScan->push_back(extractedCloud->points[k]);
                    }
                }
            }

            surfaceCloudScanDS->clear();
            downSizeFilter.setInputCloud(surfaceCloudScan);
            downSizeFilter.filter(*surfaceCloudScanDS);

            *surfaceCloud += *surfaceCloudScanDS;
        }
    }

    void freeCloudInfoMemory()
    {
        cloudInfo.startRingIndex.clear();
        cloudInfo.endRingIndex.clear();
        cloudInfo.pointColInd.clear();
        cloudInfo.pointRange.clear();
    }

    void publishFeatureCloud()
    {
        // free cloud info memory
        freeCloudInfoMemory();
        // save newly extracted features
        cloudInfo.cloud_corner  = publishCloud(pubCornerPoints,  cornerCloud,  cloudHeader.stamp, lidarFrame);
        cloudInfo.cloud_surface = publishCloud(pubSurfacePoints, surfaceCloud, cloudHeader.stamp, lidarFrame);
        // publish to mapOptimization
        pubLaserCloudInfo.publish(cloudInfo);
    }
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "lio_sam");

    FeatureExtraction FE;

    ROS_INFO("\033[1;32m----> Feature Extraction Started.\033[0m");
   
    ros::spin();

    return 0;
}