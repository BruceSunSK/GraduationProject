#pragma once
#include <vector>
#include <chrono>
#include <iomanip>
#include <fstream>
#include <iostream>

#include <sys/stat.h>

#include <opencv2/core.hpp>


/// @brief 所有规划器的接口类
/// 设置的初衷是为了统一所有规划器接口，其中所有规划器都使用栅格代价地图进行全局规划，栅格代价地图传入的方式为cv::Mat，具体内部的存储格式由各个规划器自行定义。
/// 使用栅格代价地图的最根本原因是因为课题组为了非结构化场景，构建了多层代价地图，因此全局规划要兼容多层代价地图。
/// 而多层代价地图本质就是栅格地图，因此在此规划器都使用栅格地图进行全局规划。（而且不使用描述障碍物位置、尺寸的方式，如果有需要，可以转换成代价地图）
class GlobalPlannerInterface
{
public:
    /// @brief 所有规划器参数的接口类，用于保存规划器使用的所有参数
    struct GlobalPlannerParams
    {
        virtual ~GlobalPlannerParams() = default;
    };

    /// @brief 所有辅助规划器的接口类，用于记录规划耗时等信息，并打印规划结果、参数等信息
    class GlobalPlannerHelper
    {
    public:
        GlobalPlannerHelper(GlobalPlannerInterface * planner) : planner_(planner) {}
        virtual ~GlobalPlannerHelper() = default;

        /// @brief 打印所有的信息，包括规划器参数信息、规划地图信息、规划结果信息，并可以将结果保存到指定路径中。由外部planner调用后执行
        /// @param save 是否保存到本地
        /// @param save_dir_path 保存的路径
        void showAllInfo(const bool save = false, const std::string & save_dir_path = "") const
        {
            const std::string dt = daytime();

            std::stringstream info;
            info << "--------------- [" << planner_->planner_name_ << " Info] ---------------\n"
                 << "Daytime: " << dt << "\tAuthor: BruceSun\n\n"
                 << paramsInfo() << std::endl
                 << mapInfo() << std::endl
                 << resultInfo()
                 << "--------------- [" << planner_->planner_name_ << " Info] ---------------\n\n";
            std::cout << info.str();

            if (save)
            {
                std::string file_path = save_dir_path;
                if (file_path.back() != '/')
                {
                    file_path.push_back('/');
                }
                file_path += (planner_->planner_name_ + "/");
                createDir(file_path);
                file_path += (dt + ' ' + planner_->planner_name_ + "_All_Info.txt");

                if (saveInfo(info.str(), file_path))
                {
                    std::cout << "[" << planner_->planner_name_ << " Info]: All Info Has Saved to " << file_path << std::endl;
                }
                else
                {
                    std::cerr << "[" << planner_->planner_name_ << " Info]: All Info Failed to Save to " << file_path << std::endl;
                }
            }
        }
        /// @brief 清空当前记录的所有结果信息，便于下次记录
        virtual void resetResultInfo() = 0;

    protected:
        const GlobalPlannerInterface * planner_ = nullptr;      // helper对应的规划器，在构造时指定

        /// @brief 将规划器中设置的参数以字符串的形式输出
        /// @return 规划器的参数
        virtual std::string paramsInfo() const = 0;
        /// @brief 将规划器所使用的地图信息、起点、终点以字符串的形式输出
        /// @return 地图信息、起点、终点信息
        virtual std::string mapInfo() const = 0;
        /// @brief 将helper中保存的所有有关规划的结果以字符串的形式输出
        /// @return 规划的结果数值
        virtual std::string resultInfo() const = 0;

        /// @brief 将给定的内容info直接写入file_path中
        /// @param info 待写入的内容
        /// @param file_path 文件路径和文件名字
        /// @return 保存是否成功
        static bool saveInfo(const std::string & info, const std::string & file_path)
        {
            std::ofstream ofs(file_path, std::ios::trunc);
            if (!ofs.is_open())
            {
                return false;
            }
            ofs << info;
            ofs.close();
            return true;
        }
        /// @brief 以字符串形式"%Y-%m-%d %H:%M:%S"获得当前日前和时间。
        /// @return 日期时间字符串
        static std::string daytime()
        {
            const auto now = std::chrono::system_clock::now();              // 当前时间点
            const auto now_tt = std::chrono::system_clock::to_time_t(now);  // 转为时间戳
            const auto now_tm = std::localtime(&now_tt);                    // 根据时间戳和时区返回实际的年月日时分秒

            std::stringstream ss;
            ss << std::put_time(now_tm, "%Y-%m-%d %H:%M:%S");
            return ss.str();
        }
        /// @brief 创建指定目录，若目录存在则不创建，若目录不存在则创建目录及其父目录。如果有同名的文件，则创建失败。
        /// @param dir_path 待创建的目录路径
        /// @return true 创建成功，false 创建失败
        bool createDir(const std::string & dir_path) const
        {
            struct stat st;
            if (stat(dir_path.c_str(), &st) == 0)   // 判断路径是否存在，若存在则进一步判断
            {
                if (S_ISDIR(st.st_mode))    // 判断是否是目录
                {
                    return true;            // 无需创建
                }
                else                        // 路径存在，但不是目录
                {
                    std::cout << "[" << planner_->planner_name_ << "]: \"" << dir_path << "\" is not a directory!" << std::endl;
                    return false;
                }
            }
            else    // 路径不存在，则直接调用系统命令递归创建路径。
            {
                // ubuntu 18.04默认gcc版本为7.5.0，而c++17支持的filesystem至少需要gcc8以上，因此没有标准库可用。
                // 手动创建的话需要mkdir函数，但该函数只能创建一层文件夹，太麻烦啦，直接调用shell了，虽然很难看。
                return std::system(("mkdir -p " + dir_path).c_str()) == 0;
            }
        }
    };

public:
    GlobalPlannerInterface() = default;
    virtual ~GlobalPlannerInterface() = default;

    /// @brief 初始化规划器参数，使用拷贝进行传递
    /// @param params 待设置的规划器参数
    virtual void initParams(const GlobalPlannerParams & params) = 0;
    /// @brief 设置栅格地图，默认所有规划器都是使用栅格地图进行规划
    /// @param map 栅格地图
    /// @return 设置是否成功
    virtual bool setMap(const cv::Mat & map) = 0;
    /// @brief 设置地图的参数信息
    /// @param res 分辨率
    /// @param ori_x 原点x
    /// @param ori_y 原点y
    virtual void setMapInfo(const double res, const double ori_x, const double ori_y)
    {
        res_ = res;
        ori_x_ = ori_x;
        ori_y_ = ori_y;
        init_map_info_ = true;
    }
    /// @brief 设置规划路径的起点。以真实地图坐标形式，而非行列形式。
    /// @param x 真实地图坐标系的x值
    /// @param y 真实地图坐标系的y值
    /// @return 该点是否能够成为起点。即该点在地图内部且不在障碍物上。
    virtual bool setStartPoint(const double x, const double y) = 0;
    /// @brief 设置规划路径的起点。以真实地图坐标形式，而非行列形式。
    /// @param p 真实地图坐标系的点
    /// @return 该点是否能够成为起点。即该点在地图内部且不在障碍物上。
    virtual bool setStartPoint(const cv::Point2d & p) = 0;
    /// @brief 设置规划路径的起点的朝向。
    /// @param yaw x轴为0，右手坐标系；单位为弧度；范围为[-pi, pi]。
    /// @return 该设置有效。规划器无设置起点朝向的功能时，返回false，否则返回true。
    virtual bool setStartPointYaw(const double yaw)
    {
        std::cout << "[" << planner_name_ << "]: \"setStartPointYaw\" is not implemented in this planner!" << std::endl;
        return false;
    }
    /// @brief 设置规划路径的终点。以真实地图坐标形式，而非行列形式。
    /// @param x 真实地图坐标系的x值
    /// @param y 真实地图坐标系的y值
    /// @return 该点是否能够成为终点。即该点在地图内部且不在障碍物上。
    virtual bool setEndPoint(const double x, const double y) = 0;
    /// @brief 设置规划路径的终点。以真实地图坐标形式，而非行列形式。
    /// @param p 真实地图坐标系的点
    /// @return 该点是否能够成为终点。即该点在地图内部且不在障碍物上。
    virtual bool setEndPoint(const cv::Point2d & p) = 0;
    /// @brief 设置规划路径的终点的朝向。
    /// @param yaw x轴为0，右手坐标系；单位为弧度；范围为[-pi, pi]。
    /// @return 该设置有效。规划器无设置终点朝向的功能时，返回false，否则返回true。
    virtual bool setEndPointYaw(const double yaw)
    {
        std::cout << "[" << planner_name_ << "]: \"setEndPointYaw\" is not implemented in this planner!" << std::endl;
        return false;
    }
    /// @brief 获得处理后的地图，即算法内部真正使用的地图，常用于实际观察调参结果
    /// @param map 地图将存入该变量
    /// @return 存入是否成功
    virtual bool getProcessedMap(cv::Mat & map) const = 0;
    /// @brief 通过给定的地图、起点、终点规划出一条从起点到终点的最终路径。
    /// @param path 规划出的路径。该路径是原始地图坐标系下原始路径点。
    /// @param auxiliary_info 辅助信息。该信息是原始地图坐标系下路径规划过程中的各种关键路径点信息。例如扩展的节点，去除冗余点前的点等。
    /// @return 是否规划成功
    virtual bool getPath(std::vector<cv::Point2d> & path, std::vector<std::vector<cv::Point2d>> & auxiliary_info) = 0;
    /// @brief 打印所有的信息，包括规划器参数信息、规划地图信息、规划结果信息，并可以将结果保存到指定路径中。直接调用内部helper_的显示
    /// @param save 是否保存到本地
    /// @param save_dir_path 保存的路径
    virtual void showAllInfo(const bool save = false, const std::string & save_dir_path = "") const = 0;
    /// @brief 获取所有信息的helper，用于记录规划耗时等信息。
    /// @note 该函数返回的是一个指针，存储的数据将会随着规划而刷新数据。
    /// @return 所有信息的helper
    virtual const GlobalPlannerHelper * getAllInfo() const = 0;
    
protected:
    // 规划器名称
    std::string planner_name_ = "GlobalPlannerInterface";
    
    // 地图属性
    int rows_ = 0;          // 地图行数
    int cols_ = 0;          // 地图列数
    int channels_ = 0;      // 地图通道数，暂未使用
    double res_ = 0.0;      // 地图分辨率，用于将栅格地图中的数据直接转换到真实地图
    double ori_x_ = 0.0;    // 地图原点坐标x，用于将栅格地图中的数据直接转换到真实地图
    double ori_y_ = 0.0;    // 地图原点坐标y，用于将栅格地图中的数据直接转换到真实地图

    // 设置参数
    bool init_map_ = false;         // 地图初始化
    bool init_map_info_ = false;    // 真实地图信息初始化，即res_, ori_x_, ori_y_
    bool init_start_point_ = false; // 确定规划起点
    bool init_end_point_ = false;   // 确定规划终点
};
