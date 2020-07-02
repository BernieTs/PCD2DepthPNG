#include <QCoreApplication>
#include <QDebug>
#include <QFileInfo>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/io/png_io.h>

typedef pcl::PointXYZ PointT;
void saveDepthToPNG (const std::string &file_name, const pcl::PointCloud<PointT> &cloud);

int main(int argc, char *argv[])
{
    bool fileExists;
    bool isPCDfile;
    QString filePath;
    do{
        std::cout<<"Enter organized pcd file name(.pcd):";
        std::string fileName;
        std::cin>>fileName;
        filePath = QString::fromStdString(fileName);
        fileExists = (QFileInfo::exists(filePath) && QFileInfo(filePath).isFile())? true: false;   //確認file是否存在
        isPCDfile = (QFileInfo(filePath).suffix() == "pcd")? true :false;   //確認file是否為csv
    }while(!fileExists || !isPCDfile);

    pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
    if(pcl::io::loadPCDFile(filePath.toStdString(), *cloud) == -1)
    {
        PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
        system("PAUSE");
        return -1;
    }
    if(!cloud->isOrganized())
    {
        PCL_ERROR ("cloud is not organized \n");
        system("PAUSE");
        return -1;
    }

    // save depth image
    QString qOutputFileName = QFileInfo(filePath).baseName() + ".png";
    saveDepthToPNG(qOutputFileName.toStdString(), *cloud);

    system("PAUSE");
    return 0;
}


void saveDepthToPNG (const std::string &file_name, const pcl::PointCloud<PointT> &cloud)
{
    pcl::PointCloud<pcl::PointXYZRGBA> depth;
    depth.width    = cloud.width;
    depth.height   = cloud.height;
    depth.is_dense = cloud.is_dense;
    depth.points.resize (depth.width * depth.height);

    //bad point
    const float bad_point = std::numeric_limits<float>::quiet_NaN();
    float max = bad_point, min = bad_point;

    //先找出上下限值
    for (int i = 0; i < (int) cloud.points.size(); i++)
    {
        if(std::isfinite(cloud.points[i].z))
        {
            //qDebug()<<"cloud.points[i].z != bad_point";
            //qDebug()<<"cloud.points[i].z:"<<cloud.points[i].z;
            if(!std::isfinite(max) && !std::isfinite(min))  //第一次找到finite的z值，就將值初始化給max和min
            {
                max = cloud.points[i].z;
                min = cloud.points[i].z;
                //qDebug()<<"max == bad_point && min == bad_point";
            }
            else
            {
                if(cloud.points[i].z > max) max = cloud.points[i].z;
                if(cloud.points[i].z < min) min = cloud.points[i].z;
            }
        }
    }

    qDebug()<<"max:"<<max;
    qDebug()<<"min:"<<min;

    float scale = (max - min) / 256;
    qDebug()<<"scale:"<<scale;

    for (int i = 0; i < (int) cloud.points.size(); i++)
    {
        float t = (cloud.points[i].z - min) / scale;  //將z值做shift

        // make sure pixel value between 0 ~ 255
        t = (t > 255) ? 255 : (t < 0 || !std::isfinite(t)) ? 0 : t;

        //depth.points[i].z = t;
        depth.points[i].x = cloud.points[i].x;
        depth.points[i].y = cloud.points[i].y;
        depth.points[i].z = cloud.points[i].z;
        depth.points[i].rgba = (((int)t << 16) | ((int)t << 8) | (int)t);
    }

    pcl::io::savePNGFile(file_name, depth);

    // Save cloud data
    //pcl::io::savePCDFileASCII ("depth.pcd", depth);

    std::cout<<"Saved cloud depth data to "<<file_name<<std::endl;
}
