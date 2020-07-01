#include <QCoreApplication>
#include <QDebug>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/io/png_io.h>

typedef pcl::PointXYZ PointT;
void saveDepthToPNG (const std::string &file_name, const pcl::PointCloud<PointT> &cloud);

int main(int argc, char *argv[])
{
    std::string pcd_file("test_pcd_20200701.pcd");

    pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
    pcl::io::loadPCDFile(pcd_file, *cloud);

    // save depth image
    std::string depth_file("test_pcd_20200701.png");
    saveDepthToPNG(depth_file, *cloud);

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

    double max = 0, min = 0;

    for (int i = 0; i < (int) cloud.points.size(); i++) {
        if(cloud.points[i].z > max) max = cloud.points[i].z;
        if(cloud.points[i].z < min) min = cloud.points[i].z;
    }

    qDebug()<<"max:"<<max;
    qDebug()<<"min:"<<min;

    float scale = (max - min) / 256;
    qDebug()<<"scale:"<<scale;

    for (int i = 0; i < (int) cloud.points.size(); i++) {
        float t = cloud.points[i].z / scale;
        if(i == 6881)
        {
            qDebug()<<"t at 6881(1):"<<t;
            qDebug()<<"t:"<<cloud.points[i].z / scale;
        }

        // make sure pixel value between 0 ~ 255
        t = (t > 255) ? 255 : (t < 0) ? 0 : t;
        if(i == 6881)
        {
            qDebug()<<"t at 6881(2):"<<t;
        }


        //depth.points[i].z = t;
        depth.points[i].x = cloud.points[i].x;
        depth.points[i].y = cloud.points[i].y;
        depth.points[i].z = cloud.points[i].z;
        depth.points[i].rgba = (((int)t << 16) | ((int)t << 8) | (int)t);


        if(i == 6881)
        {
            qDebug()<<"depth.points[6881].z:"<<depth.points[i].z;
            qDebug()<<"t at 6881(3):"<<t;
            qDebug()<<"depth.points[i].rgba:"<<depth.points[i].rgba;
        }
    }


    pcl::io::savePNGFile(file_name, depth);

    // Save cloud data
    pcl::io::savePCDFileASCII ("depth.pcd", depth);
}
