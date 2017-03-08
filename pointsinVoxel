//steps in finding if points occupy an cell 
//	1. specify bounding box of cell (i.e cell size x,y,z)
//	2. Use the bounding box to split the point cloud into 'n' cells and store these n cells informaion in a tree
//	3. Iterate through each box, checking if they possess N(i of points)>=3. If so, mark them as built. 


#include <iostream>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/common.h>

#include <algorithm>
#include <stdio.h>
#include <pcl/point_types.h>




struct voxel
{
	float x, y, z, delta;
	
	unsigned int numPoints;
	unsigned int firstPoint;
	
};


struct points
{
	double x[8], y[8], z[8];
	float X, Y, Z;
};


//Visualize the damn voxel with the point cloud!!! 
void displaypoints(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &selected, const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &original)//initialize the ptr for receiving the passed cloud
{
	// Visualization of keypoints along with the original cloud
	pcl::visualization::PCLVisualizer viewer("Centroid based method display");
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> keypoints_color_handler(selected, 0, 255, 0);
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color_handler(original, 255, 0, 0);
	viewer.setBackgroundColor(0.0, 0.0, 0.0);
	viewer.addPointCloud(original, cloud_color_handler, "cloud");
	viewer.addPointCloud(selected, keypoints_color_handler, "keypoints");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 7, "keypoints");

	while (!viewer.wasStopped())
	{
		viewer.spinOnce();
	}
}

int main()
{
	
	//read scan point cloud 
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	if (pcl::io::loadPLYFile("exp1.ply", *cloud) == -1)
	{
		std::cout << "Cannot load the point zed file\n";
	}
	std::cout << "total points: " << cloud->points.size() << "\n";
		
	/*bool ans = cloud->is_dense;									// To be performed only if cloud contains NAN points
	cout << "is the cloud dense? " << ans<<"\n";
	//remove NAN points from the cloud
	std::vector<int> indices;
	pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);

	bool ans2 = cloud->is_dense;
	cout << "is the cloud dense after removing NAN? " << ans2;*/
	
	//Extract the min and maximum x,y,z dimensions of the point cloud 
	Eigen::Vector4f modelMin = Eigen::Vector4f::Zero();
	Eigen::Vector4f modelMax = Eigen::Vector4f::Zero();
	pcl::getMinMax3D(*cloud,modelMin,modelMax);

	cout << modelMax[0]<<" "<<modelMax[1]<<" "<<modelMax[2] << "\n";
	float xmin, ymin, zmin=0;
	
	xmin=modelMin[0];
	ymin = modelMin[1];
	zmin = modelMin[2];

	points bin[10] = { 0 };				//define object to store the array of voxels
	
	//Use the above  minimum and maximum dimensions to split the pointcloud into voxels 
	voxel vs;
	vs.delta =0.05;
	
	int n = (modelMax[0]-modelMin[0]) / 0.07;	//define the iteration number for "for loop"
	
	int N = 0;	//Initializing the Number of voxels
		
	for (size_t i = 1; i <= 1;i++)
	{
		bin[i].x[i] = xmin;
		bin[i].y[i] = ymin;
		bin[i].z[i] = zmin;
		bin[i].x[i+1] = xmin + vs.delta;	//point no 2 coordinates (increase along x)
		bin[i].y[i+1] = ymin;
		bin[i].z[i+1] = zmin;
		bin[i].x[i+2] = xmin + vs.delta;	//point no 3 coordinates (increase along x,y)
		bin[i].y[i+2] = ymin + vs.delta;
		bin[i].z[i+2] = zmin;
		bin[i].x[i+3] = xmin;				//point no 4 coordinates (increase along y)
		bin[i].y[i+3] = ymin + vs.delta;
		bin[i].z[i+3] = zmin;
		bin[i].x[i+4] = xmin;				//point no 5 coordinates (increase along z)
		bin[i].y[i+4] = ymin;
		bin[i].z[i+4] = zmin+vs.delta ;
		bin[i].x[i+5] = xmin + vs.delta;	//point no. 6 coordinates (increase along x,z)
		bin[i].y[i+5] = ymin;
		bin[i].z[i+5] = zmin + vs.delta;
		bin[i].x[i+6] = xmin + vs.delta;	//point no.7 coordinates (increase along x,y,z)
		bin[i].y[i+6] = ymin + vs.delta;
		bin[i].z[i+6] = zmin + vs.delta;
		bin[i].x[i+7] = xmin;				//point no.8 coordinates (increase along y,z)
		bin[i].y[i+7] = ymin + vs.delta;
		bin[i].z[i+7] = zmin+vs.delta;

		//Update the xmin, ymin, zmin values 
		xmin = bin[i].x[i+1];
		ymin = bin[i].y[i+1];
		zmin = bin[i].z[i+1];
		N =N+ 1;		//Update counter 
	}


	std::cout << "total i of cubes= " << N << "\n";

	for (size_t i = 1; i <= 1; ++i)
	{
		cout << "coordinates of cube " << i << "\n";
		for (size_t j = 1; j <= 8; j++)
		{
			cout << bin[i].x[j] << "," << bin[i].y[j] << "," << bin[i].z[j] << "\n";
		}
			
	}

	//check which is the greater number
	float big_x = 0, small_x = 0;
	big_x = small_x = bin[1].x[1];

	for (int i = 1; i <= 8; i++)
	{
		if (bin[1].x[i] > big_x)
		{
			big_x = bin[1].x[i];
		}
		if (bin[1].x[i] < small_x)
		{
			small_x = bin[1].x[i];
		}
	}

	
	float big_y = 0, small_y = 0;
	big_y = small_y = bin[1].y[1];

	for (int i = 1; i <= 8; i++)
	{
		if (bin[1].y[i] > big_y)
		{
			big_y = bin[1].y[i];
		}
		if (bin[1].y[i] < small_y)
		{
			small_y = bin[1].y[i];
		}
	}

	
	float big_z = 0, small_z = 0;
	big_z = small_z = bin[1].z[1];

	for (int i = 1; i <= 8; i++)
	{
		if (bin[1].z[i] > big_z)
		{
			big_z = bin[1].z[i];
		}
		if (bin[1].z[i] < small_z)
		{
			small_z = bin[1].z[i];
		}
	}

	cout << "biggest: " << big_x << " " << "smallest: " << small_x << "\n";
	cout << "biggest: " << big_y << " " << "smallest: " << small_y << "\n";
	cout << "biggest: " << big_z << " " << "smallest: " << small_z << "\n";

	//Lets find all the points that lie in the range between small and big for x,y,z 
	//take bin1 as example
	points inside_pts[100] = { 0 };			//define object belonging to points structure. Each object holds coordinates of points found to be inside the voxel
	
	pcl::PointCloud<pcl::PointXYZ>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZ>);

	int j = 0;

	int iterations = cloud->width*cloud->height;

	for (size_t i = 0; i <= iterations-1; i++)
	{
		float xval = cloud->points[i].x;
		if ((xval >= small_x) && (xval <= big_x))	
		{
			inside_pts[j].X = xval;
			j = j + 1;
		}
	}

	cout << "the number of points inside bin 1 is: \n";
	for (size_t i = 0; i <= j-1; i++)
	{
		cout << inside_pts[i].X << "\n";
	}
	
	//view the voxel and the points inside it
	displaypoints(inside_pts, cloud);


	return(0);
}

