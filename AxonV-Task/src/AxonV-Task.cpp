// AxonV-Task.cpp : Defines the entry point for the application.

#include <iostream>
#include <vector>
#include <optional>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/common.h>
#include <pcl/common/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/eigen.h>
#include <pcl/surface/reconstruction.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <Eigen/Dense>
#include <pcl/kdtree/kdtree_flann.h>

using namespace std;


// Irregular quadrilateral, planar conture
pcl::PointXYZ contour_points[4] = {
	pcl::PointXYZ(-0.75, 0.25, 0),
	pcl::PointXYZ(0.25, 0.25, 0),
	pcl::PointXYZ(0.75, 0.75, 0),
	pcl::PointXYZ(0.25, 0.75, 0),
};
const float pathWidth = 0.025;
const string fileName = "input.pcd";
const float raisedResultZ = 0.10;

// Function to project a 3D point onto a 2D plane // https://stackoverflow.com/questions/9605556/how-to-project-a-point-onto-a-plane-in-3d
Eigen::Vector2f projectToPlane(const pcl::PointXYZ& point, const Eigen::Vector3f& planeNormal, const Eigen::Vector3f& planePoint, const Eigen::Vector3f& base1, const Eigen::Vector3f& base2) {
	Eigen::Vector3f p = point.getVector3fMap();
	Eigen::Vector3f projection = p - (p - planePoint).dot(planeNormal) * planeNormal;	
	return Eigen::Vector2f(projection.dot(base1), projection.dot(base2));
}

// Ray Casting algorithm to check if a point is inside a polygon // https://www.youtube.com/watch?v=01E0RGb2Wzo
bool pointIsInsideContour(const Eigen::Vector2f& point, const std::vector<Eigen::Vector2f>& polygon) {
	bool inside = false;

	const float x = point.x();
	const float y = point.y();

	for (int i = 0, j = polygon.size() - 1; i < polygon.size() - 1; j = i++) {
		Eigen::Vector2f a = polygon[j];
		Eigen::Vector2f b = polygon[i];

		float xa = a.x();
		float xb = b.x();
		float ya = a.y();
		float yb = b.y();

		if (
			y < ya != y < yb &&
			x < (xb - xa) * (y - ya) / (yb - ya) + xa) {
			inside = !inside;
		}
	}

	return inside;
}

std::optional<float> calculateAverageHeight(float x, float y, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, float width) {
	float sumHeight = 0.0;
	int count = 0;

	// Define the bounds of the square area centered at (x, y)
	float minX = x - width / 2.0;
	float maxX = x + width / 2.0;
	float minY = y - width / 2.0;
	float maxY = y + width / 2.0;

	for (const auto& point : cloud->points) {
		// Check if the point is within the specified bounds
		if (point.x >= minX && point.x <= maxX && point.y >= minY && point.y <= maxY) {
			sumHeight += point.z;  
			count++;  
		}
	}
	if (count == 0) {
		return std::nullopt;
	}
	return sumHeight / static_cast<float>(count);
}

int main()
{
	// Load input point cloud.
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::io::loadPCDFile(fileName, *cloud);

	//// Plane mapping ////
	// Calculate the normal of the conture plane
	Eigen::Vector3f v1 = contour_points[1].getVector3fMap() - contour_points[0].getVector3fMap();
	Eigen::Vector3f v2 = contour_points[2].getVector3fMap() - contour_points[0].getVector3fMap();
	Eigen::Vector3f planeNormal = v1.cross(v2).normalized();

	// Choose a point on the plane (any vertex will do)
	Eigen::Vector3f planePoint = contour_points[0].getVector3fMap();

	// Define two orthogonal vectors on the plane
	Eigen::Vector3f base1 = v1.normalized();
	Eigen::Vector3f base2 = planeNormal.cross(base1);

	// Project contour vertices onto the 2D plane
	std::vector<Eigen::Vector2f> projectedVertices;
	for (const auto& contourVertex : contour_points) {
		projectedVertices.push_back(projectToPlane(contourVertex, planeNormal, planePoint, base1, base2));
	}

	// Create a new point cloud to store the contour points
	pcl::PointCloud<pcl::PointXYZ>::Ptr contourCloud(new pcl::PointCloud<pcl::PointXYZ>);

	// Check each point in the cloud
	for (const auto& point : cloud->points) {
		Eigen::Vector2f projectedPoint = projectToPlane(point, planeNormal, planePoint, base1, base2);
		if (pointIsInsideContour(projectedPoint, projectedVertices)) {
			contourCloud->points.push_back(point);
		}
	}

	//// Extrapolation and Coverage Path Planning ////
	// Determine bounding box
	Eigen::Vector4f minPt, maxPt;
	pcl::getMinMax3D(*cloud, minPt, maxPt);

	float currentX = minPt.x();
	float currentY = minPt.y();
	bool moveRight = true;

	std::vector<pcl::PointXYZ> path;

	while (currentY <= maxPt.y()) {
		while ((moveRight && currentX <= maxPt.x()) || (!moveRight && currentX >= minPt.x())) {
			std::optional<float> z = calculateAverageHeight(currentX, currentY, contourCloud, pathWidth);
			if (z.has_value()) {
				path.push_back(pcl::PointXYZ(currentX, currentY, z.value()));
			}
			currentX += (moveRight ? 1 : -1) * pathWidth;
		}
		moveRight = !moveRight;
		currentY += pathWidth;  // Move up to the next row
	}

	//// Visualization ////
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr outputCloud(new pcl::PointCloud<pcl::PointXYZRGB>());
	for (const auto& point : cloud->points) {
		pcl::PointXYZRGB pointRgb;
		pointRgb.x = point.x;
		pointRgb.y = point.y;
		pointRgb.z = point.z;
		pointRgb.r = 0;
		pointRgb.g = 0;
		pointRgb.b = 255;
		outputCloud->points.push_back(pointRgb);
	}
	for (const auto& point : contourCloud->points) {
		pcl::PointXYZRGB pointRgb;
		pointRgb.x = point.x;
		pointRgb.y = point.y;
		pointRgb.z = point.z + raisedResultZ;
		pointRgb.r = 255;
		pointRgb.g = 0;
		pointRgb.b = 0;
		outputCloud->points.push_back(pointRgb);
	}

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pathCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	for (const auto& point : path) {
		pcl::PointXYZRGB pointRgb;
		pointRgb.x = point.x;
		pointRgb.y = point.y;
		pointRgb.z = point.z + raisedResultZ * 2;
		pointRgb.r = 0;
		pointRgb.g = 255;
		pointRgb.b = 0;
		pathCloud->points.push_back(pointRgb);
	}

	// Merged output of initial and contoured
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Result Viewer"));
	viewer->addPointCloud<pcl::PointXYZRGB>(outputCloud, "outputCloud");

	// Result path
	viewer->addPolygon<pcl::PointXYZRGB>(pathCloud, 0.0, 1.0, 0.0, "pathPolygon");

	// Contour bounding box
	for (int i = 0; i < 4; i++) {
		viewer->addSphere(pcl::PointXYZRGB(contour_points[i].x, contour_points[i].y, contour_points[i].z), 0.02, 0.8, 0.6, 0.2, "sphere" + i);
	}

	// Render loop
	while (!viewer->wasStopped()) {
		viewer->spinOnce(100, true);
	}

	return 0;
}