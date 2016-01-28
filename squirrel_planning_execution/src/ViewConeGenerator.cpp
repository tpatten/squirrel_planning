#include <squirrel_planning_execution/ViewConeGenerator.h>
#include <occupancy_grid_utils/ray_tracer.h>
#include <occupancy_grid_utils/coordinate_conversions.h>
//#include <tf/Quaternion.h>
//#include <tf/Vector3.h>

#include <stdlib.h>
#include <time.h> 

namespace KCL_rosplan {

ViewConeGenerator::ViewConeGenerator(ros::NodeHandle& node_handle, const std::string& topic_name)
	: has_received_occupancy_grid_(false)
{
	navigation_grid_sub_ = node_handle.subscribe("/map", 1, &ViewConeGenerator::storeNavigationGrid, this);
	srand(time(NULL));
}

void ViewConeGenerator::storeNavigationGrid(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
	last_received_occupancy_grid_msgs_ = *msg;
	has_received_occupancy_grid_ = true;
}

std::vector<geometry_msgs::Pose> ViewConeGenerator::createViewCones(unsigned int max_view_cones, int occupancy_threshold, float fov, float view_distance, unsigned int sample_size)
{
	std::vector<geometry_msgs::Pose> result;
	if (!has_received_occupancy_grid_) {
		ROS_WARN("(ViewConeGenerator) The occupancy grid was not published yet, no poses returned.");
		return result;
	}
	
	// Initialise the processed cells list.
	std::vector<bool> processed_cells(last_received_occupancy_grid_msgs_.info.width * last_received_occupancy_grid_msgs_.info.height, false);
	for (int y = 0; y < last_received_occupancy_grid_msgs_.info.width; ++y) {
		for (int x = 0; x < last_received_occupancy_grid_msgs_.info.height; ++x) {
			if (last_received_occupancy_grid_msgs_.data[x + y * last_received_occupancy_grid_msgs_.info.width] > occupancy_threshold) {
				processed_cells[x + y * last_received_occupancy_grid_msgs_.info.width] = true;
			} else {
				processed_cells[x + y * last_received_occupancy_grid_msgs_.info.width] = false;
			}
		}
	}
	
	for (unsigned int i = 0; i < max_view_cones; ++i) {
		
		// First we generate a bunch of random view cones and rate them.
		geometry_msgs::Pose best_pose;
		std::vector<occupancy_grid_utils::Cell> best_visible_cells;
		for (unsigned int j = 0; j < sample_size; ++j) {
			int grid_x = ((float)rand() / (float)RAND_MAX) * last_received_occupancy_grid_msgs_.info.height;
			int grid_y = ((float)rand() / (float)RAND_MAX) * last_received_occupancy_grid_msgs_.info.width;
			
			occupancy_grid_utils::Cell c(grid_x, grid_y);
			geometry_msgs::Point p = occupancy_grid_utils::cellCenter(last_received_occupancy_grid_msgs_.info, c);
			float yaw = ((float)rand() / (float)RAND_MAX) * 360.0f;
			
			geometry_msgs::Pose pose;
			pose.position = p;
			
			tf::Quaternion q;
			q.setEuler(0, 0, yaw);

			pose.orientation.x = q.getX();
			pose.orientation.y = q.getY();
			pose.orientation.z = q.getZ();
			pose.orientation.w = q.getW();
			
			// Calculate the triangle points encompasses the area that is viewed.
			tf::Vector3 view_point(p.x, p.y, p.z);
			
			tf::Vector3 v0(view_distance, 0.0f, 0.0f);
			v0.rotate(tf::Vector3(0.0f, 0.0f, 1.0f), yaw);
			tf::Vector3 v0_normalised = v0.normalized();
			
			tf::Vector3 v1 = v0;
			v1.rotate(tf::Vector3(0.0f, 0.0f, 1.0f), fov / 2.0f);
			v1.normalize();
			
			float length = v0.length() / v0_normalised.dot(v1);
			v1 *= length;
			v1 += view_point;
			
			tf::Vector3 v2 = v0;
			v2.rotate(tf::Vector3(0.0f, 0.0f, 1.0f), -fov / 2.0f);
			v2.normalize();
			
			length = v0.length() / v0_normalised.dot(v2);
			v2 *= length;
			v2 += view_point;
			
			// The triangle now is view_point, v1, v2, we use a flood algorithm to determine which cells
			// are inside the viewing cone.
			//occupancy_grid_utils::Cell cell = occupancy_grid_utils::pointCell(last_received_occupancy_grid_msgs_.info, );
			
			std::vector<occupancy_grid_utils::Cell> open_list;
			open_list.push_back(c);
			
			std::vector<occupancy_grid_utils::Cell> complete_list;
			
			while (open_list.size() > 0) {
				
				occupancy_grid_utils::Cell cell = open_list[0];
				open_list.erase(open_list.begin());
				
				// Check if the cell is inside the triangle.
				geometry_msgs::Point cell_centre_point = occupancy_grid_utils::cellCenter(last_received_occupancy_grid_msgs_.info, cell);
				tf::Vector3 cell_point(cell_centre_point.x, cell_centre_point.y, cell_centre_point.z);
				
				tf::Vector3 cross_v1 = (cell_point - v1).cross(v2 - v1);
				tf::Vector3 cross_v2 = (cell_point - v2).cross(v2 - view_point);
				
				// If both cross produces have the same sign we are good.
				if (cross_v1.z() * cross_v2.z() < 0 && cell.x != c.x && cell.y != c.y) {
					continue;
				}
				
				// Make sure this cell was not already added.
				if (std::find(complete_list.begin(), complete_list.end(), cell) != complete_list.end()) {
					continue;
				}
				
				complete_list.push_back(cell);
				
				// Add its children to the list.
				occupancy_grid_utils::Cell new_cell;
				if (cell.x > 0) {
					new_cell = cell;
					new_cell.x -= 1;
					open_list.push_back(new_cell);
				}
				
				if (cell.x + 1 < last_received_occupancy_grid_msgs_.info.width) {
					new_cell = cell;
					new_cell.x += 1;
					open_list.push_back(new_cell);
				}
				
				if (cell.y > 0) {
					new_cell = cell;
					new_cell.y -= 1;
					open_list.push_back(new_cell);
				}
				
				if (cell.y + 1 < last_received_occupancy_grid_msgs_.info.height) {
					new_cell = cell;
					new_cell.y += 1;
					open_list.push_back(new_cell);
				}
			}
			
			// Next we determine which of these cell points are visible from 'view_point'.
			std::vector<occupancy_grid_utils::Cell> visible_cells;
			for (std::vector<occupancy_grid_utils::Cell>::const_iterator ci = complete_list.begin(); ci != complete_list.end(); ++ci) {
				
				const occupancy_grid_utils::Cell& cell = *ci;
				// Don't count cells that have already been processed.
				if (processed_cells[cell.x + cell.y * last_received_occupancy_grid_msgs_.info.width]) {
					continue;
				}
				
				geometry_msgs::Point point = occupancy_grid_utils::cellCenter(last_received_occupancy_grid_msgs_.info, *ci);
				
				if (canConnect(point, p, occupancy_threshold)) {
					visible_cells.push_back(cell);
				}
			}
			
			// Check if this is the best view cone thus far.
			if (visible_cells.size() > best_visible_cells.size()) {
				best_pose = pose;
				best_visible_cells = visible_cells;
			}
		}
		
		// Update the state of which cells have been observed.
		for (std::vector<occupancy_grid_utils::Cell>::const_iterator ci = best_visible_cells.begin(); ci != best_visible_cells.end(); ++ci) {
			const occupancy_grid_utils::Cell& cell = *ci;
			processed_cells[cell.x + cell.y * last_received_occupancy_grid_msgs_.info.width] = true;
		}
		result.push_back(best_pose);
	}
	
	return result;
}

bool ViewConeGenerator::canConnect(const geometry_msgs::Point& w1, const geometry_msgs::Point& w2, int occupancy_threshold)
{
	occupancy_grid_utils::RayTraceIterRange ray_range = occupancy_grid_utils::rayTrace(last_received_occupancy_grid_msgs_.info, w1, w2, true, true);
	for (occupancy_grid_utils::RayTraceIterator i = ray_range.first; i != ray_range.second; ++i)
	{
		const occupancy_grid_utils::Cell& cell = *i;

		// Check if this cell is occupied.
		if (cell.x + cell.y * last_received_occupancy_grid_msgs_.info.width < last_received_occupancy_grid_msgs_.data.size() && cell.x + cell.y * last_received_occupancy_grid_msgs_.info.width >= 0 && last_received_occupancy_grid_msgs_.data[cell.x + cell.y * last_received_occupancy_grid_msgs_.info.width] > occupancy_threshold)
		{
			return false;
		}
	}
	
	return true;
}


bool ViewConeGenerator::isBlocked(const geometry_msgs::Point& point, float min_distance) const
{
	for (float x = point.x - (min_distance + last_received_occupancy_grid_msgs_.info.resolution); x < point.x + min_distance + last_received_occupancy_grid_msgs_.info.resolution; x += last_received_occupancy_grid_msgs_.info.resolution)
	{
		for (float y = point.y - (min_distance + last_received_occupancy_grid_msgs_.info.resolution); y < point.y + min_distance + last_received_occupancy_grid_msgs_.info.resolution; y += last_received_occupancy_grid_msgs_.info.resolution)
		{
			if (sqrt(x * x + y * y) > min_distance)
			{
				continue;
			}
			geometry_msgs::Point p;
			p.x = x;
			p.y = y;
			
			occupancy_grid_utils::Cell cell = occupancy_grid_utils::pointCell(last_received_occupancy_grid_msgs_.info, p);
			if (last_received_occupancy_grid_msgs_.data[cell.x + cell.y * last_received_occupancy_grid_msgs_.info.width] > 0)
			{
				return true;
			}
		}
	}
	return false;
}

};
