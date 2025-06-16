#include "modules/navigator/navigator.h"

Navigator::Navigator(HAL* hal, DataBus* data_bus)
	: Module(hal, data_bus),
	  _local_pos_sub(data_bus->local_position_node),
	  _waypoint_pub(data_bus->waypoint_node)
{
}

void Navigator::parameters_update()
{
	param_get(NAV_ACC_RAD, &_acc_rad);
}

void Navigator::update()
{
	parameters_update();

	_local_pos = _local_pos_sub.get();

	// Reset waypoint index if there is a new mission
	if (mission_get_version() != _last_mission_version)
	{
		_last_mission_version = mission_get_version();
		_curr_wp_idx = 0;
		publish_waypoint();
	}

	if (mission_get().mission_type == mission_type_t::MISSION_WAYPOINT)
	{
		update_waypoint();
	}

	printf("Navigator curr_wp: %d\n", _curr_wp_idx);
}

void Navigator::update_waypoint()
{
	// Get target waypoint
	const mission_item_t& target_wp = mission_get().mission_items[_curr_wp_idx];

	// Convert waypoint to local NED coordinates
	double tgt_north, tgt_east;
	lat_lon_to_meters(_local_pos.ref_lat, _local_pos.ref_lon,
					  target_wp.latitude, target_wp.longitude, &tgt_north, &tgt_east);

	float dist_to_wp = distance(_local_pos.x, _local_pos.y, tgt_north, tgt_east);

	// Check distance to waypoint to determine if waypoint reached
	printf("Local pos and tgt: %f,%f %f,%f\n", _local_pos.x, _local_pos.y, tgt_north, tgt_east);
	printf("Navigator dist: %f\n", dist_to_wp);
	if ((dist_to_wp < _acc_rad) && (_curr_wp_idx < mission_get().num_items - 1))
	{
		_curr_wp_idx++; // Move to next waypoint
		publish_waypoint();
	}
}

void Navigator::publish_waypoint()
{
	double tgt_north, tgt_east, prev_north, prev_east;

	const mission_item_t& target_wp = mission_get().mission_items[_curr_wp_idx];
	const mission_item_t& prev_wp = mission_get().mission_items[_curr_wp_idx - 1];

	// Convert waypoint to local NED coordinates
	lat_lon_to_meters(_local_pos.ref_lat, _local_pos.ref_lon,
					  target_wp.latitude, target_wp.longitude, &tgt_north, &tgt_east);

	if (_curr_wp_idx == 0)
	{
		prev_north = 0;
		prev_east = 0;
	}
	else
	{
		lat_lon_to_meters(_local_pos.ref_lat, _local_pos.ref_lon,
						  prev_wp.latitude, prev_wp.longitude, &prev_north, &prev_east);
	}

	_waypoint_pub.publish(
		waypoint_s{
			.previous_north = (float)prev_north,
			.previous_east = (float)prev_east,
			.current_north = (float)tgt_north,
			.current_east = (float)tgt_east,
			.current_index = _curr_wp_idx,
			.num_waypoints = mission_get().num_items,
			.timestamp = _hal->get_time_us()
		}
	);
}
