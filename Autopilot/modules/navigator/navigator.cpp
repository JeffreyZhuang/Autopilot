#include "modules/navigator/navigator.h"

// TODO: Get EKF center (or maybe just choose random point on earth as center)

// Maybe publish global position and use global for everything
// You can calculate displacement between plane lat/lon and waypoint lat/lon in NED frame

// Yeah switch everything to lat/lon, position control only needs lat/lon

// PX4 uses local position and converts waypoint lat/lon to NED

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
	if (mission_get_version() > _last_mission_version)
	{
		_last_mission_version = mission_get_version();
		_curr_wp_idx = 0;
	}

	switch (mission_get().mission_type)
	{
	case mission_type_t::MISSION_LOITER:
	case mission_type_t::MISSION_LAND:
		update_loiter_land();
		break;
	case mission_type_t::MISSION_WAYPOINT:
		update_waypoint();
		break;
	}
}

void Navigator::update_loiter_land()
{
	// Get target waypoint
	const mission_item_t& target_wp = mission_get().mission_items[_curr_wp_idx];

	// Convert waypoint to local NED coordinates
	double tgt_north, tgt_east;
	lat_lon_to_meters(_local_pos.ref_lat, _local_pos.ref_lon,
					  target_wp.latitude, target_wp.longitude, &tgt_north, &tgt_east);

	_waypoint_pub.publish(
		waypoint_s{
			.previous_north = 0,
			.previous_east = 0,
			.current_north = (float)tgt_north,
			.current_east = (float)tgt_east,
			.current_index = _curr_wp_idx,
			.num_waypoints = mission_get().num_items,
			.timestamp = _hal->get_time_us()
		}
	);
}

void Navigator::update_waypoint()
{
	// Get target waypoint
	const mission_item_t& target_wp = mission_get().mission_items[_curr_wp_idx];

	// Convert waypoint to local NED coordinates
	double tgt_north, tgt_east;
	lat_lon_to_meters(_local_pos.ref_lat, _local_pos.ref_lon,
					  target_wp.latitude, target_wp.longitude, &tgt_north, &tgt_east);

	float dist_to_wp = distance(_local_pos.x, tgt_north, _local_pos.y, tgt_east);

	// Check distance to waypoint to determine if waypoint reached
	if ((dist_to_wp < _acc_rad) && (_curr_wp_idx < mission_get().num_items - 1))
	{
		_curr_wp_idx++; // Move to next waypoint

		// Publish waypoint
		const mission_item_t& prev_wp = mission_get().mission_items[_curr_wp_idx - 1];

		double prev_north, prev_east;
		lat_lon_to_meters(_local_pos.ref_lat, _local_pos.ref_lon,
						  prev_wp.latitude, prev_wp.longitude, &prev_north, &prev_east);

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
}
