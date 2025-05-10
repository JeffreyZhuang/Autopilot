#include "modules/navigator/navigator.h"

// TODO: Get EKF center (or maybe just choose random point on earth as center)

// Maybe publish global position and use global for everything
// You can calculate displacement between plane lat/lon and waypoint lat/lon in NED frame

// Yeah switch everything to lat/lon, position control only needs lat/lon

// PX4 uses local position and converts waypoint lat/lon to NED

Navigator::Navigator(HAL* hal, Data_bus* data_bus)
	: Module(hal, data_bus),
	  _local_pos_sub(data_bus->local_position_node),
	  _telem_new_waypoint_sub(data_bus->telem_new_waypoint_node),
	  _waypoint_pub(data_bus->waypoint_node)
{
}

void Navigator::update()
{
	poll_data_bus();

	// Get target waypoint
	const Waypoint& target_wp = _waypoints[_curr_wp_idx];

	// Convert waypoint to local NED coordinates
	double tgt_north, tgt_east;
	lat_lon_to_meters(_local_pos.ref_lat, _local_pos.ref_lon,
					  target_wp.lat, target_wp.lon, &tgt_north, &tgt_east);

	// Check distance to waypoint to determine if waypoint reached
	float dist_to_wp = distance(_local_pos.x, tgt_north, _local_pos.y, tgt_east);
	float acc_rad;
	param_get(NAV_ACC_RAD, &acc_rad);
	if (dist_to_wp < acc_rad &&
		_curr_wp_idx < _telem_new_waypoint.num_waypoints - 1)
	{
		_curr_wp_idx++; // Move to next waypoint

		// Publish waypoint
		const Waypoint& prev_wp = _waypoints[_curr_wp_idx - 1];

		double prev_north, prev_east;
		lat_lon_to_meters(_local_pos.ref_lat, _local_pos.ref_lon,
						  prev_wp.lat, prev_wp.lon, &prev_north, &prev_east);

		_waypoint_pub.publish(
			waypoint_s{
				.previous_north = prev_north,
				.previous_east = prev_east,
				.previous_alt = prev_wp.alt,
				.current_north = tgt_north,
				.current_east = tgt_east,
				.current_alt = target_wp.alt,
				.current_index = _curr_wp_idx,
				.num_waypoints = _telem_new_waypoint.num_waypoints,
				.timestamp = _hal->get_time_us()
			}
		);
	}
}

void Navigator::poll_data_bus()
{
	if (_telem_new_waypoint_sub.check_new())
	{
		_telem_new_waypoint = _telem_new_waypoint_sub.get();

		_waypoints[_telem_new_waypoint.index] = Waypoint{
			_telem_new_waypoint.lat,
			_telem_new_waypoint.lon,
			_telem_new_waypoint.alt
		};
	}

	_local_pos = _local_pos_sub.get();
}
