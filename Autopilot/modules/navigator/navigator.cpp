#include "modules/navigator/navigator.h"

Navigator::Navigator(HAL* hal, Data_bus* data_bus)
	: Module(hal, data_bus),
	  _pos_est_sub(data_bus->pos_est_node),
	  _telem_new_waypoint_sub(data_bus->telem_new_waypoint_node),
	  _waypoint_pub(data_bus->waypoint_node),
	  _home_pos_pub(data_bus->home_position_node)
{
}

void Navigator::update()
{
	poll_data_bus();

	// Get target waypoint
	const Waypoint& target_wp = _waypoints[_curr_wp_idx];

	// Convert waypoint to north east coordinates
	double tgt_north, tgt_east;
	lat_lon_to_meters(_waypoints[0].lat, _waypoints[0].lon,
					  target_wp.lat, target_wp.lon, &tgt_north, &tgt_east);

	// Check distance to waypoint to determine if waypoint reached
	float rel_east = _pos_est_data.pos_e - tgt_east;
	float rel_north = _pos_est_data.pos_n - tgt_north;
	float dist_to_wp = sqrtf(rel_north * rel_north + rel_east * rel_east);
	if (dist_to_wp < NAV_ACC_RAD.get() &&
		_curr_wp_idx < _telem_new_waypoint.num_waypoints - 1)
	{
		_curr_wp_idx++; // Move to next waypoint

		// Publish waypoint
		const Waypoint& prev_wp = _waypoints[_curr_wp_idx - 1];

		double prev_north, prev_east;
		lat_lon_to_meters(_waypoints[0].lat, _waypoints[0].lon,
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

		_home_pos_pub.publish(
			home_position_s{
				.lat = _waypoints[0].lat,
				.lon = _waypoints[0].lon,
				.timestamp = _hal->get_time_us()
			}
		);
	}

	_pos_est_data = _pos_est_sub.get();
}
