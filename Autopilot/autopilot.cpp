#include <autopilot.h>

Autopilot* Autopilot::_instance = nullptr;

Autopilot::Autopilot(HAL* hal, Plane* plane): _ahrs(hal, plane, hal->main_dt),
									   	   	  _navigation(hal, plane, hal->main_dt),
											  _control(hal, plane, hal->main_dt),
											  _guidance(hal, plane),
											  _telem(hal, plane)
{
	_hal = hal;
	_plane = plane;
	_instance = this;
}

void Autopilot::setup()
{
	init_state();

	_hal->init();

	_ahrs.setup();
	_guidance.init();

	// Start tasks
	_hal->set_main_task(&Autopilot::static_main_task);
	_hal->set_background_task(&Autopilot::static_logger_task);
}

void Autopilot::main_task()
{
	update_time();
	_hal->read_sensors();

	evaluate_system_mode();

	_telem.update();
	_hal->write_storage_buffer();
}

void Autopilot::logger_task()
{
	_hal->flush_storage_buffer();
}

/**
 * System Modes
 */
void Autopilot::evaluate_system_mode()
{
	_plane->mode_id = static_cast<uint8_t>(_plane->systemMode);

	switch (_plane->systemMode)
	{
	case SystemMode::BOOT:
		boot();
		break;
	case SystemMode::FLIGHT:
		flight();
		break;
	}
}

void Autopilot::boot()
{
	set_ahrs_initial();
	_navigation.execute();

	// Calibrate barometer
	_plane->baro_offset = _plane->baro_alt;

	// Set home position to first GPS fix
	bool gnss_locked = (_plane->gnss_sats > 5) && (fabs(_plane->gnss_lat) > 0) && (fabs(_plane->gnss_lat) > 0);
	if (gnss_locked)
	{
		_plane->gnss_center_lat = _plane->gnss_lat;
		_plane->gnss_center_lon = _plane->gnss_lon;
	}

	bool transmitter_safe = (_plane->rc_throttle < THR_DEADZONE) && (_plane->manual_sw == false);

	if (gnss_locked && transmitter_safe)
	{
		_plane->systemMode = SystemMode::FLIGHT;
	}
}

void Autopilot::flight()
{
	_ahrs.update();
	_navigation.execute();

	if (_plane->manual_sw)
	{
		evaluate_auto_mode();
	}
	else
	{
		evaluate_manual_mode();
	}
}

/**
 * Auto Modes
 */
void Autopilot::evaluate_auto_mode()
{
	_plane->mode_id = static_cast<uint8_t>(_plane->autoMode);

	switch (_plane->autoMode)
	{
	case AutoMode::READY:
		ready();
		break;
	case AutoMode::TAKEOFF:
		takeoff();
		break;
	case AutoMode::MISSION:
		mission();
		break;
	case AutoMode::LAND:
		land();
		break;
	case AutoMode::FLARE:
		flare();
		break;
	case AutoMode::SAFE:
		safe();
		break;
	}
}

void Autopilot::ready()
{
	// BUT THIS RUNS ON STARTUP, NOT ON SWITCH
	// HAVE SINGLE ENUM. STILL THREE INSTANCES, EXCEPT YOU HAVE ONE MASTER INSTANCE THAT CHOOSES WHICH OF THE THREE TO RUN
	// THEN YOU ONLY NEED ONE SWITCH STATEMENT
	_plane->autoMode = AutoMode::TAKEOFF;
	takeoff_time = _plane->time;
}

void Autopilot::takeoff()
{
	if (_plane->time - takeoff_time > LAUN_MOT_DEL)
	{
		_control.update_takeoff();
	}

	if (-_plane->nav_pos_down > TAKEOFF_ALT)
	{
		_plane->autoMode = AutoMode::MISSION;
	}
}

void Autopilot::mission()
{
	_guidance.update_mission();

	_control.update_mission();

	if (_plane->waypoint_index == _plane->num_waypoints)
	{
		// Switch to land
	}
}

void Autopilot::land()
{
	_guidance.update_landing();

	_control.update_land();

	if (-_plane->nav_pos_down < LAND_FLARE_ALT)
	{
		_plane->autoMode = AutoMode::FLARE;
	}
}

void Autopilot::flare()
{

}

void Autopilot::safe()
{

}

/**
 * Manual Modes
 */
void Autopilot::evaluate_manual_mode()
{
	_plane->mode_id = static_cast<uint8_t>(_plane->manualMode);

	switch (_plane->manualMode)
	{
	case ManualMode::MANUAL:
		_control.update_manual();
		break;
	case ManualMode::STABILIZED:
		_control.update_stabilized();
		break;
	}
}

/**
 * Helper functions
 */
void Autopilot::init_state()
{
	_plane->systemMode = SystemMode::BOOT;
	_plane->manualMode = ManualMode::MANUAL;
	_plane->autoMode = AutoMode::READY;
}

void Autopilot::update_time()
{
	uint64_t time = _hal->get_time_us();
	_plane->loop_execution_time = time - _plane->time;
	_plane->time = time;
	_plane->loop_iteration++;
}

void Autopilot::set_ahrs_initial()
{
	// Get heading from mag and roll, pitch from accel, convert to quaternion and put in here
	float q0, q1, q2, q3;
	float ax = -_plane->imu_ax;
	float ay = -_plane->imu_ay;
	float az = -_plane->imu_az;
	float mx = -_plane->compass_mx;
	float my = -_plane->compass_my;
	float mz = -_plane->compass_mz;

	float roll_initial = atan2f(ay, az);
	float pitch_initial = atan2f(-ax, sqrtf(powf(ay, 2) + powf(az, 2)));
	float yaw_initial = 0;
	float norm = sqrtf(powf(mx, 2) + powf(my, 2) + powf(mz, 2));
	if (norm == 0)
	{
		q0 = 1.0f;
		q1 = 0.0f;
		q2 = 0.0f;
		q3 = 0.0f;
	}
	else
	{
		mx /= norm;
		my /= norm;
		mz /= norm;

		mx = mx * cosf(pitch_initial) + mz * sinf(pitch_initial);
		my = mx * sinf(roll_initial) * sin(pitch_initial) + my * cos(roll_initial) - mz * sinf(roll_initial) * cosf(pitch_initial);
		yaw_initial = atan2f(-my, mx);

		float cy = cosf(yaw_initial * 0.5f);
		float sy = sinf(yaw_initial * 0.5f);
		float cp = cosf(pitch_initial * 0.5f);
		float sp = sinf(pitch_initial * 0.5f);
		float cr = cosf(roll_initial * 0.5f);
		float sr = sinf(roll_initial * 0.5f);

		q0 = cr * cp * cy + sr * sp * sy;
		q1 = sr * cp * cy - cr * sp * sy;
		q2 = cr * sp * cy + sr * cp * sy;
		q3 = cr * cp * sy - sr * sp * cy;
	}

	_ahrs.set_state(q0, q1, q2, q3); // Set initial state
	_ahrs.update();
}
