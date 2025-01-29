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

void Autopilot::init()
{
	_hal->init();
	_ahrs.setup();

	_hal->set_main_task(Autopilot::static_main_task);
	_hal->set_background_task(Autopilot::static_logger_task);
}

void Autopilot::main_task()
{
	_plane->time = _hal->get_time_us();

	_hal->read_sensors();

	_ahrs.update();
	_navigation.execute();

	_guidance.update();

	if (_plane->rc_switch)
	{
		evaluate_auto_mode();
	}
	else
	{
		evaluate_manual_mode();
	}

	_hal->write_storage_buffer();

	_telem.transmit();

	_plane->loop_execution_time = _hal->get_time_us() - _plane->time;
	_plane->loop_iteration++;
}

void Autopilot::logger_task()
{
	_hal->flush_storage_buffer();
}

void Autopilot::evaluate_auto_mode()
{
	switch (_plane->autoMode)
	{
	case AutoMode::TAKEOFF:
		_control.update_takeoff();
		break;
	case AutoMode::MISSION:
		_control.update_mission();
		break;
	case AutoMode::LAND:
		_control.update_land();
		break;
	}
}

void Autopilot::evaluate_manual_mode()
{
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
