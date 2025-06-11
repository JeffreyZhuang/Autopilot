#ifndef LIB_DATA_BUS_DATA_BUS_H_
#define LIB_DATA_BUS_DATA_BUS_H_

#include "publication.h"
#include "subscription.h"
#include "nodes.h"

/**
 * @brief Centralized flight data and settings container
 *
 * This class serves as the main data repository for all flight-related
 * information including sensor data, state estimates, control commands,
 * and system state.
 */
struct DataBus
{
	Node<Modes_data> modes_node;
    Node<IMU_data> imu_node;
    Node<Mag_data> mag_node;
    Node<Baro_data> baro_node;
    Node<GNSS_data> gnss_node;
    Node<OF_data> of_node;
    Node<AHRS_data> ahrs_node;
    Node<local_position_s> local_position_node;
    Node<Power_data> power_node;
   	Node<Ctrl_cmd_data> ctrl_cmd_node;
   	Node<RC_data> rc_node;
   	Node<waypoint_s> waypoint_node;
   	Node<hitl_sensors_s> hitl_sensors_node;
   	Node<HITL_output_data> hitl_output_node;
   	Node<position_control_s> position_control_node;
   	Node<uncalibrated_imu_s> uncalibrated_imu_node;
   	Node<uncalibrated_mag_s> uncalibrated_mag_node;
};

#endif /* LIB_DATA_BUS_DATA_BUS_H_ */
