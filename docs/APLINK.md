# Autopilot Link

This is the packet structure used for telemetry, USB, and datalogging

# Packet Serialization

## Packet Format

| Byte Index | Type | Content                           | Value   | Explanation |
|   -    |-   | --------------------------------- | ------- | - |
| 0        | `uint8_t` | Start byte                 | 0xFE | Used to indicate the beginning of a new packet |
| 1          | `uint8_t`| Payload Length                    | 1 - 255 | Indicates length of the payload section |
| 2           |`uint8_t` | Message ID | 1 - 255 | ID of message type in payload |
| For n-byte payload: 3 to (3+n)  | `uint8_t[max 255]`       | Payload   |       |  Message data. Depends on message type. |
| (n+4) to (n+5) | `uint8_t[2]` | CRC-16 Checksum | | |

## Messages

### Telemetry (Message ID: 1)

| Content          | Type     | Unit |
| ---------------- | -------- | ---- |
| Roll             | int16_t  | $10^{-2}$ deg  |
| Pitch            | int16_t  | $10^{-2}$ deg  |
| Heading          | uint16_t | $10^{-2}$ deg  |
| Altitude         | int16_t  | dm   |          
| Airspeed         | uint16_t  | dm/s |
| Altitude Setpoint | int16_t  | dm   |    
| GNSS Latitude         | int32_t    | $10^{-7}$ deg  |       
| GNSS Longitude        | int32_t    | $10^{-7}$ deg  |
| Position Estimate North | float | m |
| Position Estimate East |  float | m |
| Mode ID          | uint8_t  |      |
| Target Waypoint Index | uint8_t |  |         
| Cell Voltage     | uint16_t  | mV    |    
| Battery Current          | uint16_t  | mA    |   
| Capacity Consumed | uint16_t | mAh   | 
| Autopilot Current | uint16_t | mA |
| GPS Sats | uint8_t | |
| GPS Fix | bool | | 
| Aileron | uint8_t | %|
| Elevator | uint8_t | %|
| Throttle | uint8_t | %|

### Waypoint (Message ID: 2)

| Content          | Type     | Unit |
| ---------------- | -------- | ---- |
| Waypoint Index   | uint8_t  |      |
| Total Number of Waypoints   | uint8_t  |      |
| Latitude   | int32_t  |   $10^{-7}$ deg   |
| Longitude | int32_t | $10^{-7}$ deg|
| Altitude | int16_t | dm | 

### Parameters (Message ID: 3)

### List Files (Message ID: 4)

File names is the time since epoch. List all the times since epoch.

### Read File (Message ID: 5)

Input time uint32_t for file

### Delete File (Message ID: 6)

Input time uint32_t for file, delete file given time epoch

### Command (Message ID: )

- List files
- Calibration mode (tell plane to send raw imu and mag data)
- HITL mode
- List parameters