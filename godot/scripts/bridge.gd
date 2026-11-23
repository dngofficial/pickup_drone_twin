# Adapted from warehouse_drone_twin. Upstream MIT notice is in third_party/licenses.
extends Node

const SOURCES := {
	"/perception/oracle/detections_2d": "vision_msgs/msg/Detection2DArray",
	"/perception/oracle/depth_samples": "std_msgs/msg/String",
	"/ui/request": "std_msgs/msg/String",
	"/sim/control/response": "std_msgs/msg/String",
	"/control/manual": "geometry_msgs/msg/TwistStamped",
	"/clock": "rosgraph_msgs/msg/Clock",
	"/drone/scan": "sensor_msgs/msg/LaserScan",
	"/drone/range/down": "sensor_msgs/msg/Range",
	"/drone/range/up": "sensor_msgs/msg/Range",
	"/drone/imu": "sensor_msgs/msg/Imu",
	"/drone/barometer": "sensor_msgs/msg/FluidPressure",
	"/drone/lidar/packet": "std_msgs/msg/String",
	"/drone/camera/camera_info": "sensor_msgs/msg/CameraInfo",
	"/drone/camera/image/compressed": "sensor_msgs/msg/CompressedImage",
	"/drone/camera/depth/image_raw": "sensor_msgs/msg/Image",
	"/localization/tag_observations": "urc_msgs/msg/TagObservationArray",
	"/drone/gripper/status": "urc_msgs/msg/GripperStatus",
	"/bins/delivery_event": "urc_msgs/msg/DeliveryEvent",
	"/joint_states": "sensor_msgs/msg/JointState",
}
const INPUTS := {
	"/sim/control/request": "std_msgs/msg/String",
	"/ui/response": "std_msgs/msg/String",
	"/ui/telemetry": "std_msgs/msg/String",
	"/slam/status": "std_msgs/msg/String",
	"/drone/cmd_vel": "geometry_msgs/msg/TwistStamped",
	"/drone/estop": "std_msgs/msg/Bool",
	"/drone/gripper/command": "urc_msgs/msg/GripperCommand",
	"/drone/camera/tilt_cmd": "std_msgs/msg/Float64",
}

signal received(topic: String, message: Dictionary)
var socket := WebSocketPeer.new()
var url := ""
var connected := false
var next_retry := 0
var buffer_bytes := 8388608
var status_text := "NOT STARTED"

func _ready() -> void:
	# Read flight commands/SLAM acknowledgments before each simulation tick,
	# including catch-up ticks between slow camera-rendered frames.
	process_physics_priority = -100

func start(address: String, bytes: int) -> void:
	url = address
	buffer_bytes = bytes
	if not url.is_empty():
		print("URC_ROSBRIDGE connecting to " + url)
		connect_socket()

func connect_socket() -> void:
	socket = WebSocketPeer.new()
	socket.inbound_buffer_size = buffer_bytes
	socket.outbound_buffer_size = buffer_bytes
	var result := socket.connect_to_url(url)
	status_text = "CONNECTING TO ROSBRIDGE" if result == OK else "CONNECT ERROR: " + error_string(result)
	next_retry = Time.get_ticks_msec() + 2500

func _physics_process(_delta: float) -> void:
	if url.is_empty():
		return
	socket.poll()
	if socket.get_ready_state() != WebSocketPeer.STATE_OPEN:
		if connected:
			push_warning("ROS bridge disconnected: " + url)
		connected = false
		if socket.get_ready_state() == WebSocketPeer.STATE_CLOSED:
			status_text = "ROSBRIDGE UNREACHABLE - CHECK DOCKER ROS"
		if socket.get_ready_state() == WebSocketPeer.STATE_CLOSED and Time.get_ticks_msec() >= next_retry:
			connect_socket()
		return
	if not connected:
		connected = true
		status_text = "ROSBRIDGE CONNECTED"
		print("URC_ROSBRIDGE connected to " + url)
		for topic: String in SOURCES:
			send({"op": "advertise", "topic": topic, "type": SOURCES[topic]})
		for topic: String in INPUTS:
			send({"op": "subscribe", "topic": topic, "type": INPUTS[topic], "queue_length": 1})
	while socket.get_available_packet_count() > 0:
		var message: Variant = JSON.parse_string(socket.get_packet().get_string_from_utf8())
		if message is Dictionary and message.get("op") == "publish" and INPUTS.has(message.get("topic")) and message.get("msg") is Dictionary:
			received.emit(message.topic, message.msg)

func send(message: Dictionary) -> bool:
	if not connected or socket.get_ready_state() != WebSocketPeer.STATE_OPEN:
		return false
	var payload := JSON.stringify(message).to_utf8_buffer()
	if socket.get_current_outbound_buffered_amount() + payload.size() > buffer_bytes / 4:
		return false
	return socket.send(payload, WebSocketPeer.WRITE_MODE_TEXT) == OK

func publish(topic: String, message: Dictionary) -> bool:
	return send({"op": "publish", "topic": topic, "msg": message})
