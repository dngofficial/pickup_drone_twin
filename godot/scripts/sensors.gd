extends Node

const G = preload("res://scripts/geometry.gd")
const SCAN_MASK := G.TERRAIN | G.SITE | G.TARGET
const SENSOR_UP := Vector3(0,.18,0)
const SENSOR_DOWN := Vector3(0,-.18,0)
const TAG_FLU := Basis(Vector3.FORWARD, Vector3.LEFT, Vector3.UP)
var drone: CharacterBody3D
var bridge: Node
var camera: Node3D
var settings: Dictionary
var tags: Array[Node3D] = []
var rng := RandomNumberGenerator.new()
var safety_scan := PackedFloat32Array()
var ray_directions := PackedVector3Array()
var range_down := 20.0
var range_up := 20.0
var gyro_z := 0.0
var flow_integral := Vector3.ZERO
var elapsed := 0.0
var last_scan_yaw := 0.0
var prev_yaw := 0.0
var initial_z := 0.65
var next_seq := 1
var pending_seq := 0
var pending_ms := 0
var stalled := false
var last_tag_observations: Array[Dictionary] = []
var last_pressure_pa := 101325.0

func setup(source: CharacterBody3D, link: Node, camera_stream: Node3D, config: Dictionary) -> void:
	drone = source
	bridge = link
	camera = camera_stream
	settings = config
	var azimuth_count := int(settings.lidar_azimuth_samples)
	for degrees in settings.lidar_elevations_deg:
		var e := deg_to_rad(float(degrees))
		for i in range(azimuth_count):
			var a := TAU * i / azimuth_count
			ray_directions.append(Vector3(-sin(a)*cos(e), sin(e), -cos(a)*cos(e)))

func reset(seed_value: int, visible_tags: Array[Node3D]) -> void:
	tags = visible_tags
	rng.seed = seed_value ^ 0x5271
	flow_integral = Vector3.ZERO
	elapsed = 0.0
	last_scan_yaw = drone.rotation.y
	prev_yaw = drone.rotation.y
	initial_z = drone.position.y
	next_seq = 1
	pending_seq = 0
	stalled = false
	last_tag_observations.clear()

func header(stamp: Dictionary, frame: String) -> Dictionary:
	return {"stamp": stamp, "frame_id": frame}

func ray(origin: Vector3, end: Vector3, mask := SCAN_MASK) -> Dictionary:
	var query := PhysicsRayQueryParameters3D.create(origin, end, mask)
	query.exclude = [drone.get_rid()]
	return drone.get_world_3d().direct_space_state.intersect_ray(query)

func measure(origin: Vector3, direction: Vector3, extent: float) -> float:
	var hit := ray(origin, origin + direction * extent)
	return origin.distance_to(hit.position) if not hit.is_empty() else extent

func on_status(message: Dictionary, epoch: int) -> void:
	var data: Variant = JSON.parse_string(str(message.get("data", "")))
	if data is Dictionary and int(data.get("epoch", -1)) == epoch and int(data.get("ack_seq", -1)) == pending_seq:
		pending_seq = 0
		stalled = false

func reset_link() -> void:
	pending_seq = 0
	stalled = false
	clear_integrators()

func tick(tick_index: int, dt: float, stamp: Dictionary, epoch: int) -> void:
	flow_integral += drone.global_basis.inverse() * drone.velocity * dt
	elapsed += dt
	gyro_z = wrapf(drone.rotation.y - prev_yaw, -PI, PI) / dt
	prev_yaw = drone.rotation.y
	var pos := drone.global_position
	var hz := roundi(1.0 / dt)
	if tick_index % maxi(1, roundi(hz / 20.0)) == 0:
		range_down = measure(pos + SENSOR_DOWN, Vector3.DOWN, 20.0)
		range_up = measure(pos + SENSOR_UP, Vector3.UP, 20.0)
		var relative_height := pos.y - initial_z + rng.randfn(0, float(settings.barometer_height_noise_m))
		last_pressure_pa = 101325.0 * pow(1.0 - relative_height / 44330.0, 5.255)
		publish_range("down", range_down, stamp)
		publish_range("up", range_up, stamp)
		bridge.publish("/drone/barometer", {"header": header(stamp, "base_link"), "fluid_pressure": last_pressure_pa, "variance": .13})
	var camera_stride := maxi(1, roundi(hz / float(settings.camera_hz)))
	var depth_stride := maxi(1, roundi(hz / float(settings.depth_hz)))
	if tick_index % camera_stride == 0:
		camera.request_frame(stamp, tick_index % depth_stride == 0)
	if tick_index % maxi(1, roundi(hz / float(settings.fiducial_hz))) == 0:
		last_tag_observations = observe_tags()
		bridge.publish("/localization/tag_observations", {"header": header(stamp, "camera_optical_frame"), "observations": last_tag_observations})
	var angular := G.point(Vector3(0,0,gyro_z + rng.randfn(0,float(settings.gyro_noise_radps))))
	bridge.publish("/drone/imu", {"header": header(stamp, "base_link"), "orientation": G.quaternion(Quaternion.IDENTITY),
		"orientation_covariance": [-1,0,0,0,0,0,0,0,0], "angular_velocity": angular,
		"angular_velocity_covariance": [0,0,0,0,0,0,0,0,.000001],
		"linear_acceleration": G.point(Vector3.ZERO), "linear_acceleration_covariance": [-1,0,0,0,0,0,0,0,0]})
	var scan_stride := maxi(1, roundi(hz / float(settings.lidar_hz)))
	if tick_index % scan_stride != 0:
		return
	publish_safety_scan(stamp)
	if pending_seq == 0:
		publish_lidar(stamp, epoch)
	elif Time.get_ticks_msec() - pending_ms > 1000:
		stalled = true

func publish_range(suffix: String, distance: float, stamp: Dictionary) -> void:
	bridge.publish("/drone/range/" + suffix, {"header": header(stamp, "range_" + suffix + "_link"),
		"radiation_type": 1, "field_of_view": .02, "min_range": .03, "max_range": 20.0, "range": distance})

func publish_safety_scan(stamp: Dictionary) -> void:
	safety_scan.clear()
	var origin := drone.global_position + SENSOR_UP
	for i in range(360):
		var a := -PI + TAU * i / 360.0
		var direction := drone.global_basis * Vector3(-sin(a),0,-cos(a))
		safety_scan.append(measure(origin, direction, 20.0))
	bridge.publish("/drone/scan", {"header": header(stamp,"lidar_link"), "angle_min": -PI,
		"angle_max": PI - TAU/360., "angle_increment": TAU/360., "time_increment": 0., "scan_time": .1,
		"range_min": .05, "range_max": 20., "ranges": Array(safety_scan), "intensities": []})

func publish_lidar(stamp: Dictionary, epoch: int) -> void:
	if elapsed <= 0.0:
		return
	var points: Array = []
	var origin := drone.global_position + SENSOR_UP
	var extent := float(settings.lidar_range_m)
	for local: Vector3 in ray_directions:
		var hit := ray(origin, origin + drone.global_basis * local * extent)
		if not hit.is_empty():
			var d := origin.distance_to(hit.position) + rng.randfn(0,float(settings.lidar_noise_m))
			points.append(G.as_array(G.to_ros(local * d + SENSOR_UP)).map(func(v: float) -> float: return snappedf(v,.001)))
	var flow := G.to_ros(flow_integral / elapsed)
	var mean_gyro := wrapf(drone.rotation.y - last_scan_yaw, -PI, PI) / elapsed
	var packet := {"version": 2, "epoch": epoch, "seq": next_seq, "points": points, "stamp": stamp,
		"frame_id": "base_link", "dt": elapsed, "gyro_z": mean_gyro + rng.randfn(0,float(settings.gyro_noise_radps)),
		"flow_xy": [flow.x + rng.randfn(0,float(settings.flow_noise_mps)), flow.y + rng.randfn(0,float(settings.flow_noise_mps))],
		"height": null, "baro_height": 44330.0 * (1.0 - pow(last_pressure_pa / 101325.0, 1.0/5.255)), "clearance_down": range_down}
	if bridge.publish("/drone/lidar/packet", {"data": JSON.stringify(packet)}):
		pending_seq = next_seq
		next_seq += 1
		pending_ms = Time.get_ticks_msec()
		clear_integrators()

func clear_integrators() -> void:
	elapsed = 0.0
	flow_integral = Vector3.ZERO
	last_scan_yaw = drone.rotation.y

func observe_tags() -> Array[Dictionary]:
	var seen: Array[Dictionary] = []
	var optical: Transform3D = camera.optical_pose * Transform3D(Basis(Vector3.RIGHT, PI), Vector3.ZERO)
	var inverse := optical.affine_inverse()
	var info: Dictionary = camera.camera_info({})
	var fx := float(info.k[0])
	var fy := float(info.k[4])
	for tag in tags:
		var offset := optical.origin - tag.global_position
		var distance := offset.length()
		if distance > float(settings.fiducial_max_range_m) or distance < .1:
			continue
		var facing := tag.global_basis.z.dot(offset / distance)
		var p := inverse * tag.global_position
		if p.z <= .1 or facing < .40:
			continue
		var margin: float = fx * float(tag.get_meta("width")) / p.z * .5
		if absf(p.x * fx / p.z) + margin > float(info.width)*.5 or absf(p.y * fy / p.z) + margin > float(info.height)*.5 or margin < 4.0:
			continue
		if not ray(optical.origin, tag.global_position + tag.global_basis.z * .014).is_empty() or rng.randf() < float(settings.fiducial_miss_rate):
			continue
		var pos_noise := float(settings.fiducial_position_noise_m)
		var yaw_noise := deg_to_rad(float(settings.fiducial_yaw_noise_deg))
		p += Vector3(rng.randfn(0,pos_noise),rng.randfn(0,pos_noise),rng.randfn(0,pos_noise))
		var rotation := inverse.basis * Basis(Vector3.UP, rng.randfn(0,yaw_noise)) * tag.global_basis * TAG_FLU
		var cov: Array = []
		cov.resize(36)
		cov.fill(0.0)
		for i in range(6):
			cov[7*i] = pos_noise*pos_noise if i < 3 else yaw_noise*yaw_noise
		seen.append({"tag_id": tag.get_meta("tag_id"), "confidence": clampf(facing,0,1),
			"pose": {"pose": {"position": G.point(p), "orientation": G.quaternion(rotation.get_rotation_quaternion())}, "covariance": cov}})
	return seen
