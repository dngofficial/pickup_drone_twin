extends Node3D

const G = preload("res://scripts/geometry.gd")
const Layout = preload("res://scripts/scene_manifest.gd")
const Terrain = preload("res://scripts/terrain.gd")
const Station = preload("res://scripts/station.gd")
const Bins = preload("res://scripts/bins.gd")
const Objects = preload("res://scripts/objects.gd")
const Drone = preload("res://scripts/drone.gd")
const Gripper = preload("res://scripts/gripper.gd")
const CameraStream = preload("res://scripts/camera_stream.gd")
const Sensors = preload("res://scripts/sensors.gd")
const Oracle = preload("res://scripts/oracle.gd")
const Dashboard = preload("res://scripts/dashboard.gd")
const Bridge = preload("res://scripts/bridge.gd")

var settings: Dictionary
var manifest: Dictionary
var seed_value: int
var epoch := 1
var tick_index := 0
var world: Node3D
var terrain: Node3D
var station: Node3D
var bins: Node3D
var targets: Node3D
var drone: CharacterBody3D
var gripper: Marker3D
var camera_stream: Node3D
var sensors: Node
var bridge: Node
var main_camera: Camera3D
var sun: DirectionalLight3D
var lidar_overlay: MeshInstance3D
var lidar_visible := false
var follow := false
var pending_gripper: Dictionary = {}
var request_number := 0
var readout: Label
var state_text: Label
var image_widget: TextureRect
var depth_hint: Label
var gripper_reticle: Label
var manifest_path := ""
var oracle := Oracle.new()
var dashboard: CanvasLayer
var backend_manual := false
var clock_paused := false
var step_remaining := 0

func _ready() -> void:
	settings = Layout.read_defaults()
	seed_value = int(settings.scene.seed)
	var url := OS.get_environment("URC_ROSBRIDGE_URL")
	for arg in OS.get_cmdline_user_args():
		if arg.begins_with("--seed="):
			var value := arg.trim_prefix("--seed=")
			if not value.is_valid_int():
				push_error("Expected integer scene seed")
				get_tree().quit(2)
				return
			seed_value = value.to_int()
		elif arg.begins_with("--bridge="):
			url = arg.trim_prefix("--bridge=")
		elif arg.begins_with("--manifest="):
			manifest_path = arg.trim_prefix("--manifest=")
		elif arg == "--follow":
			follow = true
	if url.is_empty():
		url = str(settings.runtime.bridge_url)
	setup_view()
	bridge = Bridge.new()
	add_child(bridge)
	bridge.received.connect(on_message)
	if not build_world():
		get_tree().quit(2)
		return
	if not manifest_path.is_empty():
		get_tree().quit()
		return
	bridge.start(url, int(settings.runtime.websocket_max_bytes))
	if DisplayServer.get_name() != "headless":
		build_panel()
		dashboard = Dashboard.new()
		add_child(dashboard)
		dashboard.setup(self)
	print("URC_SIM_READY seed=%d objects=%d tags=%d" % [seed_value, targets.bodies.size(), sensors.tags.size()])

func setup_view() -> void:
	Engine.physics_ticks_per_second = int(settings.runtime.physics_hz)
	var environment := Environment.new()
	environment.background_mode = Environment.BG_SKY
	var sky := Sky.new()
	var material := ProceduralSkyMaterial.new()
	material.sky_top_color = Color("52788b")
	material.sky_horizon_color = Color("d8baa0")
	material.ground_horizon_color = Color("d8baa0")
	material.ground_bottom_color = Color("b3916f")
	sky.sky_material = material
	environment.sky = sky
	environment.ambient_light_source = Environment.AMBIENT_SOURCE_COLOR
	environment.ambient_light_color = Color("f0e1ce")
	environment.ambient_light_energy = .40
	var world_environment := WorldEnvironment.new()
	world_environment.environment = environment
	add_child(world_environment)
	sun = DirectionalLight3D.new()
	sun.shadow_enabled = true
	sun.directional_shadow_max_distance = 40
	add_child(sun)
	main_camera = Camera3D.new()
	main_camera.fov = 49.0
	main_camera.far = 150.0
	main_camera.cull_mask = 1 | G.DISPLAY
	add_child(main_camera)
	main_camera.make_current()
	lidar_overlay = MeshInstance3D.new()
	lidar_overlay.layers = G.DISPLAY
	add_child(lidar_overlay)

func build_world() -> bool:
	manifest = Layout.build(seed_value, settings)
	if manifest.is_empty():
		return false
	world = Node3D.new()
	world.name = "Site"
	add_child(world)
	terrain = Terrain.new()
	world.add_child(terrain)
	terrain.build(manifest)
	station = Station.new()
	station.name = "GroundStation"
	world.add_child(station)
	var tags: Array[Node3D] = station.build()
	bins = Bins.new()
	world.add_child(bins)
	bins.build(manifest, settings.scene)
	tags.append_array(bins.tags)
	manifest["tag_poses_map"] = []
	for tag in tags:
		manifest.tag_poses_map.append({"tag_id": tag.get_meta("tag_id"),
			"position": G.point(G.to_ros(tag.global_position)),
			"orientation": G.quaternion(G.ros_rotation(tag.global_basis))})
	targets = Objects.new()
	world.add_child(targets)
	targets.build(manifest, settings.scene)
	drone = Drone.new()
	drone.position = G.to_godot(G.xyz(manifest.initial_base_map_xyz_yaw))
	drone.rotation.y = float(manifest.initial_base_map_xyz_yaw[3])
	drone.speed_limit_mps = float(settings.mission.transit_speed_mps)
	drone.payload_speed_coefficient = float(settings.safety.payload_speed_coefficient_per_kg)
	drone.watchdog_ms = roundi(float(settings.runtime.watchdog_wall_s) * 1000.0)
	world.add_child(drone)
	gripper = Gripper.new()
	gripper.name = "Gripper"
	gripper.position = Vector3(0,-.55,0)
	drone.add_child(gripper)
	gripper.install(drone, targets, settings.mission)
	camera_stream = CameraStream.new()
	world.add_child(camera_stream)
	camera_stream.setup(settings.sensors)
	camera_stream.frame_ready.connect(on_camera_frame)
	sensors = Sensors.new()
	world.add_child(sensors)
	sensors.setup(drone, bridge, camera_stream, settings.sensors)
	sensors.reset(seed_value, tags)
	oracle.rng.seed = seed_value ^ 0x3491
	sun.rotation_degrees = Vector3(float(manifest.sun_pitch_deg), float(manifest.sun_yaw_deg), 0.0)
	sun.light_energy = float(manifest.sun_energy)
	mark_search_area()
	if not manifest_path.is_empty():
		var file := FileAccess.open(manifest_path, FileAccess.WRITE)
		if file == null:
			push_error("Cannot write scene manifest to " + manifest_path)
			return false
		file.store_string(JSON.stringify(manifest, "\t") + "\n")
	return true

func mark_search_area() -> void:
	var shape := ImmediateMesh.new()
	var color := G.material(Color("eccf87"))
	color.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
	var b: Array = manifest.search_bounds_xy
	var corners := [Vector2(b[0],b[2]),Vector2(b[1],b[2]),Vector2(b[1],b[3]),Vector2(b[0],b[3])]
	shape.surface_begin(Mesh.PRIMITIVE_LINES, color)
	for i in range(4):
		var a: Vector2 = corners[i]
		var c: Vector2 = corners[(i+1)%4]
		var steps := int(a.distance_to(c) * 2.0)
		for step in range(steps):
			if step % 2 == 1:
				continue
			for j in [step, step+1]:
				var p := a.lerp(c, float(j) / steps)
				shape.surface_add_vertex(G.to_godot(Vector3(p.x,p.y,Layout.elevation(manifest,p.x,p.y)+.022)))
	shape.surface_end()
	var line := G.mesh(world, shape, Vector3.ZERO, null)
	line.layers = G.DISPLAY

func build_panel() -> void:
	var layer := CanvasLayer.new()
	add_child(layer)
	var header := panel(layer, Vector2(20,18), Vector2(1240,69))
	var title := label(header, "URC   /   PICKUP DRONE TWIN", Vector2(18,9), 27, Color("f6ead3"))
	title.add_theme_color_override("font_shadow_color", Color("172329"))
	label(header, "TERRAIN SIMULATOR   /   SITE 01", Vector2(19,43), 13, Color("a1b8b6"))
	state_text = label(header, "", Vector2(870,23), 19, Color("6ae0c8"))
	var left := panel(layer, Vector2(20,105), Vector2(276,205))
	label(left, "FLIGHT TELEMETRY", Vector2(16,11), 14, Color("a1b8b6"))
	readout = label(left, "", Vector2(16,36), 16, Color("f6ead3"))
	var right := panel(layer, Vector2(877,105), Vector2(383,270))
	label(right, "ONBOARD CAMERA", Vector2(16,11), 14, Color("a1b8b6"))
	depth_hint = label(right, "", Vector2(16,238), 13, Color("a1b8b6"))
	image_widget = TextureRect.new()
	image_widget.position = Vector2(16,36)
	image_widget.size = Vector2(351,197)
	image_widget.expand_mode = TextureRect.EXPAND_IGNORE_SIZE
	image_widget.stretch_mode = TextureRect.STRETCH_KEEP_ASPECT_CENTERED
	image_widget.mouse_filter = Control.MOUSE_FILTER_IGNORE
	image_widget.texture = camera_stream.rgb_view.get_texture()
	right.add_child(image_widget)
	gripper_reticle = label(right, "+", Vector2.ZERO, 25, Color("64e5ce"))
	gripper_reticle.mouse_filter = Control.MOUSE_FILTER_IGNORE
	gripper_reticle.hide()
	var controls := panel(layer, Vector2(20,713), Vector2(1240,69))
	label(controls, "WASD  Move      Q/E  Height      ←/→  Yaw      C  View      T  Camera tilt      L  LiDAR", Vector2(16,8), 16, Color("f6ead3"))
	label(controls, "G  Gripper       Space  Hold / resume       X  E-stop       U  Clear latch       R  Reset at home", Vector2(16,36), 16, Color("a1b8b6"))

func panel(layer: CanvasLayer, at: Vector2, extent: Vector2) -> Panel:
	var node := Panel.new()
	node.position = at
	node.size = extent
	node.mouse_filter = Control.MOUSE_FILTER_IGNORE
	var style := StyleBoxFlat.new()
	style.bg_color = Color("15252de9")
	style.border_color = Color("3e5a5d")
	style.set_border_width_all(1)
	style.set_corner_radius_all(9)
	node.add_theme_stylebox_override("panel", style)
	layer.add_child(node)
	return node

func label(parent: Control, value: String, at: Vector2, px: int, color: Color) -> Label:
	var node := Label.new()
	node.text = value
	node.position = at
	node.add_theme_font_size_override("font_size", px)
	node.add_theme_color_override("font_color", color)
	parent.add_child(node)
	return node

func stamp() -> Dictionary:
	@warning_ignore("integer_division")
	var nanos := tick_index * 1000000000 / int(settings.runtime.physics_hz)
	@warning_ignore("integer_division")
	var seconds := nanos / 1000000000
	return {"sec": seconds, "nanosec": nanos % 1000000000}

func _physics_process(dt: float) -> void:
	if not is_instance_valid(drone) or not manifest_path.is_empty():
		return
	if not bridge.connected:
		backend_manual = false
		pending_gripper.clear()
		drone.halt()
		sensors.reset_link()
		return
	if clock_paused and step_remaining <= 0:
		return
	tick_index += 1
	if step_remaining > 0:
		step_remaining -= 1
	var now := stamp()
	if backend_manual and not drone.paused and get_window().has_focus():
		var f := float(Input.is_physical_key_pressed(KEY_W)) - float(Input.is_physical_key_pressed(KEY_S))
		var l := float(Input.is_physical_key_pressed(KEY_A)) - float(Input.is_physical_key_pressed(KEY_D))
		var z := float(Input.is_physical_key_pressed(KEY_E)) - float(Input.is_physical_key_pressed(KEY_Q))
		var yaw := float(Input.is_physical_key_pressed(KEY_LEFT)) - float(Input.is_physical_key_pressed(KEY_RIGHT))
		var velocity := Vector3(f*.8,l*.8,z*.42)
		bridge.publish("/control/manual",{"header":sensors.header(now,"base_link"),"twist":{"linear":G.point(velocity),"angular":G.point(Vector3(0,0,yaw*.65))}})
	if sensors.stalled:
		drone.halt()
	flight_tick(dt, now)
	if clock_paused and step_remaining == 0:
		PhysicsServer3D.set_active.call_deferred(false)

func flight_tick(dt: float, now: Dictionary) -> void:
	if not pending_gripper.is_empty():
		gripper.command(pending_gripper.command, pending_gripper.request_id, targets.bodies)
		pending_gripper.clear()
	var allowed := constrain_motion(drone.desired)
	drone.motion_step(dt, allowed, drone.desired_yaw)
	camera_stream.align(drone.global_transform)
	sensors.tick(tick_index, dt, now, epoch)
	for event in bins.deliveries(targets.bodies, dt, epoch):
		event["header"] = sensors.header(now, "map")
		bridge.publish("/bins/delivery_event", event)
	bridge.publish("/clock", {"clock": now})
	if tick_index % 12 == 0:
		var observation: Dictionary = oracle.sample(targets.bodies,camera_stream,sensors,settings.perception,now,epoch)
		bridge.publish("/drone/camera/camera_info",observation.info)
		bridge.publish("/perception/oracle/detections_2d",observation.boxes)
		bridge.publish("/perception/oracle/depth_samples",{"data":JSON.stringify(observation.depth)})
	if tick_index % 3 == 0:
		bridge.publish("/drone/gripper/status", gripper.status(now))
		bridge.publish("/joint_states", {"header": sensors.header(now, ""), "name": ["camera_tilt_joint"],
			"position": [deg_to_rad(camera_stream.tilt_degrees)], "velocity": [], "effort": []})
	if lidar_visible and tick_index % 6 == 0:
		draw_lidar()

func constrain_motion(flu: Vector3) -> Vector3:
	var map_velocity := G.to_ros(drone.global_basis * G.to_godot(flu))
	var pose := G.to_ros(drone.global_position)
	var fence: Array = settings.safety.geofence_xy
	var margin := float(settings.safety.fence_margin_m) + .5
	var stop_x: float = map_velocity.x * absf(map_velocity.x) / (2.0*drone.acceleration)
	var stop_y: float = map_velocity.y * absf(map_velocity.y) / (2.0*drone.acceleration)
	if (pose.x + stop_x <= float(fence[0])+margin and map_velocity.x < 0) or (pose.x + stop_x >= float(fence[1])-margin and map_velocity.x > 0):
		map_velocity.x = 0
	if (pose.y + stop_y <= float(fence[2])+margin and map_velocity.y < 0) or (pose.y + stop_y >= float(fence[3])-margin and map_velocity.y > 0):
		map_velocity.y = 0
	var agl := pose.z - Layout.elevation(manifest, pose.x, pose.y)
	if map_velocity.z > 0 and agl + .12 >= float(settings.safety.geofence_max_agl_m):
		map_velocity.z = 0
	if map_velocity.z < 0:
		var ground_ray: Dictionary = sensors.ray(drone.global_position, drone.global_position + Vector3.DOWN * .79, G.TERRAIN | G.SITE)
		if not ground_ray.is_empty():
			var clearance: float = drone.global_position.y - ground_ray.position.y
			map_velocity.z = -minf(absf(map_velocity.z), clampf((clearance-.555)*4.0,0.0,.17))
	return G.to_ros(drone.global_basis.inverse() * G.to_godot(map_velocity))

func on_camera_frame(info: Dictionary, jpeg: PackedByteArray, depth: PackedByteArray) -> void:
	if not bridge.connected:
		return
	bridge.publish("/drone/camera/camera_info", info)
	bridge.publish("/drone/camera/image/compressed", {"header": info.header, "format": "rgb8; jpeg compressed bgr8", "data": Marshalls.raw_to_base64(jpeg)})
	if not depth.is_empty():
		bridge.publish("/drone/camera/depth/image_raw", {"header": info.header, "height": info.height,
			"width": info.width, "encoding": "16UC1", "is_bigendian": 0, "step": 2 * info.width,
			"data": Marshalls.raw_to_base64(depth)})

func stamp_age(data: Dictionary) -> float:
	var now := stamp()
	return float(now.sec) - float(data.get("sec", -1)) + (float(now.nanosec) - float(data.get("nanosec", -1))) * 1e-9

func on_message(topic: String, msg: Dictionary) -> void:
	if topic == "/ui/telemetry":
		var value: Variant = JSON.parse_string(msg.get("data",""))
		if value is Dictionary:
			backend_manual = bool(value.status.manual)
			if is_instance_valid(dashboard):
				dashboard.update(value)
		return
	if topic == "/ui/response":
		if is_instance_valid(dashboard):
			dashboard.response(JSON.parse_string(msg.get("data","")))
		return
	if topic == "/sim/control/request":
		var request: Variant = JSON.parse_string(msg.get("data",""))
		if request is Dictionary and int(request.get("epoch",epoch)) == epoch:
			scene_request.call_deferred(request)
		return
	if topic == "/slam/status":
		sensors.on_status(msg, epoch)
	elif topic == "/drone/estop":
		if msg.get("data", false):
			pending_gripper.clear()
			drone.latch_estop()
		else:
			drone.clear_estop()
	elif topic == "/drone/camera/tilt_cmd" and is_finite(float(msg.get("data", NAN))):
		camera_stream.tilt_degrees = clampf(rad_to_deg(float(msg.data)), 0.0, 90.0)
	elif topic == "/drone/cmd_vel" and msg.has("header") and msg.has("twist"):
		var age := stamp_age(msg.header.get("stamp", {}))
		if age >= 0.0 and age <= float(settings.runtime.command_max_age_sim_s) and msg.header.get("frame_id") == "base_link":
			var vector: Dictionary = msg.twist.get("linear", {})
			drone.set_command(Vector3(vector.get("x",0.0),vector.get("y",0.0),vector.get("z",0.0)), float(msg.twist.get("angular", {}).get("z",0.0)))
	elif topic == "/drone/gripper/command" and not drone.estop:
		var age := stamp_age(msg.get("header", {}).get("stamp", {}))
		if age >= 0.0 and age <= float(settings.runtime.command_max_age_sim_s):
			pending_gripper = {"request_id": str(msg.get("request_id","")), "command": str(msg.get("command",""))}

func _unhandled_key_input(event: InputEvent) -> void:
	if not event is InputEventKey or not event.pressed or event.echo or not is_instance_valid(drone):
		return
	match event.keycode:
		KEY_C: follow = not follow
		KEY_L: lidar_visible = not lidar_visible
		KEY_T:
			if bridge.connected and backend_manual:
				camera_stream.tilt_degrees = float(settings.sensors.servo_camera_tilt_deg if camera_stream.tilt_degrees < 80.0 else settings.sensors.search_camera_tilt_deg)
		KEY_X:
			ui_request("estop")
			pending_gripper.clear()
			drone.latch_estop()
		KEY_U:
			ui_request("clear_estop")
		KEY_SPACE:
			ui_request("resume_mission" if drone.paused or (is_instance_valid(dashboard) and dashboard.is_paused()) else "pause_mission")
		KEY_ENTER:
			ui_request("start_mission", {"mission_id":"pilot-"+str(epoch)})
		KEY_M:
			if is_instance_valid(dashboard): dashboard.toggle_map()
		KEY_V:
			ui_request("set_manual", {"enabled":not backend_manual})
		KEY_G:
			if backend_manual:
				ui_request("manual_grip", {"command":"open" if gripper.closed else "close"})
		KEY_R:
			ui_request("reset_scene", {"seed":seed_value})

func reset_scene() -> void:
	remove_child(world)
	world.queue_free()
	epoch += 1
	pending_gripper.clear()
	if not build_world():
		get_tree().quit(2)
		return
	if is_instance_valid(image_widget):
		image_widget.texture = camera_stream.rgb_view.get_texture()

func draw_lidar() -> void:
	var points: PackedFloat32Array = sensors.safety_scan
	if points.is_empty():
		return
	var style := G.material(Color("66c7b8"))
	style.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
	var shape := ImmediateMesh.new()
	shape.surface_begin(Mesh.PRIMITIVE_LINES, style)
	var origin := drone.global_position + Vector3(0,.18,0)
	for i in range(0, points.size(), 8):
		var angle := -PI + TAU*i/360.
		shape.surface_add_vertex(origin)
		shape.surface_add_vertex(origin + drone.global_basis * Vector3(-sin(angle),0,-cos(angle)) * minf(points[i],5.0))
	shape.surface_end()
	lidar_overlay.mesh = shape

func _process(dt: float) -> void:
	if not is_instance_valid(drone):
		return
	lidar_overlay.visible = lidar_visible
	var anchor := drone.global_position
	if follow:
		var eye := anchor + drone.global_basis * Vector3(1.5,1.4,2.5)
		main_camera.position = main_camera.position.lerp(eye, minf(dt*7.0,1.0))
		main_camera.look_at(anchor + Vector3(0,.11,-.35))
	else:
		main_camera.position = G.to_godot(Vector3(-9.5,-16.5,23.0))
		main_camera.look_at(G.to_godot(Vector3(8.6,-.5,0.0)))
	if is_instance_valid(readout):
		var xyz := G.to_ros(anchor)
		var display := "MANUAL FLIGHT" if backend_manual else "ROS 2 CONTROL"
		if drone.estop:
			display = "E-STOP LATCHED"
		elif drone.paused:
			display = "HOLD  /  SPACE TO RESUME"
		state_text.text = display
		var link := "ROS CONNECTED" if bridge.connected else "WAITING FOR ROS"
		var payload: String = "%.2f kg" % drone.payload_mass_kg if gripper.held != null else gripper.reason
		readout.text = "Seed  %d   •   %s\nPosition  %.1f  %.1f  %.1f m\nDown range  %.2f m\nGripper  %s\nBin A   %d   /   Bin B   %d\nVisible station tags  %d" % [seed_value,link,xyz.x,xyz.y,xyz.z,sensors.range_down,payload,bins.counts.bin_a,bins.counts.bin_b,sensors.last_tag_observations.size()]
		depth_hint.text = "%d × %d     /     RGB + METRIC DEPTH     /     %d°" % [camera_stream.resolution.x, camera_stream.resolution.y, roundi(camera_stream.tilt_degrees)]
		update_reticle()

func update_reticle() -> void:
	var color_camera: Camera3D = camera_stream.rgb_camera
	var grip_point := gripper.global_position
	gripper_reticle.visible = camera_stream.tilt_degrees >= 80.0 and camera_stream.produced > 0 and not color_camera.is_position_behind(grip_point)
	if not gripper_reticle.visible:
		return
	var normalized := color_camera.unproject_position(grip_point) / Vector2(camera_stream.resolution)
	gripper_reticle.visible = Rect2(.04,.04,.92,.92).has_point(normalized)
	gripper_reticle.position = image_widget.position + image_widget.size * normalized - gripper_reticle.size * .5

func ui_request(method: String, args: Dictionary = {}) -> void:
	if not bridge.connected:
		return
	request_number += 1
	bridge.publish("/ui/request",{"data":JSON.stringify({"request_id":"godot-%d-%d" % [epoch,request_number],"epoch":epoch,"method":method,"args":args})})

func scene_request(request: Dictionary) -> void:
	var ok := true
	var was_paused := clock_paused
	match str(request.get("operation","")):
		"pause":
			clock_paused = true
			PhysicsServer3D.set_active(false)
			drone.halt()
		"resume":
			if not drone.estop:
				clock_paused = false
				PhysicsServer3D.set_active(true)
				drone.paused = false
				drone.halt()
			else:
				ok = false
		"step":
			if clock_paused:
				PhysicsServer3D.set_active(true)
				step_remaining = clampi(int(request.get("ticks",6)),1,60)
			else:
				ok = false
		"reset":
			if not drone.estop and gripper.held == null and drone.global_position.y < .85 and drone.velocity.length() < .08:
				seed_value = int(request.get("seed",seed_value))
				reset_scene()
			else:
				ok = false
		_:
			ok = false
	bridge.publish("/sim/control/response",{"data":JSON.stringify({"ok":ok,"operation":request.get("operation"),"epoch":epoch,"request_id":request.get("request_id",""),"was_paused":was_paused})})
