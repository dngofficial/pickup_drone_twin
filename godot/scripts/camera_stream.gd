extends Node3D

const G = preload("res://scripts/geometry.gd")
const DEPTH_SHADER = preload("res://shaders/metric_depth.gdshader")

signal frame_ready(info: Dictionary, jpeg: PackedByteArray, depth_mm: PackedByteArray)
var rgb_view: SubViewport
var depth_view: SubViewport
var rgb_camera: Camera3D
var depth_camera: Camera3D
var enabled := false
var resolution: Vector2i
var tilt_degrees := 60.0
var vertical_fov: float
var pending := false
var pending_stamp: Dictionary
var want_depth := false
var optical_pose := Transform3D.IDENTITY
var produced := 0

func setup(settings: Dictionary) -> void:
	resolution = Vector2i(int(settings.camera_width), int(settings.camera_height))
	vertical_fov = float(settings.camera_vertical_fov_deg)
	tilt_degrees = float(settings.search_camera_tilt_deg)
	enabled = DisplayServer.get_name() != "headless"
	if not enabled:
		return
	rgb_view = make_viewport("ColorCamera")
	depth_view = make_viewport("MetricDepth")
	rgb_camera = make_camera(rgb_view, 1)
	depth_camera = make_camera(depth_view, 1 | G.DEPTH_QUAD)
	var quad := QuadMesh.new()
	quad.size = Vector2(2.0, 2.0)
	var material := ShaderMaterial.new()
	material.shader = DEPTH_SHADER
	material.render_priority = 100
	var pass_mesh := G.mesh(depth_camera, quad, Vector3(0,0,-1), material)
	pass_mesh.layers = G.DEPTH_QUAD
	pass_mesh.cast_shadow = GeometryInstance3D.SHADOW_CASTING_SETTING_OFF
	RenderingServer.frame_post_draw.connect(read_frame)

func make_viewport(view_name: String) -> SubViewport:
	var view := SubViewport.new()
	view.name = view_name
	view.size = resolution
	view.world_3d = get_world_3d()
	view.render_target_update_mode = SubViewport.UPDATE_DISABLED
	add_child(view)
	return view

func make_camera(view: SubViewport, mask: int) -> Camera3D:
	var camera := Camera3D.new()
	camera.keep_aspect = Camera3D.KEEP_HEIGHT
	camera.fov = vertical_fov
	camera.near = .035
	camera.far = 60.0
	camera.cull_mask = mask
	view.add_child(camera)
	camera.make_current()
	return camera

func align(drone_pose: Transform3D) -> void:
	var mount := Transform3D(Basis(Vector3.RIGHT, -deg_to_rad(tilt_degrees)), G.to_godot(Vector3(.20, 0.0, -.15)))
	optical_pose = drone_pose * mount

func request_frame(stamp: Dictionary, depth: bool) -> void:
	if not enabled:
		return
	# If rendering services several physics ticks at once, keep the paired
	# capture instead of letting an RGB-only request displace it.
	if pending and (want_depth or not depth):
		return
	pending = true
	pending_stamp = stamp
	want_depth = depth
	rgb_camera.global_transform = optical_pose
	rgb_view.render_target_update_mode = SubViewport.UPDATE_ONCE
	if depth:
		depth_camera.global_transform = optical_pose
		depth_view.render_target_update_mode = SubViewport.UPDATE_ONCE

func camera_info(stamp: Dictionary) -> Dictionary:
	var f := float(resolution.y) / (2.0 * tan(deg_to_rad(vertical_fov * .5)))
	var cx := (float(resolution.x) - 1.0) * .5
	var cy := (float(resolution.y) - 1.0) * .5
	return {"header": {"stamp": stamp, "frame_id": "camera_optical_frame"},
		"height": resolution.y, "width": resolution.x, "distortion_model": "plumb_bob",
		"d": [0.0,0.0,0.0,0.0,0.0], "k": [f,0.0,cx,0.0,f,cy,0.0,0.0,1.0],
		"r": [1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0],
		"p": [f,0.0,cx,0.0,0.0,f,cy,0.0,0.0,0.0,1.0,0.0], "binning_x": 0, "binning_y": 0}

func read_frame() -> void:
	if not pending:
		return
	pending = false
	var image := rgb_view.get_texture().get_image()
	if image == null or image.is_empty():
		return
	var jpeg := image.save_jpg_to_buffer(.80)
	var depth_mm := PackedByteArray()
	if want_depth:
		var packed := depth_view.get_texture().get_image()
		if packed != null and not packed.is_empty():
			packed.convert(Image.FORMAT_RGB8)
			var channels := packed.get_data()
			depth_mm.resize(resolution.x * resolution.y * 2)
			for i in range(resolution.x * resolution.y):
				var index := 3 * i
				var mm := (channels[index] >> 2) | ((channels[index + 1] >> 2) << 6) | ((channels[index + 2] >> 4) << 12)
				depth_mm.encode_u16(2 * i, mm)
	produced += 1
	frame_ready.emit(camera_info(pending_stamp), jpeg, depth_mm)
