extends SceneTree

const G = preload("res://scripts/geometry.gd")
var destination := ""
var first_seed := 0
var last_seed := 1
var frames := 12

func _initialize() -> void:
	for arg in OS.get_cmdline_user_args():
		if arg.begins_with("--output="): destination = arg.trim_prefix("--output=")
		if arg.begins_with("--first="): first_seed = arg.trim_prefix("--first=").to_int()
		if arg.begins_with("--last="): last_seed = arg.trim_prefix("--last=").to_int()
		if arg.begins_with("--frames="): frames = arg.trim_prefix("--frames=").to_int()
	if DisplayServer.get_name()=="headless" or destination.is_empty() or last_seed <= first_seed or frames < 1:
		push_error("A graphical renderer, --output, and a valid seed range are required")
		quit(2)
	else:
		run.call_deferred()

func run() -> void:
	var scene: Node = preload("res://scenes/main.tscn").instantiate()
	root.add_child(scene)
	var rng := RandomNumberGenerator.new()
	for seed_value in range(first_seed,last_seed):
		scene.seed_value = seed_value
		scene.reset_scene()
		for i in range(65): await physics_frame
		scene.set_physics_process(false)
		var stream: Node3D = scene.camera_stream
		while stream.pending: await stream.frame_ready
		PhysicsServer3D.set_active(false)
		var bodies: Array[RigidBody3D] = scene.targets.bodies
		var originals: Dictionary = {}
		for body in bodies:
			for child in body.get_children():
				if child is MeshInstance3D:
					originals[child] = child.material_override
		rng.seed = seed_value ^ 0x1671
		for frame in range(frames):
			var anchor: Vector3 = bodies[frame % bodies.size()].global_position
			var origin := anchor + Vector3(rng.randf_range(-1.15,1.15),rng.randf_range(1.0,4.0),rng.randf_range(-1.15,1.15))
			stream.tilt_degrees = rng.randf_range(66.0,90.0)
			stream.align(Transform3D(Basis(Vector3.UP,rng.randf_range(-PI,PI)),origin))
			var h := {"sec":seed_value-first_seed,"nanosec":frame}
			stream.request_frame(h,false)
			var captured: Array = await stream.frame_ready
			for i in range(bodies.size()):
				var material := G.material(Color8(0,22+i*19,249))
				material.shading_mode = BaseMaterial3D.SHADING_MODE_UNSHADED
				for child in bodies[i].get_children():
					if child is MeshInstance3D: child.material_override = material
			stream.request_frame(h,false)
			await stream.frame_ready
			var mask: Image = stream.rgb_view.get_texture().get_image()
			for child: MeshInstance3D in originals: child.material_override = originals[child]
			var labels := visible_boxes(mask,bodies)
			var split := "test" if posmod(seed_value,10)==9 else ("val" if posmod(seed_value,10)==8 else "train")
			var stem := "seed_%06d_frame_%04d" % [seed_value,frame]
			for key in ["images","labels","metadata"]:
				DirAccess.make_dir_recursive_absolute(destination.path_join(key).path_join(split))
			var image_file := FileAccess.open(destination.path_join("images/%s/%s.jpg" % [split,stem]),FileAccess.WRITE)
			image_file.store_buffer(captured[1])
			var label_file := FileAccess.open(destination.path_join("labels/%s/%s.txt" % [split,stem]),FileAccess.WRITE)
			for line: String in labels: label_file.store_line(line)
			var metadata := FileAccess.open(destination.path_join("metadata/%s/%s.json" % [split,stem]),FileAccess.WRITE)
			metadata.store_string(JSON.stringify({"seed":seed_value,"frame":frame,"camera_info":captured[0],"camera_origin_map":G.as_array(G.to_ros(stream.optical_pose.origin)),"camera_rotation_xyzw":G.quaternion(G.ros_rotation(stream.optical_pose.basis)),"tilt_degrees":stream.tilt_degrees,"labels":labels.size(),"label_version":1}))
		PhysicsServer3D.set_active(true)
		scene.set_physics_process(true)
		print("SYNTHETIC seed=%d frames=%d" % [seed_value,frames])
	quit()

func visible_boxes(mask: Image, bodies: Array[RigidBody3D]) -> Array[String]:
	mask.convert(Image.FORMAT_RGB8)
	var pixels := mask.get_data()
	var width := mask.get_width()
	var height := mask.get_height()
	var lows: Array[Vector2] = []
	var highs: Array[Vector2] = []
	var totals := PackedInt32Array()
	for _i in range(bodies.size()):
		lows.append(Vector2(width,height)); highs.append(Vector2.ZERO); totals.append(0)
	for y in range(height):
		for x in range(width):
			var base := (y*width+x)*3
			if pixels[base]>6 or pixels[base+2]<240 or pixels[base+2]>251:
				continue
			var id := roundi((pixels[base+1]-21)/19.0)
			if id < 0 or id >= bodies.size() or absi(pixels[base+1]-(22+id*19))>5:
				continue
			lows[id] = lows[id].min(Vector2(x,y))
			highs[id] = highs[id].max(Vector2(x+1,y+1))
			totals[id] += 1
	var lines: Array[String] = []
	for i in range(bodies.size()):
		if totals[i] < 12: continue
		var center := (lows[i]+highs[i])*.5
		var extent := highs[i]-lows[i]
		var label := 0 if bodies[i].get_meta("class_id")=="bottle" else 1
		lines.append("%d %.7f %.7f %.7f %.7f" % [label,center.x/width,center.y/height,extent.x/width,extent.y/height])
	return lines
