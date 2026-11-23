extends RefCounted

const G = preload("res://scripts/geometry.gd")
const SOURCE = "res://config/defaults.json"

static func read_defaults() -> Dictionary:
	var parsed: Variant = JSON.parse_string(FileAccess.get_file_as_string(SOURCE))
	assert(parsed is Dictionary and parsed.get("contract_version") == 1, "Invalid Godot defaults")
	return parsed

static func build(seed_value: int, settings: Dictionary) -> Dictionary:
	var scene: Dictionary = settings.scene
	var rng := RandomNumberGenerator.new()
	rng.seed = seed_value
	var params := {
		"seed": seed_value, "version": 1, "terrain_phase": rng.randf_range(0, TAU),
		"terrain_amplitude": tan(deg_to_rad(float(scene.max_slope_deg))) * 0.83,
		"sun_yaw_deg": rng.randf_range(-60.0, -20.0),
		"sun_pitch_deg": rng.randf_range(-53.0, -35.0),
		"sun_energy": rng.randf_range(0.80, 1.04),
		"station_world_xyz_yaw": scene.station_world_xyz_yaw,
		"initial_base_map_xyz_yaw": scene.initial_base_map_xyz_yaw,
		"bin_a_map_xyz": scene.bin_a_map_xyz, "bin_b_map_xyz": scene.bin_b_map_xyz,
		"search_bounds_xy": settings.mission.search_bounds_xy,
		"rocks": [], "objects": []
	}
	var bounds: Array = params.search_bounds_xy
	for i in range(int(scene.get("rock_count",28))):
		var x := rng.randf_range(float(bounds[0]), float(bounds[1]) + 3.0)
		var y := rng.randf_range(float(bounds[2]) - 2.5, float(bounds[3]) + 2.5)
		params.rocks.append({
			"xy": [x, y], "radius_m": rng.randf_range(0.22, 0.70),
			"height_m": rng.randf_range(0.18, float(scene.max_rock_height_m)),
			"yaw": rng.randf_range(0, TAU), "variation": rng.randf(),
		})
	var placement_separation := float(scene.object_min_separation_m)
	var total := int(scene.bottles) + int(scene.hammers)
	for i in range(total):
		var class_id := "bottle" if i < int(scene.bottles) else "hammer"
		var found := false
		for attempt in range(512):
			var location := Vector2(rng.randf_range(float(bounds[0]) + 0.5, float(bounds[1]) - 0.5), rng.randf_range(float(bounds[2]) + 0.5, float(bounds[3]) - 0.5))
			if not is_clear(params, location, placement_separation):
				continue
			var yaw := rng.randf_range(0, TAU)
			params.objects.append({
				"id": "%s_%02d" % [class_id, i + 1], "class_id": class_id,
				"xyz": [location.x, location.y, elevation(params, location.x, location.y) + 0.14],
				"yaw": yaw, "sideways": class_id == "hammer" or rng.randf() < 0.65,
				"color_variant": rng.randi_range(0, 2),
			})
			found = true
			break
		if not found:
			push_error("Seed %s cannot place %d configured objects." % [seed_value, total])
			return {}
	return params

static func is_clear(manifest: Dictionary, p: Vector2, separation: float) -> bool:
	for rock: Dictionary in manifest.rocks:
		if p.distance_to(Vector2(rock.xy[0], rock.xy[1])) < float(rock.radius_m) + 0.62:
			return false
	for item: Dictionary in manifest.objects:
		if p.distance_to(Vector2(item.xyz[0], item.xyz[1])) < separation:
			return false
	return true

static func elevation(manifest: Dictionary, x: float, y: float) -> float:
	# Keep all station and bin pads flat, then blend into low hills analytically.
	var fade := smoothstep(2.6, 8.0, x)
	var phase := float(manifest.terrain_phase)
	return fade * float(manifest.terrain_amplitude) * (sin(x * 0.31 + phase) + 0.60 * cos(y * 0.42 - phase) + 0.32 * sin((x + y) * 0.68))
