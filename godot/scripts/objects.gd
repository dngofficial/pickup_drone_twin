extends Node3D

const G = preload("res://scripts/geometry.gd")
var bodies: Array[RigidBody3D] = []

func build(manifest: Dictionary, config: Dictionary) -> void:
	for spec: Dictionary in manifest.objects:
		var body := RigidBody3D.new()
		body.name = spec.id
		body.position = G.to_godot(G.xyz(spec.xyz))
		body.rotation.y = float(spec.yaw)
		body.collision_layer = G.TARGET
		body.collision_mask = G.TERRAIN | G.SITE | G.TARGET | G.DRONE
		body.linear_damp = .5
		body.angular_damp = 1.0
		body.continuous_cd = true
		body.set_meta("class_id", spec.class_id)
		add_child(body)
		if spec.class_id == "bottle":
			body.mass = float(config.bottle_mass_kg)
			bottle(body, int(spec.color_variant))
			if spec.sideways:
				body.rotate_object_local(Vector3.RIGHT, PI/2)
		else:
			body.mass = float(config.hammer_mass_kg)
			hammer(body, int(spec.color_variant))
		bodies.append(body)

func bottle(body: RigidBody3D, variant: int) -> void:
	var colors := [Color("06b9e1"), Color("78d9ec"), Color("059bb6")]
	var model := G.fitted_model(body, "res://assets/cc0/water_bottle.glb", Vector3(.09,.28,.09), Vector3.ZERO)
	if model != null:
		var palette := [G.material(colors[variant], .38), G.material(Color("163854")), G.material(Color("f1f2dd"))]
		for i in range(mini(model.mesh.get_surface_count(), palette.size())):
			model.set_surface_override_material(i, palette[i])
		G.model_hull(body, model)
	var ring := Marker3D.new()
	ring.name = "PickPoint"
	ring.position = Vector3(0,0,0)
	body.add_child(ring)

func hammer(body: RigidBody3D, variant: int) -> void:
	var colors := [Color("f47b26"), Color("bd481d"), Color("e99337")]
	var axes := Basis(Vector3.UP, Vector3.FORWARD, Vector3.LEFT)
	var model := G.fitted_model(body, "res://assets/cc-by/hammer.glb", Vector3(.19,.056,.371), Vector3.ZERO, axes)
	if model != null:
		model.set_surface_override_material(0, G.material(colors[variant], .61))
		model.set_surface_override_material(1, G.material(Color("4d545c"), .39))
		for i in range(model.mesh.get_surface_count()):
			var shape := ConvexPolygonShape3D.new()
			var vertices := PackedVector3Array()
			for v: Vector3 in model.mesh.surface_get_arrays(i)[Mesh.ARRAY_VERTEX]:
				vertices.append(model.transform * v)
			shape.points = vertices
			G.attach_shape(body, shape)
	var pick := Marker3D.new()
	pick.name = "PickPoint"
	pick.position = Vector3(-.018,0,.043)
	body.add_child(pick)
