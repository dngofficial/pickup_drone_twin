extends RefCounted

const TERRAIN := 1
const DRONE := 2
const TARGET := 4
const SITE := 8
const DISPLAY := 2
const DEPTH_QUAD := 1 << 19

static func to_godot(ros: Vector3) -> Vector3:
	return Vector3(-ros.y, ros.z, -ros.x)

static func to_ros(godot: Vector3) -> Vector3:
	return Vector3(-godot.z, -godot.x, godot.y)

static func xyz(values: Array) -> Vector3:
	return Vector3(float(values[0]), float(values[1]), float(values[2]))

static func as_array(v: Vector3) -> Array:
	return [v.x, v.y, v.z]

static func point(v: Vector3) -> Dictionary:
	return {"x": v.x, "y": v.y, "z": v.z}

static func quaternion(q: Quaternion) -> Dictionary:
	return {"x": q.x, "y": q.y, "z": q.z, "w": q.w}

static func ros_rotation(basis_godot: Basis) -> Quaternion:
	var x := to_ros(basis_godot * Vector3.FORWARD)
	var y := to_ros(basis_godot * Vector3.LEFT)
	var z := to_ros(basis_godot * Vector3.UP)
	return Basis(x, y, z).orthonormalized().get_rotation_quaternion()

static func material(color: Color, roughness := 0.8) -> StandardMaterial3D:
	var result := StandardMaterial3D.new()
	result.albedo_color = color
	result.roughness = roughness
	return result

static func mesh(parent: Node3D, shape: Mesh, at: Vector3, material_override: Material) -> MeshInstance3D:
	var instance := MeshInstance3D.new()
	instance.mesh = shape
	instance.material_override = material_override
	instance.position = at
	parent.add_child(instance)
	return instance

static func box(parent: Node3D, at: Vector3, size: Vector3, paint: Material) -> MeshInstance3D:
	var shape := BoxMesh.new()
	shape.size = size
	return mesh(parent, shape, at, paint)

static func cylinder(parent: Node3D, at: Vector3, radius: float, height: float, paint: Material, top := -1.0) -> MeshInstance3D:
	var shape := CylinderMesh.new()
	shape.bottom_radius = radius
	shape.top_radius = top if top >= 0.0 else radius
	shape.height = height
	shape.radial_segments = 20
	return mesh(parent, shape, at, paint)

static func attach_shape(body: CollisionObject3D, shape: Shape3D, at := Vector3.ZERO) -> CollisionShape3D:
	var node := CollisionShape3D.new()
	node.shape = shape
	node.position = at
	body.add_child(node)
	return node

static func box_shape(body: CollisionObject3D, at: Vector3, size: Vector3) -> CollisionShape3D:
	var shape := BoxShape3D.new()
	shape.size = size
	return attach_shape(body, shape, at)

static func solid_box(parent: Node3D, at: Vector3, size: Vector3, paint: Material, layer := SITE) -> StaticBody3D:
	var body := StaticBody3D.new()
	body.position = at
	body.collision_layer = layer
	body.collision_mask = 0
	parent.add_child(body)
	box(body, Vector3.ZERO, size, paint)
	box_shape(body, Vector3.ZERO, size)
	return body

static func sign(parent: Node3D, message: String, at: Vector3, color: Color, pixel_size := 0.009) -> Label3D:
	var label := Label3D.new()
	label.text = message
	label.position = at
	label.font_size = 48
	label.pixel_size = pixel_size
	label.modulate = color
	label.outline_size = 0
	label.layers = DISPLAY
	parent.add_child(label)
	return label

static var mesh_sources: Dictionary = {}

static func model_source(path: String) -> Dictionary:
	if not mesh_sources.has(path):
		var importer := GLTFDocument.new()
		var state := GLTFState.new()
		var result := importer.append_from_buffer(FileAccess.get_file_as_bytes(path), path.get_base_dir(), state)
		if result != OK:
			push_error("Cannot read required model " + path)
			return {}
		var imported := importer.generate_scene(state)
		mesh_sources[path] = find_model_mesh(imported, Transform3D.IDENTITY)
		imported.free()
	return mesh_sources[path]

static func find_model_mesh(node: Node, transform: Transform3D) -> Dictionary:
	var combined := transform
	if node is Node3D:
		combined *= node.transform
	if node is MeshInstance3D and node.mesh != null:
		return {"mesh": node.mesh, "transform": combined}
	for child in node.get_children():
		var candidate := find_model_mesh(child, combined)
		if not candidate.is_empty():
			return candidate
	return {}

static func fitted_model(parent: Node3D, path: String, size: Vector3, at: Vector3, orientation := Basis.IDENTITY) -> MeshInstance3D:
	var asset := model_source(path)
	if asset.is_empty():
		return null
	var mesh_resource: Mesh = asset.mesh
	var pose: Transform3D = Transform3D(orientation, Vector3.ZERO) * asset.transform
	var box_bounds := pose * mesh_resource.get_aabb()
	var ratio := size / box_bounds.size
	var fit := Transform3D(Basis.from_scale(ratio), at - box_bounds.get_center() * ratio)
	var model := mesh(parent, mesh_resource, Vector3.ZERO, null)
	model.transform = fit * pose
	return model

static func model_hull(body: CollisionObject3D, model: MeshInstance3D) -> CollisionShape3D:
	var mesh_hull := model.mesh.create_convex_shape()
	var vertices := PackedVector3Array()
	for v in mesh_hull.points:
		vertices.append(model.transform * v)
	var shape := ConvexPolygonShape3D.new()
	shape.points = vertices
	return attach_shape(body, shape)
