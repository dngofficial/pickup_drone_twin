extends Marker3D

const G = preload("res://scripts/geometry.gd")
var drone: CharacterBody3D
var container: Node3D
var held: RigidBody3D
var token := ""
var serial := 0
var closed := false
var reason := "open"
var request_id := ""
var processed_requests: Array[String] = []
var jaw_left: Node3D
var jaw_right: Node3D
var payload_shapes: Array[CollisionShape3D] = []
var capture_radius := .10
var max_relative_speed := .08

func install(owner_drone: CharacterBody3D, target_container: Node3D, settings: Dictionary) -> void:
	drone = owner_drone
	container = target_container
	capture_radius = float(settings.grasp_capture_radius_m)
	max_relative_speed = float(settings.grasp_max_relative_speed_mps)
	var paint := G.material(Color("66c7b8"), .46)
	G.box(self, Vector3(0,.12,0), Vector3(.13,.09,.14), paint)
	G.box(self, Vector3(0,.17,0), Vector3(.05,.15,.05), G.material(Color("313e48")))
	jaw_left = G.box(self, Vector3(-.095,.043,0), Vector3(.023,.13,.12), paint)
	jaw_right = G.box(self, Vector3(.095,.043,0), Vector3(.023,.13,.12), paint)
	for visual: MeshInstance3D in [jaw_left, jaw_right]:
		visual.layers = G.DISPLAY
	for node in get_children():
		if node is MeshInstance3D:
			node.layers = G.DISPLAY

func command(action: String, id: String, bodies: Array[RigidBody3D]) -> void:
	if id.is_empty() or processed_requests.has(id) or action not in ["open", "close"]:
		return
	request_id = id
	processed_requests.append(id)
	if processed_requests.size() > 128:
		processed_requests.pop_front()
	if drone.estop:
		reason = "gripper held by emergency stop"
		return
	if action == "open":
		release()
	else:
		closed = true
		if held == null:
			capture(bodies)
	jaw_left.position.x = -.043 if closed else -.095
	jaw_right.position.x = .043 if closed else .095

func capture(bodies: Array[RigidBody3D]) -> void:
	if drone.desired.length() > max_relative_speed or absf(drone.desired_yaw) > .2:
		reason = "hold still before closing"
		return
	var best_distance := capture_radius
	var candidate: RigidBody3D
	for body in bodies:
		if body.freeze or (body.linear_velocity - drone.velocity).length() > max_relative_speed or body.angular_velocity.length() > .3:
			continue
		var point: Marker3D = body.get_node("PickPoint")
		var distance := point.global_position.distance_to(global_position)
		if distance >= best_distance:
			continue
		var ray := PhysicsRayQueryParameters3D.create(global_position, point.global_position, G.TERRAIN | G.SITE)
		if not get_world_3d().direct_space_state.intersect_ray(ray).is_empty():
			continue
		candidate = body
		best_distance = distance
	if candidate == null:
		reason = "missed grasp"
		return
	held = candidate
	serial += 1
	token = "payload-%04d" % serial
	held.set_meta("released_token", "")
	held.freeze = true
	held.collision_layer = 0
	held.collision_mask = 0
	held.reparent(self, true)
	for child in held.get_children():
		if child is CollisionShape3D:
			var proxy := G.attach_shape(drone, child.shape)
			proxy.global_transform = child.global_transform
			payload_shapes.append(proxy)
	drone.payload_mass_kg = held.mass
	reason = "attached"

func release() -> void:
	closed = false
	reason = "open"
	if held == null:
		return
	for proxy in payload_shapes:
		proxy.disabled = true
		proxy.queue_free()
	payload_shapes.clear()
	held.reparent(container, true)
	held.set_meta("released_token", token)
	held.collision_layer = G.TARGET
	held.collision_mask = G.TERRAIN | G.SITE | G.TARGET | G.DRONE
	held.freeze = false
	held.sleeping = false
	held.linear_velocity = drone.velocity
	held.angular_velocity = Vector3.ZERO
	held = null
	drone.payload_mass_kg = 0.0
	reason = "released"

func status(stamp: Dictionary) -> Dictionary:
	return {"header": {"stamp": stamp, "frame_id": "gripper_link"}, "request_id": request_id,
		"closed": closed, "attached": held != null, "payload_token": token,
		"payload_mass_kg": drone.payload_mass_kg, "slip_detected": false, "reason": reason}
