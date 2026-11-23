# Adapted from warehouse_drone_twin. Upstream MIT notice is in third_party/licenses.
extends CharacterBody3D
const G = preload("res://scripts/geometry.gd")
var desired := Vector3.ZERO
var desired_yaw := 0.0
var last_command_ms := -10000
var estop := false
var paused := false
var rotors: Array[Node3D] = []
var payload_mass_kg := 0.0
var acceleration := 1.8
var speed_limit_mps := .8
var payload_speed_coefficient := 1.5
var watchdog_ms := 500
var vertical_limit_mps := .6

func _ready() -> void:
	name = "PickupDrone"
	collision_layer = G.DRONE
	collision_mask = G.TERRAIN | G.SITE | G.TARGET
	motion_mode = CharacterBody3D.MOTION_MODE_FLOATING
	G.box_shape(self, Vector3(0,-.08,0), Vector3(.97,.51,.97))
	var dark = G.material(Color("232a31"),0.35)
	var black = G.material(Color("10151a"),0.2)
	var grey = G.material(Color("5e6a77"),0.55)
	var red = G.material(Color("ed644a"))
	var lens = G.material(Color("18384a"),0.7)
	G.box(self,Vector3.ZERO,Vector3(.34,.16,.48),dark)
	G.box(self,Vector3(0,.11,.015),Vector3(.23,.07,.33),grey)
	for x in [-1,1]:
		for z in [-1,1]:
			var arm = G.box(self,Vector3(x*.19,0,z*.19),Vector3(.10,.08,.56),dark)
			arm.rotation.y = float(x*z)*PI/4.0
			G.cylinder(self,Vector3(x*.38,.05,z*.38),.075,.10,black)
			G.box(self,Vector3(x*.29,-.04,z*.29),Vector3(.13,.025,.15),red if z == -1 else grey)
			var rotor = Node3D.new()
			rotor.position = Vector3(x*.38,.115,z*.38)
			add_child(rotor)
			G.box(rotor,Vector3.ZERO,Vector3(.40,.012,.035),grey)
			rotors.append(rotor)
	for x in [-1,1]:
		var leg = G.box(self,Vector3(x*.17,-.20,.06),Vector3(.035,.35,.035),dark)
		leg.rotation.z = x*.2
		G.box(self,Vector3(x*.20,-.36,.02),Vector3(.045,.035,.45),black)
	G.box(self,Vector3(0,-.17,-.07),Vector3(.035,.18,.035),grey)
	G.box(self,Vector3(0,-.27,-.10),Vector3(.15,.105,.11),black)
	var eye = G.cylinder(self,Vector3(0,-.27,-.165),.038,.025,lens)
	eye.rotation.x = PI/2
	G.cylinder(self,Vector3(0,.20,0),.075,.055,dark)
	G.cylinder(self,Vector3(0,.23,0),.065,.012,red)
	G.box(self,Vector3(0,-.13,.13),Vector3(.12,.06,.12),black)
	G.cylinder(self,Vector3(0,-.17,.13),.028,.018,lens)
	var title := G.sign(self, "SCOUT 01", Vector3(0,.51,0), Color("a2fbff"), .0055)
	title.billboard = BaseMaterial3D.BILLBOARD_ENABLED

	set_camera_exclusion(self)

func set_camera_exclusion(node: Node) -> void:
	if node is MeshInstance3D:
		node.layers = G.DISPLAY
	for child in node.get_children():
		set_camera_exclusion(child)

func set_command(body_flu: Vector3, yaw_rate: float) -> void:
	if estop or paused or not body_flu.is_finite() or not is_finite(yaw_rate):
		return
	desired = body_flu
	desired_yaw = clampf(yaw_rate, -1.2, 1.2)
	last_command_ms = Time.get_ticks_msec()

func command_fresh() -> bool:
	return Time.get_ticks_msec() - last_command_ms <= watchdog_ms

func halt() -> void:
	desired = Vector3.ZERO
	desired_yaw = 0.0
	last_command_ms = -10000
	velocity = Vector3.ZERO

func latch_estop() -> void:
	estop = true
	paused = true
	halt()

func clear_estop() -> void:
	estop = false
	halt()

func horizontal_limit() -> float:
	return speed_limit_mps / (1.0 + payload_speed_coefficient * payload_mass_kg)

func motion_step(dt: float, linear: Vector3, yaw_rate: float) -> void:
	if estop or paused:
		halt()
		return
	var fresh := command_fresh()
	if not fresh:
		linear = Vector3.ZERO
		yaw_rate = 0.0
	rotation.y = wrapf(rotation.y + yaw_rate * dt, -PI, PI)
	var limit := horizontal_limit()
	var horizontal := Vector2(linear.x, linear.y).limit_length(limit)
	var local_velocity := G.to_godot(Vector3(horizontal.x, horizontal.y, clampf(linear.z, -vertical_limit_mps, vertical_limit_mps)))
	velocity = velocity.move_toward(global_basis * local_velocity, acceleration * dt)
	move_and_slide()
	for rotor in rotors:
		rotor.rotation.y += dt * 56.0
