extends CanvasLayer

const Topdown = preload("res://scripts/topdown_map.gd")
const Overlay = preload("res://scripts/detection_overlay.gd")
var simulator: Node
var info: Dictionary = {}
var map_panel: Control
var overlays: Control
var mission_card: Panel
var status_line: Label
var progress: ProgressBar
var station_line: Label
var totals: Label
var seed_field: LineEdit
var message_line: Label
var detector: OptionButton
var policy: OptionButton
var selected_bin: OptionButton
var mission_button: Button
var sim_button: Button

func setup(host: Node) -> void:
	simulator = host
	layer = 2
	mission_card = host.panel(self,Vector2(877,390),Vector2(383,308))
	host.label(mission_card,"MISSION CONTROL",Vector2(14,9),14,Color("9bb8bb"))
	status_line = host.label(mission_card,"WAITING FOR ROS 2",Vector2(14,32),20,Color("74e2c5"))
	progress = ProgressBar.new()
	progress.position = Vector2(14,62)
	progress.size = Vector2(355,18)
	progress.show_percentage = false
	mission_card.add_child(progress)
	station_line = host.label(mission_card,"WAITING FOR MISSION TELEMETRY",Vector2(14,82),13,Color("9bb8bb"))
	station_line.tooltip_text = "ROS bridge: " + str(host.bridge.url)
	totals = host.label(mission_card,"",Vector2(14,105),14,Color("f6ead3"))
	mission_button = button(mission_card,"Start",Vector2(14,151),81,mission_action)
	button(mission_card,"Pilot",Vector2(103,151),81,func(): send("set_manual",{"enabled":not bool(info.get("status",{}).get("manual",false))}))
	button(mission_card,"E-stop",Vector2(192,151),81,func(): send("estop"))
	button(mission_card,"Clear",Vector2(281,151),87,func(): send("clear_estop"))
	detector = options(mission_card,Vector2(14,188),100,["Oracle","YOLOv8"])
	detector.item_selected.connect(func(i: int): send("set_detector",{"mode":"oracle" if i==0 else "yolo"}))
	policy = options(mission_card,Vector2(121,188),148,["Sort by class","Nearest bin","Select bin"])
	selected_bin = options(mission_card,Vector2(276,188),92,["Bin A","Bin B"])
	policy.item_selected.connect(func(_i): set_policy())
	selected_bin.item_selected.connect(func(_i): set_policy())
	seed_field = LineEdit.new()
	seed_field.position = Vector2(14,227)
	seed_field.size = Vector2(69,29)
	seed_field.text = str(host.seed_value)
	mission_card.add_child(seed_field)
	button(mission_card,"Reset",Vector2(89,227),65,func():
		if seed_field.text.is_valid_int(): send("reset_scene",{"seed":seed_field.text.to_int()}))
	sim_button = button(mission_card,"Pause sim",Vector2(160,227),99,func():
		send("resume_sim" if simulator.clock_paused else "pause_sim"))
	button(mission_card,"Step",Vector2(265,227),51,func(): send("step_sim",{"ticks":6}))
	button(mission_card,"CLI",Vector2(322,227),47,show_commands)
	message_line = host.label(mission_card,"Click a green track on the map to target it.",Vector2(14,266),12,Color("9bb8bb"))
	map_panel = Topdown.new()
	map_panel.position = Vector2(20,329)
	map_panel.size = Vector2(355,321)
	add_child(map_panel)
	map_panel.selected.connect(func(track: String): send("select_target",{"track_id":track}))
	map_panel.hide()
	overlays = Overlay.new()
	overlays.position = Vector2(893,141)
	overlays.size = Vector2(351,197)
	overlays.mouse_filter = Control.MOUSE_FILTER_IGNORE
	add_child(overlays)

func button(parent: Node, title: String, pos: Vector2, width: float, action: Callable) -> Button:
	var item := Button.new()
	item.position = pos
	item.size = Vector2(width,31)
	item.text = title
	# Space/Enter are flight shortcuts, not implicit presses of the last button.
	item.focus_mode = Control.FOCUS_NONE
	item.add_theme_font_size_override("font_size",13)
	item.pressed.connect(action)
	parent.add_child(item)
	return item

func mission_action() -> void:
	var state: Dictionary = info.get("status", {})
	if bool(state.get("estop_latched", false)):
		message_line.text = "E-stop latched: click Clear, then Resume."
	elif str(state.get("state", "IDLE")) == "IDLE":
		send("start_mission", {"mission_id":"mission-"+str(simulator.epoch)})
	elif is_paused() or bool(state.get("manual", false)) or simulator.clock_paused:
		send("resume_mission")
	else:
		send("pause_mission")

func options(parent: Node, pos: Vector2, width: float, labels: Array) -> OptionButton:
	var item := OptionButton.new()
	item.position = pos
	item.size = Vector2(width,31)
	item.add_theme_font_size_override("font_size",13)
	for text in labels:
		item.add_item(text)
	parent.add_child(item)
	return item

func set_policy() -> void:
	send("set_bin_policy",{"policy":["by_class","nearest","ui_select"][policy.selected],"selected_bin":"bin_a" if selected_bin.selected==0 else "bin_b"})

func show_commands() -> void:
	var dialog := AcceptDialog.new()
	dialog.title = "ROS 2 visualization"
	dialog.dialog_text = "Windows / Docker (native Godot)\ndocker compose up -d --build ros\nOptional RViz viewer: docker compose up -d --build rviz\nhttp://localhost:6080/vnc.html?autoconnect=true&resize=scale\n\nNative Linux (start ROS once)\nros2 launch urc_bringup twin.launch.py sim:=godot rviz:=true\n\nBridge: " + str(simulator.bridge.url)
	dialog.confirmed.connect(dialog.queue_free)
	dialog.canceled.connect(dialog.queue_free)
	add_child(dialog)
	dialog.popup_centered(Vector2i(680,210))

func send(method: String, args: Dictionary={}) -> void:
	if simulator.bridge.connected:
		simulator.ui_request(method,args)
	else:
		message_line.text = "Start the ROS 2 launch first."

func response(value: Variant) -> void:
	if value is Dictionary:
		var detail := str(value.get("result","")) if value.get("ok",false) else str(value.get("error","Rejected"))
		message_line.text = detail.left(52)
		message_line.tooltip_text = detail

func is_paused() -> bool:
	return bool(info.get("status",{}).get("paused",false))

func toggle_map() -> void:
	map_panel.visible = not map_panel.visible

func update(value: Dictionary) -> void:
	var first := info.is_empty()
	info = value
	var state: Dictionary = value.status
	update_buttons()
	status_line.text = str(state.state).replace("_"," ") + ("  /  HOLD" if state.paused else "")
	if state.state == "FAILED":
		message_line.text = str(state.reason).left(52)
		message_line.tooltip_text = str(state.reason)
	elif bool(state.estop_latched):
		message_line.text = "E-stop latched: click Clear, then Resume."
	progress.value = 100.0 * float(state.coverage_fraction)
	station_line.text = "Station %s   ·   %.0fs    Battery %.0f%%" % [str(value.quality),float(value.fix_age),100.*float(state.battery_fraction)]
	totals.text = "A  %d bottle / %d hammer    B  %d bottle / %d hammer\nPayload   %s" % [state.bin_a_bottles,state.bin_a_hammers,state.bin_b_bottles,state.bin_b_hammers,str(state.payload_class) if not str(state.payload_class).is_empty() else "empty"]
	detector.select(0 if state.detector=="oracle" else 1)
	var yolo_available := bool(value.get("yolo_available", false))
	detector.set_item_disabled(1, not yolo_available)
	detector.tooltip_text = "Choose Oracle or the trained YOLO model." if yolo_available else "YOLO needs trained bottle/hammer weights and a .classes.json sidecar. See docs/DOCKER_DESKTOP.md."
	policy.select(maxi(0,["by_class","nearest","ui_select"].find(state.bin_policy)))
	map_panel.update(value)
	overlays.boxes = value.get("boxes",[])
	overlays.queue_redraw()
	if first:
		map_panel.show()
		if not yolo_available and not bool(state.estop_latched) and state.state != "FAILED":
			message_line.text = "Oracle active. YOLO needs trained weights."

func update_buttons() -> void:
	var state: Dictionary = info.get("status", {})
	var idle := str(state.get("state", "IDLE")) == "IDLE"
	mission_button.text = "Start" if idle else ("Resume" if is_paused() or bool(state.get("manual", false)) or simulator.clock_paused else "Pause")
	mission_button.disabled = not simulator.bridge.connected or str(state.get("state", "")) == "FAILED"
	mission_button.tooltip_text = "Click Clear first to release E-stop." if bool(state.get("estop_latched", false)) else "Start, pause, or resume the mission."
	sim_button.text = "Resume sim" if simulator.clock_paused else "Pause sim"
	sim_button.disabled = not simulator.bridge.connected

func _process(_delta: float) -> void:
	if not is_instance_valid(simulator) or not is_instance_valid(station_line):
		return
	update_buttons()
	if not simulator.bridge.connected:
		info.clear()
		status_line.text = "WAITING FOR ROS 2"
		station_line.text = simulator.bridge.status_text
		if not overlays.boxes.is_empty():
			overlays.boxes = []
			overlays.queue_redraw()
