extends SceneTree

const Dashboard = preload("res://scripts/dashboard.gd")

class FakeBridge extends RefCounted:
	var connected := true
	var url := "ws://127.0.0.1:9090"
	var status_text := "CONNECTED"

class FakeHost extends Node:
	var bridge := FakeBridge.new()
	var clock_paused := false
	var seed_value := 42
	var epoch := 1
	var requests: Array[String] = []
	func panel(parent: Node, at: Vector2, extent: Vector2) -> Panel:
		var item := Panel.new()
		item.position = at
		item.size = extent
		parent.add_child(item)
		return item
	func label(parent: Node, value: String, at: Vector2, _px: int, _color: Color) -> Label:
		var item := Label.new()
		item.text = value
		item.position = at
		parent.add_child(item)
		return item
	func ui_request(method: String, _args: Dictionary = {}) -> void:
		requests.append(method)

var failures: Array[String] = []

func check(value: bool, detail: String) -> void:
	if not value:
		failures.append(detail)
		push_error(detail)

func _initialize() -> void:
	run.call_deferred()

func run() -> void:
	var host := FakeHost.new()
	root.add_child(host)
	var dashboard := Dashboard.new()
	root.add_child(dashboard)
	dashboard.setup(host)
	dashboard.info = {"status":{"state":"IDLE", "paused":false, "manual":false, "estop_latched":false}}
	dashboard.update_buttons()
	check(dashboard.mission_button.text == "Start", "Idle mission must show Start")
	dashboard.mission_button.pressed.emit()
	check(host.requests.back() == "start_mission", "Start must start a mission")
	dashboard.info.status.state = "TRANSPORT"
	dashboard.update_buttons()
	check(dashboard.mission_button.text == "Pause", "Active mission must show Pause")
	dashboard.mission_button.pressed.emit()
	check(host.requests.back() == "pause_mission", "Pause must hold the active mission")
	dashboard.info.status.paused = true
	dashboard.update_buttons()
	check(dashboard.mission_button.text == "Resume", "Held mission must show Resume")
	dashboard.mission_button.pressed.emit()
	check(host.requests.back() == "resume_mission", "Resume must not start a second mission")
	dashboard.info.status.estop_latched = true
	var sent := host.requests.size()
	dashboard.mission_button.pressed.emit()
	check(host.requests.size() == sent and "Clear" in dashboard.message_line.text, "Resume must explain the E-stop latch")
	dashboard.info.status.estop_latched = false
	host.clock_paused = true
	dashboard.update_buttons()
	check(dashboard.sim_button.text == "Resume sim", "Frozen simulation must show Resume sim")
	dashboard.sim_button.pressed.emit()
	check(host.requests.back() == "resume_sim", "Resume sim must use the actual clock state")
	check(dashboard.mission_button.focus_mode == Control.FOCUS_NONE, "Mission button must not consume Space")
	check(dashboard.sim_button.focus_mode == Control.FOCUS_NONE, "Simulation button must not consume Space")
	host.bridge.connected = false
	dashboard.update_buttons()
	check(dashboard.mission_button.disabled and dashboard.sim_button.disabled, "Disconnected controls must be disabled")
	if failures.is_empty():
		print("PASS: dashboard Start/Pause/Resume, E-stop guidance, simulation resume, and shortcut focus")
	quit(0 if failures.is_empty() else 1)
