#!/usr/bin/env python3
"""
Map Manager ROS Node
====================
Handles SLAM mapping, map saving/loading, and room label management.
Commands are sent as JSON strings on /map_manager/command.
Status updates are published as JSON strings on /map_manager/status (latched).

Command reference:
  {"action": "start_mapping"}
  {"action": "stop_mapping"}
  {"action": "save_map", "name": "hotel_grand_floor1"}
  {"action": "list_maps"}
  {"action": "load_map", "name": "hotel_grand_floor1"}
  {"action": "save_label", "map": "hotel_grand_floor1", "name": "Bathroom", "x": 1.5, "y": 2.3, "theta": 0.0}
  {"action": "list_labels", "map": "hotel_grand_floor1"}
  {"action": "delete_label", "map": "hotel_grand_floor1", "name": "Bathroom"}
  {"action": "set_active_map", "name": "hotel_grand_floor1"}
  {"action": "get_active_map"}
  {"action": "get_robot_pose"}
  {"action": "delete_map", "name": "hotel_grand_floor1"}
"""

import rospy
import json
import os
import math
import subprocess
import yaml
import signal
import time
import threading

from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped
import tf

# ── Paths ─────────────────────────────────────────────────────────────────────
MAPS_DIR = os.path.expanduser(
    "/home/krish/catkin_ws/src/kinova-movo/movo_demos/maps"
)
WAYPOINTS_FILE = (
    "/home/krish/catkin_ws/src/kinova-movo/movo_nav/movo_waypoints.yaml"
)
ACTIVE_MAP_FILE = os.path.join(MAPS_DIR, ".active_map")


class MapManager:
    def __init__(self):
        rospy.init_node("map_manager")

        os.makedirs(MAPS_DIR, exist_ok=True)

        self.gmapping_proc    = None
        self.map_server_proc  = None
        self.nav_proc         = None   # map_nav.launch (map_server + amcl + move_base)
        self._active_nav_map  = None   # name of map currently being served by nav_proc
        self.tf_listener = tf.TransformListener()
        self._save_lock = threading.Lock()
        self._cached_map_msg = None   # latest OccupancyGrid while mapping
        self._map_cache_sub  = None   # subscriber active during mapping
        self._last_known_pose = None  # continuously updated from TF

        # Background thread: poll TF every 0.5s so we always have a recent pose
        self._pose_poll_thread = threading.Thread(target=self._poll_pose, daemon=True)
        self._pose_poll_thread.start()

        self.cmd_sub = rospy.Subscriber(
            "/map_manager/command", String, self.handle_command, queue_size=20
        )
        self.status_pub = rospy.Publisher(
            "/map_manager/status", String, queue_size=10, latch=True
        )
        # Publish robot pose as a proper ROS topic at 5 Hz so the UI can
        # subscribe to it directly — no tf2_web_republisher needed.
        self.pose_pub = rospy.Publisher(
            "/map_manager/robot_pose", PoseStamped, queue_size=5
        )
        self._pose_broadcast_thread = threading.Thread(target=self._broadcast_pose, daemon=True)
        self._pose_broadcast_thread.start()
        # Latched waypoints list — Navigation tab subscribes to this.
        # Published here so the UI always has the list even if goto_points.py
        # isn't running or hasn't started yet.
        self.waypoints_list_pub = rospy.Publisher(
            "/ui_waypoints_list", String, queue_size=1, latch=True
        )
        # Nav-ready flag — also published here so buttons enable even if
        # goto_points.py hasn't connected to move_base yet.
        self.nav_ready_pub = rospy.Publisher(
            "/ui_nav_ready", Bool, queue_size=1, latch=True
        )
        self.nav_ready_pub.publish(Bool(data=False))

        rospy.loginfo("Map Manager ready. Maps directory: %s", MAPS_DIR)
        self._publish({"type": "ready", "message": "Map Manager ready"})

        # Auto-start navigation if there is already an active map from a
        # previous session — so the user doesn't have to click anything.
        saved_active = self._read_active_map()
        if saved_active:
            rospy.loginfo("Auto-starting navigation for saved active map: %s", saved_active)
            # Publish waypoints list immediately (before nav comes up) so the
            # Navigation tab has buttons as soon as rosbridge connects.
            self._publish_waypoints_list(saved_active)
            threading.Thread(
                target=lambda: self.start_navigation(saved_active),
                daemon=True,
            ).start()

    # ── Core publish helper ───────────────────────────────────────────────────

    def _publish(self, data):
        self.status_pub.publish(String(data=json.dumps(data)))

    # ── Command router ────────────────────────────────────────────────────────

    def handle_command(self, msg):
        try:
            cmd = json.loads(msg.data)
        except json.JSONDecodeError as e:
            self._publish({"type": "error", "message": f"Invalid JSON: {e}"})
            return

        action = cmd.get("action", "")
        rospy.loginfo("MapManager command: %s", action)

        handlers = {
            "start_mapping":    lambda: self.start_mapping(),
            "stop_mapping":     lambda: self.stop_mapping(),
            "ready_to_save":    lambda: self.ready_to_save(),
            "save_map":         lambda: self.save_map(cmd.get("name", "new_map")),
            "list_maps":        lambda: self.list_maps(),
            "load_map":         lambda: self.start_navigation(cmd.get("name")),
            "start_navigation": lambda: self.start_navigation(cmd.get("name")),
            "stop_navigation":  lambda: self.stop_navigation(),
            "save_label":       lambda: self.save_label(cmd),
            "list_labels":      lambda: self.list_labels(cmd.get("map")),
            "delete_label":     lambda: self.delete_label(cmd.get("map"), cmd.get("name")),
            "set_active_map":   lambda: self.set_active_map(cmd.get("name")),
            "get_active_map":   lambda: self.get_active_map(),
            "get_robot_pose":   lambda: self.get_robot_pose(),
            "delete_map":       lambda: self.delete_map(cmd.get("name")),
        }

        # Long-running handlers must run in a daemon thread so the ROS
        # callback thread is never blocked (e.g. time.sleep inside start_navigation).
        THREADED_ACTIONS = {
            "start_mapping", "stop_mapping", "ready_to_save", "save_map",
            "load_map", "start_navigation", "stop_navigation", "set_active_map",
        }

        handler = handlers.get(action)
        if handler:
            if action in THREADED_ACTIONS:
                threading.Thread(target=handler, daemon=True).start()
            else:
                try:
                    handler()
                except Exception as e:
                    rospy.logerr("MapManager error in %s: %s", action, e)
                    self._publish({"type": "error", "message": str(e)})
        else:
            self._publish({"type": "error", "message": f"Unknown action: {action}"})

    # ── Mapping ───────────────────────────────────────────────────────────────

    def start_mapping(self):
        if self.gmapping_proc and self.gmapping_proc.poll() is None:
            self._publish({"type": "error", "message": "Mapping is already running."})
            return

        try:
            self.gmapping_proc = subprocess.Popen(
                ["roslaunch", "movo_demos", "mapping.launch", "local:=true"],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                preexec_fn=os.setsid,   # create a new process group so we can kill it
            )
            rospy.loginfo("Started gmapping (PID %s)", self.gmapping_proc.pid)

            # Subscribe to /map and keep caching every update.
            # This gives us the data even after the gmapping node is killed.
            self._cached_map_msg = None
            from nav_msgs.msg import OccupancyGrid as _OG
            self._map_cache_sub = rospy.Subscriber(
                "/map", _OG, self._cache_map, queue_size=1
            )

            self._publish({
                "type": "mapping_started",
                "message": (
                    "SLAM mapping started. Drive the robot around the entire "
                    "environment, covering all rooms and hallways."
                ),
            })
        except Exception as e:
            self._publish({"type": "error", "message": f"Failed to start mapping: {e}"})

    def stop_mapping(self):
        if self.gmapping_proc and self.gmapping_proc.poll() is None:
            try:
                os.killpg(os.getpgid(self.gmapping_proc.pid), signal.SIGTERM)
                self.gmapping_proc.wait(timeout=8)
            except (subprocess.TimeoutExpired, ProcessLookupError):
                try:
                    os.killpg(os.getpgid(self.gmapping_proc.pid), signal.SIGKILL)
                except ProcessLookupError:
                    pass
            self.gmapping_proc = None
            self._publish({
                "type": "mapping_stopped",
                "message": "Mapping stopped. You can now save the map.",
            })
        else:
            self._publish({"type": "info", "message": "No mapping process was running."})

    def ready_to_save(self):
        """Wizard soft-stop: signal the UI to advance to the save step WITHOUT
        killing gmapping.  The process keeps running so that map→base_link TF
        stays live through the label step.  gmapping is killed later by
        set_active_map() when the user finishes labeling."""
        self._publish({
            "type":    "mapping_stopped",
            "message": "Ready to save. gmapping still active for live position.",
        })

    def _cache_map(self, msg):
        """Callback: store the latest OccupancyGrid in memory."""
        self._cached_map_msg = msg

    # ── Map save / load ───────────────────────────────────────────────────────

    def save_map(self, name):
        name = _sanitize(name)
        if not name:
            self._publish({"type": "error", "message": "Map name cannot be empty."})
            return

        out_base = os.path.join(MAPS_DIR, name)

        def _do_save():
            try:
                from nav_msgs.msg import OccupancyGrid

                # Prefer the in-memory cache (populated while gmapping ran).
                # Fall back to wait_for_message in case map_server is publishing /map.
                map_msg = self._cached_map_msg
                if map_msg is None:
                    self._publish({"type": "info", "message": "No cached map — waiting for /map (10s)\u2026"})
                    rospy.loginfo("No cached map, trying wait_for_message (10s)\u2026")
                    map_msg = rospy.wait_for_message("/map", OccupancyGrid, timeout=10.0)
                else:
                    rospy.loginfo("Using cached map (%dx%d)", map_msg.info.width, map_msg.info.height)

                width      = map_msg.info.width
                height     = map_msg.info.height
                resolution = map_msg.info.resolution
                origin_x   = map_msg.info.origin.position.x
                origin_y   = map_msg.info.origin.position.y

                # ── Build PGM pixel data ─────────────────────────────────────
                # OccupancyGrid: 0=free, 100=occupied, -1=unknown
                # PGM pixel (negate=0): 254=free, 0=occupied, 205=unknown
                # Thresholds match map_server defaults (free<0.196, occ>=0.65)
                FREE_THRESH = 19.6   # 0.196 * 100
                OCC_THRESH  = 65.0   # 0.65  * 100

                pixels = bytearray(width * height)
                for i, cell in enumerate(map_msg.data):
                    row = i // width
                    col = i % width
                    # OccupancyGrid row 0 = bottom; PGM row 0 = top → flip
                    pgm_idx = (height - 1 - row) * width + col
                    if cell == -1 or (FREE_THRESH < cell < OCC_THRESH):
                        pixels[pgm_idx] = 205   # unknown / ambiguous → grey
                    elif cell <= FREE_THRESH:
                        pixels[pgm_idx] = 254   # free → near-white
                    else:
                        pixels[pgm_idx] = 0     # occupied → black

                # ── Write PGM (binary P5) ────────────────────────────────────
                pgm_path = out_base + ".pgm"
                with open(pgm_path, "wb") as f:
                    header = f"P5\n{width} {height}\n255\n"
                    f.write(header.encode("ascii"))
                    f.write(bytes(pixels))

                # ── Write YAML sidecar ───────────────────────────────────────
                yaml_path = out_base + ".yaml"
                map_meta = {
                    "image":           pgm_path,
                    "resolution":      float(resolution),
                    "origin":          [float(origin_x), float(origin_y), 0.0],
                    "negate":          0,
                    "occupied_thresh": 0.65,
                    "free_thresh":     0.196,
                }
                with open(yaml_path, "w") as f:
                    yaml.dump(map_meta, f, default_flow_style=False)

                rospy.loginfo("Map saved: %s (.pgm + .yaml)", out_base)
                
                # Start a map_server to serve the newly saved map on /map
                # so the labeling canvas shows the correct map (not stale gmapping)
                self._stop_nav_proc()  # stop any old map_server first
                try:
                    yaml_path = os.path.join(MAPS_DIR, name + ".yaml")
                    self.map_server_proc = subprocess.Popen(
                        ["rosrun", "map_server", "map_server", yaml_path],
                        stdout=subprocess.DEVNULL,
                        stderr=subprocess.DEVNULL,
                        preexec_fn=os.setsid,
                    )
                    rospy.loginfo("Started map_server for '%s' (PID %s)", name, self.map_server_proc.pid)
                    time.sleep(1.0)  # give map_server time to come up
                except Exception as e:
                    rospy.logwarn("Could not start map_server for labeling: %s", e)
                
                self._publish({
                    "type":    "map_saved",
                    "name":    name,
                    "message": f"Map '{name}' saved successfully.",
                })

            except rospy.ROSException:
                self._publish({
                    "type":    "error",
                    "message": (
                        "No map data available. "
                        "Start SLAM mapping first, drive the robot around, "
                        "then save — do not close the Map Manager node between steps."
                    ),
                })
            except Exception as e:
                rospy.logerr("save_map error: %s", e)
                self._publish({"type": "error", "message": f"Failed to save map: {e}"})

        threading.Thread(target=_do_save, daemon=True).start()


    def list_maps(self):
        maps = []
        try:
            for fname in sorted(os.listdir(MAPS_DIR)):
                if not fname.endswith(".yaml"):
                    continue
                if fname.endswith("_labels.yaml"):
                    continue
                if fname.startswith("."):
                    continue

                map_name = fname[:-5]
                pgm_path = os.path.join(MAPS_DIR, map_name + ".pgm")
                label_path = os.path.join(MAPS_DIR, map_name + "_labels.yaml")

                label_count = 0
                if os.path.exists(label_path):
                    try:
                        with open(label_path) as f:
                            data = yaml.safe_load(f) or {}
                        label_count = len(data.get("labels", {}))
                    except Exception:
                        pass

                maps.append({
                    "name": map_name,
                    "has_pgm": os.path.exists(pgm_path),
                    "label_count": label_count,
                })
        except Exception as e:
            self._publish({"type": "error", "message": str(e)})
            return

        self._publish({
            "type": "maps_list",
            "maps": maps,
            "active": self._read_active_map(),
        })

    def start_navigation(self, name):
        """Launch the full navigation stack (map_server + amcl + move_base)
        via map_nav.launch for the given map name."""
        if not name:
            self._publish({"type": "error", "message": "No map name provided."})
            return

        yaml_path = os.path.join(MAPS_DIR, name + ".yaml")
        if not os.path.exists(yaml_path):
            self._publish({"type": "error", "message": f"Map '{name}' not found."})
            return

        # Stop any previously running navigation launch
        self._stop_nav_proc()

        # Always keep the waypoints file in sync with this map's labels
        self._sync_waypoints(name)

        self._publish({"type": "info", "message": f"Starting navigation for map '{name}'…"})

        try:
            self.nav_proc = subprocess.Popen(
                [
                    "roslaunch", "movo_demos", "map_nav.launch",
                    "sim:=false",
                    "local:=true",
                    f"map_file:={name}",
                ],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                preexec_fn=os.setsid,
            )
            rospy.loginfo("Started map_nav.launch for map '%s' (PID %s)", name, self.nav_proc.pid)
            # Give the stack a few seconds to come up before telling the UI
            time.sleep(3.0)
            self.nav_ready_pub.publish(Bool(data=True))
            self._publish({
                "type":    "map_loaded",
                "name":    name,
                "message": f"Navigation started for map '{name}' (map_server + AMCL + move_base).",
            })
        except Exception as e:
            self._publish({"type": "error", "message": f"Failed to start navigation: {e}"})

    def stop_navigation(self):
        """Shut down the navigation stack launched by start_navigation."""
        self._stop_nav_proc()
        self._active_nav_map = None
        self.nav_ready_pub.publish(Bool(data=False))
        self._publish({"type": "info", "message": "Navigation stack stopped."})

    def _stop_nav_proc(self):
        """Internal helper — kill nav_proc and map_server_proc if running."""
        for attr in ("nav_proc", "map_server_proc"):
            proc = getattr(self, attr, None)
            if proc and proc.poll() is None:
                try:
                    os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
                    proc.wait(timeout=8)
                except (subprocess.TimeoutExpired, ProcessLookupError):
                    try:
                        os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
                    except ProcessLookupError:
                        pass
            setattr(self, attr, None)

    # kept for any callers that still use the old name
    def load_map(self, name):
        return self.start_navigation(name)

    def delete_map(self, name):
        if not name:
            self._publish({"type": "error", "message": "No map name provided."})
            return

        for ext in [".pgm", ".yaml", "_labels.yaml"]:
            p = os.path.join(MAPS_DIR, name + ext)
            if os.path.exists(p):
                os.remove(p)

        if self._read_active_map() == name:
            if os.path.exists(ACTIVE_MAP_FILE):
                os.remove(ACTIVE_MAP_FILE)

        self._publish({
            "type": "map_deleted",
            "name": name,
            "message": f"Map '{name}' deleted.",
        })

    # ── Room labels ───────────────────────────────────────────────────────────

    def save_label(self, cmd):
        map_name = cmd.get("map")
        label_name = cmd.get("name")
        x = float(cmd.get("x", 0.0))
        y = float(cmd.get("y", 0.0))
        theta = float(cmd.get("theta", 0.0))

        if not map_name or not label_name:
            self._publish({"type": "error", "message": "'map' and 'name' are required."})
            return

        label_path = os.path.join(MAPS_DIR, map_name + "_labels.yaml")

        with self._save_lock:
            data = {"labels": {}}
            if os.path.exists(label_path):
                try:
                    with open(label_path) as f:
                        data = yaml.safe_load(f) or {"labels": {}}
                except Exception:
                    pass

            qz = math.sin(theta / 2.0)
            qw = math.cos(theta / 2.0)

            data.setdefault("labels", {})[label_name] = {
                "position": [round(x, 4), round(y, 4), 0.0],
                "orientation": [0.0, 0.0, round(qz, 6), round(qw, 6)],
            }

            with open(label_path, "w") as f:
                yaml.dump(data, f, default_flow_style=False)

        # If this is the active map, keep waypoints file in sync
        if self._read_active_map() == map_name:
            self._sync_waypoints(map_name)

        self._publish({
            "type": "label_saved",
            "name": label_name,
            "map": map_name,
            "message": f"Label '{label_name}' saved on map '{map_name}'.",
        })

    def list_labels(self, map_name):
        if not map_name:
            self._publish({"type": "error", "message": "No map name provided."})
            return

        label_path = os.path.join(MAPS_DIR, map_name + "_labels.yaml")
        labels = []
        if os.path.exists(label_path):
            try:
                with open(label_path) as f:
                    data = yaml.safe_load(f) or {}
                for k, v in data.get("labels", {}).items():
                    pos = v.get("position", [0, 0, 0])
                    ori = v.get("orientation", [0, 0, 0, 1])
                    theta = 2 * math.atan2(ori[2], ori[3])
                    labels.append({
                        "name": k,
                        "x": pos[0],
                        "y": pos[1],
                        "theta": theta,
                    })
            except Exception as e:
                self._publish({"type": "error", "message": f"Could not read labels: {e}"})
                return

        self._publish({
            "type": "labels_list",
            "map": map_name,
            "labels": labels,
        })

    def delete_label(self, map_name, label_name):
        if not map_name or not label_name:
            self._publish({"type": "error", "message": "'map' and 'name' are required."})
            return

        label_path = os.path.join(MAPS_DIR, map_name + "_labels.yaml")
        if not os.path.exists(label_path):
            self._publish({"type": "error", "message": "No labels file found for this map."})
            return

        with self._save_lock:
            with open(label_path) as f:
                data = yaml.safe_load(f) or {"labels": {}}
            if label_name not in data.get("labels", {}):
                self._publish({"type": "error", "message": f"Label '{label_name}' not found."})
                return
            del data["labels"][label_name]
            with open(label_path, "w") as f:
                yaml.dump(data, f, default_flow_style=False)

        if self._read_active_map() == map_name:
            self._sync_waypoints(map_name)

        self._publish({
            "type": "label_deleted",
            "name": label_name,
            "map": map_name,
            "message": f"Label '{label_name}' deleted.",
        })

    # ── Active map ────────────────────────────────────────────────────────────

    def set_active_map(self, name):
        if not name:
            self._publish({"type": "error", "message": "No map name provided."})
            return

        yaml_path = os.path.join(MAPS_DIR, name + ".yaml")
        if not os.path.exists(yaml_path):
            self._publish({"type": "error", "message": f"Map '{name}' not found."})
            return

        # Stop gmapping if it is still running (e.g. the wizard kept it alive
        # through the label step so TF stayed live).  Do this before writing
        # the active-map file so the navigation stack gets a clean start.
        if self.gmapping_proc and self.gmapping_proc.poll() is None:
            try:
                os.killpg(os.getpgid(self.gmapping_proc.pid), signal.SIGTERM)
                self.gmapping_proc.wait(timeout=8)
            except (subprocess.TimeoutExpired, ProcessLookupError):
                try:
                    os.killpg(os.getpgid(self.gmapping_proc.pid), signal.SIGKILL)
                except ProcessLookupError:
                    pass
            self.gmapping_proc = None
            rospy.loginfo("set_active_map: stopped gmapping")

        with open(ACTIVE_MAP_FILE, "w") as f:
            f.write(name)

        self._sync_waypoints(name)

        # Start the full navigation stack for the new active map
        self.start_navigation(name)

        self._publish({
            "type": "active_map_set",
            "name": name,
            "message": f"'{name}' is now the active map — navigation started.",
        })

    def get_active_map(self):
        self._publish({
            "type": "active_map",
            "name": self._read_active_map(),
        })

    # ── Robot pose ────────────────────────────────────────────────────────────

    def get_robot_pose(self):
        # Try live TF. Use lookupTransform directly — canTransform + lookupTransform
        # has a race condition and both must use the same rospy.Time(0) semantics
        # ("give me the latest available transform", not "at current wall-clock time").
        try:
            (trans, rot) = self.tf_listener.lookupTransform(
                '/map', '/base_link', rospy.Time(0)
            )
            theta = 2 * math.atan2(rot[2], rot[3])
            pose = {
                'x': round(trans[0], 4),
                'y': round(trans[1], 4),
                'theta': round(theta, 4),
            }
            self._last_known_pose = pose
            self._publish({"type": "robot_pose", **pose})
            return
        except (tf.LookupException, tf.ConnectivityException,
                tf.ExtrapolationException, Exception):
            pass

        # Fall back to the last pose cached by the background poller
        if self._last_known_pose:
            self._publish({
                "type": "robot_pose",
                **self._last_known_pose,
                "stale": True,
            })
            return

        self._publish({
            "type": "error",
            "message": (
                "Cannot get robot position: no TF transform available yet. "
                "Make sure gmapping or AMCL is running, or use "
                "\"Click on map\" to place the label manually."
            ),
        })

    # ── Internal helpers ──────────────────────────────────────────────────────

    def _poll_pose(self):
        """Background thread: keep _last_known_pose updated from TF."""
        # Give the TF buffer a moment to receive its first transforms
        rospy.sleep(2.0)
        rate = rospy.Rate(2)  # 2 Hz
        while not rospy.is_shutdown():
            try:
                (trans, rot) = self.tf_listener.lookupTransform(
                    '/map', '/base_link', rospy.Time(0)
                )
                self._last_known_pose = {
                    'x': round(trans[0], 4),
                    'y': round(trans[1], 4),
                    'theta': round(2 * math.atan2(rot[2], rot[3]), 4),
                }
            except (tf.LookupException, tf.ConnectivityException,
                    tf.ExtrapolationException):
                pass  # transform not yet available — try again next tick
            except Exception:
                pass
            rate.sleep()

    def _broadcast_pose(self):
        """Publish latest pose as PoseStamped at 5 Hz for the UI canvas."""
        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            pose = self._last_known_pose
            if pose:
                msg = PoseStamped()
                msg.header.stamp = rospy.Time.now()
                msg.header.frame_id = 'map'
                msg.pose.position.x = pose['x']
                msg.pose.position.y = pose['y']
                msg.pose.position.z = 0.0
                # Convert yaw back to quaternion
                msg.pose.orientation.z = math.sin(pose['theta'] / 2.0)
                msg.pose.orientation.w = math.cos(pose['theta'] / 2.0)
                self.pose_pub.publish(msg)
            rate.sleep()

    def _read_active_map(self):
        if os.path.exists(ACTIVE_MAP_FILE):
            with open(ACTIVE_MAP_FILE) as f:
                return f.read().strip() or None
        return None

    def _sync_waypoints(self, map_name):
        """Copy the room labels for map_name into movo_waypoints.yaml."""
        label_path = os.path.join(MAPS_DIR, map_name + "_labels.yaml")
        waypoints = {"waypoints": {}}

        if os.path.exists(label_path):
            try:
                with open(label_path) as f:
                    data = yaml.safe_load(f) or {}
                for k, v in data.get("labels", {}).items():
                    waypoints["waypoints"][k] = {
                        "position": v["position"],
                        "orientation": v["orientation"],
                    }
            except Exception as e:
                rospy.logwarn("Could not sync waypoints: %s", e)

        with open(WAYPOINTS_FILE, "w") as f:
            yaml.dump(waypoints, f, default_flow_style=False)

        rospy.loginfo(
            "Synced %d waypoints from '%s' to %s",
            len(waypoints["waypoints"]), map_name, WAYPOINTS_FILE
        )
        self._publish_waypoints_list(map_name)

    def _publish_waypoints_list(self, map_name):
        """Publish the label names for map_name as a latched JSON list on
        /ui_waypoints_list so the Navigation tab always has the buttons."""
        label_path = os.path.join(MAPS_DIR, map_name + "_labels.yaml")
        names = []
        if os.path.exists(label_path):
            try:
                with open(label_path) as f:
                    data = yaml.safe_load(f) or {}
                names = list(data.get("labels", {}).keys())
            except Exception:
                pass
        self.waypoints_list_pub.publish(String(data=json.dumps(names)))
        rospy.loginfo("Published waypoints list for '%s': %s", map_name, names)

    def shutdown(self):
        """Clean up subprocesses on node shutdown."""
        for proc in [self.gmapping_proc, self.map_server_proc]:
            if proc and proc.poll() is None:
                try:
                    os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
                except Exception:
                    pass


# ── Helpers ───────────────────────────────────────────────────────────────────

def _sanitize(name):
    """Lower-case, replace spaces/slashes with underscores, strip dots."""
    import re
    return re.sub(r"[^\w-]", "_", name.strip()).lower()


# ── Entry point ───────────────────────────────────────────────────────────────

if __name__ == "__main__":
    mgr = MapManager()
    rospy.on_shutdown(mgr.shutdown)
    rospy.spin()
