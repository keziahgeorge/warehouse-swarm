"""
Warehouse Swarm Simulation Visualizer (Live Interactive Tkinter GUI)

Patched:
  - Episode-aware: RWARE resets the world every few hundred steps. The GUI
    now shows the episode number / step-in-episode and resets coordinator
    state on every reset.
  - Diagnostics: shows how many live robots are idle (NOOP) and how many
    are carrying a shelf, so you can tell "stuck" from "working".
"""

import sys
import os
from pathlib import Path
import time
import tkinter as tk
from tkinter import ttk

# Ensure core and project root are on sys.path
_ROOT_DIR = Path(__file__).resolve().parent.parent
_CORE_DIR = _ROOT_DIR / "core"
_EXP_DIR = _ROOT_DIR / "experiments"
for _p in [str(_CORE_DIR), str(_EXP_DIR), str(_ROOT_DIR)]:
    if _p not in sys.path:
        sys.path.insert(0, _p)

import numpy as np
from multi_robot_vecenv import MultiRobotVecEnv
from zone_partitioner import ZonePartitioner
from hierarchical_coordinator import HierarchicalCoordinator


class WarehouseGUI:
    # Color palette
    ZONE_COLORS = ["#f8f9fa", "#f1f3f5", "#e9ecef", "#dee2e6"]
    ZONE_BORDER = "#adb5bd"

    ROBOT_COLORS = {
        0: "#228be6",  # fast_light -> Blue
        1: "#fa5252",  # heavy_load -> Red
        2: "#40c057",  # balanced   -> Green
    }
    FAILED_ROBOT_COLOR = "#495057"

    TASK_COLORS = {
        0: "#15aabf",
        1: "#e64980",
        2: "#fab005",
    }

    CELL_SIZE = 36  # pixels per grid cell

    def __init__(self, root, env_id="rware-small-16ag-v2", n_zones=4, seed=42):
        self.root = root
        self.root.title("Warehouse Swarm — Hierarchical Zone-Based Multi-Robot Coordination")
        self.root.configure(bg="#212529")

        self.running = False
        self.step_delay = 80  # ms between steps
        self.step_count = 0
        self.episode = 1
        self.episode_steps = 0

        self.n_robots = 16
        self.robot_types = [0, 1, 2, 0, 1, 2, 0, 1, 2, 0, 1, 2, 0, 1, 2, 0]
        self.last_actions = [0] * self.n_robots

        self.vec_env = MultiRobotVecEnv(
            env_id=env_id, robot_types=self.robot_types, seed=seed
        )
        self.obs = self.vec_env.reset()
        self.env = self.vec_env.hetero_env
        self.ua = self.env.unwrapped

        # RWARE grid_size is (rows, cols) -> (height, width)
        self.grid_h, self.grid_w = self.ua.grid_size[0], self.ua.grid_size[1]

        self.partitioner = ZonePartitioner(self.grid_w, self.grid_h, n_zones=n_zones)
        self.zone_assignment = self.partitioner.assign_robots_to_zones(self.ua.agents)
        self.partitioner.report_robot_assignment(self.zone_assignment)
        self.coordinator = HierarchicalCoordinator(
            self.env, self.partitioner, self.zone_assignment, bid_model_path="bid_head_trained.pt"
        )

        self._build_ui()
        self._draw_grid_base()
        self._update_display()

        self.root.update_idletasks()
        self.root.geometry("")

    def _build_ui(self):
        header = tk.Frame(self.root, bg="#343a40", padx=16, pady=10)
        header.pack(fill=tk.X)

        title = tk.Label(
            header,
            text="Autonomous Warehouse Swarm — Hierarchical Coordination Simulation",
            font=("Segoe UI", 15, "bold"),
            fg="#f8f9fa",
            bg="#343a40",
        )
        title.pack(side=tk.LEFT)

        subtitle = tk.Label(
            header,
            text="16 Robots | 4 Spatial Zones | Decentralized Auctions + A*",
            font=("Segoe UI", 10),
            fg="#adb5bd",
            bg="#343a40",
        )
        subtitle.pack(side=tk.RIGHT)

        body = tk.Frame(self.root, bg="#212529", padx=12, pady=12)
        body.pack(fill=tk.BOTH, expand=True)

        canvas_frame = tk.Frame(body, bg="#111", relief=tk.RIDGE, bd=2)
        canvas_frame.pack(side=tk.LEFT, padx=(0, 12), fill=tk.BOTH, expand=True)

        c_w = self.grid_w * self.CELL_SIZE
        c_h = self.grid_h * self.CELL_SIZE
        self.canvas = tk.Canvas(
            canvas_frame, width=c_w, height=c_h, bg="#ffffff", highlightthickness=0
        )
        self.canvas.pack(padx=8, pady=8)

        sidebar = tk.Frame(body, bg="#2b3035", width=340, padx=14, pady=10)
        sidebar.pack(side=tk.RIGHT, fill=tk.Y)

        # Controls
        ctrl_frame = tk.LabelFrame(
            sidebar, text=" Controls ", font=("Segoe UI", 10, "bold"),
            fg="#ced4da", bg="#2b3035", padx=10, pady=8
        )
        ctrl_frame.pack(fill=tk.X, pady=(0, 10))

        self.btn_run = tk.Button(
            ctrl_frame, text="▶ Start Live Sim", bg="#2b8a3e", fg="white",
            font=("Segoe UI", 10, "bold"), relief=tk.FLAT, padx=10, pady=6,
            command=self.toggle_run
        )
        self.btn_run.pack(side=tk.LEFT, expand=True, fill=tk.X, padx=2)

        self.btn_step = tk.Button(
            ctrl_frame, text="Step ❯", bg="#495057", fg="white",
            font=("Segoe UI", 10), relief=tk.FLAT, padx=8, pady=6,
            command=self.step_simulation
        )
        self.btn_step.pack(side=tk.LEFT, expand=True, fill=tk.X, padx=2)

        speed_frame = tk.Frame(sidebar, bg="#2b3035")
        speed_frame.pack(fill=tk.X, pady=(0, 10))
        tk.Label(speed_frame, text="Speed:", bg="#2b3035", fg="#ced4da", font=("Segoe UI", 9)).pack(side=tk.LEFT)
        self.speed_slider = tk.Scale(
            speed_frame, from_=20, to=250, orient=tk.HORIZONTAL,
            bg="#2b3035", fg="#ced4da", highlightthickness=0,
            command=lambda v: setattr(self, 'step_delay', int(v))
        )
        self.speed_slider.set(self.step_delay)
        self.speed_slider.pack(side=tk.RIGHT, fill=tk.X, expand=True)

        # Metrics
        stats_box = tk.LabelFrame(
            sidebar, text=" Swarm Metrics ", font=("Segoe UI", 10, "bold"),
            fg="#ced4da", bg="#2b3035", padx=10, pady=8
        )
        stats_box.pack(fill=tk.X, pady=(0, 10))

        self.lbl_step = tk.Label(stats_box, text="Step: 0", font=("Segoe UI", 10), bg="#2b3035", fg="#f8f9fa", anchor="w")
        self.lbl_step.pack(fill=tk.X)

        self.lbl_episode = tk.Label(stats_box, text="Episode: 1 (step 0)", font=("Segoe UI", 10), bg="#2b3035", fg="#f8f9fa", anchor="w")
        self.lbl_episode.pack(fill=tk.X)

        self.lbl_deliveries = tk.Label(stats_box, text="Deliveries: 0", font=("Segoe UI", 10, "bold"), bg="#2b3035", fg="#69db7c", anchor="w")
        self.lbl_deliveries.pack(fill=tk.X)

        self.lbl_match = tk.Label(stats_box, text="Capability Match Rate: 100.0%", font=("Segoe UI", 10), bg="#2b3035", fg="#74c0fc", anchor="w")
        self.lbl_match.pack(fill=tk.X)

        self.lbl_handoffs = tk.Label(stats_box, text="Cross-Zone Handoffs: 0", font=("Segoe UI", 10), bg="#2b3035", fg="#ffd43b", anchor="w")
        self.lbl_handoffs.pack(fill=tk.X)

        self.lbl_comm = tk.Label(stats_box, text="Comm Events/Robot/Step: 0.00", font=("Segoe UI", 10), bg="#2b3035", fg="#ced4da", anchor="w")
        self.lbl_comm.pack(fill=tk.X)

        self.lbl_idle = tk.Label(stats_box, text="Idle robots (NOOP): 0/16", font=("Segoe UI", 10), bg="#2b3035", fg="#ced4da", anchor="w")
        self.lbl_idle.pack(fill=tk.X)

        self.lbl_carry = tk.Label(stats_box, text="Carrying a shelf: 0", font=("Segoe UI", 10), bg="#2b3035", fg="#ced4da", anchor="w")
        self.lbl_carry.pack(fill=tk.X)

        self.lbl_stranded = tk.Label(stats_box, text="Stranded Tasks (Awaiting Rescue): 0", font=("Segoe UI", 10), bg="#2b3035", fg="#ff8787", anchor="w")
        self.lbl_stranded.pack(fill=tk.X)

        # Fault injection
        fault_box = tk.LabelFrame(
            sidebar, text=" Fault Injection (Test Resilience) ",
            font=("Segoe UI", 10, "bold"), fg="#ff8787", bg="#2b3035", padx=10, pady=8
        )
        fault_box.pack(fill=tk.X, pady=(0, 10))

        fault_select_frame = tk.Frame(fault_box, bg="#2b3035")
        fault_select_frame.pack(fill=tk.X, pady=4)
        tk.Label(fault_select_frame, text="Select Robot:", bg="#2b3035", fg="#ced4da", font=("Segoe UI", 9)).pack(side=tk.LEFT)

        self.fault_robot_var = tk.StringVar(value="Robot 1")
        options = [f"Robot {i} ({self.env.TYPE_NAMES[self.robot_types[i]]})" for i in range(self.n_robots)]
        self.combo_fault = ttk.Combobox(
            fault_select_frame, textvariable=self.fault_robot_var, values=options, state="readonly", width=18
        )
        self.combo_fault.pack(side=tk.RIGHT)

        self.btn_fail = tk.Button(
            fault_box, text="⚡ Inject Failure", bg="#e03131", fg="white",
            font=("Segoe UI", 9, "bold"), relief=tk.FLAT, pady=4,
            command=self.inject_selected_failure
        )
        self.btn_fail.pack(fill=tk.X, pady=(4, 0))

        # Legend
        legend_box = tk.LabelFrame(
            sidebar, text=" Legend ", font=("Segoe UI", 10, "bold"),
            fg="#ced4da", bg="#2b3035", padx=10, pady=8
        )
        legend_box.pack(fill=tk.X)

        self._add_legend_item(legend_box, "#228be6", "Robot: fast_light")
        self._add_legend_item(legend_box, "#fa5252", "Robot: heavy_load")
        self._add_legend_item(legend_box, "#40c057", "Robot: balanced")
        self._add_legend_item(legend_box, "#495057", "Robot: Failed / Deactivated")
        self._add_legend_item(legend_box, "#fab005", "Gold: Requested Task Shelf (U/H/S)")
        self._add_legend_item(legend_box, "#20c997", "Green Base: Delivery Goal")

    def _add_legend_item(self, parent, color, text):
        row = tk.Frame(parent, bg="#2b3035")
        row.pack(fill=tk.X, pady=2)
        box = tk.Canvas(row, width=14, height=14, bg=color, highlightthickness=0)
        box.pack(side=tk.LEFT, padx=(0, 6))
        lbl = tk.Label(row, text=text, font=("Segoe UI", 8), bg="#2b3035", fg="#ced4da")
        lbl.pack(side=tk.LEFT)

    def _draw_grid_base(self):
        self.canvas.delete("base")
        c = self.CELL_SIZE

        for z in range(self.partitioner.n_zones):
            col = z // self.partitioner.n_rows
            row = z % self.partitioner.n_rows
            x1 = col * self.partitioner.zone_col_width * c
            y1 = row * self.partitioner.zone_row_height * c
            x2 = (col + 1) * self.partitioner.zone_col_width * c
            y2 = (row + 1) * self.partitioner.zone_row_height * c

            bg_color = self.ZONE_COLORS[z % len(self.ZONE_COLORS)]
            self.canvas.create_rectangle(
                x1, y1, x2, y2, fill=bg_color, outline=self.ZONE_BORDER, width=2, tags="base"
            )
            self.canvas.create_text(
                x1 + 45, y1 + 18, text=f"ZONE {z}", fill="#ced4da",
                font=("Segoe UI", 11, "bold"), tags="base"
            )

        for gx, gy in self.ua.goals:
            self.canvas.create_rectangle(
                gx * c, gy * c, (gx + 1) * c, (gy + 1) * c,
                fill="#8ce99a", outline="#37b24d", width=2, tags="base"
            )

    def _update_display(self):
        self.canvas.delete("dynamic")
        c = self.CELL_SIZE

        # 1. Shelves
        active_shelves = set(self.ua.request_queue)
        for shelf in self.ua.shelfs:
            sx, sy = shelf.x, shelf.y
            is_requested = shelf in active_shelves
            fill_c = "#ffd43b" if is_requested else "#adb5bd"
            outline_c = "#e67700" if is_requested else "#868e96"

            self.canvas.create_rectangle(
                sx * c + 5, sy * c + 5, (sx + 1) * c - 5, (sy + 1) * c - 5,
                fill=fill_c, outline=outline_c, width=1.5, tags="dynamic"
            )
            if is_requested:
                task_t = self.env.shelf_task_type.get(shelf, 2)
                task_char = ["U", "H", "S"][task_t]
                self.canvas.create_text(
                    sx * c + c / 2, sy * c + c / 2, text=task_char,
                    fill="#343a40", font=("Segoe UI", 8, "bold"), tags="dynamic"
                )

        # 2. Robots
        failed_flags = self.coordinator.fault_layers[0].actual_failed
        n_carrying = 0
        for i, agent in enumerate(self.ua.agents):
            rx, ry = agent.x, agent.y
            r_type = self.robot_types[i]
            is_failed = failed_flags[i]

            robot_col = self.FAILED_ROBOT_COLOR if is_failed else self.ROBOT_COLORS[r_type]
            outline_col = "#ff6b6b" if (is_failed and agent.carrying_shelf is not None) else "#212529"
            outline_w = 3 if (is_failed and agent.carrying_shelf is not None) else 2

            self.canvas.create_oval(
                rx * c + 3, ry * c + 3, (rx + 1) * c - 3, (ry + 1) * c - 3,
                fill=robot_col, outline=outline_col, width=outline_w, tags="dynamic"
            )

            if agent.carrying_shelf is not None:
                n_carrying += 1
                self.canvas.create_oval(
                    rx * c + 8, ry * c + 8, (rx + 1) * c - 8, (ry + 1) * c - 8,
                    fill="#ffe066", outline="#f08c00", width=1.5, tags="dynamic"
                )

            self.canvas.create_text(
                rx * c + c / 2, ry * c + c / 2, text=str(i),
                fill="white" if not agent.carrying_shelf else "#212529",
                font=("Segoe UI", 9, "bold"), tags="dynamic"
            )

        # 3. Sidebar
        tot_d = self.coordinator.total_match_count() + self.coordinator.total_mismatch_count()
        match_rate = (
            (self.coordinator.total_match_count() / tot_d * 100) if tot_d > 0 else 100.0
        )
        n_idle = sum(
            1 for i, a in enumerate(self.last_actions)
            if a == 0 and not failed_flags[i]
        )
        n_alive = sum(1 for f in failed_flags if not f)
        n_stranded = len(self.coordinator.stranded_tasks)

        self.lbl_step.config(text=f"Step: {self.step_count:,}")
        self.lbl_episode.config(text=f"Episode: {self.episode} (step {self.episode_steps})")
        self.lbl_deliveries.config(text=f"Deliveries: {tot_d}")
        self.lbl_match.config(text=f"Capability Match Rate: {match_rate:.1f}%")
        self.lbl_handoffs.config(text=f"Cross-Zone Handoffs: {self.coordinator.cross_zone_handoffs}")
        self.lbl_comm.config(
            text=f"Comm Events/Robot/Step: {self.coordinator.avg_comm_per_robot_per_step():.3f}"
        )
        self.lbl_idle.config(text=f"Idle robots (NOOP): {n_idle}/{n_alive}")
        self.lbl_carry.config(text=f"Carrying a shelf: {n_carrying}")
        self.lbl_stranded.config(text=f"Stranded Tasks (Awaiting Rescue): {n_stranded}")

    def step_simulation(self):
        try:
            pursue, bids, nearest_tasks = self.coordinator.step(self.obs)
            actions = self.coordinator.pathfinder.get_actions_for_pursuit(
                self.obs, pursue, nearest_tasks
            )
            actions = self.coordinator.get_fault_action_overrides(actions)
            self.last_actions = list(actions)

            self.obs, rewards, dones, infos = self.vec_env.step(actions)
            self.step_count += 1
            self.episode_steps += 1

            if np.any(dones):
                # RWARE ended the episode and auto-reset the world
                self.episode += 1
                self.episode_steps = 0
                self.coordinator.reset_episode()

            self.coordinator.clear_stale_beliefs()
        except Exception as e:
            print(f"[GUI] Step error at step {self.step_count}: {e}")
            import traceback; traceback.print_exc()
            self.running = False
            self.btn_run.config(text="▶ Start Live Sim", bg="#2b8a3e")
            return

        self._update_display()

    def _loop(self):
        if self.running:
            self.step_simulation()
            self.root.after(self.step_delay, self._loop)

    def toggle_run(self):
        self.running = not self.running
        if self.running:
            self.btn_run.config(text="⏸ Pause", bg="#f59f00")
            self._loop()
        else:
            self.btn_run.config(text="▶ Start Live Sim", bg="#2b8a3e")

    def inject_selected_failure(self):
        selection = self.combo_fault.get()
        if selection:
            robot_id = int(selection.split()[1])
            self.coordinator.inject_failure(robot_id)
            print(f"[UI] Injected physical failure into Robot {robot_id}")
            self._update_display()


def main():
    root = tk.Tk()
    app = WarehouseGUI(root)
    root.mainloop()


if __name__ == "__main__":
    main()