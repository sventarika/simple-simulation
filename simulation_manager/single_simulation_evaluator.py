from __future__ import annotations

import json
import math
import matplotlib.pyplot as plt
import numpy as np

from loguru import logger
from matplotlib import patches
from typing import TYPE_CHECKING
from reportlab.lib.pagesizes import A4
from reportlab.pdfgen import canvas
from reportlab.platypus import Table, TableStyle
from reportlab.lib import colors

if TYPE_CHECKING:
    from pathlib import Path

from simple_scenario.rendering import create_scenario_plot_ax
from simple_scenario.scenario import Scenario


class SingleSimulationEvaluator:

    def __init__(self, scenario: str | Path | dict | Scenario, scenario_description: dict, test_result: dict, result_dir: str) -> None:

        # Init scenario
        self._scenario = Scenario.from_x(scenario)
        self._scenario_name = self._scenario.id
        self._lanelet_wrapper = self._scenario.get_cr_interface().lanelet_network_wrapper

        # Read scenario description (TODO: Replace by ChallengeDescription)
        self._scenario_speed_limit = self._scenario.road.speed_limit  # TODO: Read from ChallengeDescription (isn"t it a limit?)
        self._scenario_limits = scenario_description["scenario_limits"]
        self._operation_mode = self._scenario_limits["category"]
        self._operation_mode_mapping = {
            "physical_limits_timeout_possible": "1",
            "physical_limits_goal_possible": "1",
            "emergency_operation_goal_possible": "1.1",
            "emergency_operation_timeout_possible": "1.1",
            "normal_operation_goal_possible": "1.2",
            "normal_operation_timeout_possible": "1.2",
            "collision": "2"
        }
        self._expected_termination_reason = scenario_description["expected_termination_reason"]
        self._operation_mode_index = self._operation_mode_mapping[self._operation_mode]
        self._collision_speed = -1

        # Read test_result
        self._ego_states = test_result["ego_states"]
        self._actual_termination_reason = test_result["termination_reason"]

        # Set up result dir
        self._result_dir = result_dir
        self._result_dir.mkdir(exist_ok=True)
        self._report_dir = self._result_dir

        # Limits + 1 per cent tolerance
        self._a_max_long = 4*1.01 if self._operation_mode_index == "1.2" else 6*1.01 # m/s^2
        self._a_max_lat = 1*1.01 if self._operation_mode_index == "1.2" else 2*1.01 # m/s^2
        self._thw_min = 1.8*1.01 if self._operation_mode_index == "1.2" else 0.9*1.01 # s
        self._v_max = self._scenario_speed_limit*1.01 # km/h
        self._reaction_time = 1 # s

        # Initialize all used lists
        self._a_long_list = []
        self._a_lat_list = []
        self._t = []
        self._v_long_ego_list = []
        self._v_long_lead_list = []
        self._v_lat_list = []
        self._ego_s_list = []
        self._ego_t_list = []
        self._thw_list = []
        self._ttc_list = []
        self._ttb_list = []
        self._tts_list = []


    def get_scenario_id(self) -> str:
        """Return the value of scenario_id."""
        return self._scenario.id

    def get_operation_mode_index(self) -> str:
        """Return the value of self._operation_mode_index."""
        return self._operation_mode_index

    # Method to generate plots used for the test report
    def generate_plot(self, x: float | None=None, y1: float | None=None, y2: float | None=None, xlabel: str="", y1label: str="", y2label: str="", lower_limit: float=0, upper_limit: float=1, filename: str="file.png", *mark) -> None:
        if y2:
            f, ax = plt.subplots(figsize=(10, 6))
            ax.set_xlabel(xlabel)
            ax.set_ylabel(y1label)
            ax.set_xlim(0, self._scenario.duration)
            ax.set_ylim(lower_limit, upper_limit)
            ax.plot(x, y1, marker="x", label=y1label)
            ax.legend(loc="upper left", bbox_to_anchor=(0.8, 1.1))
            ax.grid()
            ax2 = ax.twinx()
            ax2.set_ylabel(y2label)
            ax2.set_ylim(lower_limit, upper_limit)
            ax2.plot(x, y2, marker="o", color="red", label=y2label)
            ax2.legend(loc="upper left", bbox_to_anchor=(0.6, 1.1))
            f.savefig(self._result_dir / f"{self._scenario_name}_{filename}", format="png", dpi=300)
            plt.close(f)
        elif y1:
            f, ax = plt.subplots(figsize=(10, 6))
            ax.set_xlabel(xlabel)
            ax.set_ylabel(y1label)
            ax.set_xlim(0, self._scenario.duration)
            ax.set_ylim(lower_limit, upper_limit)
            ax.plot(x, y1, marker="x")
            ax.grid()
            for m in mark:
                rect = patches.Rectangle((0, m[0]), width=self._scenario.duration, height=m[1], linewidth=1, edgecolor="none", facecolor=m[2], alpha=0.4)
                ax.add_patch(rect)
            f.savefig(self._result_dir / f"{self._scenario_name}_{filename}", format="png", dpi=300)
            plt.close(f)
        else:
            f, ax = plt.subplots(figsize=(10,2))
            ax.axhline(0, color="black", linewidth=2)
            ax.set_xlim(lower_limit, upper_limit)
            for m in mark:
                ax.scatter(m[0], 0, color=m[2], marker="x", s=100)
                if m[2] == "green":
                    ax.text(m[0], 0.022, m[1], horizontalalignment="center", color=m[2])
                else:
                    ax.text(m[0], 0.01, m[1], horizontalalignment="center", color=m[2])
            ax.get_yaxis().set_visible(False)
            ax.spines["left"].set_color("none")
            ax.spines["right"].set_color("none")
            ax.spines["top"].set_color("none")
            ax.spines["bottom"].set_position(("data", 0))
            ax.set_xlabel(xlabel)
            f.savefig(self._result_dir / f"{self._scenario_name}_{filename}", format="png", dpi=300)

    # Method to calculate all needed metrics
    def calc_metrics(self) -> None:

        time_limit = self._scenario.duration

        # Actual time in the test (is less equal than scenario time)
        n_test_time_steps = len(self._ego_states)
        dt = self._scenario.dt
        self._t = dt * np.arange(n_test_time_steps)

        self._a_long_list = [ego_state["action"][1] for ego_state in self._ego_states]
        self._v_long_ego_list = [ego_state["v"] for ego_state in self._ego_states]

        # Calculate ego_s & TTC & TTB & TTS & THW
        for time_step, ego_state in enumerate(self._ego_states):
            dhw = np.inf
            thw = np.inf
            ttc = np.inf
            ttb = np.inf
            tts = np.inf

            # Replicate observation
            other_obj_obs = {}
            for vehicle in self._scenario.vehicles:
                other_obj_obs[vehicle.id] = {
                    "x": vehicle.x[time_step],
                    "y": vehicle.y[time_step],
                    "v": vehicle.v[time_step],
                    "length": vehicle.length,
                    "width": vehicle.width
                }

            # Find lead vehicle
            ego_llt_id = self._lanelet_wrapper.find_lanelet_id_by_position(ego_state["x"], ego_state["y"])
            surrounding_vehicles = self._lanelet_wrapper.find_surrounding_vehicles(ego_state["x"], ego_state["y"], other_obj_obs)
            lead_vehicle = surrounding_vehicles.lead

            # Append ego_s to list
            ego_s, ego_t = self._lanelet_wrapper.from_cart_to_llt_frenet(ego_llt_id, ego_state["x"], ego_state["y"])
            self._ego_s_list.append(ego_s)
            self._ego_t_list.append(ego_t)

            # ego width
            ego_width =  ego_state["width"]

            # Calclulate TTC & TTB & TTS for whole scenario
            if lead_vehicle:
                self._v_long_lead_list.append(lead_vehicle["v"])

                # lead width
                lead_width = lead_vehicle["width"]

                lead_s, lead_t = self._lanelet_wrapper.from_cart_to_llt_frenet(ego_llt_id, lead_vehicle["x"], lead_vehicle["y"])

                # Ego front bumper and lead rear bumper
                ego_s = ego_s + ego_state["length"] / 2
                lead_s = lead_s - lead_vehicle["length"] / 2

                dhw = np.round(lead_s - ego_s, 2)
                thw = np.round(dhw / ego_state["v"], 4)

                dv = ego_state["v"] - lead_vehicle["v"]

                # TTC & TTB
                if dv > 0:
                    ttc =  np.round(dhw / dv, 4)
                    d_ego_brake = (dv ** 2) / (2 * self._a_max_long)
                    d_available = abs(dhw) - abs(d_ego_brake)
                    ttb = (d_available/dv) - self._reaction_time
                    if ttb > time_limit:
                        ttb = np.inf
                    if ttc > time_limit:
                        ttc = np.inf

                # TTS
                d_t = abs(ego_t - lead_t)
                if d_t > ( (ego_width/2) + (lead_width/2)):
                    tts = np.inf
                else:
                    y_eva = -abs(d_t) + (ego_width/2) + (lead_width/2)
                    t_steer = math.sqrt(2*y_eva)/self._a_max_lat
                    tts = ttc -  t_steer - self._reaction_time
                    if tts > time_limit:
                        tts = np.inf

                self._collision_speed = self._v_long_ego_list[-1] - self._v_long_lead_list[-1]

            self._ttc_list.append(ttc)
            self._ttb_list.append(ttb)
            self._thw_list.append(thw)
            self._tts_list.append(tts)

        self._v_lat_list = np.diff(self._ego_t_list, 0)
        self._a_lat_list = np.diff(self._v_lat_list, 0)

    # Create plots with generate_plot() method
    def plot_scenario(self) -> None:
        # Plot TTB & TTC over t
        self.generate_plot(self._t, self._ttc_list, self._ttb_list, "t [s]", "ttc [s]",
                           "ttb [s]", filename="t-ttc-ttb.png", lower_limit=0, upper_limit=self._scenario.duration)

        # Plot TTB over t
        self.generate_plot(self._t, self._ttb_list, None, "t [s]", "ttb [s]",
                           None, 0, 10, "t-ttb.png", [0, 1, "red"],
                           [self._scenario_limits["limits"]["min_ttb_best_case"]-0.05,
                            0.1, "green" ])

        # Plot TTS & THW over t
        self.generate_plot(self._t, self._ttb_list, self._thw_list, xlabel="t [s]",
                           y1label="tts [s]", y2label="thw [s]", filename="t-tts-thw.png", lower_limit=0, upper_limit=15)

        # Plot TTS over t
        self.generate_plot(self._t, self._tts_list, None, "t [s]",
                           "tts [s]", None , 0, 10, "t-tts.png", [0, 0.47, "red"],
                           [self._scenario_limits["limits"]["min_tts_best_case"]-0.025,
                            0.05, "green"])

        # Plot v & a_long over t
        self.generate_plot(self._t, [v*3.6 for v in self._v_long_ego_list], self._a_long_list,
                           "t [s]", "v [m/s]", "a long [m/s^2]", filename="t-v-a_long.png")

        # Plot a_long over t
        self.generate_plot(self._t, self._a_long_list, None, "t [s]", "a_long [m/s^2]",
                           None, -7, 7, "a_long.png", [-self._a_max_long,-10, "red"], [self._a_max_long,10, "red"])
        # Plot a_lat over t
        self.generate_plot(self._t, self._a_lat_list, None, "t [s]", "a_lat [m/s^2]",
                           None, -3, 3, "a_lat.png", [-self._a_max_lat,-10, "red"], [self._a_max_lat,10, "red"])

        # Plot THW over t
        self.generate_plot(self._t, self._thw_list, None, "t [s]", "thw [s]",
                           None, 0, 6, "thw.png", [0, self._thw_min, "red"])

        # Plot v_max over t        max_v = round(max(abs(i) for i in self._v_long_ego_list)*3.6)
        self.generate_plot(self._t, [i*3.6 for i in self._v_long_ego_list] , None, "t [s]", "v [m/s]",
                           None, 0, self._scenario_speed_limit + 20, "v.png",
                           [self._scenario_speed_limit, self._scenario_speed_limit+20, "red"])

        # Render Scenario overview
        with create_scenario_plot_ax(self._result_dir, f"{self._scenario_name}", dpi=300, figw=16, figh=9, clean=False) as ax:

            self._scenario.render(ax, dpi = 600, clean = False)

            x_values = []
            y_values = []
            for ego_state in self._ego_states:
                x_values.append(ego_state["x"])
                y_values.append(ego_state["y"])

            ax.plot(x_values, y_values)

    # Method to check success criteria
    def check_success_criteria(self) -> dict:
        """
        Check ALL success criteria. Selection of suitable success criteria is done in PDF creation.
        """

        success_criteria = {"passed": True, "collision_passed": False,
                            "a_long_passed": False, "a_lat_passed": False,
                            "thw_min_passed": False, "v_max_passed": False,
                            "goal_reached": False, "reacted": False, "low_collision_speed": False}
        if(self._actual_termination_reason != "is_collision"):
            success_criteria["collision_passed"] = True

        if(max(abs(i) for i in self._a_long_list) < self._a_max_long):
            success_criteria["a_long_passed"] = True

        if(max(abs(i) for i in self._a_lat_list) < self._a_max_lat):
            success_criteria["a_lat_passed"] = True

        if(min(self._thw_list) > self._thw_min):
            success_criteria["thw_min_passed"] = True

        if(max(self._v_long_ego_list)*3.6<= self._v_max):
            success_criteria["v_max_passed"] = True

        # Not the "goal line", but just a check whether termination reason is expected termination reason
        if(self._expected_termination_reason==self._actual_termination_reason ):
            success_criteria["goal_reached"] = True

        if(max(abs(i) for i in self._a_long_list) > 4
           or max(abs(i) for i in self._a_lat_list) > 0.5):
            success_criteria["reacted"] = True

        if(self._collision_speed*3.6 <= 45):
            success_criteria["low_collision_speed"] = True

        if self._operation_mode_index in ("1.1", "1.2"):
            if False in [success_criteria["a_lat_passed"], success_criteria["a_long_passed"],
                         success_criteria["collision_passed"], success_criteria["thw_min_passed"],
                         success_criteria["v_max_passed"], success_criteria["goal_reached"]]:
                success_criteria["passed"] = False
        else:  # noqa: PLR5501
            if False in [success_criteria["reacted"], success_criteria["low_collision_speed"]]:
                success_criteria["passed"] = False

        return success_criteria

    # Central Method to create test report with all plots and metrics
    def create_test_report(self) -> None:  # noqa: PLR0912

        self.calc_metrics()

        # Plots for PDF
        self.plot_scenario()
        # Success criteria check for table in PDF
        success_criteria = self.check_success_criteria()

        # -- PDF Creation --

        # Operation mode
        operation_mode_string = ""
        if self._operation_mode_index == "1.2":
            operation_mode_string = "1.2 - Normal operation"
        elif self._operation_mode_index == "1.1":
            operation_mode_string = "1.1 - Emergency operation"
        elif self._operation_mode_index == "1":
            operation_mode_string = "1 - Avoid collision in physical limits possible"
        elif self._operation_mode_index == "2":
            operation_mode_string = "2 - Collision unavoidable"

        # Build table depending von operation mode
        if self._operation_mode_index == "1.2":
            data = [["Success Criteria", ""],
            ["1: no Collision", ("passed" if success_criteria["collision_passed"] else "failed")],
            ["2: Longitudinal acceleration", ("passed" if success_criteria["a_long_passed"] else "failed")],
            ["3: Lateral acceleration", ("passed" if success_criteria["a_lat_passed"] else "failed")],
            ["4: Minimal distance", ("passed" if success_criteria["thw_min_passed"] else "failed")],
            ["5: Maximum speed", ("passed" if success_criteria["v_max_passed"] else "failed")],
            [f"6: Expected goal '{self._expected_termination_reason}' reached", ("passed" if success_criteria["goal_reached"] else "failed")]]
        elif self._operation_mode_index == "1.1":
            data = [["Success Criteria", ""],
            ["1: No collision", ("passed" if success_criteria["collision_passed"] else "failed")],
            ["2: Longitudinal acceleration", ("passed" if success_criteria["a_long_passed"] else "failed")],
            ["3: Lateral acceleration", ("passed" if success_criteria["a_lat_passed"] else "failed")],
            ["4: Minimal distance", ("passed" if success_criteria["thw_min_passed"] else "failed")],
            ["5: Maximum speed", ("passed" if success_criteria["v_max_passed"] else "failed")],
            [f"6: Expected goal '{self._expected_termination_reason}' reached", ("passed" if success_criteria["goal_reached"] else "failed")]]
        else:
            data = [["Success Criteria", ""],
                ["1: System reacted", ("passed" if success_criteria["reacted"] else "failed")],
                ["2: Speed at collision <= 45km/h", (f"passed -> v = {round(self._collision_speed*3.6)} km/h" if success_criteria["low_collision_speed"] else f"failed -> v_col = {round(self._collision_speed*3.6)} km/h")]]

        ## create PDF
        c = canvas.Canvas(str(self._report_dir / f"{self._scenario.id}_report.pdf"), pagesize=A4)
        w, h = A4

        ## --SITE 1--
        ## Header
        c.setFont("Helvetica", 10)
        c.drawString(w-50, 25, "1")

        ## Title
        c.setFont("Helvetica-Bold", 18)
        c.drawString(50,h-50, "Test Report")

        ## Scenario infos
        c.setFont("Helvetica", 14)
        info_lines = [
        ("Scenario ID:", self._scenario.id),
        ("Category:", operation_mode_string),
        ("Termination reason:", self._actual_termination_reason)
        ]
        y = h - 150
        y = h - 150
        for line in info_lines:
            c.drawString(50, y, line[0])
            c.drawString(200, y, line[1])
            y -= 30  # vertival distance between lines

        ## Table with success criteria
        table = Table(data, colWidths=[250, 200])
        style = TableStyle([
            ("FONTNAME", (0, 0), (-1, 0), "Helvetica-Bold"),
            ("FONTNAME", (1, 1), (1, -1), "Helvetica-Bold"),
            ("FONTNAME", (0, 1), (0, -1), "Helvetica"),
            ("ALIGN", (0, 0), (-1, -1), "LEFT"),
            ("INNERGRID", (0, 0), (-1, -1), 0.25, colors.black),
            ("BOX", (0, 0), (-1, -1), 0.25, colors.black)
        ])
        for i, row in enumerate(data):
            if str(row[1]).split(" ")[0] == 0:
                continue
            if str(row[1]).split(" ")[0] == "passed":
                style.add("TEXTCOLOR", (1, i), (1, i), colors.green)
            elif str(row[1]).split(" ")[0] == "failed":
                style.add("TEXTCOLOR", (1, i), (1, i), colors.red)
        table.setStyle(style)
        if self._operation_mode_index not in ["2","1"]:
            table.wrapOn(c, 50, h-355)
            table.drawOn(c, 50, h-355)
        else:
            table.wrapOn(c, 50, h-290)
            table.drawOn(c, 50, h-290)

        # Plot scenario
        c.drawImage(self._result_dir / f"{self._scenario_name}.png",30, h-120, width = 35 *15, height = 35)

        # Plots with metrics
        c.setFont("Helvetica", 8)
        c.drawImage(self._result_dir / f"{self._scenario_name}_a_long.png",20, h-550, width = 180 *1.6, height = 180)
        c.drawString(110,h-385, "longitudinal acceleration over time")
        c.drawImage(self._result_dir / f"{self._scenario_name}_a_lat.png",300, h-550, width = 180 *1.6, height = 180)
        c.drawString(395,h-385, "lateral acceleration over time")
        c.drawImage(self._result_dir / f"{self._scenario_name}_thw.png",20, h-750, width = 180 *1.6, height = 180)
        c.drawString(120,h-585, "time headway over time")
        c.drawImage(self._result_dir / f"{self._scenario_name}_v.png",300, h-750, width = 180 *1.6, height = 180)
        c.drawString(420,h-585, "speed over time")

        c.showPage()

        ## --SITE 2--
        c.setFont("Helvetica", 10)
        c.drawString(w-50, 25, "2")

        ## Title
        c.setFont("Helvetica-Bold", 14)
        c.drawString(50,h-50, "Relative classification")
        c.setFont("Helvetica", 12)
        # First row
        c.drawString(50,h-80, "The ")
        c.setFont("Helvetica-Bold", 12)
        c.setFillColor(colors.red)
        c.drawString(73,h-80, "red")
        c.setFont("Helvetica", 12)
        c.setFillColor(colors.black)
        c.drawString(97,h-80, "area represents an area that is considered critical by a human driver.")
        # Second row
        c.drawString(50,h-95, "The ")
        c.setFont("Helvetica-Bold", 12)
        c.setFillColor(colors.green)
        c.drawString(73,h-95, "green")
        c.setFont("Helvetica", 12)
        c.setFillColor(colors.black)
        c.drawString(105,h-95, " value is the best-case reaction if the system were to decelerate to the maximum")
        # Third row
        c.drawString(50, h-110, "immediately at the start of the scenario. It describes the minimum TTS/TTB occurring")
        c.drawString(50, h-125, "during immediate deceleration with maximum longitudinal deceleration.")

        ## Plots
        c.drawImage(self._result_dir / f"{self._scenario_name}_t-ttb.png",50, h-450, width = 280 *1.6, height = 280)
        c.drawImage(self._result_dir / f"{self._scenario_name}_t-tts.png",50, h-730, width = 280 *1.6, height = 280)

        c.showPage()

        c.save()

        logger.info("done")

        # Delete saved plots
        for f in self._result_dir.glob("*.png"):
            f.unlink()
           # logger.info(f"Deleted file: {f}")

        # Write sucess criteria to json file (ALL)
        output = {"scenario_id": self._scenario.id, "operation_mode": self._operation_mode_index, "success_criteria": success_criteria}
        with (self._report_dir / f"{self._scenario.id}_report.json").open("w", newline="") as file:
                json.dump(output, file, indent=4)

        logger.info(f"Test report for scenario {self._scenario.id} created.")
