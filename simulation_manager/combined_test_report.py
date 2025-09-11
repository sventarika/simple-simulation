from loguru import logger
from pathlib import Path
from reportlab.lib.pagesizes import A4
from reportlab.pdfgen import canvas
from reportlab.platypus import Table, TableStyle
from reportlab.lib import colors


class CombinedTestReport:

    def __init__(self, result_dir: Path, evaluation_list: list) -> None:
        self._evaluation_list = evaluation_list
        self._export_dir = result_dir
        self._export_dir.mkdir(exist_ok=True)

    def create_combined_report(self) -> None:
        data = [["Scenario", "Mode", "Collision", "a_lon", "a_lat", "d", "v_max"]]
        data_col = [["Scenario", "Mode", "System reacted", "Low collision speed"]]

        for evaluation in self._evaluation_list:
            id_ = evaluation.get_scenario_id()
            success_criteria = evaluation.check_success_criteria()
            operation_mode_index = evaluation.get_operation_mode_index()

            if operation_mode_index not in ("1", "2"):
                data.append([id_, evaluation.get_operation_mode_index(), "passed" if success_criteria["collision_passed"] else "failed", "passed" if success_criteria["a_long_passed"] else "failed",
                                "passed" if success_criteria["a_lat_passed"] else "failed", "passed" if success_criteria["thw_min_passed"] else "failed",
                                "passed" if success_criteria["v_max_passed"] else "failed"])
            else:
                data_col.append([id_, evaluation.get_operation_mode_index(), ("passed" if success_criteria["reacted"] else "failed"),
                                 "passed" if success_criteria["low_collision_speed"] else "failed"])

        c = canvas.Canvas(self._export_dir /  "combined_report.pdf", pagesize=A4)
        w, h = A4

        ## --Site 1--
        ## Header
        c.setFont("Helvetica", 10)
        c.drawString(w-50, 25, "1")

        ## Title
        c.setFont("Helvetica-Bold", 18)
        c.drawString(50,h-50, "Combined Test Report")

        c.setFont("Helvetica", 12)
        c.drawString(50,h-90, "Scenarios in normal or emergency operation")

        ## Table 1 with normal and emergency operation
        table = Table(data, colWidths=[80, 60])
        style = TableStyle([
            ("FONTNAME", (0, 0), (-1, 0), "Helvetica-Bold"),
            ("FONTNAME", (1, 1), (-1, -1), "Helvetica-Bold"),
            ("FONTNAME", (0, 1), (0, -1), "Helvetica"),
            ("ALIGN", (0, 0), (-1, -1), "LEFT"),
            ("INNERGRID", (0, 0), (-1, -1), 0.25, colors.black),
            ("BOX", (0, 0), (-1, -1), 0.25, colors.black)
        ])
        for i, row in enumerate(data):
            for j, cell in enumerate(row):
                if cell == "passed":
                  style.add("TEXTCOLOR", (j, i), (j, i), colors.green)
                elif cell == "failed":
                  style.add("TEXTCOLOR", (j, i), (j, i), colors.red)

        table.setStyle(style)

        table_h = 30 * len(data)
        table.wrapOn(c, 50, h-table_h-20)
        table.drawOn(c, 50, h-table_h-20)

        # TABLE 2 with collision scenarios
        c.setFont("Helvetica", 12)
        table2_h = 30 * len(data_col)
        c.drawString(50,h-table_h-60, "Scenarios with collision")

        table2 = Table(data_col, colWidths=[80, 40, 100, 120])
        style2 = TableStyle([
            ("FONTNAME", (0, 0), (-1, 0), "Helvetica-Bold"),
            ("FONTNAME", (1, 1), (-1, -1), "Helvetica-Bold"),
            ("FONTNAME", (0, 1), (0, -1), "Helvetica"),
            ("ALIGN", (0, 0), (-1, -1), "LEFT"),
            ("INNERGRID", (0, 0), (-1, -1), 0.25, colors.black),
            ("BOX", (0, 0), (-1, -1), 0.25, colors.black)
        ])
        for i, row in enumerate(data_col):
            for j, cell in enumerate(row):
                if cell == "passed":
                  style2.add("TEXTCOLOR", (j, i), (j, i), colors.green)
                elif cell == "failed":
                  style2.add("TEXTCOLOR", (j, i), (j, i), colors.red)

        table2.setStyle(style2)

        table2.wrapOn(c, 50, h-table2_h-table_h-60)
        table2.drawOn(c, 50, h-table2_h-table_h-60)

        c.showPage()

        c.save()

        logger.info("Combined Test report created.")
