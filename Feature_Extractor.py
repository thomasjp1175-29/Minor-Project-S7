"""Feature Extractor - Smart Feature Recognition"""

import traceback
import adsk
import adsk.core
import adsk.fusion
import os

from itertools import combinations

app = adsk.core.Application.get()
ui = app.userInterface


def solve_reactions(p1, p2, p3, cog_x, cog_y):
    """Solve R1+R2+R3=1, sum(Ri*xi)=xg, sum(Ri*yi)=yg via Cramer's rule."""
    (x1, y1), (x2, y2), (x3, y3) = p1, p2, p3

    def det3(a):
        return (a[0][0]*(a[1][1]*a[2][2]-a[1][2]*a[2][1])
                - a[0][1]*(a[1][0]*a[2][2]-a[1][2]*a[2][0])
                + a[0][2]*(a[1][0]*a[2][1]-a[1][1]*a[2][0]))

    A = [[1, 1, 1], [x1, x2, x3], [y1, y2, y3]]
    b = [1, cog_x, cog_y]
    detA = det3(A)
    if abs(detA) < 1e-9:
        return None
    R = []
    for i in range(3):
        Ai = [row[:] for row in A]
        for r in range(3):
            Ai[r][i] = b[r]
        R.append(det3(Ai) / detA)
    return R


def candidate_primary_points(bbox, hole_centers, hole_radius, margin):
    minP, maxP = bbox.minPoint, bbox.maxPoint
    corners = [
        (maxP.x - margin, maxP.y - margin),
        (maxP.x - margin, minP.y + margin),
        (minP.x + margin, maxP.y - margin),
        (minP.x + margin, minP.y + margin),
    ]
    return [
        (cx, cy) for cx, cy in corners
        if all(((cx - hx) ** 2 + (cy - hy) ** 2) ** 0.5 > hole_radius * 3
               for hx, hy in hole_centers)
    ]


def choose_primary_points(corners, cog_x, cog_y):
    best, best_spread = None, None
    for combo in combinations(corners, 3):
        R = solve_reactions(combo[0], combo[1], combo[2], cog_x, cog_y)
        if R is None or any(r < 0 for r in R):
            continue  # COG falls outside this triangle -> unstable
        spread = max(R) - min(R)
        if best_spread is None or spread < best_spread:
            best_spread, best = spread, (combo, R)
    return best


# ---------------------------------------------------------
# SMART CYLINDER CLASSIFIER
# ---------------------------------------------------------
def classify_cylinder(face):

    cyl = adsk.core.Cylinder.cast(face.geometry)

    if not cyl:
        return "Unknown Cylinder"

    diameter = cyl.radius * 2
    edge_count = face.edges.count

    # Through hole
    # Current workpiece holes:
    # Diameter = 0.50 cm
    # Edge Count = 2
    if edge_count == 2 and diameter <= 0.60:
        return "Through Hole"

    # Corner fillet
    # Current workpiece corner fillets:
    # Diameter = 1.00 cm
    # Edge Count = 4
    if edge_count == 4:
        return "Corner Fillet"

    return "Unknown Cylinder"


# ---------------------------------------------------------
# MAIN FUSION SCRIPT
# ---------------------------------------------------------
def run(_context: str):
    try:
        design = adsk.fusion.Design.cast(app.activeProduct)

        if not design:
            ui.messageBox("No active design found.")
            return

        rootComp = design.rootComponent

        if rootComp.bRepBodies.count == 0:
            ui.messageBox("No body found.")
            return

        body = rootComp.bRepBodies.item(0)

        report = "SMART FEATURE RECOGNITION REPORT\n\n"

        through_holes = 0
        corner_fillets = 0
        unknown_cylinders = 0
        feature_number = 0
        hole_centres = []

        # -------------------------------------------------
        # ANALYSE ALL FACES
        # -------------------------------------------------
        for face in body.faces:
            geometry = face.geometry
            if not geometry:
                continue

            face_type = str(geometry.objectType).lower()
            if "cylinder" not in face_type:
                continue

            cyl = adsk.core.Cylinder.cast(geometry)
            if not cyl:
                continue

            feature_number += 1
            feature_type = classify_cylinder(face)

            # Count features
            if feature_type == "Through Hole":
                through_holes += 1
                hole_centres.append((cyl.origin.x, cyl.origin.y))
            elif feature_type == "Corner Fillet":
                corner_fillets += 1
            else:
                unknown_cylinders += 1

            axis = cyl.axis
            origin = cyl.origin

            report += (
                "====================================\n"
                f"Feature {feature_number}\n"
                "====================================\n"
            )
            report += f"Type     : {feature_type}\n"
            report += (
                f"Center   : "
                f"({origin.x:.2f}, "
                f"{origin.y:.2f}, "
                f"{origin.z:.2f})\n"
            )
            report += f"Radius   : {cyl.radius:.2f} cm\n"
            report += f"Diameter : {cyl.radius * 2:.2f} cm\n"
            report += (
                f"Axis     : "
                f"({axis.x:.2f}, "
                f"{axis.y:.2f}, "
                f"{axis.z:.2f})\n"
            )
            report += f"Area     : {face.area:.2f} cm²\n"
            report += f"Edges    : {face.edges.count}\n\n"

        # -------------------------------------------------
        # 3-2-1 PRIMARY SUPPORT POINTS
        # (runs ONCE, after the loop over all faces is done)
        # -------------------------------------------------
        cog = body.physicalProperties.centerOfMass
        bbox = body.boundingBox

        corners = candidate_primary_points(bbox, hole_centres, hole_radius=0.25, margin=1.0)
        result = choose_primary_points(corners, cog.x, cog.y)

        if result:
            pts, reactions = result
            report += "\n3-2-1 PRIMARY SUPPORT POINTS\n"
            for (px, py), r in zip(pts, reactions):
                report += f"  ({px:.2f}, {py:.2f}) cm  -> reaction share: {r*100:.1f}%\n"
        else:
            report += "\nNo valid 3-point support triangle found (check margin/hole_radius)\n"

        # -------------------------------------------------
        # FEATURE SUMMARY
        # -------------------------------------------------
        report += (
            "\n====================================\n"
            "FEATURE SUMMARY\n"
            "====================================\n\n"
        )
        report += f"Total Cylindrical Features : {feature_number}\n"
        report += f"Through Holes              : {through_holes}\n"
        report += f"Corner Fillets             : {corner_fillets}\n"
        report += f"Unknown Cylinders          : {unknown_cylinders}\n"

        # -------------------------------------------------
        # SAVE REPORT + NOTIFY
        # -------------------------------------------------
        output_dir = r"C:\Users\THOMAS JOHN\OneDrive\Documents\Minor project\Fixture_Reports"
        os.makedirs(output_dir, exist_ok=True)
        output_path = os.path.join(output_dir, "Feature_Report.txt")
        with open(output_path, "w") as f:
            f.write(report)

        ui.messageBox(f"Report saved to:\n{output_path}\n\n(Full report is in the file — this popup may be truncated)")

    except:
        ui.messageBox("SCRIPT ERROR\n\n" + traceback.format_exc())