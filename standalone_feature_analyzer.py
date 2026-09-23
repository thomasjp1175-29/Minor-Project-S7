"""
Standalone Feature Analyzer - CadQuery port of Feature_Extractor.py

Runs OUTSIDE Fusion 360. Takes a .stp file path, returns the same kind
of report + 3-2-1 support points your Fusion script produces, plus a
.glb mesh for a web viewer.

Install:
    pip install cadquery

This is a library, not a script to double-click - see analyze_step()
at the bottom for the entry point a FastAPI endpoint would call.
"""

import os
from itertools import combinations
import cadquery as cq
from cadquery.occ_impl.shapes import Shape


# ---------------------------------------------------------------------
# CAD (Z-up, mm) -> glTF (Y-up, meters) coordinate conversion
# ---------------------------------------------------------------------
def cad_to_gltf_point(x, y, z):
    """(x, y, z) in mm, Z-up  ->  (x, z, -y) in meters, Y-up."""
    return (x / 1000.0, z / 1000.0, -y / 1000.0)


# ---------------------------------------------------------------------
# SMART CYLINDER CLASSIFIER  (same thresholds as Fusion version)
# ---------------------------------------------------------------------
def classify_cylinder(face):
    """
    face: a cadquery Face object whose geomType() == 'CYLINDER'
    Returns ('Through Hole' | 'Corner Fillet' | 'Unknown Cylinder', info)
    """
    geom = face._geomAdaptor()  # BRepAdaptor_Surface
    cyl = geom.Cylinder()
    radius = cyl.Radius()
    diameter = radius * 2
    edge_count = len(face.Edges())

    origin = cyl.Location()  # gp_Pnt
    axis_dir = cyl.Axis().Direction()

    info = {
        "center": (origin.X(), origin.Y(), origin.Z()),
        "axis": (axis_dir.X(), axis_dir.Y(), axis_dir.Z()),
        "radius": radius,
        "diameter": diameter,
        "area": face.Area(),
        "edge_count": edge_count,
    }

    # Through hole. STEP files are in mm (vs Fusion's cm), so the
    # threshold is 10x Fusion's: diameter <= 6.0 mm covers the 5mm holes.
    # edge_count is 2 or 3: OpenCascade adds a seam edge for full-revolution
    # cylinders that Fusion's kernel doesn't count separately.
    if edge_count in (2, 3) and diameter <= 6.0:
        return "Through Hole", info

    # Corner fillet (Fusion threshold: edge_count == 4)
    if edge_count == 4:
        return "Corner Fillet", info

    return "Unknown Cylinder", info


# ---------------------------------------------------------------------
# 3-2-1 SUPPORT POINT SOLVER  (identical math to the Fusion version)
# ---------------------------------------------------------------------
def solve_reactions(p1, p2, p3, cog_x, cog_y):
    (x1, y1), (x2, y2), (x3, y3) = p1, p2, p3

    def det3(a):
        return (a[0][0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1])
                - a[0][1] * (a[1][0] * a[2][2] - a[1][2] * a[2][0])
                + a[0][2] * (a[1][0] * a[2][1] - a[1][1] * a[2][0]))

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
    corners = [
        (bbox.xmax - margin, bbox.ymax - margin),
        (bbox.xmax - margin, bbox.ymin + margin),
        (bbox.xmin + margin, bbox.ymax - margin),
        (bbox.xmin + margin, bbox.ymin + margin),
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
            continue
        spread = max(R) - min(R)
        if best_spread is None or spread < best_spread:
            best_spread, best = spread, (combo, R)
    return best


# ---------------------------------------------------------------------
# MAIN ENTRY POINT - what your FastAPI endpoint calls
# ---------------------------------------------------------------------
def analyze_step(stp_path: str, glb_out_path: str = None):
    result = cq.importers.importStep(stp_path)
    shape = result.val()

    faces = shape.Faces()

    report_lines = ["SMART FEATURE RECOGNITION REPORT", ""]
    through_holes = corner_fillets = unknown_cylinders = feature_number = 0
    hole_centers = []

    for face in faces:
        if face.geomType() != "CYLINDER":
            continue

        feature_number += 1
        feature_type, info = classify_cylinder(face)

        if feature_type == "Through Hole":
            through_holes += 1
            hole_centers.append((info["center"][0], info["center"][1]))
        elif feature_type == "Corner Fillet":
            corner_fillets += 1
        else:
            unknown_cylinders += 1

        report_lines.append(f"========== Feature {feature_number} ==========")
        report_lines.append(f"Type     : {feature_type}")
        report_lines.append(f"Center   : {tuple(round(v, 2) for v in info['center'])}")
        report_lines.append(f"Radius   : {info['radius']:.2f} mm")
        report_lines.append(f"Diameter : {info['diameter']:.2f} mm")
        report_lines.append(f"Axis     : {tuple(round(v, 2) for v in info['axis'])}")
        report_lines.append(f"Area     : {info['area']:.2f} mm^2")
        report_lines.append(f"Edges    : {info['edge_count']}")
        report_lines.append("")

    # --- mass properties + bounding box (CadQuery/OCC equivalents of
    #     Fusion's body.physicalProperties / body.boundingBox) ---
    cog = shape.Center()        # cq.Vector - center of mass
    bbox = shape.BoundingBox()

    corners = candidate_primary_points(bbox, hole_centers, hole_radius=2.5, margin=10.0)
    support_result = choose_primary_points(corners, cog.x, cog.y)

    support_points = []
    if support_result:
        pts, reactions = support_result
        report_lines.append("3-2-1 PRIMARY SUPPORT POINTS")
        for (px, py), r in zip(pts, reactions):
            support_points.append({"x": px, "y": py, "reaction_pct": r * 100})
            report_lines.append(f"  ({px:.2f}, {py:.2f}) mm  -> reaction share: {r*100:.1f}%")
    else:
        report_lines.append("No valid 3-point support triangle found")

    report_lines.append("")
    report_lines.append("==============================")
    report_lines.append("FEATURE SUMMARY")
    report_lines.append("==============================")
    report_lines.append(f"Through Holes     : {through_holes}")
    report_lines.append(f"Corner Fillets    : {corner_fillets}")
    report_lines.append(f"Unknown Cylinders : {unknown_cylinders}")

    report_text = "\n".join(report_lines)

    # --- export a mesh for the web viewer ---
    # CadQuery doesn't export GLB directly, so we go STEP -> STL -> GLB
    # (via trimesh), and rotate Z-up (CAD) into Y-up (glTF) on the way.
    # pip install trimesh
    if glb_out_path:
        import trimesh
        import tempfile

        with tempfile.NamedTemporaryFile(suffix=".stl", delete=False) as tmp_stl:
            stl_path = tmp_stl.name
        cq.exporters.export(shape, stl_path, exportType="STL")

        mesh = trimesh.load(stl_path)
        rotation = trimesh.transformations.rotation_matrix(
            angle=-1.5707963267948966,  # -90 degrees, in radians
            direction=[1, 0, 0],
        )
        mesh.apply_scale(0.001)      # mm -> m
        mesh.apply_transform(rotation)  # Z-up -> Y-up
        mesh.export(glb_out_path)

        os.unlink(stl_path)

    # Support points + COG converted into the same space as the exported
    # mesh, so the frontend can place hotspots directly on the model.
    support_points_gltf = [
        {**pt, **dict(zip(("x", "y", "z"), cad_to_gltf_point(pt["x"], pt["y"], 0)))}
        for pt in support_points
    ]
    cog_gltf = dict(zip(("x", "y", "z"), cad_to_gltf_point(cog.x, cog.y, cog.z)))

    return {
        "report_text": report_text,
        "support_points": support_points,          # original CAD mm coordinates
        "support_points_gltf": support_points_gltf,  # meters, matches the .glb
        "through_holes": through_holes,
        "corner_fillets": corner_fillets,
        "unknown_cylinders": unknown_cylinders,
        "center_of_mass": {"x": cog.x, "y": cog.y, "z": cog.z},
        "center_of_mass_gltf": cog_gltf,
    }


if __name__ == "__main__":
    import sys
    if len(sys.argv) < 2:
        print("Usage: python standalone_feature_analyzer.py path/to/part.stp")
        sys.exit(1)
    result = analyze_step(sys.argv[1])
    print(result["report_text"])
