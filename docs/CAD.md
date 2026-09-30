# Fusion construction export

Select a candidate in the results browser, then download its Fusion Python
script or coordinate CSV. Both buttons are enabled only when the active home
solver produces six valid legs. JSON remains available for diagnostic
candidates with an invalid home pose. A candidate whose home geometry is valid
may export construction geometry even when its workspace, cycle, or another
requirement fails; the script embeds the candidate ID, run settings, diagnostic
flag, and failed categories.

The CSV records six base anchors in the base frame, six solved horn tips and
six platform anchors in world coordinates at the home pose, plus base and
platform centroids. When a leg's base anchor, horn tip and platform anchor are
collinear, the CSV also carries a `Plane guide N` row (`kind` `plane_guide`,
world frame) offset 10 mm from the base anchor perpendicular to the horn, so
the Fusion script can still define that leg's sketch plane; it is not a
mechanical point. Coordinates and lengths are millimeters. The CSV also
records the selected candidate ID and each point's `kind` and frame. The home pose has
zero requested translation/rotation; its world platform points include the
layout's home-height offset.

Run the downloaded `.py` file using Fusion's Scripts and Add-Ins panel. It
creates a new direct-design document with named construction points and a
construction plane plus two named construction-line sketches per leg: one
horn and one rod. It creates no solid bodies or manufactured parts. Fusion's
API uses centimeters internally, so the script divides each millimeter
coordinate by 10 when constructing a point. Each leg's sketch plane contains
its base, horn, and platform points; a guide point is used only if those three
points are collinear. Sketch coordinates are converted from model space before
adding the lines.

The generated script uses Autodesk's documented [construction point API](https://help.autodesk.com/cloudhelp/ENU/Fusion-360-API/files/fusion_ConstructionPointInput_setByPoint.htm),
[three-point construction plane API](https://help.autodesk.com/cloudhelp/ENU/Fusion-360-API/files/ConstructionPlaneInput_setByThreePoints.htm),
[sketch coordinate conversion](https://help.autodesk.com/cloudhelp/ENU/Fusion-360-API/files/fusion_Sketch_modelToSketchSpace.htm),
and [internal unit convention](https://help.autodesk.com/cloudhelp/ENU/Fusion-360-API/files/Units_UM.htm).

Automated checks verify coordinates, horn/rod lengths, names, gating, and
script structure. The script has not yet been verified by a recorded live
import into Fusion; that verification is tracked in issue #31, so treat the
generated geometry as unconfirmed until it is done.
