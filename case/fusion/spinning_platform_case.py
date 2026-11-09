"""Spinning Platform V2 case: parametric Fusion model.

Run inside Fusion (Utilities > Scripts and Add-Ins, or via the Fusion MCP):
    exec(open('<repo>/case/fusion/spinning_platform_case.py').read()); build(clean=True)
It builds into the active design. Every dimension is a user parameter
(Modify > Change Parameters), and the sketches are driven by those parameters.

Coordinates: origin = motor shaft axis at the underside of the foot plate (rubber feet go
below that), +Y = front (pot), Z up. PCB coords: X = x_kicad - 150, Y = 150 - y_kicad.
Position dimensions are distances from the origin, so a parameter that changes sign
needs that sketch fixed by hand.

Whole-case weighing, like a kitchen scale:
  table → feet → FOOT PLATE → pillar → load cell → post → FLOOR PLATE → PCB/columns → SHELL
The shell (walls + closed top, printed upside down) hangs on the cell ~1 mm above the foot
plate; its skirt hides that gap. The only opening on top is the shaft hole, inside a collar.
The bearing sits on the top, the platform rests on the bearing (drive pins + D-shaft hub,
lifts off for cleaning). USB-C and the charge LED sit side by side in a panel on the foot
plate (back), so the cable is never weighed; the pot is alone on the front, centred.

Assembly:
 1. Shell: motor ears → top (M3 thread-forming from below), HX711 → its posts,
    pot → front wall (nut from inside, through the open bottom).
 2. Floor plate: cell fixed end → post (screws from under the floor), PCB → standoffs.
 3. Wire up, slide floor + PCB into the shell, 4× M3 from under the floor through
    standoffs + PCB into the shell's columns.
 4. Foot plate: USB-C → back panel; pillar up through the floor/PCB cutouts, 2 screws from
    under the foot plate into the cell's free end. Stick on rubber feet.
 5. Bearing bottom plate → top (rotate the top plate 45° to reach the corner holes),
    platform drops on.
"""
import math
import adsk.core
import adsk.fusion

# name, expression, comment. Plain numbers are mm.
PARAMS = [
    # shell
    ('base_outer', '110', 'outer square size'),
    ('base_r', '4', 'outer corner radius'),
    ('wall', '2.5', 'wall thickness'),
    ('inner', 'base_outer - 2*wall', ''),
    ('top_t', '4', 'closed top (thick enough for blind screw pilots)'),
    ('skirt_z', '0.5', 'shell bottom edge, just above the foot plate underside'),
    # foot plate (not weighed) and the gap that is also the overload stop
    ('foot_t', '4', ''),
    ('foot_gap', '1', 'foot plate to skirt, all round'),
    ('foot_sq', 'inner - 2*foot_gap', ''),
    ('foot_r', '1', ''),
    ('float_gap', '1', 'foot plate top to floor plate underside'),
    ('stop_h', '0.3', 'stop bumps: overload travel = float_gap - stop_h'),
    # floor plate + PCB
    ('floor_t', '2', ''),
    ('floor_bot_z', 'foot_t + float_gap', ''),
    ('floor_top_z', 'floor_bot_z + floor_t', ''),
    ('floor_sq', 'inner - 0.4 mm', 'slides into the shell'),
    ('floor_r', 'base_r - wall - 0.2 mm', ''),
    ('standoff_h', '3', ''),
    ('pcb_bottom_z', 'floor_top_z + standoff_h', ''),
    ('pcb_t', '1.6', ''),
    ('pcb_top_z', 'pcb_bottom_z + pcb_t', ''),
    ('pcb_clear', '1', 'post/pillar to PCB cutout edge'),
    ('batt_h', '23', '21700 in holder, above the PCB'),
    ('top_in_z', 'pcb_top_z + batt_h + 1.4 mm', 'underside of the top'),
    ('top_z', 'top_in_z + top_t', 'top surface'),
    ('column_d', '6', 'shell columns down to the PCB mounting holes'),
    # bearing + platform
    ('bearing_h', '8', 'lazy-susan thickness (TODO confirm)'),
    ('bearing_sq', '72', 'lazy-susan square size'),
    ('bearing_bot_hole', '31.5', 'bottom plate holes at (±, ±) (TODO)'),
    ('bearing_top_hole', '24', 'top plate holes at (±,0) and (0,±) (TODO)'),
    ('pilot_depth', '3', 'blind M3 pilots in the top'),
    ('platform_d', '112', 'covers the bearing corner sweep'),
    ('platform_t', '4', ''),
    ('platform_z', 'top_z + bearing_h', ''),
    ('hub_d', '12', ''),
    ('peg_d', '3', 'drive pins into the bearing top plate holes (TODO hole size - 0.4)'),
    ('peg_h', '1.5', ''),
    ('collar_id', '14', 'raised ring around the shaft hole'),
    ('collar_od', '18', ''),
    ('collar_h', '2', ''),
    # load cell (TODO measure on arrival)
    ('cell_l', '80', ''), ('cell_w', '12.7', ''), ('cell_h', '12.7', ''),
    ('cell_y', '-14.35', 'cell centre line'),
    ('cell_x0', '-38', 'west (fixed) end'),
    ('cell_bot_z', 'pcb_top_z + 5.7 mm', ''),
    ('cell_hole_d', '5.5', 'M5 clearance (TODO; M4 = 4.5)'),
    ('cell_hole_end', '5', 'first hole centre from the cell end (TODO)'),
    ('cell_hole_pitch', '15', 'between the two holes at one end (TODO)'),
    ('mount_len', 'cell_hole_end + cell_hole_pitch + 5 mm', 'post/pillar length: covers both screws'),
    ('mount_w', 'cell_w - 2 mm', 'post/pillar width (narrower than the cell keeps the PCB cutouts off traces)'),
    # motor 28BYJ-48
    ('motor_body_d', '28.2', ''), ('motor_body_h', '19', ''),
    ('motor_off', '8', 'body centre north of the shaft'),
    ('motor_ear_pitch', '35', ''),
    ('motor_boss_d', '9.5', ''),
    ('shaft_d', '5', ''), ('shaft_flat', '3', 'across the flats'), ('shaft_len', '9.5', 'above the motor face'),
    ('m3_pilot', '2.5', 'M3 thread-forming pilot'),
    # front wall: the pot alone, centred
    ('wall_mid_z', '(skirt_z + top_z)/2', ''),
    ('pot_x', '0', ''), ('pot_hole', '7', ''), ('pot_wall', '4', ''), ('pot_boss_d', '16', ''),
    ('pot_body_d', '17', 'assumed'), ('pot_depth', '11', ''),
    # USB-C panel socket + RGB charge LED side by side on the foot plate, back edge.
    # The pair (flange left edge .. LED holder flange right edge) is centred on x = 0.
    ('led_pitch', '16', 'USB-C centre to LED centre'),
    ('led_hole', '5.14', 'RGB LED panel holder'), ('led_wall', '4.5', 'panel thickness the holder wants'),
    ('led_flange_d', '8', 'holder flange outside the panel (TODO measure)'),
    ('led_depth', '10', 'LED + holder + legs behind the panel (TODO measure)'),
    ('usb_cut_w', '13', ''), ('usb_cut_h', '3.5', ''),
    ('usb_flange_w', '20.1', ''), ('usb_flange_h', '7', ''), ('usb_flange_t', '1.6', ''),
    ('usb_hole_d', '2.3', 'M2 clearance, nuts behind the plate'), ('usb_hole_pitch', '16.1', ''),
    ('usb_nut_d', '5.5', 'M2 nut + spinner room behind the plate'),
    ('usb_x', '-(led_pitch + led_flange_d/2 - usb_flange_w/2)/2', 'centres the USB-C + LED pair'),
    ('led_x', 'usb_x + led_pitch', ''),
    ('usb_body_w', '9', 'parts on the back of the USB-C board (measured)'), ('usb_body_h', '3', 'measured'), ('usb_depth', '12', 'measured, without wires (they bend up through the floor plate notch)'),
    ('usb_panel_t', '3', 'plate: 1.6 flange recess + 1.4 wall'),
    ('usb_face_y', '-base_outer/2', 'plate face, flush with the shell'),
    ('usb_side_gap', '0.5', 'skirt notch beside the plate'),
    ('usb_l', 'usb_x - usb_flange_w/2 - 3 mm', 'plate west edge'),
    ('usb_r', 'led_x + led_flange_d/2 + 3 mm', 'plate east edge'),
    ('usb_win_clear', '1', 'skirt notch above the plate (> overload travel)'),
    ('usb_panel_top', 'pcb_top_z - usb_win_clear', 'puts the gap above the plate in front of the PCB edge'),
    ('usb_z', 'usb_panel_top/2', 'USB-C and LED centred on the plate'),
    ('led_boss_w', '9', 'LED boss behind the plate'),
    # HX711 (Soldered 333005, 22×22) flat under the top
    ('hx_x0', '24', ''), ('hx_y0', '-4', ''), ('hx_post_h', '4', ''),
    ('hx_post_d', '6.5', ''), ('hx_screw', '2.6', ''),
]

PCB_MOUNT = [(-47.5, 25), (48.5, 24.5), (-47.5, -47.5), (47.5, -47.5)]  # from KiCad, fixed
STOP_BUMPS = [(-30, 44), (30, 44), (-30, -44), (30, -44)]
PARTS = ('shell', 'floor', 'foot', 'platform', 'reference')


AX = {'x': 0, 'y': 1, 'z': 2}
OP = adsk.fusion.FeatureOperations
VI = adsk.core.ValueInput.createByString


class Ctx:
    def __init__(self, design):
        self.design = design
        self.um = design.unitsManager
        self.root = design.rootComponent

    def val(self, expr):  # cm
        return self.um.evaluateExpression(expr, 'mm')


def _mm(expr):
    """Plain numbers get a unit; expressions are left alone."""
    try:
        float(expr)
        return expr + ' mm'
    except ValueError:
        return expr


def ensure_params(c):
    ups = c.design.userParameters
    for name, expr, comment in PARAMS:
        p = ups.itemByName(name)
        if p:
            p.expression = _mm(expr)
        else:
            ups.add(name, VI(_mm(expr)), 'mm', comment)


def new_comp(c, name):
    for occ in c.root.occurrences:
        if occ.component.name == name:
            raise RuntimeError(f'component {name} already exists')
    occ = c.root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
    occ.component.name = name
    return occ.component


def plane(c, comp, base, expr):
    """Offset construction plane from comp's base plane ('xy'|'xz'|'yz') at world coord expr."""
    basep = {'xy': comp.xYConstructionPlane, 'xz': comp.xZConstructionPlane,
             'yz': comp.yZConstructionPlane}[base]
    if expr in ('0', 0):
        return basep
    axis = {'xy': 2, 'xz': 1, 'yz': 0}[base]
    want = c.val(expr)
    for e in (expr, f'-({expr})'):
        pi = comp.constructionPlanes.createInput()
        pi.setByOffset(basep, VI(e))
        p = comp.constructionPlanes.add(pi)
        o = p.geometry.origin
        if abs([o.x, o.y, o.z][axis] - want) < 1e-6:
            p.name = f'{base}@{expr}'
            return p
        p.deleteMe()
    raise RuntimeError(f'could not place plane {base}@{expr}')


class Sk:
    """Sketch whose geometry is placed by world-coordinate expressions and dimensioned with them."""

    def __init__(self, c, comp, pl, name):
        self.c = c
        self.sk = comp.sketches.add(pl)
        self.sk.name = name
        self.sk.isComputeDeferred = True
        xd, yd = self.sk.xDirection, self.sk.yDirection
        self.map = {}  # world axis -> 'h' | 'v'
        for a, i in AX.items():
            if abs([xd.x, xd.y, xd.z][i]) > 0.99:
                self.map[a] = 'h'
            elif abs([yd.x, yd.y, yd.z][i]) > 0.99:
                self.map[a] = 'v'
        o = self.sk.sketchToModelSpace(adsk.core.Point3D.create(0, 0, 0))
        for a in self.map:
            if abs([o.x, o.y, o.z][AX[a]]) > 1e-6:
                raise RuntimeError('sketch origin is not on the world origin')
        n = xd.crossProduct(yd)
        self.normal = n

    def P(self, w):
        """w: dict axis->expr (in-plane axes). Returns sketch-space Point3D."""
        xyz = [0.0, 0.0, 0.0]
        for a, e in w.items():
            xyz[AX[a]] = self.c.val(e)
        # fill the out-of-plane coordinate from the plane itself
        pt = self.sk.modelToSketchSpace(adsk.core.Point3D.create(*xyz))
        return adsk.core.Point3D.create(pt.x, pt.y, 0)

    def fix(self, sp, w):
        """Dimension sketch point sp to the origin using the world expressions in w."""
        dims = self.sk.sketchDimensions
        gc = self.sk.geometricConstraints
        org = self.sk.originPoint
        for a, e in w.items():
            v = self.c.val(e)
            hv = self.map[a]
            if abs(v) < 1e-9:
                (gc.addVerticalPoints if hv == 'h' else gc.addHorizontalPoints)(org, sp)
                continue
            ori = (adsk.fusion.DimensionOrientations.HorizontalDimensionOrientation if hv == 'h'
                   else adsk.fusion.DimensionOrientations.VerticalDimensionOrientation)
            g = sp.geometry
            tp = adsk.core.Point3D.create(g.x / 2 + 0.3, g.y / 2 + 0.3, 0)
            try:
                d = dims.addDistanceDimension(org, sp, ori, tp)
            except RuntimeError as ex:
                # Fusion calls a dimension "over constrained" when the point already lines up
                # with another dimensioned point. Nudge it off, dimension it, let the expression
                # put it back.
                if 'OVER_CONSTRAINTS' not in str(ex):
                    raise
                n = 0.0013 if v > 0 else -0.0013
                sp.move(adsk.core.Vector3D.create(n if hv == 'h' else 0, n if hv == 'v' else 0, 0))
                d = dims.addDistanceDimension(org, sp, ori, tp)
            d.parameter.expression = e if v > 0 else f'-({e})'

    def circle(self, w, d_expr):
        cir = self.sk.sketchCurves.sketchCircles.addByCenterRadius(self.P(w), self.c.val(d_expr) / 2)
        g = cir.centerSketchPoint.geometry
        dd = self.sk.sketchDimensions.addDiameterDimension(
            cir, adsk.core.Point3D.create(g.x + 0.5, g.y + 0.5, 0))
        dd.parameter.expression = d_expr
        self.fix(cir.centerSketchPoint, w)
        return cir

    def point(self, w):
        sp = self.sk.sketchPoints.add(self.P(w))
        self.fix(sp, w)
        return sp

    def rect(self, a, a0, a1, b, b0, b1):
        """Axis-aligned rectangle from world axis a in [a0,a1], axis b in [b0,b1]."""
        w0 = {a: a0, b: b0}
        w1 = {a: a1, b: b1}
        lines = self.sk.sketchCurves.sketchLines.addTwoPointRectangle(self.P(w0), self.P(w1))
        gc = self.sk.geometricConstraints
        for ln in lines:
            s, e = ln.startSketchPoint.geometry, ln.endSketchPoint.geometry
            try:
                if abs(s.y - e.y) < 1e-9:
                    gc.addHorizontal(ln)
                else:
                    gc.addVertical(ln)
            except Exception:
                pass  # already constrained
        p0, p1 = self.P(w0), self.P(w1)
        pts = {}
        for ln in lines:
            for sp in (ln.startSketchPoint, ln.endSketchPoint):
                pts[sp.entityToken] = sp
        for sp in pts.values():
            g = sp.geometry
            if g.isEqualTo(p0):
                self.fix(sp, w0)
            elif g.isEqualTo(p1):
                self.fix(sp, w1)
        return lines

    def line(self, w0, w1):
        ln = self.sk.sketchCurves.sketchLines.addByTwoPoints(self.P(w0), self.P(w1))
        self.fix(ln.startSketchPoint, w0)
        self.fix(ln.endSketchPoint, w1)
        return ln

    def done(self):
        self.sk.isComputeDeferred = False
        return self.sk

    def profiles(self, near=None):
        self.done()
        col = adsk.core.ObjectCollection.create()
        if near is None:
            for p in self.sk.profiles:
                col.add(p)
            return col
        tgt = self.P(near)
        best = min(self.sk.profiles,
                   key=lambda p: p.areaProperties().centroid.distanceTo(tgt))
        col.add(best)
        return col

    def dir(self, world_dir):
        """Extent direction for a world axis direction like '+z' or '-y'."""
        i = AX[world_dir[1]]
        s = 1 if world_dir[0] == '+' else -1
        n = [self.normal.x, self.normal.y, self.normal.z][i] * s
        return (adsk.fusion.ExtentDirections.PositiveExtentDirection if n > 0
                else adsk.fusion.ExtentDirections.NegativeExtentDirection)


def extrude(comp, sk, profs, dist_expr, world_dir, op, body=None, name=None):
    ef = comp.features.extrudeFeatures
    inp = ef.createInput(profs, op)
    inp.setOneSideExtent(adsk.fusion.DistanceExtentDefinition.create(VI(dist_expr)), sk.dir(world_dir))
    if body is not None:
        inp.participantBodies = [body]
    f = ef.add(inp)
    if name:
        f.name = name
    return f


def fillet_vertical(comp, body, half_expr, c, r_expr, name):
    half = c.val(half_expr)
    col = adsk.core.ObjectCollection.create()
    for e in body.edges:
        g = e.geometry
        if not isinstance(g, adsk.core.Line3D):
            continue
        s, t = g.startPoint, g.endPoint
        if abs(s.x - t.x) > 1e-6 or abs(s.y - t.y) > 1e-6:
            continue
        if abs(abs(s.x) - half) < 1e-5 and abs(abs(s.y) - half) < 1e-5:
            col.add(e)
    if col.count != 4:
        raise RuntimeError(f'{name}: expected 4 vertical corner edges, found {col.count}')
    ff = comp.features.filletFeatures
    inp = ff.createInput()
    inp.edgeSetInputs.addConstantRadiusEdgeSet(col, VI(r_expr), False)
    f = ff.add(inp)
    f.name = name
    return f


# ------------------------------------------------------------------ parts
POST_X = ('cell_x0', 'cell_x0 + mount_len')                       # fixed end, on the floor plate
PILLAR_X = ('cell_x0 + cell_l - mount_len', 'cell_x0 + cell_l')   # free end, on the foot plate
MOUNT_Y = ('cell_y - mount_w/2', 'cell_y + mount_w/2')


def cell_holes(xs, from_end):
    """Two screw positions in a mount block, measured from the cell end."""
    x_end = xs[0] if from_end == 'west' else xs[1]
    sg = '+' if from_end == 'west' else '-'
    return [{'x': f'{x_end} {sg} cell_hole_end', 'y': 'cell_y'},
            {'x': f'{x_end} {sg} (cell_hole_end + cell_hole_pitch)', 'y': 'cell_y'}]


def build_shell(c):
    comp = new_comp(c, 'Shell')
    bot = plane(c, comp, 'xy', 'skirt_z')
    s = Sk(c, comp, bot, 'Outline')
    s.rect('x', '-base_outer/2', 'base_outer/2', 'y', '-base_outer/2', 'base_outer/2')
    body = extrude(comp, s, s.profiles(), 'top_z - skirt_z', '+z', OP.NewBodyFeatureOperation, name='Box').bodies.item(0)
    body.name = 'Shell'
    fillet_vertical(comp, body, 'base_outer/2', c, 'base_r', 'Outer corners')
    s = Sk(c, comp, bot, 'Cavity')
    s.rect('x', '-inner/2', 'inner/2', 'y', '-inner/2', 'inner/2')
    extrude(comp, s, s.profiles(), 'top_in_z - skirt_z', '+z', OP.CutFeatureOperation, body, 'Cavity')
    fillet_vertical(comp, body, 'inner/2', c, 'base_r - wall', 'Inner corners')

    top_in = plane(c, comp, 'xy', 'top_in_z')
    top = plane(c, comp, 'xy', 'top_z')
    # columns down to the PCB mounting holes; M3 from under the floor plate threads in here
    s = Sk(c, comp, top_in, 'Columns')
    for x, y in PCB_MOUNT:
        s.circle({'x': str(x), 'y': str(y)}, 'column_d')
    extrude(comp, s, s.profiles(), 'top_in_z - pcb_top_z', '-z', OP.JoinFeatureOperation, body, 'Columns')
    s = Sk(c, comp, plane(c, comp, 'xy', 'pcb_top_z'), 'Column pilots')
    for x, y in PCB_MOUNT:
        s.circle({'x': str(x), 'y': str(y)}, 'm3_pilot')
    extrude(comp, s, s.profiles(), 'top_in_z - pcb_top_z - 1 mm', '+z', OP.CutFeatureOperation, body, 'Column pilots')

    # collar + shaft hole: the only opening in the top
    s = Sk(c, comp, top, 'Collar')
    s.circle({'x': '0', 'y': '0'}, 'collar_od')
    extrude(comp, s, s.profiles(), 'collar_h', '+z', OP.JoinFeatureOperation, body, 'Collar')
    s = Sk(c, comp, top, 'Collar bore')
    s.circle({'x': '0', 'y': '0'}, 'collar_id')
    extrude(comp, s, s.profiles(), 'collar_h', '+z', OP.CutFeatureOperation, body, 'Collar bore')
    s = Sk(c, comp, top_in, 'Shaft hole')
    s.circle({'x': '0', 'y': '0'}, 'motor_boss_d + 1.5 mm')
    extrude(comp, s, s.profiles(), 'top_t', '+z', OP.CutFeatureOperation, body, 'Shaft hole')

    # blind pilots: bearing bottom plate from above, motor ears from below
    s = Sk(c, comp, top, 'Bearing pilots')
    for sx in ('-', ''):
        for sy in ('-', ''):
            s.circle({'x': f'{sx}bearing_bot_hole', 'y': f'{sy}bearing_bot_hole'}, 'm3_pilot')
    extrude(comp, s, s.profiles(), 'pilot_depth', '-z', OP.CutFeatureOperation, body, 'Bearing pilots')
    s = Sk(c, comp, top_in, 'Motor pilots')
    for sx in ('-', ''):
        s.circle({'x': f'{sx}motor_ear_pitch/2', 'y': 'motor_off'}, 'm3_pilot')
    extrude(comp, s, s.profiles(), 'pilot_depth', '+z', OP.CutFeatureOperation, body, 'Motor pilots')

    # HX711 posts under the top
    hx = [{'x': 'hx_x0 + 3 mm', 'y': 'hx_y0 + 3 mm'}, {'x': 'hx_x0 + 19 mm', 'y': 'hx_y0 + 3 mm'}]
    s = Sk(c, comp, top_in, 'HX711 posts')
    for w in hx:
        s.circle(w, 'hx_post_d')
    extrude(comp, s, s.profiles(), 'hx_post_h', '-z', OP.JoinFeatureOperation, body, 'HX711 posts')
    s = Sk(c, comp, plane(c, comp, 'xy', 'top_in_z - hx_post_h'), 'HX711 pilots')
    for w in hx:
        s.circle(w, 'hx_screw')
    extrude(comp, s, s.profiles(), 'hx_post_h + 1 mm', '+z', OP.CutFeatureOperation, body, 'HX711 pilots')

    # front wall: pot
    inside = plane(c, comp, 'xz', 'inner/2')
    outside = plane(c, comp, 'xz', 'base_outer/2')
    for nm, x, d, t, hole in (('Pot', 'pot_x', 'pot_boss_d', 'pot_wall', 'pot_hole'),):
        s = Sk(c, comp, inside, f'{nm} boss')
        s.circle({'x': x, 'z': 'wall_mid_z'}, d)
        extrude(comp, s, s.profiles(), f'{t} - wall', '-y', OP.JoinFeatureOperation, body, f'{nm} boss')
        s = Sk(c, comp, outside, f'{nm} hole')
        s.circle({'x': x, 'z': 'wall_mid_z'}, hole)
        extrude(comp, s, s.profiles(), f'{t} + 1 mm', '-y', OP.CutFeatureOperation, body, f'{nm} hole')

    # back wall: notch the foot plate's USB-C + LED plate sits in, face flush
    s = Sk(c, comp, plane(c, comp, 'xz', '-base_outer/2'), 'USB-C notch')
    s.rect('x', 'usb_l - usb_side_gap', 'usb_r + usb_side_gap', 'z', 'skirt_z - 1 mm', 'usb_panel_top + usb_win_clear')
    extrude(comp, s, s.profiles(), 'wall + 1 mm', '+y', OP.CutFeatureOperation, body, 'USB-C notch')
    return comp


def build_floor(c):
    comp = new_comp(c, 'Floor plate')
    bot = plane(c, comp, 'xy', 'floor_bot_z')
    s = Sk(c, comp, bot, 'Floor')
    s.rect('x', '-floor_sq/2', 'floor_sq/2', 'y', '-floor_sq/2', 'floor_sq/2')
    body = extrude(comp, s, s.profiles(), 'floor_t', '+z', OP.NewBodyFeatureOperation, name='Floor').bodies.item(0)
    body.name = 'Floor plate'
    fillet_vertical(comp, body, 'floor_sq/2', c, 'floor_r', 'Corners')

    top = plane(c, comp, 'xy', 'floor_top_z')
    s = Sk(c, comp, top, 'Standoffs')
    for x, y in PCB_MOUNT:
        s.circle({'x': str(x), 'y': str(y)}, '7 mm')
    extrude(comp, s, s.profiles(), 'standoff_h', '+z', OP.JoinFeatureOperation, body, 'Standoffs')
    s = Sk(c, comp, bot, 'Shell screws')
    for x, y in PCB_MOUNT:
        s.circle({'x': str(x), 'y': str(y)}, '3.4 mm')
    extrude(comp, s, s.profiles(), 'floor_t + standoff_h', '+z', OP.CutFeatureOperation, body, 'Shell screws')
    s = Sk(c, comp, bot, 'Shell screw heads')
    for x, y in PCB_MOUNT:
        s.circle({'x': str(x), 'y': str(y)}, '6.5 mm')
    extrude(comp, s, s.profiles(), '3 mm', '+z', OP.CutFeatureOperation, body, 'Shell screw heads')

    # fixed-end post, screws from under the floor into the cell
    s = Sk(c, comp, top, 'Cell post')
    s.rect('x', *POST_X, 'y', *MOUNT_Y)
    extrude(comp, s, s.profiles(), 'cell_bot_z - floor_top_z', '+z', OP.JoinFeatureOperation, body, 'Cell post')
    holes = cell_holes(POST_X, 'west')
    s = Sk(c, comp, bot, 'Cell screws')
    for w in holes:
        s.circle(w, 'cell_hole_d')
    extrude(comp, s, s.profiles(), 'cell_bot_z - floor_bot_z', '+z', OP.CutFeatureOperation, body, 'Cell screws')
    s = Sk(c, comp, bot, 'Cell screw heads')
    for w in holes:
        s.circle(w, 'cell_hole_d + 4.5 mm')
    extrude(comp, s, s.profiles(), '3 mm', '+z', OP.CutFeatureOperation, body, 'Cell screw heads')

    # opening for the foot plate's pillar, and the notch over the USB-C socket
    s = Sk(c, comp, bot, 'Pillar opening')
    s.rect('x', f'{PILLAR_X[0]} - pcb_clear', f'{PILLAR_X[1]} + pcb_clear',
           'y', f'{MOUNT_Y[0]} - pcb_clear', f'{MOUNT_Y[1]} + pcb_clear')
    extrude(comp, s, s.profiles(), 'floor_t', '+z', OP.CutFeatureOperation, body, 'Pillar opening')
    s = Sk(c, comp, bot, 'USB-C notch')
    s.rect('x', 'usb_l - 1 mm', 'usb_r + 1 mm',
           'y', '-floor_sq/2 - 1 mm', 'usb_face_y + usb_panel_t + usb_depth + 2 mm')
    extrude(comp, s, s.profiles(), 'floor_t', '+z', OP.CutFeatureOperation, body, 'USB-C notch')
    return comp


def build_foot(c):
    comp = new_comp(c, 'Foot plate')
    xy = comp.xYConstructionPlane
    s = Sk(c, comp, xy, 'Foot plate')
    s.rect('x', '-foot_sq/2', 'foot_sq/2', 'y', '-foot_sq/2', 'foot_sq/2')
    body = extrude(comp, s, s.profiles(), 'foot_t', '+z', OP.NewBodyFeatureOperation, name='Foot').bodies.item(0)
    body.name = 'Foot plate'
    fillet_vertical(comp, body, 'foot_sq/2', c, 'foot_r', 'Corners')
    top = plane(c, comp, 'xy', 'foot_t')

    # pillar up to the cell's free end, screws from under the foot plate
    s = Sk(c, comp, top, 'Pillar')
    s.rect('x', *PILLAR_X, 'y', *MOUNT_Y)
    extrude(comp, s, s.profiles(), 'cell_bot_z - foot_t', '+z', OP.JoinFeatureOperation, body, 'Pillar')
    holes = cell_holes(PILLAR_X, 'east')
    s = Sk(c, comp, xy, 'Cell screws')
    for w in holes:
        s.circle(w, 'cell_hole_d')
    extrude(comp, s, s.profiles(), 'cell_bot_z', '+z', OP.CutFeatureOperation, body, 'Cell screws')
    s = Sk(c, comp, xy, 'Cell screw heads')
    for w in holes:
        s.circle(w, 'cell_hole_d + 4.5 mm')
    extrude(comp, s, s.profiles(), '3 mm', '+z', OP.CutFeatureOperation, body, 'Cell screw heads')

    # overload stops
    s = Sk(c, comp, top, 'Stop bumps')
    for x, y in STOP_BUMPS:
        s.circle({'x': f'{x} mm', 'y': f'{y} mm'}, '6 mm')
    extrude(comp, s, s.profiles(), 'stop_h', '+z', OP.JoinFeatureOperation, body, 'Stop bumps')

    # USB-C + LED plate on the back edge, face flush with the shell, on a tongue of the foot plate
    s = Sk(c, comp, xy, 'USB-C tongue')
    s.rect('x', 'usb_l', 'usb_r', 'y', 'usb_face_y + usb_panel_t', '-foot_sq/2 + 1 mm')
    extrude(comp, s, s.profiles(), 'foot_t', '+z', OP.JoinFeatureOperation, body, 'USB-C tongue')
    s = Sk(c, comp, xy, 'USB-C plate')
    s.rect('x', 'usb_l', 'usb_r', 'y', 'usb_face_y', 'usb_face_y + usb_panel_t')
    extrude(comp, s, s.profiles(), 'usb_panel_top', '+z', OP.JoinFeatureOperation, body, 'USB-C plate')
    s = Sk(c, comp, xy, 'LED boss')
    s.rect('x', 'led_x - led_boss_w/2', 'led_x + led_boss_w/2', 'y', 'usb_face_y + usb_panel_t', 'usb_face_y + led_wall')
    extrude(comp, s, s.profiles(), 'pcb_bottom_z - 1 mm', '+z', OP.JoinFeatureOperation, body, 'LED boss')
    # pockets in the foot plate under the socket and the LED (both sit lower than its top)
    s = Sk(c, comp, top, 'USB-C pocket')
    s.rect('x', 'usb_x - usb_body_w/2 - 0.5 mm', 'usb_x + usb_body_w/2 + 0.5 mm',
           'y', 'usb_face_y + usb_panel_t', 'usb_face_y + usb_panel_t + usb_depth + 1 mm')
    extrude(comp, s, s.profiles(), 'foot_t - (usb_z - usb_body_h/2 - 0.5 mm)', '-z', OP.CutFeatureOperation, body, 'USB-C pocket')
    s = Sk(c, comp, top, 'LED pocket')
    s.rect('x', 'led_x - led_hole/2 - 0.5 mm', 'led_x + led_hole/2 + 0.5 mm',
           'y', 'usb_face_y + led_wall', 'usb_face_y + led_wall + led_depth + 1 mm')
    extrude(comp, s, s.profiles(), 'foot_t - (usb_z - led_hole/2 - 0.5 mm)', '-z', OP.CutFeatureOperation, body, 'LED pocket')
    face = plane(c, comp, 'xz', 'usb_face_y')
    s = Sk(c, comp, face, 'USB-C opening')
    s.rect('x', 'usb_x - usb_cut_w/2', 'usb_x + usb_cut_w/2', 'z', 'usb_z - usb_cut_h/2', 'usb_z + usb_cut_h/2')
    extrude(comp, s, s.profiles(), 'usb_panel_t + 1 mm', '+y', OP.CutFeatureOperation, body, 'USB-C opening')
    s = Sk(c, comp, face, 'USB-C flange')
    s.rect('x', 'usb_x - usb_flange_w/2 - 0.15 mm', 'usb_x + usb_flange_w/2 + 0.15 mm',
           'z', 'usb_z - usb_flange_h/2 - 0.15 mm', 'usb_z + usb_flange_h/2 + 0.15 mm')
    extrude(comp, s, s.profiles(), 'usb_flange_t', '+y', OP.CutFeatureOperation, body, 'USB-C flange pocket')
    s = Sk(c, comp, face, 'USB-C screws')
    for sx in ('-', '+'):
        s.circle({'x': f'usb_x {sx} usb_hole_pitch/2', 'z': 'usb_z'}, 'usb_hole_d')
    extrude(comp, s, s.profiles(), 'usb_panel_t + 1 mm', '+y', OP.CutFeatureOperation, body, 'USB-C screws')
    s = Sk(c, comp, face, 'LED hole')
    s.circle({'x': 'led_x', 'z': 'usb_z'}, 'led_hole')
    extrude(comp, s, s.profiles(), 'led_wall + 1 mm', '+y', OP.CutFeatureOperation, body, 'LED hole')
    return comp


def build_platform(c):
    comp = new_comp(c, 'Platform')
    pz = plane(c, comp, 'xy', 'platform_z')
    s = Sk(c, comp, pz, 'Platform')
    s.circle({'x': '0', 'y': '0'}, 'platform_d')
    body = extrude(comp, s, s.profiles(), 'platform_t', '+z', OP.NewBodyFeatureOperation, name='Platform').bodies.item(0)
    body.name = 'Platform'

    # hub reaches down inside the collar onto the shaft
    hub_pl = plane(c, comp, 'xy', 'top_z + 0.5 mm')
    s = Sk(c, comp, hub_pl, 'Hub')
    s.circle({'x': '0', 'y': '0'}, 'hub_d')
    extrude(comp, s, s.profiles(), 'platform_z - top_z - 0.5 mm', '+z', OP.JoinFeatureOperation, body, 'Hub')
    s = Sk(c, comp, hub_pl, 'D-shaft')
    s.circle({'x': '0', 'y': '0'}, 'shaft_d + 0.25 mm')
    for sg in ('-', ''):
        s.line({'x': f'{sg}(shaft_flat + 0.25 mm)/2', 'y': '-4 mm'},
               {'x': f'{sg}(shaft_flat + 0.25 mm)/2', 'y': '4 mm'})
    # blind: 1 mm past the shaft tip (motor face = top_in_z)
    extrude(comp, s, s.profiles(near={'x': '0', 'y': '0'}), 'top_in_z + shaft_len + 1 mm - top_z - 0.5 mm', '+z',
            OP.CutFeatureOperation, body, 'D-shaft')

    # drive pins into the bearing top plate's holes (platform just rests on the bearing)
    s = Sk(c, comp, pz, 'Drive pins')
    for w in ({'x': 'bearing_top_hole', 'y': '0'}, {'x': '-bearing_top_hole', 'y': '0'},
              {'x': '0', 'y': 'bearing_top_hole'}, {'x': '0', 'y': '-bearing_top_hole'}):
        s.circle(w, 'peg_d')
    extrude(comp, s, s.profiles(), 'peg_h', '-z', OP.JoinFeatureOperation, body, 'Drive pins')
    return comp


def build_reference(c):
    """Bought parts as see-through placeholders for fit checks (not printed)."""
    comp = new_comp(c, 'Reference parts')
    NB = OP.NewBodyFeatureOperation

    def box(name, z0, a, a0, a1, b, b0, b1, h, base='xy', d='+z'):
        s = Sk(c, comp, plane(c, comp, base, z0), name)
        s.rect(a, a0, a1, b, b0, b1)
        bd = extrude(comp, s, s.profiles(), h, d, NB, name=name).bodies.item(0)
        bd.name = name
        return bd

    def cyl(name, z0, w, dia, h, base='xy', d='+z'):
        s = Sk(c, comp, plane(c, comp, base, z0), name)
        s.circle(w, dia)
        bd = extrude(comp, s, s.profiles(), h, d, NB, name=name).bodies.item(0)
        bd.name = name
        return bd

    pcb = box('PCB', 'pcb_bottom_z', 'x', '-51.5 mm', '51.5 mm', 'y', '-51.5 mm', '37 mm', 'pcb_t')
    # the PCB cutouts this case needs (post + pillar with pcb_clear all round)
    s = Sk(c, comp, plane(c, comp, 'xy', 'pcb_bottom_z'), 'PCB cutouts')
    for xs in (POST_X, PILLAR_X):
        s.rect('x', f'{xs[0]} - pcb_clear', f'{xs[1]} + pcb_clear',
               'y', f'{MOUNT_Y[0]} - pcb_clear', f'{MOUNT_Y[1]} + pcb_clear')
    extrude(comp, s, s.profiles(), 'pcb_t', '+z', OP.CutFeatureOperation, pcb, 'PCB cutouts')

    box('Load cell', 'cell_bot_z', 'x', 'cell_x0', 'cell_x0 + cell_l', 'y', 'cell_y - cell_w/2', 'cell_y + cell_w/2', 'cell_h')
    box('21700 holder', 'pcb_top_z', 'x', '-43 mm', '40 mm', 'y', '-48.5 mm', '-23.5 mm', 'batt_h')
    cyl('Motor', 'top_in_z - motor_body_h', {'x': '0', 'y': 'motor_off'}, 'motor_body_d', 'motor_body_h')
    bearing = box('Bearing', 'top_z', 'x', '-bearing_sq/2', 'bearing_sq/2', 'y', '-bearing_sq/2', 'bearing_sq/2', 'bearing_h')
    s = Sk(c, comp, plane(c, comp, 'xy', 'top_z'), 'Bearing bore')
    s.circle({'x': '0', 'y': '0'}, '45 mm')
    extrude(comp, s, s.profiles(), 'bearing_h', '+z', OP.CutFeatureOperation, bearing, 'Bearing bore')
    s = Sk(c, comp, plane(c, comp, 'xy', 'platform_z'), 'Bearing top holes')
    for w in ({'x': 'bearing_top_hole', 'y': '0'}, {'x': '-bearing_top_hole', 'y': '0'},
              {'x': '0', 'y': 'bearing_top_hole'}, {'x': '0', 'y': '-bearing_top_hole'}):
        s.circle(w, 'peg_d + 0.4 mm')
    extrude(comp, s, s.profiles(), 'peg_h + 0.5 mm', '-z', OP.CutFeatureOperation, bearing, 'Bearing top holes')
    box('HX711', 'top_in_z - hx_post_h - 1.6 mm', 'x', 'hx_x0', 'hx_x0 + 22 mm', 'y', 'hx_y0', 'hx_y0 + 22 mm', '1.6 mm')
    cyl('Pot', 'inner/2 - pot_wall + wall', {'x': 'pot_x', 'z': 'wall_mid_z'}, 'pot_body_d', 'pot_depth', 'xz', '-y')
    box('USB-C socket', 'usb_face_y + usb_panel_t', 'x', 'usb_x - usb_body_w/2', 'usb_x + usb_body_w/2',
        'z', 'usb_z - usb_body_h/2', 'usb_z + usb_body_h/2', 'usb_depth', 'xz', '+y')
    cyl('Charge LED', 'usb_face_y + led_wall', {'x': 'led_x', 'z': 'usb_z'}, '5.8 mm', 'led_depth', 'xz', '+y')
    for b in comp.bRepBodies:
        b.opacity = 0.45
    return comp


# ------------------------------------------------------------------ checks
def checks(c):
    v = lambda e: c.val(e) * 10  # mm
    msgs = []

    def need(ok, msg):
        if not ok:
            msgs.append(msg)

    need(abs(v('pot_x')) < 0.01, 'pot is not centred')
    need(abs(v('usb_x - usb_flange_w/2') + v('led_x + led_flange_d/2')) < 0.01, 'USB-C + LED pair is not centred')
    need(v('led_x - led_flange_d/2') - v('usb_x + usb_flange_w/2 + 0.15 mm') >= 1, 'LED holder flange touches the USB-C flange')
    need(v('led_x - led_boss_w/2') - v('usb_x + usb_hole_pitch/2 + usb_nut_d/2') >= 0.5, 'LED boss leaves no room for the USB-C nut')
    need(v('usb_hole_pitch/2 - usb_nut_d/2') - v('usb_body_w/2') >= 0.5, 'USB-C nuts hit the socket body')
    need(v('led_depth') <= v('usb_depth'), 'LED reaches past the floor plate notch')
    need(v('platform_d') / 2 >= v('bearing_sq') * math.sqrt(2) / 2 + 3, "platform doesn't cover the bearing corners")
    need(v('bearing_sq') / 2 < v('base_outer') / 2, 'bearing overhangs the shell')
    need(v('inner/2 - pot_depth') > 37 + 1, 'pot body reaches over the PCB')
    need(v('wall_mid_z - pot_body_d/2') > v('floor_top_z') + 1, 'pot body hits the floor plate')
    need(v('wall_mid_z + pot_body_d/2') < v('top_in_z') - 1, 'pot body hits the top')
    need(v('float_gap') - v('stop_h') < v('usb_win_clear'), 'case can hit the USB-C panel before the stops')
    need(v('usb_face_y + usb_panel_t') <= -51.5 - 0.3, 'USB-C plate reaches the PCB edge')
    need(v('usb_panel_top') + v('usb_win_clear') <= v('pcb_top_z') + 1e-6, 'gap above the plate is not backed by the PCB edge')
    need(v('usb_panel_top') >= v('pcb_bottom_z') + 0.3, 'gap above the plate is not backed by the PCB edge')
    need(v('usb_z + usb_body_h/2') + v('float_gap - stop_h') + 0.3 <= v('pcb_bottom_z'), 'PCB can land on the USB-C socket')
    need(v('usb_z + 2.9 mm') + v('float_gap - stop_h') + 0.3 <= v('pcb_bottom_z'), 'PCB can land on the LED')
    need(v('usb_z - usb_flange_h/2 - 0.15 mm') >= 0.8, 'sliver under the USB-C flange recess too thin')
    need(v('skirt_z') < v('foot_t'), 'skirt does not hide the gap')
    need(v('collar_id') > v('hub_d') + 1, 'hub rubs the collar')
    need(v('top_in_z - motor_body_h') > v('pcb_top_z') + 3, 'motor too close to the PCB')
    return msgs


def interference(c):
    """Solid overlaps between all bodies (should be none)."""
    col = adsk.core.ObjectCollection.create()
    for occ in c.root.allOccurrences:
        for b in occ.bRepBodies:
            col.add(b)
    res = c.design.analyzeInterference(c.design.createInterferenceInput(col))
    return [f'{r.entityOne.name} × {r.entityTwo.name}: {r.interferenceBody.volume * 1000:.1f} mm³' for r in res]


def reset(c):
    """Remove what a previous build() made (its components and parameters)."""
    names = {'Shell', 'Floor plate', 'Foot plate', 'Platform', 'Reference parts',
             'Base', 'Deck', 'Riser'}  # last three: the old floating-deck layout
    for occ in list(c.root.occurrences):
        if occ.component.name in names:
            occ.deleteMe()
    ups = c.design.userParameters
    for p in reversed([ups.item(i) for i in range(ups.count)]):
        p.deleteMe()


COLOURS = {  # display only
    'Shell': 'Plastic - Matte (White)', 'Floor plate': 'Plastic - Matte (Blue)',
    'Foot plate': 'Plastic - Matte (Gray)', 'Platform': 'Plastic - Matte (Yellow)',
    'PCB': 'Plastic - Matte (Green)', 'Load cell': 'Plastic - Matte (Red)',
    'Bearing': 'Plastic - Matte (Black)', 'Motor': 'Plastic - Matte (Yellow)',
}


def colourise(c):
    app = adsk.core.Application.get()
    lib = app.materialLibraries.itemByName('Fusion Appearance Library')
    if not lib:
        return
    for occ in c.root.occurrences:
        for b in occ.component.bRepBodies:
            name = COLOURS.get(b.name)
            if name:
                a = c.design.appearances.itemByName(name) or c.design.appearances.addByCopy(lib.appearances.itemByName(name), name)
                b.appearance = a


def build(parts=PARTS, clean=False):
    app = adsk.core.Application.get()
    design = adsk.fusion.Design.cast(app.activeProduct)
    design.designType = adsk.fusion.DesignTypes.ParametricDesignType
    # newer Fusion: a Part design holds one component; the case needs several
    if design.designIntent == adsk.fusion.DesignIntentTypes.PartDesignIntentType:
        design.designIntent = adsk.fusion.DesignIntentTypes.HybridDesignIntentType
    c = Ctx(design)
    if clean:
        reset(c)
    ensure_params(c)
    fns = {'shell': build_shell, 'floor': build_floor, 'foot': build_foot,
           'platform': build_platform, 'reference': build_reference}
    for p in parts:
        fns[p](c)
    colourise(c)
    for m in checks(c):
        print('CHECK FAILED:', m)
    return c
