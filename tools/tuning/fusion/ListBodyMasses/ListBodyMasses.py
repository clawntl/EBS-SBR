"""Fusion script: list every solid body with its material, volume, mass and
centre of mass, so you can see where the design's mass comes from.

Install: Utilities > Add-Ins > Scripts and Add-Ins > Create (Python script),
name it ListBodyMasses, replace its .py with this file, then Run.

Writes ~/Desktop/fusion_body_masses.csv (one row per body, heaviest first)
and shows a summary: totals by material and the 15 heaviest bodies.
"""
import adsk.core, adsk.fusion, traceback, os, csv


def run(context):
    ui = None
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface
        design = adsk.fusion.Design.cast(app.activeProduct)
        if not design:
            ui.messageBox('Open a design first.')
            return
        root = design.rootComponent
        rows = []

        def add(body, path, occ_visible):
            if not body.isSolid:
                return  # surface bodies have no mass
            pp = body.physicalProperties
            com = pp.centerOfMass  # cm, in assembly coordinates
            mat = body.material.name if body.material else '(none)'
            rows.append({
                'path': path + '/' + body.name,
                'material': mat,
                'density_g_cm3': round(pp.density * 1000.0, 4),  # API: kg/cm^3
                'volume_cm3': round(pp.volume, 3),
                'mass_g': round(pp.mass * 1000.0, 3),            # API: kg
                'com_x_mm': round(com.x * 10, 2),
                'com_y_mm': round(com.y * 10, 2),
                'com_z_mm': round(com.z * 10, 2),
                'visible': bool(body.isVisible and occ_visible),
            })

        for b in root.bRepBodies:
            add(b, root.name, True)
        for occ in root.allOccurrences:
            for b in occ.bRepBodies:  # proxies: positions are in assembly space
                add(b, occ.fullPathName, occ.isVisible)

        rows.sort(key=lambda r: -r['mass_g'])
        out = os.path.join(os.path.expanduser('~'), 'Desktop', 'fusion_body_masses.csv')
        with open(out, 'w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
            w.writeheader()
            w.writerows(rows)

        total = sum(r['mass_g'] for r in rows)
        vis = sum(r['mass_g'] for r in rows if r['visible'])
        by_mat = {}
        for r in rows:
            m = by_mat.setdefault(r['material'], [0.0, 0])
            m[0] += r['mass_g']
            m[1] += 1
        lines = ['Total %.1f g (visible only %.1f g), %d bodies' % (total, vis, len(rows)), '',
                 'By material:']
        for name, (g, n) in sorted(by_mat.items(), key=lambda kv: -kv[1][0]):
            lines.append('  %-28s %8.1f g  (%d bodies)' % (name, g, n))
        lines += ['', 'Heaviest bodies:']
        for r in rows[:15]:
            lines.append('  %7.1f g  %-18s %s%s' % (r['mass_g'], r['material'][:18], r['path'],
                                                    '' if r['visible'] else '  [hidden]'))
        lines += ['', 'Full list: ' + out]
        ui.messageBox('\n'.join(lines), 'Body masses')
    except Exception:
        if ui:
            ui.messageBox('Failed:\n{}'.format(traceback.format_exc()))
