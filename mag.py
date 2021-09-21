import cadquery as cq

def make_mag(clearance=0, inner_clearance=None):
    epsilon=.001 if clearance == 0 else 0
    if inner_clearance is None:
        inner_clearance = clearance
    prev = cq.Workplane("XY").workplane(offset=-clearance)
    l = []
    for diam,thick in [
            (7, 1 + clearance + inner_clearance),
            (5, 4-1-1-1.3 - inner_clearance + inner_clearance),
            (3.5, 1.3 - 2*inner_clearance),
            (5, 1 + clearance + inner_clearance),
    ]:
        m = (prev
             .rect(14 + diam + 2*clearance, diam + 2*clearance)
             .extrude(thick, combine=False)
             .edges("|Z").fillet(diam/2 - epsilon)
        )
        prev = m.faces("+Z and >Z").workplane()
        l.append(m)
    m = l.pop(0)
    for m2 in l:
        m = m.union(m2)
    return m

show_object(make_mag(0.25), name='mag')
