"""Printable D405 wrist mount R2, mm. CadQuery 2.x, Python 3.10.

World: Robotiq vendor closed frame, Y toward tips, +Z camera side.
Local camera: +X image-right, +Y image-up, +Z scene, front glass z=0.
No calibration, contact mechanics or strength simulation is implied.
"""
from pathlib import Path
import math, json
import cadquery as cq
import numpy as np
import trimesh
from shapely.geometry import MultiPoint

OUT=Path(__file__).resolve().parent
STUDY=OUT.parent/'wrist_mount_research_20260905'
ORIGIN=(0,70,64)
PITCH=30.0
ANGLE=PITCH-90

def box(a,b,c,centre):
    return cq.Workplane('XY').box(a,b,c).val().translate(centre)

def rounded_box(a,b,c,centre,r):
    return cq.Workplane('XY').box(a,b,c).edges('|Z').fillet(r).val().translate(centre)

def world(s):
    # Local frame [(-1,0,0),(0,-sin(a),-cos(a)),(0,cos(a),-sin(a))].
    return s.rotate((0,0,0),(1,0,0),ANGLE).rotate((0,0,0),(0,1,0),180).translate(ORIGIN)

def local(s):
    return s.translate(tuple(-v for v in ORIGIN)).rotate((0,0,0),(0,1,0),180).rotate((0,0,0),(1,0,0),-ANGLE)

def bounds(s):
    b=s.BoundingBox();return [[b.xmin,b.ymin,b.zmin],[b.xmax,b.ymax,b.zmax]]

def build():
    seat=cq.importers.importStep(str(OUT/'reference'/'preserved_two_screw_seat.step')).val()
    # Rear camera contact face z=-23; plate 5 mm -> M3x8 + 0.5 washer = 2.5 insertion.
    plate=rounded_box(49.6,49.6,5,(0,0,-25.5),3)
    for y in [-13,13]:
        plate=plate.cut(rounded_box(33,9,7,(0,y,-25.5),2))
    # Relief beside USB Micro-B; symmetric so either 180-degree camera roll fits.
    for x in [-24,24]:
        plate=plate.cut(box(12,20,7,(x,0,-25.5)))
    # 2.5 mm cable-tie slots in side rails, paired above/below the open USB area.
    for x in [-21.5,21.5]:
        for y in [-15,15]:
            plate=plate.cut(rounded_box(1.8,3.4,7,(x,y,-25.5),.5))
    cage=plate
    # Two continuous protective rails. No window bridges or corner fingers.
    # Open sides let the USB housing slide in and leave space for the cable.
    # 43.6 mm clear spacing for the 42 mm camera (0.8 mm each side).
    for y in [-23.3,23.3]:
        wall=box(49.6,3,26,(0,y,-10))
        cage=cage.fuse(wall)
    # Rounded exterior corners on the guard, matching the rear frame.
    cage=cage.intersect(rounded_box(49.6,49.6,50,(0,0,-10),3)).clean()
    carrier=seat.fuse(world(cage))
    # 5 mm ribs stay clear of the M3 bores; blend with broad contact to the seat.
    a=math.radians(PITCH);up=np.array([0,-math.sin(a),-math.cos(a)]);f=np.array([0,math.cos(a),-math.sin(a)])
    def yz(u,z):
        v=np.array(ORIGIN)+up*u+f*z;return (v[1],v[2])
    profile=list(MultiPoint([(38,20),(41,20),(41,39),yz(22,-26),yz(-22,-26)]).convex_hull.exterior.coords)[:-1]
    for x in [-17,12]:
        rib=cq.Workplane('YZ',origin=(x,0,0)).polyline(profile).close().extrude(5).val()
        # Preserve a flat rear plane for printing and screw/washer access.
        rib=rib.cut(world(box(100,180,100,(0,0,-78))))
        rib=rib.cut(world(box(43.6,43.6,80,(0,0,16.99))))
        # Clear the fixed Robotiq housing: its round shell reaches Z=37.5;
        # the retained mounting face is Y=36.3. Relief is 1 mm ahead of
        # that face and 2.5 mm above the shell, without cutting the seat.
        relief=box(100,150,140,(0,-37.7,-30))
        relief=cq.Workplane(obj=relief).edges('|X').fillet(2).val()
        rib=rib.cut(relief)
        carrier=carrier.fuse(rib)
    # Recut camera holes through the complete finished body, never just plate.
    for x in [-10,10]:
        bore=cq.Solid.makeCylinder(1.7,12,cq.Vector(x,0,-31),cq.Vector(0,0,1))
        carrier=carrier.cut(world(bore))
    carrier=carrier.clean()
    # Nominal envelope for verification. USB housing is a conservative box on
    # both sides, not an assertion of the vendor's complete housing shape.
    camera=box(42,42,23,(0,0,-11.5))
    plug=box(40,20,10,(41,0,-18.2)).fuse(box(40,20,10,(-41,0,-18.2)))
    camera_with_bulges=camera.fuse(box(8,16,12,(23,0,-17))).fuse(box(8,16,12,(-23,0,-17)))
    cl=local(carrier)
    assert carrier.isValid() and len(carrier.Solids())==1
    qa={'status':'Printable prototype R2; physical fit and motion check required',
        'camera_front_centre_world_mm':ORIGIN,'pitch_deg':PITCH,
        'two_screw_interface':{'axes':'Y','centres_XZ_mm':[[-6,27],[6,27]],'mating_face_Y_mm':36.3,
        'through_diameter_mm':4.5,'counterbore_diameter_mm':8,'preserved_seat_missing_mm3':seat.cut(carrier).Volume()},
        'solid_count':len(carrier.Solids()),'brep_valid':carrier.isValid(),
        'volume_mm3':carrier.Volume(),'world_bounds_mm':bounds(carrier),
        'nominal_camera_overlap_mm3':cl.intersect(camera_with_bulges).Volume(),
        'usb_plug_envelope_overlap_mm3':cl.intersect(plug).Volume(),
        'camera_screw_insertion_mm':2.5,'camera_screws':'2 x M3x8 socket head, 0.5 mm plain washer',
        'camera_cavity_spacing_mm':43.6,'front_guard_projection_mm':3.0}
    assert qa['two_screw_interface']['preserved_seat_missing_mm3']<1e-5
    assert qa['nominal_camera_overlap_mm3']<1e-5
    assert qa['usb_plug_envelope_overlap_mm3']<1e-5
    # Straight front-loading swept clearance, including oversize USB housing.
    insertion=box(42.4,42.4,103,(0,0,28.5))
    for x in [-23,23]:insertion=insertion.fuse(box(8.4,16.4,110,(x,0,27)))
    qa['front_insertion_sweep_overlap_mm3']=cl.intersect(insertion).Volume()
    assert qa['front_insertion_sweep_overlap_mm3']<1e-5
    # Camera screw head/driver access behind mounting face and gripper drivers.
    tests=[]
    for x in [-10,10]:
        tests.append(cl.intersect(cq.Solid.makeCylinder(4,50,cq.Vector(x,0,-78.01),cq.Vector(0,0,1))).Volume())
    for x in [-6,6]:
        tests.append(carrier.intersect(cq.Solid.makeCylinder(3.8,100,cq.Vector(x,42.301,27),cq.Vector(0,1,0))).Volume())
    qa['driver_access_overlaps_mm3']=tests
    assert max(tests)<1e-5
    cq.exporters.export(carrier,str(OUT/'D405_wrist_mount_R2.step'))
    cq.exporters.export(world(camera),str(OUT/'D405_envelope_reference.step'))
    # Flat rear plate on bed, camera aperture pointing up. Support elsewhere.
    printpart=cl.translate((0,0,28))
    cq.exporters.export(printpart,str(OUT/'D405_wrist_mount_R2_PRINT.stl'),tolerance=.03,angularTolerance=.08)
    mesh=trimesh.load(OUT/'D405_wrist_mount_R2_PRINT.stl',force='mesh')
    qa['stl']={'watertight':bool(mesh.is_watertight),'winding_consistent':bool(mesh.is_winding_consistent),
        'connected_components':len(mesh.split()),'volume_mm3':float(mesh.volume),'bounds_mm':mesh.bounds.tolist(),
        'dimensions_mm':mesh.extents.tolist(),'triangles':len(mesh.faces)}
    assert mesh.is_watertight and mesh.is_winding_consistent and len(mesh.split())==1 and mesh.volume>0
    assert abs(mesh.bounds[0,2])<.001
    v,t=carrier.tessellate(.12)
    (OUT/'mount_mesh.json').write_text(json.dumps({'vertices_mm':[p.toTuple() for p in v],'indices':[i for tri in t for i in tri]}))
    (OUT/'mechanical_checks.json').write_text(json.dumps(qa,indent=2))
    print(json.dumps(qa,indent=2))

if __name__=='__main__':build()
