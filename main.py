import bpy
import bmesh
from mathutils import Vector

# GEARS IMPORTS
from simplefractions import simplest_in_interval
from typing import Union
import math
from math import sqrt
from typing import Union
from scipy.spatial import distance


def purge():
    bpy.ops.object.select_all(action='SELECT')
    bpy.ops.object.delete(use_global=False)

    # why three times? maybe no need
    bpy.ops.outliner.orphans_purge()
    bpy.ops.outliner.orphans_purge()
    bpy.ops.outliner.orphans_purge()


class GearSetup:
    def __init__(self, dp, norm_gear_amount, norm_gear_diam, last_gear_diam):
        self.dp = dp
        self.norm_gear_teeth_num = int(dp * norm_gear_diam)
        self.last_gear_teeth_num = int(dp * last_gear_diam)
        self.norm_gear_amount = int(norm_gear_amount)
        self.norm_gear_rad = round(float(norm_gear_diam), 4) / 2
        self.last_gear_rad = round(float(last_gear_diam), 4) / 2

    def __str__(self):
        return f"YOYO Gear  SetUp is:\n" \
               f"DP is {self.dp}\n" \
               f"Gear sizes: {self.norm_gear_diam} for the normal and {self.last_gear_diam} for the last\n" \
               f"Gear amounts: {self.norm_gear_amount} of normal and 1 of last\n" \
               f"Gear teeth num is: {self.norm_gear_teeth_num} for norm and {self.last_gear_teeth_num} for last"


# ToDo
class WingsUserInfo:
    def __init__(self, point1: list, point2: list, length_eps: float, width_constraint: Union[float, int],
                 angle1: Union[float, int], angle2: Union[float, int], ratio_eps: float, connecting_point_body: list,
                 connecting_point_another_wing: list
                 , normal: list):
        self.wing_ends = [point1, point2]
        self.length_eps = length_eps
        self.ratio_eps = ratio_eps
        self.width_constraints = width_constraint
        self.angels = [angle1, angle2]
        self.length = distance.euclidean(point1, point2)
        self.connnecting_point_body = connecting_point_body
        self.connnecting_point_another_wing = connecting_point_another_wing
        self.normal = normal

    # l for length_eps and r for ratio
    def change_eps(self, new_eps, indicator):
        if indicator == 'r':
            self.ratio_eps = new_eps
        if indicator == 'l':
            self.length_eps = new_eps


class BodyGear:
    def __init__(self, teeth_num, rad, location):
        self.location = location
        self.teeth_num = int(teeth_num)
        self.radius = rad

class WingsInfoOutput:
    def __init__(self, points: list, gear_setup: GearSetup, user_info: WingsUserInfo, body_gear: BodyGear):

        self.points_on_main_wing = [body_gear.location] + points
        self.gear_num = gear_setup.norm_gear_amount + 1
        self.gear_setup = gear_setup
        self.user_info = user_info
        self.pegs = [user_info.connnecting_point_body]
        self.pegs.extend([point for point in points[1:-1]])
        self.pegs.append(user_info.connnecting_point_another_wing)
        self.body_gear = body_gear

        self.holes = [['mw', points[0]], ['mb', points[-1]]]

    def __str__(self):
        return f"YOYO the ending setup is:\n" \
               f"Coordinates are: {self.points_on_main_wing}\n" \
               f"Peg coords are: {self.pegs}\n" \
               f"Amount of gears is: {self.gear_num}" \
               f"Teeth in norm: {self.gear_setup.norm_gear_teeth_num}, Teeth in last: {self.gear_setup.last_gear_teeth_num}"


# get_gears returns the gearsetup object
# BLENDER CONSTANTS
VOXELSIZE = 0.01
LENGTH = 0
LENGTH_EPS = 1 / 30
MAX_WIDTH = 0
RATIO = [0, 0]
DP = 10
MAX_ITERATIONS = 20
SUCCESS_ID = 0
ERROR_ID = 1
PEG_RADIUS = 0.5
PEG_GEAR_RAD_DIFF = 0.02
PEG_LENGTH = 1
GEAR_WIDTH = 0.2
DEDENDUM = 0.1
HANDLE_OG_LENGTH = 0.416

# A function to get the simplest fraction eps close to x
my_get_simplest_from_interval = lambda x, eps: simplest_in_interval(x - eps, x + eps)


# getting the ratio from the angels we want to move
def get_ratio(angle1: Union[float, int], angle2: Union[float, int], max_denomi: int = 30,
              ratio_eps: Union[float, int] = 0.0001, ratio_cons: list = [0.2, 2]) -> int:
    global RATIO, SUCCESS_ID, ERROR_ID
    ratio = float(angle1) / float(angle2)
    simpler_ratio = my_get_simplest_from_interval(ratio, ratio_eps).as_integer_ratio()
    if (simpler_ratio[1] / simpler_ratio[0]) < ratio_cons[0] or (simpler_ratio[1] / simpler_ratio[0]) > ratio_cons[1]:
        print("You stupido ratio out of constraints try again")
        return ERROR_ID
    if simpler_ratio[1] > max_denomi:
        print("You stupido daminator too large try again")
        return ERROR_ID
    RATIO = [simpler_ratio[0], simpler_ratio[1]]
    return SUCCESS_ID


def denominator_to_dp(denom) -> int:
    if denom == 1:
        return 25
    if denom <= 3:
        return 24
    if denom < 7:
        return denom * 7
    if denom < 10:
        return denom * 3
    if denom <= 15:
        return denom * 2
    return denom


# setting the global variables
def set_globals(max_length: Union[float, int], length_eps: Union[float, int],
                max_width: Union[float, int]) -> int:
    global LENGTH, LENGTH_EPS, MAX_WIDTH, RATIO, DP
    LENGTH = max_length
    LENGTH_EPS = length_eps
    MAX_WIDTH = max_width
    # this is okay because of the mas denominator constraint
    DP = denominator_to_dp(RATIO[1])
    return SUCCESS_ID


# this function will find the optimal gear amount and size to fit the constraint and desirable ratio
def calc_gear_amount():
    global LENGTH, LENGTH_EPS, MAX_WIDTH, DP, MAX_ITERATIONS, RATIO
    amount_to_fit = 0
    cur_last_diam = 0
    dis = LENGTH
    cur_norm_diam = (math.floor(MAX_WIDTH * DP)) / DP
    iter = 0
    while iter < MAX_ITERATIONS:
        cur_last_diam = (cur_norm_diam * RATIO[0]) / RATIO[1]
        length_left = LENGTH - cur_last_diam / 2 - cur_norm_diam / 2
        amount_to_fit = math.floor(length_left / cur_norm_diam)
        # checking for zoogiyot can change
        """if amount_to_fit % 2 == 0:
            continue"""
        dis = amount_to_fit * cur_norm_diam + LENGTH - length_left
        if abs(LENGTH - dis) <= LENGTH_EPS:
            break
        amount_to_fit += 1
        dis += cur_norm_diam
        if abs(LENGTH - dis) <= LENGTH_EPS:
            break
        cur_norm_diam -= 1 / DP
        iter += 1

    if iter >= MAX_ITERATIONS:
        print("cannot find a fitting gear setup try again")
        return

    return GearSetup(DP, amount_to_fit + 1, cur_norm_diam, cur_last_diam)


# getting the actual gears
def get_gear_setup(user_info: WingsUserInfo, max_denomi: Union[float, int] = 30, ratio_cons=None):
    if ratio_cons is None:
        ratio_cons = [0.1, 2]
    if get_ratio(user_info.angels[0], user_info.angels[1], max_denomi, user_info.ratio_eps, ratio_cons) == ERROR_ID:
        return None
    # setting all of the global variables
    if set_globals(user_info.length, user_info.length_eps, user_info.width_constraints) == ERROR_ID:
        return None
    return calc_gear_amount()


# returns a point on the line between the two points with dis being the distance between the new and point1
# in direction, 1 for towards p2 -1 for the opposite direction
def new_point_on_line(points, dis, direction):
    global ERROR_ID, SUCCESS_ID
    x1, y1, z1 = points[0]
    x2, y2, z2 = points[1]
    ans = dis ** 2 / ((x1 - x2) ** 2 + (y1 - y2) ** 2 + (z1 - z2) ** 2)
    if ans < 0:
        return ERROR_ID
    ans = direction * math.sqrt(ans)
    return [x1 + ans * (x2 - x1), y1 + ans * (y2 - y1), z1 + ans * (z2 - z1)]


def get_gear_setup_and_locations(user_info: WingsUserInfo):
    gear_setup = get_gear_setup(user_info)
    gear_coords = []
    for i in range(gear_setup.norm_gear_amount):
        gear_coords.append(
            [round(coor, 4) for coor in new_point_on_line(user_info.wing_ends, gear_setup.norm_gear_rad * 2 * i, 1)])
    gear_coords.append(
        [round(coor, 4) for coor in new_point_on_line(user_info.wing_ends, gear_setup.norm_gear_rad * 2 * (
                gear_setup.norm_gear_amount - 0.5) + gear_setup.last_gear_rad, 1)])
    
    start_coor = gear_coords[0]
    if start_coor[0] < 0:
        to_mul = -1
    else:
        to_mul = 1
    main_teeth_num = abs(math.floor((abs(start_coor[0]) - gear_setup.norm_gear_rad)* gear_setup.dp))
    main_rad = abs((abs(main_teeth_num) / gear_setup.dp )/ 2)
    return WingsInfoOutput(gear_coords, gear_setup, user_info, BodyGear(teeth_num=main_teeth_num, rad=main_rad,
                             location=[to_mul * (abs(start_coor[0]) - main_rad - gear_setup.norm_gear_rad), start_coor[1], start_coor[2]]))




def merge_meshes(objs, voxel_size=VOXELSIZE):
    assert all(obj.type == 'MESH' for obj in objs)

    for obj in bpy.context.scene.objects:
        obj.select_set(False)

    for obj in objs:
        obj.select_set(True)
        bpy.context.view_layer.objects.active = obj

    bpy.ops.object.join()
    bpy.ops.object.modifier_add(type='REMESH')
    bpy.context.object.modifiers["Remesh"].voxel_size = voxel_size
    bpy.ops.object.convert(target='MESH')
    return bpy.context.object


def full_gear_with_hole(location, number_of_teeth, radius, base, dedendum=DEDENDUM):
    bpy.ops.mesh.primitive_gear(align='WORLD', location=location,
                                rotation=(0, 0, 0),
                                change=False,
                                number_of_teeth=number_of_teeth,
                                radius=radius, width=GEAR_WIDTH,
                                dedendum=dedendum,
                                base=base)

    gear = bpy.context.object

    bpy.ops.mesh.primitive_cylinder_add(
        radius=radius - dedendum - base,
        depth=2 * GEAR_WIDTH,
        location=location,
    )

    inner_cylinder = bpy.context.object

    bpy.ops.mesh.primitive_cylinder_add(
        radius=radius - dedendum,
        depth=2 * GEAR_WIDTH,
        location=location,
    )

    outer_cylinder = bpy.context.object
    # currently pointing at outer
    bpy.ops.object.modifier_add(type='BOOLEAN')
    outer_cylinder.modifiers["Boolean"].operation = 'DIFFERENCE'
    outer_cylinder.modifiers["Boolean"].object = inner_cylinder

    for obj in bpy.context.scene.objects:
        obj.select_set(False)

    outer_cylinder.select_set(True)
    bpy.context.view_layer.objects.active = outer_cylinder

    bpy.ops.object.convert(target='MESH')

    for obj in bpy.context.scene.objects:
        obj.select_set(False)

    inner_cylinder.select_set(True)
    bpy.context.view_layer.objects.active = inner_cylinder

    bpy.ops.object.delete(use_global=False, confirm=False)

    gear.select_set(True)
    outer_cylinder.select_set(True)
    bpy.context.view_layer.objects.active = outer_cylinder
    bpy.context.view_layer.objects.active = gear

    bpy.ops.object.join()
    return bpy.context.object


def generate_gears_for_printing(gears_to_generate, start: list, printer_max):
    box = max(gears_to_generate.gear_setup.norm_gear_rad, gears_to_generate.gear_setup.last_gear_rad) * 2 + 0.4
    sides = math.ceil(math.sqrt(gears_to_generate.gear_num))
    count = 0
    if box * sides > printer_max:
        print("too large for printer do manually")
        return None

    for i in range(sides):
        for b in range(sides):
            if count < gears_to_generate.gear_setup.norm_gear_amount:
                location = [start[0] + i * box, start[1] + b * box, start[2]]
                number_of_teeth = gears_to_generate.gear_setup.norm_gear_teeth_num
                radius = gears_to_generate.gear_setup.norm_gear_rad
                base = gears_to_generate.gear_setup.norm_gear_rad - DEDENDUM - (
                            gears_to_generate.gear_setup.norm_gear_rad - DEDENDUM) / 2 - PEG_GEAR_RAD_DIFF

                full_gear_with_hole(location, number_of_teeth, radius, base, dedendum=DEDENDUM)

                
            elif count == gears_to_generate.gear_setup.norm_gear_amount:
                location = [start[0] + i * box, start[1] + b * box, start[2]]
                number_of_teeth = gears_to_generate.gear_setup.norm_gear_teeth_num
                radius = gears_to_generate.gear_setup.last_gear_rad
                base = gears_to_generate.gear_setup.last_gear_rad - DEDENDUM - (
                            gears_to_generate.gear_setup.last_gear_rad - DEDENDUM) / 2 - PEG_GEAR_RAD_DIFF

                full_gear_with_hole(location, number_of_teeth, radius, base, dedendum=DEDENDUM)
                
            count += 1


def generate_gears_on_body(gears_to_generate: WingsInfoOutput, peg_length=PEG_LENGTH):
    count = 0
    phi, theta = normal_to_angles(gears_to_generate.user_info.normal)
    first_gear = None
    last_gear = None
    body_gear = None
    for loc in gears_to_generate.points_on_main_wing:
        if count == 0:
            
            number_of_teeth = gears_to_generate.body_gear.teeth_num
            radius = gears_to_generate.body_gear.radius
            base = radius - 0.1 - (radius - 0.1) / 2 - PEG_GEAR_RAD_DIFF
            norms = gears_to_generate.user_info.normal
            place_main_gear(loc, norms, radius, number_of_teeth, "HANDLE")
            #full_gear_with_hole(loc, number_of_teeth, radius, base)
            bpy.context.object.rotation_euler[1] = theta
            bpy.context.object.rotation_euler[2] = phi
            
        elif count == gears_to_generate.gear_num:

            # We want inner radius to be PRG_RADIUS + some epsilon
            # inner_radius = radius - dedendum - base
            # base = radius - dedendum - inner_radius
            # base = gears_to_generate.gear_setup.last_gear_rad - 0.1 - (gears_to_generate.gear_setup.norm_gear_rad - 0.1) / 2

            number_of_teeth = gears_to_generate.gear_setup.last_gear_teeth_num
            radius = gears_to_generate.gear_setup.last_gear_rad
            base = gears_to_generate.gear_setup.last_gear_rad - 0.1 - (
                        gears_to_generate.gear_setup.last_gear_rad - 0.1) / 2 - PEG_GEAR_RAD_DIFF

            full_gear_with_hole(loc, number_of_teeth, radius, base)

            bpy.context.object.rotation_euler[1] = theta
            bpy.context.object.rotation_euler[2] = phi
            
            last_gear = bpy.context.object
        else:

            number_of_teeth = gears_to_generate.gear_setup.norm_gear_teeth_num
            radius = gears_to_generate.gear_setup.norm_gear_rad
            base = gears_to_generate.gear_setup.norm_gear_rad - 0.1 - (
                        gears_to_generate.gear_setup.norm_gear_rad - 0.1) / 2 - PEG_GEAR_RAD_DIFF

            full_gear_with_hole(loc, number_of_teeth, radius, base)

            bpy.context.object.rotation_euler[1] = theta
            bpy.context.object.rotation_euler[2] = phi
            
            if first_gear is None:
                first_gear =  bpy.context.object

        count += 1

    return first_gear, last_gear


def cylinder_between(point1, point2, r=PEG_RADIUS, extend=1):
    dx = point2[0] - point1[0]
    dy = point2[1] - point1[1]
    dz = point2[2] - point1[2]
    dist = math.sqrt(dx ** 2 + dy ** 2 + dz ** 2)

    bpy.ops.mesh.primitive_cylinder_add(
        radius=r,
        depth=dist * extend,
        location=(dx / 2 + point1[0], dy / 2 + point1[1], dz / 2 + point1[2]),
    )

    phi = math.atan2(dy, dx)
    theta = math.acos(dz / dist)

    bpy.context.object.rotation_euler[1] = theta
    bpy.context.object.rotation_euler[2] = phi
    return bpy.context.object


def normal_to_angles(normal: list):
    point1 = [x for x in normal]
    point2 = [0 for x in normal]

    dx = point2[0] - point1[0]
    dy = point2[1] - point1[1]
    dz = point2[2] - point1[2]
    dist = math.sqrt(dx ** 2 + dy ** 2 + dz ** 2)

    phi = math.atan2(dy, dx)
    theta = math.acos(dz / dist)
    return phi, theta


def generate_pegs(gears_to_generate, peg_length=PEG_LENGTH):
    peg_radius = (gears_to_generate.gear_setup.norm_gear_rad - 0.1) / 2
    normal_length = math.sqrt(sum(x ** 2 for x in gears_to_generate.user_info.normal))
    scaling_factor = PEG_LENGTH / normal_length
    scaled_normal = [x * scaling_factor / math.sqrt(3) for x in gears_to_generate.user_info.normal]
    pegs = []
    for location in gears_to_generate.pegs:
        dest = [location[i] + scaled_normal[i] for i in range(len(location))]
        pegs.append(cylinder_between([location[i] - scaled_normal[i]  for i in range(3)], dest, peg_radius))

    center0 = [gears_to_generate.pegs[0][i] + scaled_normal[i] * 4 / 5 for i in range(len(gears_to_generate.pegs[0]))]
    center1 = [gears_to_generate.pegs[-1][i] + scaled_normal[i] * 4 / 5 for i in range(len(gears_to_generate.pegs[-1]))]

    return pegs, cylinder_between(center0, center1, peg_radius / 5, extend=1.1)


def move_object(objA, pointA, pointB, theta, phi):
    for obj in bpy.context.scene.objects:
        obj.select_set(False)

    objA.select_set(True)

    cur_cursor = Vector(bpy.context.scene.cursor.location)

    # move cursor to pointA
    bpy.context.scene.cursor.rotation_euler = (0, 0, 0)
    bpy.context.scene.cursor.location = pointA

    # move origin of objA to cursor
    bpy.ops.object.origin_set(type='ORIGIN_CURSOR', center='MEDIAN')

    # move objA to pointB

    objA.location = Vector((0, 0, 0))
    bpy.ops.transform.translate(value=pointB, orient_axis_ortho='X', orient_type='GLOBAL',
                                orient_matrix=((1, 0, 0), (0, 1, 0), (0, 0, 1)), orient_matrix_type='GLOBAL')

    objA.rotation_euler[1] = theta
    objA.rotation_euler[2] = phi


def place_main_gear(gear_location, gear_normal, main_gear_rad, main_gear_teeth_num, handle_name):
    global HANDLE_OG_LENGTH
    bpy.ops.mesh.primitive_gear(align='WORLD', location=gear_location,
                                rotation=(0, 0, 0),
                                change=False,
                                number_of_teeth=main_gear_teeth_num,
                                radius=main_gear_rad, width=0.2,
                                base=main_gear_rad - 0.1)

    phi, theta = normal_to_angles(gear_normal)

    bpy.context.object.rotation_euler[1] = theta
    bpy.context.object.rotation_euler[2] = phi

    handle = bpy.context.scene.objects[handle_name]

    for obj in bpy.context.scene.objects:
        obj.select_set(False)
    handle.select_set(True)
    bpy.context.view_layer.objects.active = handle
    handle_scale_factor = 0.55 / HANDLE_OG_LENGTH
    bpy.ops.transform.resize(value=(handle_scale_factor, handle_scale_factor, handle_scale_factor))

    normal_length = math.sqrt(sum(x ** 2 for x in gear_normal))
    scaling_factor = 0.5 / normal_length
    scaled_normal = [x * scaling_factor / math.sqrt(3) for x in gear_normal]
    dest = [gear_location[i] - scaled_normal[i] for i in range(len(gear_normal))]

    move_object(handle, handle.location, dest, theta, phi)


def generate_pegs_with_hole(gears_to_generate):
    pegs, hole_cylinder = generate_pegs(gears_to_generate)

    for obj in bpy.context.scene.objects:
        obj.select_set(False)

    for peg in pegs:
        peg.select_set(True)
        bpy.context.view_layer.objects.active = peg

        bpy.ops.object.modifier_add(type='BOOLEAN')
        peg.modifiers["Boolean"].operation = 'DIFFERENCE'
        peg.modifiers["Boolean"].object = hole_cylinder
        bpy.ops.object.convert(target='MESH')

        peg.select_set(False)


    hole_cylinder.select_set(True)
    bpy.context.view_layer.objects.active = hole_cylinder

    bpy.ops.object.delete(use_global=False, confirm=False)

    return pegs

def create_hole_in_wing(wing1, wing2, pointA, pointB, radius):
    inner_cylinder = cylinder_between(pointA,pointB,radius)

    for obj in bpy.context.scene.objects:
        obj.select_set(False)


    for wing in [wing1, wing2]:
        wing.select_set(True)
        bpy.context.view_layer.objects.active = wing

        bpy.ops.object.modifier_add(type='BOOLEAN')
        wing.modifiers["Boolean"].operation = 'DIFFERENCE'
        wing.modifiers["Boolean"].object = inner_cylinder


        bpy.ops.object.convert(target='MESH')
        wing.select_set(False)


    inner_cylinder.select_set(True)
    bpy.context.view_layer.objects.active = inner_cylinder

    bpy.ops.object.delete(use_global=False, confirm=False)
    return wing1, wing2

def merge_wing_peg_gear(first_gear, last_gear, pegs, wing1, wing2):
    first_peg = pegs[0]
    last_peg = pegs[-1]
    
    merge_meshes([first_gear, wing1] + pegs[0:-1])
    
    merge_meshes([last_gear, last_peg, wing2])
    
        


def export(path):
    from pathlib import Path

    context = bpy.context
    scene = context.scene
    viewlayer = context.view_layer

    obs = [o for o in scene.objects if o.type == 'MESH']
    bpy.ops.object.select_all(action='DESELECT')

    path = Path(path)
    for ob in obs:
        viewlayer.objects.active = ob
        ob.select_set(True)
        stl_path = path / f"{ob.name}.stl"
        bpy.ops.export_mesh.stl(
            filepath=str(stl_path),
            use_selection=True)
        ob.select_set(False)

def handle_a_wing(view_vs_export, wing1, wing2, start_point_on_first_wing: list, end_point_on_first_wing: list, width_constraint: Union[float, int], angle1: Union[float, int], angle2: Union[float, int]
         ,normal: list, export_path = None):
    user_info = WingsUserInfo(point1=start_point_on_first_wing, point2=end_point_on_first_wing, length_eps=1/30, width_constraint=width_constraint, angle1=angle1,
                         angle2=angle2, ratio_eps=1/36, connecting_point_body=start_point_on_first_wing, connecting_point_another_wing=end_point_on_first_wing,
                         normal=normal)
    generation_info = get_gear_setup_and_locations(user_info)
    pegs_list = generate_pegs_with_hole(generation_info)
    if view_vs_export == 'v':
        first_gear, last_gear = generate_gears_on_body(generation_info)
        merge_wing_peg_gear(first_gear, last_gear, pegs_list, wing1, wing2)
    if view_vs_export == 'e':
        generate_gears_for_printing(generation_info, [0,0,0])
        


def main(view_vs_export, main_wing1, main_wing2, another_wing1, another_wing2, start_point_on_first_wing: list, end_point_on_first_wing: list, width_constraint: Union[float, int], angle1: Union[float, int], angle2: Union[float, int]
         ,normal: list, export_path = None):
    handle_a_wing(view_vs_export, main_wing1, another_wing1,
                  start_point_on_first_wing=start_point_on_first_wing, end_point_on_first_wing=end_point_on_first_wing, width_constraint=width_constraint,
                  angle1=angle1, angle2=angle2, normal=normal, export_path=export_path)
    start_point_on_first_wing = [-start_point_on_first_wing[0], start_point_on_first_wing[1], start_point_on_first_wing[2]]
    end_point_on_first_wing = [-end_point_on_first_wing[0], end_point_on_first_wing[1], end_point_on_first_wing[2]]
    handle_a_wing(view_vs_export, main_wing2, another_wing2,
                  start_point_on_first_wing=start_point_on_first_wing, end_point_on_first_wing=end_point_on_first_wing,
                  width_constraint=width_constraint,
                  angle1=angle1, angle2=angle2, normal=normal, export_path=export_path)
    
    if view_vs_export == 'e':
        export(export_path)



if __name__ == "__main__":
    # purge()
    main('v', bpy.context.scene.objects["MainWing2"], bpy.context.scene.objects["MainWing1"], bpy.context.scene.objects["AnotherWing2"],
                    bpy.context.scene.objects["AnotherWing1"],
                  start_point_on_first_wing=[-1, -0.5, 1.9], end_point_on_first_wing=[-2.9, -0.5, 2.6], width_constraint=1.,
                  angle1=30, angle2=60, normal=[0, 1, 0], export_path="C:\\Users\\Tommy\\Documents")
