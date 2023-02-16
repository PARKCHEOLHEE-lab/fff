import math
import random
import Rhino.Geometry as rg
import ghpythonlib.components as gh
import ghpythonlib.treehelpers as gt
import ghpythonlib.parallel as gp


TOLERANCE = 0.001
TOLERANCE_MICRO = 0.00001

SUN_RAD = 5
HEMISPHERE_RAD = 150
MOVE_DIST = HEMISPHERE_RAD * 1.7

PARAPET_HEIGHT = 1

ROTATE_90 = math.pi * 0.5
ROTATE_180 = math.pi
ROTATE_270 = math.pi * 1.5

WORLD_XY = rg.Plane.WorldXY
random.seed(0)


class Utils:
    def is_close(self, n1, n2, tolerance=TOLERANCE):
        return abs(n1 - n2) <= tolerance


class Environment:
    def __init__(self, brep, sun_position):
        self.brep = brep
        self.sun_position = sun_position
        self.__generate()
        
    def __generate(self):
        self.__gen_environment_bbox()
        self.__gen_hemisphere()
        self.__gen_sun()
        self.__gen_sun_facing_plane()
        self.__gen_sun_facing_angle()
        
    def __gen_environment_bbox(self):
        self.environment_bbox = self.brep.GetBoundingBox(WORLD_XY)
        self.environment_bbox_faces, _, _ = gh.DeconstructBrep(self.environment_bbox)
        self.environment_bbox_bottom_face = sorted(self.environment_bbox_faces, key=lambda f: gh.Area(geometry=f).centroid.Z)[0]
        _, self.environment_bbox_bottom_face_centroid = gh.Area(self.environment_bbox_bottom_face)
        self.environment_bbox_bottom_face_scaling, _ = gh.Scale(
            geometry=self.environment_bbox_bottom_face,
            center=self.environment_bbox_bottom_face_centroid,
            factor=HEMISPHERE_RAD * 0.1,
        )
        
    def __gen_hemisphere(self):
        self.sphere = gh.Sphere(
            base=self.environment_bbox_bottom_face_centroid,
            radius=HEMISPHERE_RAD,
        )
        _, self.hemisphere = self.sphere.Split(self.environment_bbox_bottom_face_scaling, TOLERANCE)
        self.interval = rg.Interval(0, 1)
        self.hemisphere.Faces[0].SetDomain(0, self.interval)
        self.hemisphere.Faces[0].SetDomain(1, self.interval)
        
    def __gen_sun(self):
        self.sun_point = self.hemisphere.Faces[0].Evaluate(
            u=self.sun_position[0][0], 
            v=self.sun_position[0][1], 
            numberDerivatives=0,
        )[1]
        
        self.sun = gh.Sphere(base=self.sun_point, radius=SUN_RAD)
        
    def __gen_sun_facing_plane(self):
        self.sun_point_to_face_centroid = rg.Line(self.sun_point, self.environment_bbox_bottom_face_centroid)
        self.projected_sun_point_to_face_centroid = gh.Project(
            curve=self.sun_point_to_face_centroid,
            brep=self.environment_bbox_bottom_face_scaling,
            direction=rg.Point3d(0, 0, 1)
        )
        
        self.sun_vector = self.environment_bbox_bottom_face_centroid - self.sun_point
        self.plane = (
            WORLD_XY if self.projected_sun_point_to_face_centroid is None 
            else gh.HorizontalFrame(self.projected_sun_point_to_face_centroid, 0.5)
        )
        
    def __gen_sun_facing_angle(self):
        if self.projected_sun_point_to_face_centroid is None:
            self.sun_facing_angle = 0
        
        else:
            x1, y1, _ = self.projected_sun_point_to_face_centroid.PointAtStart
            x2, y2, _ = self.projected_sun_point_to_face_centroid.PointAtEnd
            
            self.sun_facing_angle = math.atan2(y2 - y1, x2 - x1)
            self.sun_facing_degree = math.degrees(self.sun_facing_angle) - 360
            self.converted_sun_facing_angle = math.radians(self.sun_facing_degree)
                        

class VoxelConditions:
    UNDEFINED = -1
    NONE = 0
    
    ROOF = 3
    EXTERIOR = 4
    EXTERIOR_BOTH = 5
    EXTERIOR_CORNER = 6
    EXTERIOR_CORNER_O = 7
    EXTERIOR_CORNER_U = 8
    INTERIOR = 9


class VoxelUnits:
    def __init__(self, exterior_unit, exterior_both_unit, exterior_corner_unit, exterior_corner_o_unit, exterior_corner_u_unit):
        self.unit_bounding_box = gh.BoundingBox(exterior_unit, WORLD_XY).box
        self.exterior_unit_centroid = gh.Volume(self.unit_bounding_box).centroid
        
        self.voxel_face_edges = gh.DeconstructBrep(self.unit_bounding_box.ToBrep().Faces[0].ToBrep()).edges
        self.scale_factor = self.voxel_size / self.voxel_face_edges[0].GetLength()
        
        self.rotated_exterior_unit = self.__get_scaled_unit(exterior_unit)
        self.rotated_exterior_both_unit = self.__get_scaled_unit(exterior_both_unit)
        self.rotated_exterior_corner_unit = self.__get_scaled_unit(exterior_corner_unit)
        self.rotated_exterior_corner_u_unit = self.__get_scaled_unit(exterior_corner_u_unit)
        self.rotated_exterior_corner_o_unit = self.__get_scaled_unit(exterior_corner_o_unit)
        
    def __get_joined_mesh(self, unit):
        return gh.MeshJoin(rg.Mesh.CreateFromBrep(unit, rg.MeshingParameters(0)))
        
    def __get_rotated_unit(self, unit):
        return gh.Rotate(
            self.__get_joined_mesh(unit),
            self.converted_sun_facing_angle, 
            self.exterior_unit_centroid
        ).geometry
        
    def __get_scaled_unit(self, unit):
        return gh.Scale(
            self.__get_rotated_unit(unit),
            self.exterior_unit_centroid,
            self.scale_factor
        ).geometry if self.is_needed_resize else self.__get_rotated_unit(unit)


class Voxel:
    voxel_3x3_map = None
    voxel_shade = None

    def __init__(
        self, 
        voxel_geom=None,
        voxel_box=None,
        voxel_geom_centroid=None,
        voxel_condition=None,
        roof_color_index=None,
        is_roof=False,
        is_exterior=False,
    ):
        self.voxel_geom = voxel_geom
        self.voxel_box = voxel_box
        self.voxel_geom_centroid = voxel_geom_centroid
        self.voxel_condition = voxel_condition
        self.roof_color_index = roof_color_index
        self.is_roof = is_roof
        self.is_exterior = is_exterior
        
    def get_voxel_object(
            self, 
            voxel_geom=None, 
            voxel_box=None,
            is_roof=None, 
            voxel_condition=VoxelConditions.NONE, 
            roof_color_index=None
        ):
            
        voxel_geom_centroid = gh.Volume(voxel_geom).centroid
            
        return Voxel(
            voxel_geom=voxel_geom,
            voxel_box=voxel_box,
            voxel_geom_centroid=voxel_geom_centroid,
            voxel_condition=voxel_condition,
            roof_color_index=roof_color_index,
            is_roof=is_roof,
        )


class VoxelShape(Utils, Environment, Voxel, VoxelConditions, VoxelUnits):
    def __init__(
        self, 
        brep, 
        voxel_size, 
        exterior_unit,
        exterior_both_unit,
        exterior_corner_unit, 
        exterior_corner_o_unit,
        exterior_corner_u_unit, 
        sun_position, 
        is_needed_shades=False,
        is_needed_resize=False,
    ):
        self.brep = brep
        self.voxel_size = voxel_size
        self.exterior_unit = exterior_unit
        self.exterior_both_unit = exterior_both_unit
        self.exterior_corner_unit = exterior_corner_unit
        self.exterior_corner_o_unit = exterior_corner_o_unit
        self.exterior_corner_u_unit = exterior_corner_u_unit
        self.sun_position = sun_position
        self.is_needed_shades = is_needed_shades
        self.is_needed_resize = is_needed_resize
        self.__voxelate()
        
    def __voxelate(self):
        self.__gen_moved_brep()
        self.__gen_environment()
        self.__gen_moved_brep_bbox()
        self.__gen_grid()
        self.__gen_voxels()
        self.__gen_voxel_3x3_map()
        self.__gen_placed_voxel_unit()
        self.__gen_voxel_shape_parapet()
        self.__gen_voxel_shades()
        
    def __gen_moved_brep(self):
        self.moved_brep, _ = gh.Move(geometry=self.brep, motion=rg.Point3d(MOVE_DIST, 0, 0))
        self.moved_brep_mesh = rg.Mesh.CreateFromBrep(self.moved_brep, rg.MeshingParameters(0))
        
    def __gen_environment(self):
        Utils.__init__(self)
        Environment.__init__(self, self.moved_brep, sun_position)
        Voxel.__init__(self)
        VoxelConditions.__init__(self)
        VoxelUnits.__init__(
            self, 
            self.exterior_unit, 
            self.exterior_both_unit,
            self.exterior_corner_unit, 
            self.exterior_corner_o_unit, 
            self.exterior_corner_u_unit
        )
        
    def __gen_moved_brep_bbox(self):
        self.moved_brep_bbox, _ = gh.BoundingBox(content=self.moved_brep, plane=self.plane)
        self.moved_brep_bbox_faces, _, _ = gh.DeconstructBrep(brep=self.moved_brep_bbox)
        self.moved_brep_bbox_bottom_face = sorted(
            self.moved_brep_bbox_faces, 
            key=lambda f: gh.Area(geometry=f).centroid.Z
        )[0]
        
    def __gen_grid(self):
        moved_brep_bbox_bottom_face_segments, _ = gh.Explode(curve=self.moved_brep_bbox_bottom_face, recursive=True)
        
        seg_1, seg_2, _, _ = moved_brep_bbox_bottom_face_segments
        seg_1_vector = seg_1.Line.PointAtLength(self.voxel_size) - seg_1.Line.PointAt(0)
        seg_2_vector = seg_2.Line.PointAtLength(self.voxel_size) - seg_2.Line.PointAt(0)
        
        grid_origin = gh.HorizontalFrame(curve=seg_1, parameter=0)
        base_rectangle, _ = gh.Rectangle(plane=grid_origin, x_size=self.voxel_size, y_size=self.voxel_size, radius=0)
        
        self.x_cols = int(seg_1.Line.Length // self.voxel_size) + 2
        self.y_cols = int(seg_2.Line.Length // self.voxel_size) + 2
        self.z_cols = int(self.moved_brep_bbox.Z[1] // self.voxel_size) + 2
        
        self.cols = (self.x_cols, self.y_cols, self.z_cols)
        
        x_grid = []
        for x in range(self.x_cols):
            motion = seg_1_vector * x
            moved_rectangle, _ = gh.Move(geometry=base_rectangle, motion=motion)
            x_grid.append(moved_rectangle)
        x_grid = gh.Move(geometry=x_grid[::-1], motion=gh.Negative(seg_1_vector)).geometry
        
        base_grid = []
        for y in range(self.y_cols):
            motion = seg_2_vector * y
            moved_rectangles, _ = gh.Move(geometry=x_grid, motion=motion)
            base_grid.extend(moved_rectangles)
        
        self.grid = []
        for z in range(self.z_cols):
            motion = rg.Point3d(0, 0, z * self.voxel_size)
            moved_rectangles, _ = gh.Move(geometry=base_grid, motion=motion)
            self.grid.extend(moved_rectangles)
            
        self.grid_center_lowest_z = self.grid[0].Center.Z
            
    def __gen_voxels(self):
        self.grid_centroid = [
            g.Center if not self.is_close(g.Center.Z, self.grid_center_lowest_z)
            else g.Center + rg.Point3d(0, 0, TOLERANCE_MICRO) 
            for g in self.grid
        ]
        
        self.inside_grid_centroid = gh.MeshInclusion(
            mesh=gh.MeshJoin(self.moved_brep_mesh), 
            point=self.grid_centroid, 
            strict=False
        )
        
        (
            self.voxel_3d_list,
            self.grid_3d_list,
            self.roof_3d_list, 
            self.exterior_3d_list, 
            self.exterior_both_3d_list, 
            self.exterior_corner_3d_list,
            self.exterior_corner_o_3d_list,
            self.exterior_corner_u_3d_list,
            self.interior_3d_list
        ) = self.__get_voxels_condition_list()
        
        self.voxels_objects = []
        
        for zi, voxels in enumerate(self.voxel_3d_list):
            none_voxel = self.get_voxel_object(voxel_condition=self.NONE)
            for yi, solid_conditions in enumerate(voxels):
                if all([not bool(y) for y in solid_conditions]):
                    self.voxels_objects.extend([none_voxel] * len(solid_conditions))
                    continue
                
                roof_conditions = self.roof_3d_list[zi][yi]
                exterior_conditions = self.exterior_3d_list[zi][yi]
                exterior_both_conditions = self.exterior_both_3d_list[zi][yi]
                exterior_corner_conditions = self.exterior_corner_3d_list[zi][yi]
                exterior_corner_o_conditions = self.exterior_corner_o_3d_list[zi][yi]
                exterior_corner_u_conditions = self.exterior_corner_u_3d_list[zi][yi]
                interior_conditions = self.interior_3d_list[zi][yi]
                
                for xi, sc in enumerate(solid_conditions):
                    if not bool(sc):
                        self.voxels_objects.append(none_voxel)
                        continue
                    
                    voxel_box = gh.BoxRectangle(self.grid_3d_list[zi][yi][xi], self.voxel_size)
                    voxel_geom = rg.Mesh.CreateFromBox(voxel_box, 1, 1, 1)
                    
                    voxel_condition = self.UNDEFINED
                    
                    is_roof = False
                    if bool(roof_conditions[xi]):
                        is_roof = True
                        
                    if bool(exterior_corner_conditions[xi]):
                        voxel_condition = self.EXTERIOR_CORNER
                    elif bool(exterior_corner_o_conditions[xi]):
                        voxel_condition = self.EXTERIOR_CORNER_O
                    elif bool(exterior_corner_u_conditions[xi]):
                        voxel_condition = self.EXTERIOR_CORNER_U
                    elif bool(exterior_conditions[xi]):
                        voxel_condition = self.EXTERIOR       
                    elif bool(exterior_both_conditions[xi]):
                        voxel_condition = self.EXTERIOR_BOTH             
                    elif bool(interior_conditions[xi]):
                        voxel_condition = self.INTERIOR
                        
                    voxels[yi][xi] = voxel_condition
                    
                    self.voxels_objects.append(
                        self.get_voxel_object(
                            voxel_geom=voxel_geom, 
                            voxel_box=voxel_box,
                            is_roof=is_roof, 
                            voxel_condition=voxel_condition, 
                            roof_color_index=random.randint(0, 2) if is_roof else None,
                        )
                    )
        
        self.voxels_objects_flattened = self.voxels_objects[:]
        
        self.voxel_geom_3d_list = self.__get_reshaped_list([v.voxel_geom for v in self.voxels_objects], *self.cols)
        self.voxels_objects = self.__get_reshaped_list(self.voxels_objects, *self.cols)
        
    def __gen_voxel_3x3_map(self):
        for zi, voxels in enumerate(self.voxels_objects):
            for yi, conditions in enumerate(voxels):
                if all(c.voxel_condition == self.NONE for c in conditions):
                    continue
                
                prev_voxels = voxels[(yi - 1) % len(voxels)]
                next_voxels = voxels[(yi + 1) % len(voxels)]
                
                for xi, voxel_object in enumerate(conditions):
                    if voxel_object.voxel_condition == self.NONE:
                        continue
                        
                    prev_xi = (xi - 1) % len(conditions)
                    next_xi = (xi + 1) % len(conditions)
                    
                    voxel_3x3_map = [[0] * 3 for _ in range(3)]
                     
                    north_x = prev_voxels[xi].voxel_condition
                    south_x = next_voxels[xi].voxel_condition
                    west_x = conditions[prev_xi].voxel_condition
                    east_x = conditions[next_xi].voxel_condition
                    
                    neast_x = prev_voxels[next_xi].voxel_condition
                    nwest_x = prev_voxels[prev_xi].voxel_condition
                    seast_x = next_voxels[next_xi].voxel_condition
                    swest_x = next_voxels[prev_xi].voxel_condition
                    
                    voxel_3x3_map[0][0] = nwest_x
                    voxel_3x3_map[0][1] = north_x
                    voxel_3x3_map[0][2] = neast_x
                    
                    voxel_3x3_map[1][0] = west_x
                    voxel_3x3_map[1][1] = voxel_object.voxel_condition
                    voxel_3x3_map[1][2] = east_x
                    
                    voxel_3x3_map[2][0] = swest_x
                    voxel_3x3_map[2][1] = south_x
                    voxel_3x3_map[2][2] = seast_x
                    
                    voxels[yi][xi].voxel_3x3_map = voxel_3x3_map
                    
    def __gen_voxel_shades(self):
        if self.is_needed_shades:
            voxels_geom_flattened = [v.voxel_geom for v in self.voxels_objects_flattened if v.voxel_condition != self.NONE]
            for voxels in self.voxels_objects:
                for y_voxels in voxels:
                    for vi, v in enumerate(y_voxels):
                        if (
                            v.voxel_condition == self.NONE 
                            or v.voxel_condition not in (self.EXTERIOR, self.EXTERIOR_CORNER, self.ROOF)
                        ):
                            continue
                        
                        ray = rg.Line(self.sun_point, v.voxel_geom_centroid)
                        shade = 0
                        for ov in voxels_geom_flattened:
                            intersects = rg.Intersect.Intersection.MeshLine(ov, ray)
                            if len(intersects) > 0:
                                shade += len(intersects)
                        
                        y_voxels[vi].voxel_shade = shade
                        
    def __gen_placed_voxel_unit(self):
        for voxel_object in self.voxels_objects_flattened:
            each_voxel_condition = voxel_object.voxel_condition
            
            if each_voxel_condition in (self.NONE, self.INTERIOR):
                continue
                
            voxel_3x3_map = voxel_object.voxel_3x3_map
                
            north = voxel_3x3_map[0][1]
            west = voxel_3x3_map[1][0]
            east = voxel_3x3_map[1][2]
            south = voxel_3x3_map[2][1]
            
            nwest = not north and not west
            neast = not north and not east
            swest = not south and not west
            seast = not south and not east
            
            vector = voxel_object.voxel_geom_centroid - self.exterior_unit_centroid
            rotate_angle = 0
            
            if each_voxel_condition == self.EXTERIOR:
                base_unit = self.rotated_exterior_unit
                
                if not east:
                    rotate_angle = ROTATE_270
                elif not west:
                    rotate_angle = ROTATE_90
                elif not south:
                    rotate_angle = ROTATE_180
                
            elif each_voxel_condition == self.EXTERIOR_CORNER:
                base_unit = self.rotated_exterior_corner_unit
                
                if neast:
                    rotate_angle = ROTATE_270
                elif swest:
                    rotate_angle = ROTATE_90
                elif seast:
                    rotate_angle = ROTATE_180
                
            elif each_voxel_condition == self.EXTERIOR_BOTH:
                base_unit = self.rotated_exterior_both_unit
                
                if not east and not west:
                    rotate_angle = ROTATE_90
                        
            elif each_voxel_condition == self.EXTERIOR_CORNER_O:
                base_unit = self.rotated_exterior_corner_o_unit
                
            elif each_voxel_condition == self.EXTERIOR_CORNER_U:
                base_unit = self.rotated_exterior_corner_u_unit
                
                if nwest and neast:
                    rotate_angle = ROTATE_270
                elif swest and seast:
                    rotate_angle = ROTATE_90
                elif neast and seast:
                    rotate_angle = ROTATE_180
             
            if voxel_object.is_roof:
                 voxel_object.voxel_condition = self.ROOF
                 
            voxel_object.voxel_geom = gh.Move(base_unit, vector).geometry
            voxel_object.voxel_geom = gh.Rotate(
                voxel_object.voxel_geom, rotate_angle, voxel_object.voxel_geom_centroid
            ).geometry
        
    def __gen_voxel_shape_parapet(self):
        self.roof_faces = []
        if self.is_needed_resize:
            self.joined_voxel_geoms_mesh = []
            self.roof_voxels_objects = []
            for voxels in self.voxels_objects:
                for y_voxels in voxels:
                    for v in y_voxels:
                        if v.is_roof:
                            self.roof_faces.append(v.voxel_box.ToBrep().Faces[5].ToBrep())
                            self.roof_voxels_objects.append(v)
                            
                        if v.voxel_condition != self.NONE:
                            self.joined_voxel_geoms_mesh.append(v.voxel_geom)
                            
            self.joined_voxel_geom_mesh = gh.MeshJoin(self.joined_voxel_geoms_mesh)
            self.merged_roof_faces = gh.BrepJoin(gh.MergeFaces(self.roof_faces).breps).breps
            self.all_parapet_edges = gh.BrepEdges(self.merged_roof_faces).naked
            
            self.parapet_edges = []
            for i, parapet_edge in enumerate(self.all_parapet_edges):
                moved_parapet_edge = gh.Move(parapet_edge, rg.Point3d(0, 0, PARAPET_HEIGHT)).geometry
                moved_parapet_edge_centroid = moved_parapet_edge.PointAtNormalizedLength(0.5)
                closest_point = gh.MeshClosestPoint(moved_parapet_edge_centroid, self.joined_voxel_geom_mesh).point
                
                if self.is_close(closest_point.DistanceTo(moved_parapet_edge_centroid), PARAPET_HEIGHT):
                    self.parapet_edges.append(parapet_edge)
            
            self.parapet = gh.Extrude(self.parapet_edges, rg.Point3d(0, 0, PARAPET_HEIGHT))
            self.parapet_mesh = [rg.Mesh.CreateFromBrep(parapet)[0] for parapet in self.parapet]
            
            for rvi, roof_voxel in enumerate(self.roof_voxels_objects):
                for pi, parapet_edge in enumerate(self.parapet_edges):
                    parapet_edge_centroid = parapet_edge.PointAtNormalizedLength(0.5)
                    roof_face_centroid = roof_voxel.voxel_geom_centroid + rg.Point3d(0, 0, self.voxel_size / 2)
                    
                    if self.is_close(roof_face_centroid.DistanceTo(parapet_edge_centroid), self.voxel_size / 2):
                        roof_voxel.voxel_geom = gh.MeshJoin([roof_voxel.voxel_geom, self.parapet_mesh[pi]])
        
    def __get_reshaped_list(self, one_dim_list, x_shape, y_shape, z_shape):
        x_divided_list = [
            one_dim_list[x : x + x_shape] for x in range(0, len(one_dim_list), x_shape)
        ]
        
        y_divided_list = [
            x_divided_list[y : y + y_shape] for y in range(0, len(x_divided_list), y_shape)
        ]
        
        z_divided_list = [
            y_divided_list[z : z + z_shape] for z in range(0, len(y_divided_list), z_shape)
        ]
        
        return z_divided_list[0]
        
    def __get_voxels_condition_list(self):
        inside_grid_centroid_integer = [int(b) for b in self.inside_grid_centroid]
        voxel_3d_list = self.__get_reshaped_list(inside_grid_centroid_integer, *self.cols)
        
        
        roof_3d_list = self.__get_default_3d_list()
        exterior_3d_list = self.__get_default_3d_list()
        exterior_both_3d_list = self.__get_default_3d_list()
        exterior_corner_3d_list = self.__get_default_3d_list()
        exterior_corner_u_3d_list = self.__get_default_3d_list()
        exterior_corner_o_3d_list = self.__get_default_3d_list()
        
        for zi, z_list in enumerate(voxel_3d_list):
            prev_zi = (zi - 1) % len(voxel_3d_list)
            next_zi = (zi + 1) % len(voxel_3d_list)
            
            for yi, y_list in enumerate(z_list):
                prev_yi = (yi - 1) % len(z_list)
                next_yi = (yi + 1) % len(z_list)
                
                for xi, x in enumerate(y_list):
                    prev_x = bool(y_list[(xi - 1) % len(y_list)])
                    next_x = bool(y_list[(xi + 1) % len(y_list)])
                    back_x = bool(z_list[prev_yi][xi])
                    frnt_x = bool(z_list[next_yi][xi])
                    
                    exterior_count = [prev_x, next_x, back_x, frnt_x].count(False)
                    
                    is_roof = bool(x) and not bool(voxel_3d_list[next_zi][yi][xi])
                    is_exterior_both = bool(x) and (not prev_x and not next_x) or (not back_x and not frnt_x)
                    is_exterior = bool(x) and any([not prev_x, not next_x, not back_x, not frnt_x]) and not is_exterior_both
                    is_exterior_corner = bool(x) and exterior_count == 2 and not is_exterior_both
                    is_exterior_corner_u = bool(x) and exterior_count == 3
                    is_exterior_corner_o = bool(x) and exterior_count == 4
                    
                    if is_roof:
                        roof_3d_list[zi][yi][xi] = 1
                    if is_exterior:
                        exterior_3d_list[zi][yi][xi] = 1
                    if is_exterior_both:
                        exterior_both_3d_list[zi][yi][xi] = 1
                    if is_exterior_corner:
                        exterior_corner_3d_list[zi][yi][xi] = 1
                    if is_exterior_corner_o:
                        exterior_corner_o_3d_list[zi][yi][xi] = 1
                    if is_exterior_corner_u:
                        exterior_corner_u_3d_list[zi][yi][xi] = 1
                        
        interior_3d_list = self.__get_default_3d_list()
        for zi, (z_list, e_list) in enumerate(zip(voxel_3d_list, exterior_3d_list)):
            for yi, (z, e) in enumerate(zip(z_list, e_list)):
                for xi, (zx, ex) in enumerate(zip(z, e)):
                    interior_3d_list[zi][yi][xi] = zx - ex
        
        grid_3d_list = self.__get_reshaped_list(self.grid, *self.cols)
        
        return (
            voxel_3d_list, 
            grid_3d_list, 
            roof_3d_list, 
            exterior_3d_list, 
            exterior_both_3d_list,
            exterior_corner_3d_list, 
            exterior_corner_o_3d_list,
            exterior_corner_u_3d_list,
            interior_3d_list
        )
        
    def __get_default_3d_list(self):
        zeros = [0] * len(self.grid_centroid)
        return self.__get_reshaped_list(zeros, *self.cols)


if __name__ == "__main__":
    voxel_shape = VoxelShape(
        brep=brep, 
        voxel_size=3.5,
        exterior_unit=exterior_unit,
        exterior_both_unit=exterior_both_unit,
        exterior_corner_unit=exterior_corner_unit,
        exterior_corner_o_unit=exterior_corner_o_unit,
        exterior_corner_u_unit=exterior_corner_u_unit,
        sun_position=sun_position,
        is_needed_shades=False,
        is_needed_resize=True,
    )
    
    sun_vector = voxel_shape.sun_vector
    environment = gh.DeconstructBrep(voxel_shape.hemisphere).edges + [voxel_shape.sun]
    
    voxel_geoms_roof_faces = [
        gh.Move(face, rg.Point3d(0, 0, PARAPET_HEIGHT * 0.2)).geometry 
        for face in voxel_shape.roof_faces
    ]
    
    voxels_geoms = []
    voxels_conditions = []
    voxels_shades = []
    voxel_geoms_roof_faces_colors = []
    
    for voxels in voxel_shape.voxels_objects:
        for y_voxels in voxels:
            for v in y_voxels:
                if v.voxel_condition == voxel_shape.NONE:
                    continue
                
                voxels_geoms.append(v.voxel_geom)
                voxels_conditions.append(v.voxel_condition)
                voxels_shades.append(v.voxel_shade)
                
                if v.is_roof:
                    voxel_geoms_roof_faces_colors.append(v.roof_color_index)
