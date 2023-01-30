"""need to cleanup code"""
import math
import Rhino.Geometry as rg
import ghpythonlib.components as gh
import ghpythonlib.treehelpers as gt


TOLERANCE = 0.001
TOLERANCE_MICRO = 0.00001
HEMISPHERE_RAD = 150
SUN_RAD = 5
MOVE_DIST = HEMISPHERE_RAD * 1.7


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
        
    def __gen_environment_bbox(self):
        self.environment_bbox = self.brep.GetBoundingBox(rg.Plane.WorldXY)
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
            rg.Plane.WorldXY if self.projected_sun_point_to_face_centroid is None 
            else gh.HorizontalFrame(self.projected_sun_point_to_face_centroid, 0.5)
        )


class VoxelConditions:
    UNDEFINED = -1
    ROOF = 3
    EXTERIOR = 4
    INTERIOR = 5


class Voxel:
    def __init__(
        self, 
        voxel_geom=None,
        voxel_geom_centroid=None,
        voxel_condition=None,
        facing_angle=None,
        is_roof=False,
        is_exterior=False,
        is_sun_facing=False,
    ):
        self.voxel_geom = voxel_geom
        self.voxel_geom_centroid = voxel_geom_centroid
        self.voxel_condition = voxel_condition
        self.facing_angle = facing_angle
        self.is_roof = is_roof
        self.is_exterior = is_exterior
        self.is_sun_facing = is_sun_facing
        
    """ archived
    
    def get_voxel_information(self, voxel_geom, sun_centroid, remain_voxels_centroids):
        voxel_geom_centroid = gh.Volume(voxel_geom).centroid
        is_roof, is_exterior, is_sun_facing = self.__get_voxel_statement(voxel_geom, remain_voxels_centroids)
        
        return Voxel(
            voxel_geom=voxel_geom,
            voxel_geom_centroid=voxel_geom_centroid,
            facing_angle=self.__get_facing_angle(voxel_geom_centroid, sun_centroid),
            is_roof=is_roof,
            is_exterior=is_exterior,
            is_sun_facing=False,
        )
        
    def __get_voxel_statement(self, voxel_geom, remain_voxels_centroids):
        def __is_close(n1, n2, tolerance=TOLERANCE):
            return abs(n1 - n2) <= tolerance
        
        voxel_geom_faces_centroids, _ = gh.FaceNormals(voxel_geom)
        remain_voxels_centroids_cloud = rg.PointCloud(remain_voxels_centroids)
        
        c = 0
        is_roof = False
        is_exterior = False
        is_sun_facing = False
        
        for fci, face_centroid in enumerate(voxel_geom_faces_centroids[1:]):
            closest_point_idx = remain_voxels_centroids_cloud.ClosestPoint(face_centroid)
            statement_checker = __is_close(
                remain_voxels_centroids[closest_point_idx].DistanceTo(face_centroid), 
                self.voxel_size / 2
            )
            
            if fci == 0:
                is_roof = not statement_checker
                
            elif fci in (1, 2, 3, 4) and statement_checker:
                c += 1
                
        is_exterior = c != 4 and not is_roof
        
        return is_roof, is_exterior, False
        
    archived """
        
    def get_voxel_information_v2(self, voxel_geom, sun_centroid, voxel_condition):
        voxel_geom_centroid = gh.Volume(voxel_geom).centroid
        
        return Voxel(
            voxel_geom=voxel_geom,
            voxel_geom_centroid=voxel_geom_centroid,
            facing_angle=self.__get_facing_angle(voxel_geom_centroid, sun_centroid),
            voxel_condition=voxel_condition
        )
        
    def __get_facing_angle(self, voxel_geom_centroid, sun_centroid):
        x1, y1, _ = voxel_geom_centroid
        x2, y2, _ = sun_centroid
        return math.atan2(y2 - y1, x2 - x1)


class VoxelShape(Voxel, VoxelConditions, Environment):
    def __init__(self, brep, voxel_size, sun_position):
        self.brep = brep
        self.voxel_size = voxel_size
        self.sun_position = sun_position
        self.__voxelate()
        
    def __len__(self):
        return len(self.voxels)
        
    def __getitem__(self, i):
        return self.voxels[i]
        
    def __voxelate(self):
        self.__gen_moved_brep()
        self.__gen_environment()
        self.__gen_moved_brep_bbox()
        self.__gen_grid()
        self.__gen_voxels()
        self.__gen_voxel_shades()
        
    def __gen_moved_brep(self):
        self.moved_brep, _ = gh.Move(geometry=self.brep, motion=rg.Point3d(MOVE_DIST, 0, 0))
        self.moved_brep_mesh = rg.Mesh.CreateFromBrep(self.moved_brep, rg.MeshingParameters(0))
        
    def __gen_environment(self):
        Environment.__init__(self, self.moved_brep, sun_position)
        Voxel.__init__(self)
        VoxelConditions.__init__(self)
        
    def __gen_moved_brep_bbox(self):
        self.moved_brep_bbox, _ = gh.BoundingBox(content=self.moved_brep, plane=self.plane)
        self.moved_brep_bbox_faces, _, _ = gh.DeconstructBrep(brep=self.moved_brep_bbox)
        self.moved_brep_bbox_bottom_face = sorted(self.moved_brep_bbox_faces, key=lambda f: gh.Area(geometry=f).centroid.Z)[0]
        
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
        
        x_grid = []
        for x in range(self.x_cols):
            motion = seg_1_vector * x
            moved_rectangle, _ = gh.Move(geometry=base_rectangle, motion=motion)
            x_grid.append(moved_rectangle)
        
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
            
    def __gen_voxels(self):
        self.grid_centroid = [
            g.Center if g.Center.Z != 0 
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
            self.interior_3d_list
        ) = self.__get_voxels_condition_list()
        
        self.voxels_objects = []
        _, sun_centroid = gh.Volume(self.sun)
        
        for zi, voxels in enumerate(self.voxel_3d_list):
            for yi, solid_conditions in enumerate(voxels):
                if all([not bool(y) for y in solid_conditions]):
                    continue
                
                roof_conditions = self.roof_3d_list[zi][yi]
                exterior_conditions = self.exterior_3d_list[zi][yi]
                interior_conditions = self.interior_3d_list[zi][yi]
                
                for xi, sc in enumerate(solid_conditions):
                    if not bool(sc):
                        continue
                    
                    voxel_geom = (
                        rg.Mesh.CreateFromBox(
                            gh.BoxRectangle(self.grid_3d_list[zi][yi][xi], self.voxel_size),
                            1,
                            1,
                            1
                        )
                    )
                    
                    voxel_condition = self.UNDEFINED
                    if bool(roof_conditions[xi]):
                        voxel_condition = self.ROOF
                    elif bool(exterior_conditions[xi]):
                        voxel_condition = self.EXTERIOR                    
                    elif bool(interior_conditions[xi]):
                        voxel_condition = self.INTERIOR  
                        
                    self.voxels_objects.append(
                        self.get_voxel_information_v2(voxel_geom, sun_centroid, voxel_condition)
                    )
        
        """ archived
        
        self.inside_grid = [
             self.grid[ci] for ci, p in enumerate(self.inside_grid_centroid) if p
        ]
        
        self.voxels_centroids = [
            g.Center + rg.Point3d(0, 0, self.voxel_size / 2) for g in self.inside_grid
        ]
        
        self.voxels = [
            rg.Mesh.CreateFromBox(v, 1, 1, 1) for v in gh.BoxRectangle(self.inside_grid, self.voxel_size)
        ]
        
        
        for vi, voxel_geom in enumerate(self.voxels):
            remain_voxels_centroids = self.voxels_centroids[:vi] + self.voxels_centroids[vi+1:]
            voxel_object = self.get_voxel_information(voxel_geom, sun_centroid, remain_voxels_centroids)
            
            self.voxels_objects.append(voxel_object)
            
        arcihved """
        
    def __gen_voxel_shades(self):
        roof_exterior_centroid_ray = [
            gh.Line(self.sun_point, v.voxel_geom_centroid)
            for v in self.voxels_objects 
            if v.voxel_condition in (VoxelConditions.EXTERIOR, VoxelConditions.ROOF)
        ]
        
        self.roof_exterior_voxels = [
            v.voxel_geom
            for v in self.voxels_objects 
            if v.voxel_condition in (VoxelConditions.EXTERIOR, VoxelConditions.ROOF)
        ]
        
        self.shades = []
        for ray in roof_exterior_centroid_ray:
            shade = []
            for voxel in self.voxels_objects:
                _, count = rg.Intersect.Intersection.MeshLine(voxel.voxel_geom, ray)
                
                if count is not None:
                    shade.append(count[0])
            
            self.shades.append(sum(shade))

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
        voxel_3d_list = self.__get_reshaped_list(
            inside_grid_centroid_integer, self.x_cols, self.y_cols, self.z_cols
        )
        
        roof_3d_list = self.__get_reshaped_list([0] * len(self.grid_centroid), self.x_cols, self.y_cols, self.z_cols)
        exterior_3d_list = self.__get_reshaped_list([0] * len(self.grid_centroid), self.x_cols, self.y_cols, self.z_cols)
        for zi, z_list in enumerate(voxel_3d_list):
            prev_zi = (zi - 1) % len(voxel_3d_list)
            next_zi = (zi + 1) % len(voxel_3d_list)
            
            for yi, y_list in enumerate(z_list):
                prev_yi = (yi - 1) % len(z_list)
                next_yi = (yi + 1) % len(z_list)
                
                for xi, x in enumerate(y_list):
                    prev_x = y_list[(xi - 1) % len(y_list)]
                    next_x = y_list[(xi + 1) % len(y_list)]
                    back_x = z_list[prev_yi][xi]
                    frnt_x = z_list[next_yi][xi]
                    
                    is_roof = bool(x) and not bool(voxel_3d_list[next_zi][yi][xi])
                    is_exterior = bool(x) and any([not prev_x, not next_x, not back_x, not frnt_x])
                    if is_roof:
                        roof_3d_list[zi][yi][xi] = 1
                    
                    if is_exterior:
                        exterior_3d_list[zi][yi][xi] = 1
                        
        interior_3d_list = self.__get_reshaped_list([0] * len(self.grid_centroid), self.x_cols, self.y_cols, self.z_cols)
        for zi, (z_list, e_list) in enumerate(zip(voxel_3d_list, exterior_3d_list)):
            for yi, (z, e) in enumerate(zip(z_list, e_list)):
                for xi, (zx, ex) in enumerate(zip(z, e)):
                    interior_3d_list[zi][yi][xi] = zx - ex
        
        for zi, (r_list, e_list) in enumerate(zip(roof_3d_list, exterior_3d_list)):
            for yi, (r, e) in enumerate(zip(r_list, e_list)):
                for xi, (rx, ex) in enumerate(zip(r, e)):
                    if bool(rx) and bool(ex):
                        exterior_3d_list[zi][yi][xi] = 0
        
        grid_3d_list = self.__get_reshaped_list(self.grid, self.x_cols, self.y_cols, self.z_cols)
        
        return voxel_3d_list, grid_3d_list, roof_3d_list, exterior_3d_list, interior_3d_list



if __name__ == "__main__":
    print(brep)
    voxel_shape = VoxelShape(
        brep=brep, 
        voxel_size=voxel_size, 
        sun_position=sun_position
    )
    
    environment = gh.DeconstructBrep(voxel_shape.hemisphere).edges + [voxel_shape.sun]
    sun_vector = voxel_shape.sun_vector
    
    voxels = [v.voxel_geom for v in voxel_shape.voxels_objects]
    angles = [v.facing_angle for v in voxel_shape.voxels_objects]
    conditions = [v.voxel_condition for v in voxel_shape.voxels_objects]
    shades = voxel_shape.shades
    roof_exterior_voxels = voxel_shape.roof_exterior_voxels
    