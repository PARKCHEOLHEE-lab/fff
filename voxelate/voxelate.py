﻿"""need to cleanup code"""
import math
import Rhino.Geometry as rg
import rhinoscriptsyntax as rs
import ghpythonlib.components as gh



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
        self.environment_bbox = rg.Brep.GetBoundingBox(self.brep, rg.Plane.WorldXY)
        self.environment_bbox_faces, _, _ = gh.DeconstructBrep(self.environment_bbox)
        self.environment_bbox_bottom_face = sorted(self.environment_bbox_faces, key=lambda f: gh.Area(geometry=f).centroid.Z)[0]
        _, self.environment_bbox_bottom_face_centroid = gh.Area(self.environment_bbox_bottom_face)
        self.environment_bbox_bottom_face_scaling, _ = gh.Scale(
            geometry=self.environment_bbox_bottom_face,
            center=self.environment_bbox_bottom_face_centroid,
            factor=10,
        )
        
    def __gen_hemisphere(self):
        self.sphere = gh.Sphere(
            base=self.environment_bbox_bottom_face_centroid,
            radius=100,
        )
        _, self.hemisphere = self.sphere.Split(self.environment_bbox_bottom_face_scaling, 0.001)
        self.interval = rg.Interval(0, 1)
        self.hemisphere.Faces[0].SetDomain(0, self.interval)
        self.hemisphere.Faces[0].SetDomain(1, self.interval)
        
    def __gen_sun(self):
        self.sun_point = self.hemisphere.Faces[0].Evaluate(
            u=self.sun_position[0][0], 
            v=self.sun_position[0][1], 
            numberDerivatives=0,
        )[1]
        
        self.sun = gh.Sphere(base=self.sun_point, radius=5)
        
    def __gen_sun_facing_plane(self):
        self.sun_point_to_face_centroid = rg.Line(self.sun_point, self.environment_bbox_bottom_face_centroid)
        self.projected_sun_point_to_face_centroid = gh.Project(
            curve=self.sun_point_to_face_centroid,
            brep=self.environment_bbox_bottom_face_scaling,
            direction=rg.Point3d(0, 0, 1)
        )
        
        self.plane = (
            rg.Plane.WorldXY if self.projected_sun_point_to_face_centroid is None 
            else gh.HorizontalFrame(self.projected_sun_point_to_face_centroid, 0.5)
        )


class Voxel:
    def __init__(
        self, 
        voxel_geom=None,
        facing_angle=None,
        is_roof=False,
        is_exterior=False,
        is_sun_facing=False,
    ):
        self.voxel_geom = voxel_geom
        self.facing_angle = facing_angle
        self.is_roof = is_roof
        self.is_exterior = is_exterior
        self.is_sun_facing = is_sun_facing
        
    def get_voxel_information(self, voxel_geom, sun_centroid):
        facing_angle = self.__get_facing_angle(gh.Volume(voxel_geom).centroid, sun_centroid)
        voxel_geom.ToBrep().Faces

        return Voxel(
            voxel_geom=voxel_geom, 
            facing_angle=facing_angle,
            is_roof=False,
            is_exterior=False,
            is_sun_facing=False,
        )
        
    def __get_facing_angle(self, voxel_geom_centroid, sun_centroid):
        x1, y1, _ = voxel_geom_centroid
        x2, y2, _ = sun_centroid
        return math.atan2(y2 - y1, x2 - x1)


class VoxelBrep(Voxel, Environment):
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
        
    def __gen_moved_brep(self):
        self.moved_brep, _ = gh.Move(geometry=self.brep, motion=rg.Point3d(150, 0, 0))
        self.moved_brep_mesh = rg.Mesh.CreateFromBrep(self.moved_brep, rg.MeshingParameters(0))
        
    def __gen_environment(self):
        Environment.__init__(self, self.moved_brep, sun_position)
        Voxel.__init__(self)
        
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
        
        x_grid = []
        for x in range(int(seg_1.Line.Length // self.voxel_size) + 1):
            motion = seg_1_vector * x
            moved_rectangle, _ = gh.Move(geometry=base_rectangle, motion=motion)
            x_grid.append(moved_rectangle)
        
        base_grid = []
        for y in range(int(seg_2.Line.Length // self.voxel_size) + 1):
            motion = seg_2_vector * y
            moved_rectangles, _ = gh.Move(geometry=x_grid, motion=motion)
            base_grid.extend(moved_rectangles)
        
        self.grid = []
        for z in range(int(self.moved_brep_bbox.Z[1] // self.voxel_size) + 1):
            motion = rg.Point3d(0, 0, z * self.voxel_size)
            moved_rectangles, _ = gh.Move(geometry=base_grid, motion=motion)
            self.grid.extend(moved_rectangles)
            
    def __gen_voxels(self):
        self.grid_centroid = [g.Center for g in self.grid]
        self.inside_grid_centroid = gh.MeshInclusion(
            mesh=gh.MeshJoin(self.moved_brep_mesh), 
            point=self.grid_centroid, 
            strict=False
        )
        
        self.inside_grid = [
             self.grid[ci] for ci, p in enumerate(self.inside_grid_centroid) if p
        ]
        
        _, sun_centroid = gh.Volume(self.sun)
        self.voxels = gh.BoxRectangle(self.inside_grid, self.voxel_size)
        self.voxels_objects = []
        for vi, voxel_geom in enumerate(self.voxels):
            other_voxels = self.voxels[:vi] + self.voxels[vi+1:]
            voxel_object = self.get_voxel_information(voxel_geom, sun_centroid)
            
            
            self.voxels_objects.append(voxel_object)
            print(len(other_voxels))
            
        self.voxels_mesh = [
            rg.Mesh.CreateFromBox(v, 1, 1, 1) for v in self.voxels
        ]


if __name__ == "__main__":
    voxel_brep = VoxelBrep(
        brep=brep, 
        voxel_size=3, 
        sun_position=sun_position
    )
    
    voxels = [v.voxel_geom for v in voxel_brep.voxels_objects]
    angles = [v.facing_angle for v in voxel_brep.voxels_objects]
    environment = voxel_brep.hemisphere, voxel_brep.sun
    a = voxel_brep.voxels_mesh
