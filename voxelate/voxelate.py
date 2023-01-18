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
        
    def __gen_environment_bbox(self):
        self.environment_bbox = rg.Brep.GetBoundingBox(self.brep, rg.Plane.WorldXY)
        self.environment_bbox_faces, _, _ = gh.DeconstructBrep(self.environment_bbox)
        self.environment_bbox_bottom_face = sorted(self.environment_bbox_faces, key=lambda f: gh.Area(geometry=f).centroid.Z)[0]
        self.environment_bbox_bottom_face_scaling, _ = gh.Scale(
            geometry=self.environment_bbox_bottom_face,
            center=gh.Area(self.environment_bbox_bottom_face).centroid,
            factor=10,
        )
        
    def __gen_hemisphere(self):
        self.sphere = gh.Sphere(
            base=gh.Area(self.environment_bbox_bottom_face).centroid,
            radius=100,
        )
        _, self.hemisphere = self.sphere.Split(self.environment_bbox_bottom_face_scaling, 0.001)
        self.interval = rg.Interval(0, 1)
        self.hemisphere.Faces[0].SetDomain(0, self.interval)
        self.hemisphere.Faces[0].SetDomain(1, self.interval)
        
    def __gen_sun(self):
        self.sun = self.hemisphere.Faces[0].Evaluate(
            u=self.sun_position[0][0], 
            v=self.sun_position[0][1], 
            numberDerivatives=0,
        )[1]
        

class VoxelBrep(Environment):
    def __init__(self, brep, voxel_size, voxel_angle, sun_position):
        self.brep = brep
        self.voxel_size = voxel_size
        self.voxel_angle = voxel_angle
        self.sun_position = sun_position
        self.__voxelate()
        
    def __voxelate(self):
        self.__gen_moved_brep()
        self.__gen_environment()
        self.__gen_plane()
        self.__gen_moved_brep_bbox()
        self.__gen_grid()
        self.__gen_voxels()
        
    def __gen_moved_brep(self):
        self.moved_brep, _ = gh.Move(geometry=self.brep, motion=rg.Point3d(100, 0, 0))
        self.moved_brep_mesh = rg.Mesh.CreateFromBrep(self.moved_brep, rg.MeshingParameters(0))
        
    def __gen_environment(self):
        Environment.__init__(self, self.moved_brep, sun_position)
        
    def __gen_plane(self):
        self.plane = gh.RotatePlane(plane=rg.Plane.WorldXY, angle=self.voxel_angle)
        
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
        self.inside_grid = gh.CullPattern(
            list=self.grid, 
            cull_pattern=gh.MeshInclusion(
                mesh=gh.MeshJoin(self.moved_brep_mesh), 
                point=self.grid_centroid, 
                strict=False
            )
        )
        
        self.inside_grid_centroid = [g.Center for g in self.inside_grid]
        self.voxels_plane = gh.ConstructPlane(
            self.inside_grid_centroid,
            self.plane.XAxis,
            self.plane.YAxis
        )
        
        interval = rg.Interval(-self.voxel_size / 2, self.voxel_size / 2)
        self.voxels = gh.DomainBox(
            base=self.voxels_plane,
            x=interval,
            y=interval,
            z=self.voxel_size
            
        )
        
        self.voxels_mesh = [
            rg.Mesh.CreateFromBox(v, 1, 1, 1)
            for v in self.voxels
        ]           
        


if __name__ == "__main__":
    voxel_brep = VoxelBrep(
        brep=brep, 
        voxel_size=2.5, 
        voxel_angle=math.radians(degree), 
        sun_position=sun_position
    )
    
    voxels = voxel_brep.voxels_mesh
    a = voxel_brep.sun, voxel_brep.hemisphere