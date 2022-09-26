import Rhino.Geometry as rg
import ghpythonlib.components as gh
import ghpythonlib.treehelpers as gt


FENCE_INTERVAL = 20.9
FENCE_COLUMN_RADIUS = 0.38
FENCE_GRID_COUNT = 32
FENCE_HEIGHT = 17
FENCE_Y_VECTOR = rg.Point3d(0, -FENCE_HEIGHT / 4, 0)
FENCE_Z_VECTOR = rg.Point3d(0, 0, FENCE_HEIGHT)
FENCE_Y_EXTRUDE = rg.Point3d(0, 5, 0)

TOLERANCE = 0.001

def is_close(a, b, tol=TOLERANCE):
    if abs(a - b) < tol:
        return True
    return False

class DynamicMeshFence:
    def __init__(self, fence_count, directions, shape):
        self.fence_count = fence_count
        self.generate(directions, shape)
        
    def generate(self, directions, shape):
        """generate dynamic mesh fence"""
        
        self.get_directions(directions)
        self.get_merged_input_shapes(shape)
        self.get_fence_columns()
        self.get_fence_base_geoms()
        self.get_clean_trimmed_shapes()
        self.get_merged_trimmed_shapes()
        self.get_fence_contour_lines()
        
        if self.merged_trimmed_shapes is None:
            self.result_mesh_fence = gt.list_to_tree(self.fence_faces_contour_lines)
            
        else:
            self.get_splitted_fence_contour_lines()
            self.get_result_mesh_fence()
            self.result_mesh_fence = gt.list_to_tree(self.result_mesh_fence)
            
    def get_directions(self, directions):
        """calculate trim directions"""
        self.directions = directions
        if len(directions) != self.fence_count:
            diff_directions_count = abs(len(directions) - self.fence_count)
            self.directions = directions[:diff_directions_count] + [directions[-1]] * (diff_directions_count + 1)
        
    def get_merged_input_shapes(self, shape):
        """union & merge input brep shapes"""
        
        if len(shape) == 0 or all(s is None for s in shape):
            shape = gh.Sphere(rg.Point3d(0, 0, 500), 1)  # dummy shape
                
        union_shape = gh.SolidUnion(shape)
        if isinstance(union_shape, rg.Brep):
            union_shape = [union_shape]
        
        merged_shape = rg.Brep.MergeBreps(union_shape, TOLERANCE)
        self.shape = merged_shape
        
    def get_fence_columns(self):
        """generate fence columns"""
                       
        self.fence_origin = []         # List[List[Point3d]]
        self.fence_columns = []        # List[Brep]
        
        for n in range(self.fence_count + 1):
            origin = rg.Point3d(n * FENCE_INTERVAL, 0, 0)
            self.fence_origin.append(origin)
            
            column_base = rg.Circle(origin, FENCE_COLUMN_RADIUS)
            column = gh.Extrude(column_base, FENCE_Z_VECTOR)
            column = column.CapPlanarHoles(0.001)
            
            self.fence_columns.append(column)
            
    def get_fence_base_geoms(self):
        """generate base geometries for dynamic mesh fence"""

        self.trimmed_shapes = []         # List[Brep]
        self.fence_faces = []            # List[Brep]
        self.extruded_fence_faces = []   # List[Brep]
        self.adjusted_origin = []        # List[List[Point3d]]
        
        for i, direction in zip(range(len(self.fence_origin) - 1), self.directions):
            curr_origin = self.fence_origin[i]
            next_origin = self.fence_origin[i+1]
            
            adjusted_curr_origin = curr_origin + rg.Point3d(FENCE_COLUMN_RADIUS, 0, 0)
            adjusted_next_origin = next_origin + rg.Point3d(-FENCE_COLUMN_RADIUS, 0, 0)
            self.adjusted_origin.append([adjusted_curr_origin, adjusted_next_origin])
            
            fence_base_line = rg.Line(adjusted_curr_origin, adjusted_next_origin)
            fence_face = gh.Extrude(fence_base_line, FENCE_Z_VECTOR)
            self.fence_faces.append(fence_face)
            
            scaled_fence_face = gh.Scale(fence_face, gh.Area(fence_face).centroid, 0.95).geometry
            scaled_fence_face_extrusion = gh.Extrude(scaled_fence_face, FENCE_Y_VECTOR)
            if direction:
                scaled_fence_face_extrusion = gh.Extrude(scaled_fence_face, -FENCE_Y_VECTOR)
                
            self.extruded_fence_faces.append(scaled_fence_face_extrusion)
            
            each_trimmed_shapes = rg.Brep.CreateBooleanIntersection(scaled_fence_face_extrusion, self.shape, TOLERANCE)
            self.trimmed_shapes.extend(each_trimmed_shapes)
            
    def get_clean_trimmed_shapes(self):
        """remove shapes not intersecting fence face"""
        
        self.result_trimmed_shapes = []  # List[Brep]
        merged_fence_faces = rg.Brep.MergeBreps(self.fence_faces, TOLERANCE)
        
        for each_trimmed_shape in self.trimmed_shapes:
            is_intsc_shape = gh.BrepXBrep(each_trimmed_shape, merged_fence_faces).curves
            if is_intsc_shape is not None:
                self.result_trimmed_shapes.append(each_trimmed_shape)
                
    def get_merged_trimmed_shapes(self):
        self.merged_trimmed_shapes = rg.Brep.MergeBreps(self.result_trimmed_shapes, TOLERANCE)
                
    def get_fence_contour_lines(self):
        """generate base fence lines & trimmed shapes contour lines"""
        
        self.fence_faces_contour_lines = []  # List[]
        self.trimmed_shapes_contour_lines = []
        
        for (start_origin, end_origin), fence_face in zip(self.adjusted_origin, self.fence_faces):
            # generate each fence face contour lines
            each_face_contour_lines = rg.Brep.CreateContourCurves(fence_face, 
                                                                  start_origin, 
                                                                  end_origin, 
                                                                  FENCE_INTERVAL / FENCE_GRID_COUNT)[1:]
                                                                  
            each_face_contour_lines_z = rg.Brep.CreateContourCurves(fence_face, 
                                                                    rg.Point3d(0,0,FENCE_HEIGHT), 
                                                                    rg.Point3d(0,0,0), 
                                                                    FENCE_HEIGHT / FENCE_GRID_COUNT * 2.5)
                                                                    
            self.fence_faces_contour_lines.append(each_face_contour_lines + each_face_contour_lines_z)
            
            if self.merged_trimmed_shapes is None:
                continue
            
            # generate each shape contour lines
            merged_shapes_contour_lines = rg.Brep.CreateContourCurves(self.merged_trimmed_shapes, 
                                                                      start_origin, 
                                                                      end_origin, 
                                                                      FENCE_INTERVAL / FENCE_GRID_COUNT)
            
            simplified_each_shape_contour_lines = gh.SimplifyCurve(merged_shapes_contour_lines).curve
            
            if simplified_each_shape_contour_lines is None:
                self.trimmed_shapes_contour_lines.append(each_face_contour_lines + each_face_contour_lines_z)
                continue
            
            if isinstance(simplified_each_shape_contour_lines, rg.PolyCurve):
                simplified_each_shape_contour_lines = [simplified_each_shape_contour_lines]
            
            exploded_each_shape_contour_lines = []
            for shape_contour_line in simplified_each_shape_contour_lines:
                exploded_contour_lines = rg.PolylineCurve.DuplicateSegments(shape_contour_line)
                exploded_each_shape_contour_lines.extend(exploded_contour_lines)
                
            merged_shapes_contour_lines_z = rg.Brep.CreateContourCurves(self.merged_trimmed_shapes,  
                                                                        rg.Point3d(0,0,FENCE_HEIGHT), 
                                                                        rg.Point3d(0,0,0), 
                                                                        FENCE_HEIGHT / FENCE_GRID_COUNT * 2.5)
                                                                      
            simplified_each_shape_contour_lines_z = gh.SimplifyCurve(merged_shapes_contour_lines_z).curve
            
            exploded_each_shape_contour_lines_z = []
            for shape_contour_line_z in simplified_each_shape_contour_lines_z:
                exploded_contour_lines_z = rg.PolylineCurve.DuplicateSegments(shape_contour_line_z)
                exploded_each_shape_contour_lines_z.extend(exploded_contour_lines_z)
                
            each_result_contour_lines = []
            for exploded_contour_line in exploded_each_shape_contour_lines:
                start_pt = exploded_contour_line.PointAtStart
                end_pt = exploded_contour_line.PointAtEnd
                if not is_close(0, start_pt.Y) or not is_close(0, end_pt.Y):
                    each_result_contour_lines.append(exploded_contour_line)
                
            each_result_contour_lines_z = []
            for exploded_contour_line_z in exploded_each_shape_contour_lines_z:
                start_pt_z = exploded_contour_line_z.PointAtStart
                end_pt_z = exploded_contour_line_z.PointAtEnd
                if not is_close(0, start_pt_z.Y) or not is_close(0, end_pt_z.Y):
                    each_result_contour_lines_z.append(exploded_contour_line_z)
            
            self.trimmed_shapes_contour_lines.append(each_result_contour_lines + each_result_contour_lines_z)
            
    def get_splitted_fence_contour_lines(self):
        """generate splitted fence faces contour lines"""
        self.splitted_original_fence_lines = []
        for each_original_lines in self.fence_faces_contour_lines:
            each_splitted_original_fence_lines = []
            
            for original_line in each_original_lines:
                splitted_original_lines = original_line.Split(self.merged_trimmed_shapes, TOLERANCE)
                
                if len(splitted_original_lines) == 0:
                    each_splitted_original_fence_lines.append(original_line)
                    continue
                    
                else:
                    each_splitted_original_fence_lines.extend(splitted_original_lines)
                    
            self.splitted_original_fence_lines.append(each_splitted_original_fence_lines)
            
    def get_result_mesh_fence(self):
        """generate result"""
        
        self.result_mesh_fence = []
        
        for each_splitted_original_fence_lines, each_contour_lines in zip(self.splitted_original_fence_lines, 
                                                                          self.trimmed_shapes_contour_lines):
                                                                              
            each_splitted_original_fence_lines_centroids = gh.CurveMiddle(each_splitted_original_fence_lines)
            pattern = gh.PointInBreps(self.merged_trimmed_shapes, each_splitted_original_fence_lines_centroids, False).index
            negative_pattern = gh.Negative(pattern)
            fence_lines = gh.CullPattern(each_splitted_original_fence_lines, negative_pattern)
            
            result = gh.JoinCurves(fence_lines + list(each_contour_lines), False)
            self.result_mesh_fence.append(result)


if __name__ == "__main__":
    
    dynamic_mesh_fence = DynamicMeshFence(fence_count, directions, shape)
    fence_columns = dynamic_mesh_fence.fence_columns
    result_mesh_fence = dynamic_mesh_fence.result_mesh_fence
