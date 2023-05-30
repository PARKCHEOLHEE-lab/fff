import math
import Rhino.Geometry as rg
import ghpythonlib.components as gh

class KlemensTorgglerDoor:
    def __init__(self, door_size=20, handler_param=0.2):
        self.door_size = door_size
        self.handler_param = handler_param
        self._generate()
        
    def _generate(self):
        self._gen_vertices_and_anchor()
        upper_door = self._get_torggler_door(
            self.upper_anchor_plane, self.upper_vertices
        )
        
        lower_door = self._get_torggler_door(
            self.lower_anchor_plane, self.lower_vertices
        )
        
        self.klemenstorgglerdoor = upper_door + lower_door
    
    def _gen_vertices_and_anchor(self):
        self.upper_vertices = [
            rg.Point3d(0, 0, self.door_size),
            rg.Point3d(0, self.door_size / 2, self.door_size),
            rg.Point3d(0, self.door_size / 2, self.door_size / 2),
        ]
        
        self.upper_anchor_plane = self._get_anchor_plane(self.upper_vertices[1])
        
        self.lower_vertices = [
            rg.Point3d(0, 0, 0),
            rg.Point3d(0, self.door_size / 2, 0),
            rg.Point3d(0, self.door_size / 2, self.door_size / 2),
        ]
        
        self.lower_anchor_plane = self._get_anchor_plane(
            self.lower_vertices[1], is_flip=True
        )
        
    def _get_torggler_door(self, anchor_plane, vertices):
        handler_position_y = self.handler_param * self.door_size / 2 * 2
        
        moved_vertices, _ = gh.Rotate(vertices, math.pi * m / 2, anchor_plane)
        p3 = moved_vertices[0] / 2 + moved_vertices[2] / 2
        p4 = rg.Point3d(0, handler_position_y, self.door_size / 2)
        b = p3.DistanceTo(p4)
        
        handler_position_x = (
            0 
            if handler_position_y in (0, self.door_size / 2 * 2) 
            else math.sqrt(self.door_size ** 2 / 8 - b ** 2)
        )
        
        handle = rg.Point3d(
            handler_position_x, handler_position_y, self.door_size / 2
        )
    
        door_each_part = [
            rg.PolylineCurve(moved_vertices + moved_vertices[:1]),
            rg.PolylineCurve(
                [
                    moved_vertices[0], 
                    moved_vertices[2], 
                    handle, 
                    moved_vertices[0]
                ]
            )
        ]
        
        return door_each_part
        
    def _get_anchor_plane(self, vertex, is_flip=False):
        return gh.FlipPlane(
            plane=gh.YZPlane(vertex),
            reverse_x=False,
            reverse_y=False,
            swap_axes=is_flip
            
        )

if __name__ == "__main__":
    ktd = KlemensTorgglerDoor(door_size=50, handler_param=m)
    klemens_torggler_door = ktd.klemenstorgglerdoor