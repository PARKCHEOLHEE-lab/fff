import ghpythonlib.components as gh
import os

GH_PATH = ghdoc.Path
DIR = os.path.dirname(os.path.realpath(GH_PATH))
MODEL_PATH = os.path.join(DIR, "3dm")
UNIT_PATH = os.path.join(MODEL_PATH, "unit")

if __name__ == "__main__":
    
    model_3dm = [m for m in os.listdir(MODEL_PATH) if m[-3:] == "3dm"]
    unit_3dm = [u for u in os.listdir(UNIT_PATH) if u[-3:] == "3dm"]
    
    (
        _79andpark_3dm, 
        _legotowers_3dm, 
        _red7_3dm, 
        _vancouverhouse_3dm,
    ) = model_3dm
    
    (
        _exterior_unit_3dm, 
        _exterior_both_unit_3dm, 
        _exterior_corner_unit_3dm, 
        _exterior_corner_o_unit_3dm,
        _exterior_corner_u_unit_3dm,
        _
    ) = unit_3dm
    
    model = gh.Import3DM(os.path.join(MODEL_PATH, _vancouverhouse_3dm), "*", "*")
    
    exterior_unit = gh.Import3DM(os.path.join(UNIT_PATH, _exterior_unit_3dm), "*", "*")
    exterior_both_unit = gh.Import3DM(os.path.join(UNIT_PATH, _exterior_both_unit_3dm), "*", "*")
    exterior_corner_unit = gh.Import3DM(os.path.join(UNIT_PATH, _exterior_corner_unit_3dm), "*", "*")
    exterior_corner_o_unit = gh.Import3DM(os.path.join(UNIT_PATH, _exterior_corner_o_unit_3dm), "*", "*")
    exterior_corner_u_unit = gh.Import3DM(os.path.join(UNIT_PATH, _exterior_corner_u_unit_3dm), "*", "*")