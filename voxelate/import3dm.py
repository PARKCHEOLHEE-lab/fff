import ghpythonlib.components as gh
import os

GH_PATH = ghdoc.Path
DIR = os.path.dirname(os.path.realpath(GH_PATH))
MODEL_PATH = os.path.join(DIR, "3dm")
UNIT_PATH = os.path.join(MODEL_PATH, "unit")

if __name__ == "__main__":
    
    model_3dm = [m for m in os.listdir(MODEL_PATH) if m[-3:] == "3dm"]
    unit_3dm = [u for u in os.listdir(UNIT_PATH) if u[-3:] == "3dm"]
    
    _79andpark_3dm = model_3dm[0]
    _legotowers_3dm = model_3dm[1]
    _red7_3dm = model_3dm[2]
    _vancouverhouse_3dm = model_3dm[3]
    
    _exterior_unit_3dm = unit_3dm[0]
    
    model = gh.Import3DM(os.path.join(MODEL_PATH, _red7_3dm), "*", "*")
    exterior_unit = gh.Import3DM(os.path.join(UNIT_PATH, _exterior_unit_3dm), "*", "*")
