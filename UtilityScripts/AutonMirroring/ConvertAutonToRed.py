import json
from os import listdir

'''
private Pose2d flipAlliance(Pose2d poseToMirror) {
        Pose2d mirroredPose2d = new Pose2d(poseToMirror.getX(),
            (FieldConstants.kFieldWidthMeters - poseToMirror.getY()),
            Rotation2d.fromDegrees(poseToMirror.getRotation().getDegrees() * -1));
        return mirroredPose2d;
      }
'''

file_type_subfolders = {
    "path": "paths",
    "auto": "auto",
    "choreo": "choreo"
}

class Rotation2D:
    def __init__(self, degrees: float):
        self.degrees = degrees

class Pose2D:
    def __init__(self, x: float, y: float, rotation: Rotation2D):
        self.x = x
        self.y = y
        self.rot = rotation

class CoordinateFlipper:
    FIELD_WIDTH_METERS = 8.21055 # TODO: Placeholder
    NEW_FOLDER = "Red" # TODO: I assume this is how it works but still a placeholder
    WRITE_OUT_SUFFIX = "_red" # What will be added to the end of the file name

    def __init__(self, path: str|None = None):
        if path != None:
            self.load_single_file(path)

    def load_single_file(self, path: str):
        path = path.replace("\\", "/")
        with open(path, "r") as file:
            self.json = json.loads(file.read())
        self.file_format = path.split(".")[-1] # Should be either .auto, .traj, or .path
        self.cwd = "/".join(path.split("/")[:-1])
        self.name = ".".join(path.split("/")[-1].split(".")[:-1])

    def iter_over_points(self):
        if self.file_format == "auto":
            self._iter_over_points_auto()
        elif self.file_format == "traj":
            self._iter_over_points_traj()
        else: # Make path the default parser even if the file extension is wrong
            self._iter_over_points_path()

    def _iter_over_points_auto(self):
        '''Auto files contain references to .path files, so we need to run our function on all of those paths individually
        '''
        target = self.json["command"]["data"]["commands"]
        for command in target:
            if command["type"] == "path":
                flipper = CoordinateFlipper(self.cwd.removesuffix("autos") + "paths/" + command["data"]["pathName"] + ".path")
                flipper.iter_over_points()

                command["data"]["pathName"] = ".".join(flipper.write().split(".")[:-1])

    def _iter_over_points_traj(self):
        # Samples
        samples = self.json["samples"]
        for sample in samples:
            sample = self.mirror_pose_json(sample, "heading")

    def _iter_over_points_path(self):
        self.json["reversed"] = True
        self.json["folder"] = "Red"
        target = self.json["waypoints"]
        for waypoint in target:
            for object in [waypoint["anchor"], waypoint["prevControl"], waypoint["nextControl"]]:
                if object != None:
                    object = self.mirror_pose_json(object, None)
        for object in [self.json["goalEndState"], self.json["previewStartingState"]]:
            object = self.mirror_pose_json(object, "rotation", None, None)

    def mirror_pose_json(self, json: dict, rot_key: str|None = "rotation", x_key: str|None = "x", y_key: str|None = "y"):
        new_pose2d = self.mirror_pose(Pose2D(
            json[x_key] if x_key else 0,
            json[y_key] if y_key else 0,
            Rotation2D(json[rot_key]) if rot_key else Rotation2D(0)
        ))
        if x_key: json[x_key] = new_pose2d.x
        if y_key: json[y_key] = new_pose2d.y
        if rot_key: json[rot_key] = new_pose2d.rot.degrees
        return json

    def mirror_pose(self, pose: Pose2D) -> Pose2D:
        return Pose2D(
            pose.x,
            self.FIELD_WIDTH_METERS - pose.y,
            Rotation2D(pose.rot.degrees * -1)
    )

    def write(self) -> str:
        new_file_name = self.name + self.WRITE_OUT_SUFFIX + "." + self.file_format
        with open(self.cwd + "/" + new_file_name, "w+") as file:
            file.write(json.dumps(self.json, indent=2))
        return new_file_name

# We probably don't want to include .path files, since they are referenced in the .auto files and will be processed along with the .auto files
def find_files(dir: str, include_paths: bool = False) -> list[str]:
    files = listdir(dir)
    pertinent_files = []
    allowed_types = ["traj", "auto"]
    if include_paths: allowed_types.append("path")
    for file in files:
        if file.split(".")[-1] in allowed_types:
            pertinent_files.append(file)
    return pertinent_files

def process_path(path: str):

    files = find_files(path)
    for file in files:
        if not ".".join(file.split(".")[:-1]).endswith(CoordinateFlipper.WRITE_OUT_SUFFIX): # Excluding file ending, make sure this file doesn't have the suffix
            flipper = CoordinateFlipper(path + "/" + file)
            flipper.iter_over_points()
            flipper.write()

def process_deploy_path(deploy_path: str):
    process_path(deploy_path + "/choreo")
    process_path(deploy_path + "/pathplanner/autos")

# To dearest Elijah,
# The variable __name__ will be equal to "__main__" if this Python file
# is being ran by itself. It WON'T run if this Python file is being
# imported by another file. This is considered best practice.
if __name__ == "__main__":
    deploy_path = "C:/RoboticsProjects/Team7826Lib/src/main/deploy"
    process_deploy_path(deploy_path)