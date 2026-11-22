"""Blender 4.4: blender -b -P Tools/RetargetWalking.py

Source/UnrealWalking의 루프를 기존 Idle 메시와 바인드 포즈에 맞춰 변환한다.
낙상 클립은 blender -b -P Tools/RetargetWalking.py -- shove 로 변환한다.
공중 낙하 루프는 blender -b -P Tools/RetargetWalking.py -- flail 로 변환한다.
"""
import hashlib
import json
import math
import struct
import sys
from copy import deepcopy
from pathlib import Path

import bpy
from mathutils import Matrix, Quaternion, Vector


ASSETS = Path(__file__).resolve().parents[1] / "Assets/Characters/Mixamo"
# 원본 접미사는 밀어온 쪽이다. 골반이 움직이기 직전의 원본 프레임부터 굽는다.
SHOVE_FALLS = {
    "M_relaxed_ragdoll_shove_stand_B_V": ("ShoveFallForward", 16),
    "M_relaxed_ragdoll_shove_stand_F_V": ("ShoveFallBackward", 18),
    "M_relaxed_ragdoll_shove_stand_L_V": ("ShoveFallRight", 20),
    "M_relaxed_ragdoll_shove_stand_R_V": ("ShoveFallLeft", 15),
}
AIRBORNE_FLAILS = {f"M_ragdoll_flail_{pose}" for pose in ("legsFirst", "armsFirst", "armsDown")}
AXES = Matrix.Rotation(-math.pi / 2, 4, "X")
MAPPING = dict(Hips="pelvis", Spine="spine_01", Spine1="spine_03", Spine2="spine_05",
               Neck="neck_01", Head="head")
AIM = dict(Spine="Spine1", Spine1="Spine2", Spine2="Neck", Neck="Head")
for side, suffix in (("Left", "_l"), ("Right", "_r")):
    for chain in [(("Shoulder", "clavicle"), ("Arm", "upperarm"), ("ForeArm", "lowerarm"), ("Hand", "hand")),
                  (("UpLeg", "thigh"), ("Leg", "calf"), ("Foot", "foot"), ("ToeBase", "ball"))]:
        for i, (name, source) in enumerate(chain):
            MAPPING[side + name] = source + suffix
            if i + 1 < len(chain) and name != "Foot":
                AIM[side + name] = side + chain[i + 1][0]
    for finger in ("Thumb", "Index", "Middle", "Ring", "Pinky"):
        for i in range(1, 4):
            name = side + "Hand" + finger
            MAPPING[name + str(i)] = f"{finger.lower()}_0{i}{suffix}"
            if i < 3:
                AIM[name + str(i)] = name + str(i + 1)


def rotation(matrix):
    return matrix.to_quaternion().normalized()


def read_transform(node):
    x, y, z, w = node.get("rotation", (0, 0, 0, 1))
    return Matrix.LocRotScale(Vector(node.get("translation", (0, 0, 0))), Quaternion((w, x, y, z)),
                              Vector(node.get("scale", (1, 1, 1))))


def palm_frame(bones, side):
    wrist = bones[side + "Hand"].translation
    index = bones[side + "HandIndex1"].translation - wrist
    pinky = bones[side + "HandPinky1"].translation - wrist
    along = (index + pinky).normalized()
    normal = index.cross(pinky).normalized()
    return Matrix((along, normal.cross(along), normal)).transposed().to_quaternion()


def pointed(orientation, rest_direction, direction):
    return ((orientation @ rest_direction).rotation_difference(direction) @ orientation).normalized()


class Retarget:
    def __init__(self, template, rig, falling=False):
        self.nodes = template["nodes"]
        self.local = {}
        self.bind = {}
        self.parent = {}
        self.indices = {}

        def visit(index, parent=None):
            node = self.nodes[index]
            name = node["name"].split(":")[-1]
            self.indices[name] = index
            self.parent[name] = parent
            self.local[name] = read_transform(node)
            self.bind[name] = self.bind[parent] @ self.local[name] if parent else self.local[name]
            for child in node.get("children", []):
                visit(child, name)

        for index in template["scenes"][template["scene"]]["nodes"]:
            visit(index)
        self.rig = rig
        self.to_model = AXES
        self.match_ankles = not falling
        if falling:
            # 촬영을 위해 배치한 첫 프레임의 위치와 회전만 제거하고 이후 낙상 궤적은 보존한다.
            placement = Matrix.LocRotScale(rig.matrix_world.translation, rotation(rig.matrix_world), None)
            self.to_model = AXES @ placement.inverted()
        base = self.to_model @ rig.matrix_world
        self.source_bind = {name: base @ rig.data.bones[source].matrix_local for name, source in MAPPING.items()}
        self.aim_local = {}
        self.offset = {}
        for name in MAPPING:
            target = rotation(self.bind[name])
            source = rotation(self.source_bind[name])
            aligned = target
            if name in AIM:
                child = AIM[name]
                direction = self.bind[child].translation - self.bind[name].translation
                self.aim_local[name] = target.inverted() @ direction
                aligned = pointed(target, self.aim_local[name], self.source_bind[child].translation - self.source_bind[name].translation)
            self.offset[name] = source.inverted() @ aligned

        def leg_length(pose):
            return sum((pose[b].translation - pose[a].translation).length for a, b in
                       (("LeftUpLeg", "LeftLeg"), ("LeftLeg", "LeftFoot")))

        self.scale = leg_length(self.bind) / leg_length(self.source_bind)

    def sample(self):
        base = self.to_model @ self.rig.matrix_world
        source = {name: base @ self.rig.pose.bones[bone].matrix for name, bone in MAPPING.items()}
        desired = {name: rotation(matrix) @ self.offset[name] for name, matrix in source.items()}
        for name, child in AIM.items():
            desired[name] = pointed(desired[name], self.aim_local[name], source[child].translation - source[name].translation)

        bind = self.bind
        hips = bind["Hips"].translation + self.scale * (source["Hips"].translation - self.source_bind["Hips"].translation)
        ankles = {}
        for side in ("Left", "Right"):
            hand = side + "Hand"
            desired[hand] = palm_frame(source, side) @ palm_frame(bind, side).inverted() @ rotation(bind[hand])
            if not self.match_ankles:
                continue
            thigh, calf, foot, toe = (side + n for n in ("UpLeg", "Leg", "Foot", "ToeBase"))
            ankle = hips + self.scale * (source[foot].translation - source["Hips"].translation)
            ankle.y = bind[foot].translation.y + self.scale * (source[foot].translation.y - self.source_bind[foot].translation.y)
            toe_y = bind[toe].translation.y + self.scale * (source[toe].translation.y - self.source_bind[toe].translation.y)
            toe_offset = desired[foot] @ (rotation(bind[foot]).inverted() @ (bind[toe].translation - bind[foot].translation))
            ankle.y = max(ankle.y, toe_y - toe_offset.y)
            ankles[side] = ankle

        # 두 발의 월드 궤적을 보존할 수 있는 높이까지만 골반을 올린다.
        for side, ankle in ankles.items():
            thigh, calf, foot = (side + n for n in ("UpLeg", "Leg", "Foot"))
            leg = self.aim_local[thigh].length + self.aim_local[calf].length
            hip_offset = desired["Hips"] @ (rotation(bind["Hips"]).inverted() @ (bind[thigh].translation - bind["Hips"].translation))
            delta = hips + hip_offset - ankle
            vertical_reach = math.sqrt(max(0, (leg - 0.0001) ** 2 - delta.x ** 2 - delta.z ** 2))
            hips.y = min(hips.y, ankle.y + vertical_reach - hip_offset.y)

        world = {}
        keys = {}
        knees = {}
        for name, local in self.local.items():
            parent = world.get(self.parent[name], Matrix.Identity(4))
            position = parent @ local.translation
            if name not in desired:
                world[name] = parent @ local
                continue
            if self.match_ankles and name.endswith("UpLeg"):
                side = name[:-5]
                calf = side + "Leg"
                axis = ankles[side] - position
                distance = axis.length
                axis.normalize()
                pole = source[calf].translation - source[name].translation
                pole = (pole - axis * pole.dot(axis)).normalized()
                upper = self.aim_local[name].length
                lower = self.aim_local[calf].length
                along = (upper ** 2 - lower ** 2 + distance ** 2) / (2 * distance)
                knees[side] = position + axis * along + pole * math.sqrt(max(0, upper ** 2 - along ** 2))
                desired[name] = pointed(desired[name], self.aim_local[name], knees[side] - position)
            elif self.match_ankles and name in ("LeftLeg", "RightLeg"):
                desired[name] = pointed(desired[name], self.aim_local[name], ankles[name[:-3]] - position)
            local_rot = (rotation(parent).inverted() @ desired[name]).normalized()
            translation = local.translation
            if name == "Hips":
                translation = parent.inverted() @ hips
                keys[(self.indices[name], "translation")] = tuple(translation)
            keys[(self.indices[name], "rotation")] = (local_rot.x, local_rot.y, local_rot.z, local_rot.w)
            world[name] = parent @ Matrix.LocRotScale(translation, local_rot, local.to_scale())
        return keys


def save_clip(template, destination, name, frames, fps):
    doc = deepcopy(template)
    times = [(i / fps,) for i in range(len(frames))]
    # 기존 메시 및 역바인드 행렬은 Idle의 바이너리를 공유한다.
    first_animation = min(s["input"] for s in doc["animations"][0]["samplers"])
    first_view = doc["accessors"][first_animation]["bufferView"]
    doc["accessors"] = doc["accessors"][:first_animation]
    doc["bufferViews"] = doc["bufferViews"][:first_view]
    doc["buffers"][0]["uri"] = "../Idle/Idle.bin"
    data = bytearray()

    def accessor(rows):
        width = len(rows[0])
        offset = len(data)
        pack = struct.Struct("<" + "f" * width).pack
        for row in rows:
            data.extend(pack(*row))
        view = len(doc["bufferViews"])
        doc["bufferViews"].append(dict(buffer=1, byteOffset=offset, byteLength=len(data) - offset))
        item = dict(bufferView=view, componentType=5126, count=len(rows), type={1:"SCALAR", 3:"VEC3", 4:"VEC4"}[width])
        if width == 1:
            item.update(min=list(rows[0]), max=list(rows[-1]))
        doc["accessors"].append(item)
        return len(doc["accessors"]) - 1

    time_index = accessor(times)
    animation = dict(name=name, channels=[], samplers=[])
    for node, path in frames[0]:
        rows = [f[(node, path)] for f in frames]
        if path == "rotation":
            for i in range(1, len(rows)):
                if sum(a * b for a, b in zip(rows[i-1], rows[i])) < 0:
                    rows[i] = tuple(-x for x in rows[i])
        animation["channels"].append(dict(sampler=len(animation["samplers"]), target=dict(node=node, path=path)))
        animation["samplers"].append(dict(input=time_index, output=accessor(rows), interpolation="LINEAR"))
    doc["animations"] = [animation]
    doc["buffers"].append(dict(uri=name + ".bin", byteLength=len(data)))
    (destination / (name + ".bin")).write_bytes(data)
    path = destination / (name + ".gltf")
    path.write_text(json.dumps(doc, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return path


def main():
    template = json.loads((ASSETS / "Converted/Idle/Idle.gltf").read_bytes())
    manifest_path = ASSETS / "manifest.json"
    manifest = json.loads(manifest_path.read_bytes())
    idle = next(clip for clip in manifest["clips"] if clip["animation"] == "Idle")
    args = sys.argv[sys.argv.index("--") + 1:] if "--" in sys.argv else []
    if args not in ([], ["shove"], ["flail"]):
        raise ValueError("보행은 인자 없이, 지상 낙상은 -- shove, 공중 낙하는 -- flail 로 변환하세요.")
    kind = args[0] if args else "walk"
    source_folder, output_folder = {
        "walk": ("UnrealWalking", "WalkingLoops"),
        "shove": ("UnrealShove", "ShoveFalls"),
        "flail": ("UnrealFlail", "AirborneFlails"),
    }[kind]
    destination = ASSETS / "Converted" / output_folder
    source_dir = ASSETS / "Source" / source_folder
    sources = sorted(source_dir.glob("*.fbx"))
    names = {file.name.split(".")[0] for file in sources}
    if kind == "shove" and names != SHOVE_FALLS.keys():
        raise ValueError("UnrealShove의 앞뒤좌우 원본 4개가 필요합니다.")
    if kind == "flail" and names != AIRBORNE_FLAILS:
        raise ValueError("UnrealFlail의 legsFirst, armsFirst, armsDown 원본 3개가 필요합니다.")
    if kind == "walk" and len(sources) != 10:
        raise ValueError("UnrealWalking 원본 10개가 필요합니다.")
    destination.mkdir(parents=True, exist_ok=True)
    entries = []
    for file in sources:
        name = file.name.split(".")[0]
        bpy.ops.wm.read_factory_settings(use_empty=True)
        bpy.ops.import_scene.fbx(filepath=str(file), automatic_bone_orientation=False)
        scene = bpy.context.scene
        rig = next(obj for obj in scene.objects if obj.type == "ARMATURE")
        start, end = map(int, rig.animation_data.action.frame_range)
        fps = scene.render.fps / scene.render.fps_base
        scene.frame_set(start)
        retarget = Retarget(template, rig, falling=kind != "walk")
        first_frame = start
        if kind == "shove":
            name, first_frame = SHOVE_FALLS[name]
            if not start <= first_frame < end:
                raise ValueError(f"{file.name}의 낙상 시작 프레임이 범위를 벗어났습니다.")
        samples = []
        for frame in range(first_frame, end + 1):
            scene.frame_set(frame)
            samples.append(retarget.sample())
        path = save_clip(template, destination, name, samples, fps)
        entries.append(dict(animation=name, path=path.relative_to(ASSETS).as_posix(), source=file.relative_to(ASSETS).as_posix(),
                            source_sha256=hashlib.sha256(file.read_bytes()).hexdigest(), gltf_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                            blender_version=bpy.app.version_string, source_frames=[start, end], frames_per_second=fps,
                            sample_counts=[len(samples)], durations_seconds=[round((end-first_frame)/fps, 6)], channels=len(samples[0]),
                            meshes=1, skins=1, joints=idle["joints"], source_rig="Unreal", source_bone_count=len(rig.data.bones),
                            target_skeleton=idle["path"], textures=idle["textures"]))
        if kind != "walk":
            entries[-1]["exported_source_frames"] = [first_frame, end]
    prefix = destination.relative_to(ASSETS).as_posix() + "/"
    manifest["clips"] = [clip for clip in manifest["clips"] if not clip["path"].startswith(prefix)] + entries
    manifest_path.write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


if __name__ == "__main__":
    main()
