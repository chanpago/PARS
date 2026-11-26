"""팔라딘과 에리카의 게임용 메시를 생성한다. 실행: python Tools/BindGameplayCharacters.py (NumPy 필요)"""

import json
import os
from copy import deepcopy
from pathlib import Path

import numpy as np


CHARACTERS = Path(__file__).resolve().parents[1] / "Assets/Characters"
OUTPUT = CHARACTERS / "Gameplay"
IDLE = CHARACTERS / "Mixamo/Converted/Idle/Idle.gltf"
TYPES = {5121: "u1", 5123: "<u2", 5125: "<u4", 5126: "<f4"}
WIDTHS = {"SCALAR": 1, "VEC2": 2, "VEC3": 3, "VEC4": 4, "MAT4": 16}


def read_accessor(doc, buffers, index):
    item = doc["accessors"][index]
    view = doc["bufferViews"][item["bufferView"]]
    dtype = np.dtype(TYPES[item["componentType"]])
    width = WIDTHS[item["type"]]
    return np.ndarray((item["count"], width), dtype, buffers[view["buffer"]],
                      offset=view.get("byteOffset", 0) + item.get("byteOffset", 0),
                      strides=(view.get("byteStride", width * dtype.itemsize), dtype.itemsize))


def joint_binding(source, reference, source_ibm, reference_ibm):
    skin = source["skins"][0]
    parents = {child: i for i, node in enumerate(source["nodes"]) for child in node.get("children", [])}
    slots = {node: i for i, node in enumerate(skin["joints"])}
    target = {reference["nodes"][node]["name"].split(":")[-1]: i
              for i, node in enumerate(reference["skins"][0]["joints"])}
    binding = []
    transforms = []
    for node in skin["joints"]:
        name = source["nodes"][node]["name"].split(":")[-1]
        # 공용 본에 없는 눈은 머리의 원본 역바인드를 써야 얼굴 안에서의 위치가 보존된다.
        while name not in target:
            node = parents[node]
            name = source["nodes"][node]["name"].split(":")[-1]
        binding.append(target[name])
        transforms.append(np.linalg.solve(reference_ibm[target[name]], source_ibm[slots[node]]))
    return np.array(binding), np.stack(transforms)


def normalized(vectors):
    return vectors / np.linalg.norm(vectors, axis=1, keepdims=True).clip(min=1e-12)


def bind_vertices(attributes, binding, transforms):
    joints = attributes["JOINTS_0"]
    weights = attributes["WEIGHTS_0"].astype(np.float64)
    weights /= weights.sum(axis=1, keepdims=True)
    # 정점별 가중치를 유지하면서 각 본의 원본 바인드 공간을 게임 스켈레톤의 바인드 공간으로 옮긴다.
    matrix = np.einsum("vi,vijk->vjk", weights, transforms[joints])
    basis = matrix[:, :3, :3]
    attributes["POSITION"] = np.einsum("vij,vj->vi", basis, attributes["POSITION"]) + matrix[:, :3, 3]
    normals = normalized(np.linalg.solve(basis.transpose(0, 2, 1), attributes["NORMAL"][:, :, None])[:, :, 0])
    attributes["NORMAL"] = normals
    tangent = attributes["TANGENT"]
    along = np.einsum("vij,vj->vi", basis, tangent[:, :3])
    along -= normals * (along * normals).sum(axis=1, keepdims=True)
    attributes["TANGENT"] = np.column_stack((normalized(along), tangent[:, 3] * np.sign(np.linalg.det(basis))))
    joints = binding[joints]
    for slot in range(1, joints.shape[1]):
        for previous in range(slot):
            same = (joints[:, slot] == joints[:, previous]) & (weights[:, previous] > 0)
            weights[same, previous] += weights[same, slot]
            weights[same, slot] = 0
    order = np.argsort(-weights, axis=1, kind="stable")
    attributes["JOINTS_0"] = np.take_along_axis(np.where(weights > 0, joints, 0), order, axis=1)
    attributes["WEIGHTS_0"] = np.take_along_axis(weights, order, axis=1)


def bake(path, reference, reference_buffers):
    source = json.loads(path.read_bytes())
    buffers = [(path.parent / item["uri"]).read_bytes() for item in source["buffers"]]
    if len(source["skins"]) != 1 or any(n.get("skin") != 0 or set(n) - {"name", "mesh", "skin"}
                                         for n in source["nodes"] if "mesh" in n):
        raise ValueError(f"{path.name}: 예상한 단일 스킨과 메시 좌표계를 확인하세요.")

    def ibms(doc, data):
        values = read_accessor(doc, data, doc["skins"][0]["inverseBindMatrices"])
        return values.astype(np.float64).reshape(-1, 4, 4).transpose(0, 2, 1)

    binding, transforms = joint_binding(source, reference, ibms(source, buffers), ibms(reference, reference_buffers))
    doc = {key: deepcopy(reference[key]) for key in ("scene", "scenes", "nodes", "skins", "animations")}
    doc.update({key: deepcopy(source[key]) for key in ("asset", "materials", "textures", "images", "samplers")})
    for key in ("extensionsUsed", "extensionsRequired"):
        if key in source:
            doc[key] = deepcopy(source[key])
    doc["asset"]["generator"] = "PARS BindGameplayCharacters"
    for image in doc["images"]:
        image["uri"] = Path(os.path.relpath(path.parent / image["uri"], OUTPUT)).as_posix()
    doc["accessors"] = []
    doc["bufferViews"] = []
    model = {"name": path.stem, "primitives": []}
    doc["meshes"] = [model]
    packed = bytearray()

    def add_geometry(index, values, target, component_type=None):
        item = source["accessors"][index].copy()
        if component_type is not None:
            item["componentType"] = component_type
        item["count"] = len(values)
        values = np.ascontiguousarray(values, dtype=TYPES[item["componentType"]])
        if not np.isfinite(values).all():
            raise ValueError(f"{path.name}: 유효하지 않은 정점 값")
        for key, reduce in (("min", np.min), ("max", np.max)):
            if key in item:
                item[key] = reduce(values, axis=0).tolist()
        packed.extend(bytes(-len(packed) % 4))
        item.pop("byteOffset", None)
        item["bufferView"] = len(doc["bufferViews"])
        doc["bufferViews"].append(dict(buffer=0, byteOffset=len(packed), byteLength=values.nbytes, target=target))
        packed.extend(values.tobytes())
        doc["accessors"].append(item)
        return len(doc["accessors"]) - 1

    groups = {}
    for mesh in source["meshes"]:
        for primitive in mesh["primitives"]:
            src = primitive["attributes"]
            attributes = {name: read_accessor(source, buffers, i) for name, i in src.items()}
            bind_vertices(attributes, binding, transforms)
            layout = tuple(sorted((name, source["accessors"][i]["componentType"]) for name, i in src.items()))
            key = primitive["material"], primitive.get("mode", 4), layout
            groups.setdefault(key, []).append((primitive, attributes))
    # 같은 재질의 투구와 갑옷은 하나의 인덱스 버퍼로 합쳐 한 번에 그린다.
    for group in groups.values():
        first = group[0][0]
        part = first.copy()
        part["attributes"] = {
            key: add_geometry(i, np.concatenate([attrs[key] for _, attrs in group]), 34962)
            for key, i in first["attributes"].items()
        }
        indices = []
        vertex_count = 0
        for primitive, attributes in group:
            local = read_accessor(source, buffers, primitive["indices"]).astype(np.uint32)
            indices.append(local + vertex_count)
            vertex_count += len(attributes["POSITION"])
        index_type = 5123 if vertex_count <= 65535 else 5125
        part["indices"] = add_geometry(first["indices"], np.concatenate(indices), 34963, index_type)
        model["primitives"].append(part)

    # 역바인드 행렬과 대기 클립은 기존 Idle 바이너리의 같은 바이트를 공유한다.
    ref_accessors = {}
    ref_views = {}

    def shared_accessor(index):
        if index not in ref_accessors:
            item = reference["accessors"][index].copy()
            view = item["bufferView"]
            if view not in ref_views:
                ref_views[view] = len(doc["bufferViews"])
                desc = reference["bufferViews"][view].copy()
                desc["buffer"] += 1
                doc["bufferViews"].append(desc)
            item["bufferView"] = ref_views[view]
            ref_accessors[index] = len(doc["accessors"])
            doc["accessors"].append(item)
        return ref_accessors[index]

    skin = doc["skins"][0]
    skin["inverseBindMatrices"] = shared_accessor(skin["inverseBindMatrices"])
    for clip in doc["animations"]:
        for sampler in clip["samplers"]:
            for key in ("input", "output"):
                sampler[key] = shared_accessor(sampler[key])
    binary = OUTPUT / (path.stem.replace(" ", "") + ".bin")
    doc["buffers"] = [dict(uri=binary.name, byteLength=len(packed))]
    for item in reference["buffers"]:
        uri = Path(os.path.relpath(IDLE.parent / item["uri"], OUTPUT)).as_posix()
        doc["buffers"].append(dict(uri=uri, byteLength=item["byteLength"]))
    output = OUTPUT / path.name
    json_bytes = (json.dumps(doc, ensure_ascii=False, indent=1) + "\n").encode("utf-8")
    geometry_changed = not binary.exists() or binary.read_bytes() != packed
    if geometry_changed:
        binary.write_bytes(packed)
    if geometry_changed or not output.exists() or output.read_bytes() != json_bytes:
        output.write_bytes(json_bytes)


if __name__ == "__main__":
    OUTPUT.mkdir(exist_ok=True)
    idle = json.loads(IDLE.read_bytes())
    idle_buffers = [(IDLE.parent / item["uri"]).read_bytes() for item in idle["buffers"]]
    for character in ("Paladin J Nordstrom", "Erika Archer"):
        bake(CHARACTERS / (character + ".gltf"), idle, idle_buffers)
