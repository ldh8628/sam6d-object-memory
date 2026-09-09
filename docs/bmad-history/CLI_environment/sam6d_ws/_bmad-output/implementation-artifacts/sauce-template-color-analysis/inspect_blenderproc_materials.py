import blenderproc as bproc

import argparse
import bpy


parser = argparse.ArgumentParser()
parser.add_argument("--cad_path", required=True)
args = parser.parse_args()

bproc.init()
objs = bproc.loader.load_obj(args.cad_path)
print(f"CAD_PATH={args.cad_path}")
print(f"OBJECT_COUNT={len(objs)}")

for obj_idx, obj in enumerate(objs):
    blender_obj = obj.blender_obj
    mesh = blender_obj.data
    print(f"OBJECT[{obj_idx}].name={blender_obj.name}")
    print(f"OBJECT[{obj_idx}].type={blender_obj.type}")
    print(f"OBJECT[{obj_idx}].vertices={len(mesh.vertices)}")
    print(f"OBJECT[{obj_idx}].polygons={len(mesh.polygons)}")
    print(f"OBJECT[{obj_idx}].materials={len(blender_obj.material_slots)}")

    color_attrs = getattr(mesh, "color_attributes", None)
    if color_attrs is not None:
        print(f"OBJECT[{obj_idx}].color_attributes={[a.name + ':' + a.domain + ':' + a.data_type for a in color_attrs]}")
    else:
        print(f"OBJECT[{obj_idx}].color_attributes=<not available>")

    vertex_colors = getattr(mesh, "vertex_colors", None)
    if vertex_colors is not None:
        print(f"OBJECT[{obj_idx}].vertex_colors={[a.name for a in vertex_colors]}")
    else:
        print(f"OBJECT[{obj_idx}].vertex_colors=<not available>")

    uv_layers = getattr(mesh, "uv_layers", None)
    if uv_layers is not None:
        print(f"OBJECT[{obj_idx}].uv_layers={[u.name for u in uv_layers]}")
    else:
        print(f"OBJECT[{obj_idx}].uv_layers=<not available>")

    for slot_idx, slot in enumerate(blender_obj.material_slots):
        mat = slot.material
        if mat is None:
            print(f"OBJECT[{obj_idx}].material[{slot_idx}]=None")
            continue
        print(f"OBJECT[{obj_idx}].material[{slot_idx}].name={mat.name}")
        print(f"OBJECT[{obj_idx}].material[{slot_idx}].use_nodes={mat.use_nodes}")
        print(f"OBJECT[{obj_idx}].material[{slot_idx}].diffuse_color={tuple(round(v, 6) for v in mat.diffuse_color)}")
        if mat.use_nodes and mat.node_tree is not None:
            nodes = [node.bl_idname for node in mat.node_tree.nodes]
            node_details = []
            for node in mat.node_tree.nodes:
                detail = node.bl_idname
                if hasattr(node, "attribute_name"):
                    detail += f"(attribute_name={node.attribute_name!r})"
                if hasattr(node, "layer_name"):
                    detail += f"(layer_name={node.layer_name!r})"
                node_details.append(detail)
            links = [
                f"{link.from_node.bl_idname}.{link.from_socket.name}->{link.to_node.bl_idname}.{link.to_socket.name}"
                for link in mat.node_tree.links
            ]
            print(f"OBJECT[{obj_idx}].material[{slot_idx}].nodes={nodes}")
            print(f"OBJECT[{obj_idx}].material[{slot_idx}].node_details={node_details}")
            print(f"OBJECT[{obj_idx}].material[{slot_idx}].links={links}")
