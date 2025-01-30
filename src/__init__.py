bl_info = {
    "name": "Blender Fireworks",
    "author": "Ashai-project",
    "version": (1, 0),
    "blender": (4, 2, 0),
    "location": "3D View > Sidebar > Fireworks",
    "description": "Imports PLY point clouds and creates fireworks particle effects",
    "warning": "",
    "doc_url": "",
    "category": "Object",
}

import bpy
import random
import math
import numpy as np
from mathutils import Vector

class FIREWORKS_OT_ImportPointCloud(bpy.types.Operator):
    """PLY 点群データをインポート"""
    bl_idname = "fireworks.import_ply"
    bl_label = "Import PLY Point Cloud"
    bl_options = {'REGISTER', 'UNDO'}

    filepath: bpy.props.StringProperty(subtype="FILE_PATH")

    def execute(self, context):
        point_cloud_obj, point_cloud = import_ply(self.filepath)
        if point_cloud_obj:
            self.report({'INFO'}, f"PLYファイルをインポートしました: {self.filepath}")
        else:
            self.report({'ERROR'}, "インポート失敗")

        # 点群データをダウンサンプリング
        downsampled_point_cloud = downsample_point_cloud_fps(point_cloud, num_samples=1000)

        # 球状の発射台を作成
        launch_obj, launch_points = create_launch_platform()
        
        # 発射台をセグメンテーション
        inside_collection, outside_collection, inside_layers, outside_layers = segment_launch_platform(launch_points, downsampled_point_cloud, point_cloud_obj)
        return {'FINISHED'}

    def invoke(self, context, event):
        context.window_manager.fileselect_add(self)  # ファイル選択ダイアログを開く
        return {'RUNNING_MODAL'}
    
class FIREWORKS_PT_Panel(bpy.types.Panel):
    """FireworksのUIパネル"""
    bl_label = "Fireworks Control"
    bl_idname = "FIREWORKS_PT_Panel"
    bl_space_type = "VIEW_3D"
    bl_region_type = "UI"
    bl_category = "Fireworks"

    def draw(self, context):
        layout = self.layout
        layout.operator("fireworks.import_ply", text="PLYインポート")

def import_ply(file_path):
    try:
        bpy.ops.wm.ply_import(filepath=file_path)
        
        # インポートされたオブジェクトを取得
        imported_obj = bpy.context.selected_objects[-1]
        
        # AABB の中心を計算して原点に移動
        min_coords = np.min([vert.co for vert in imported_obj.data.vertices], axis=0)
        max_coords = np.max([vert.co for vert in imported_obj.data.vertices], axis=0)
        center = (min_coords + max_coords) / 2
        imported_obj.location = (imported_obj.location.x - center[0], imported_obj.location.y - center[1], imported_obj.location.z - center[2])
        
        # メッシュデータのスケールを正規化
        bpy.ops.object.transform_apply(location=True, rotation=True, scale=True)
        max_dim = max(imported_obj.dimensions)
        scale_factor = 5.0 / max_dim  # 5.0 の範囲に収める
        imported_obj.scale = (scale_factor, scale_factor, scale_factor)
        
        # Y 軸が上なので Z 軸上に回転
        imported_obj.rotation_euler = (math.radians(90), 0, 0)
        bpy.ops.object.transform_apply(location=True, rotation=True, scale=True)
        
        # メッシュデータから頂点座標を取得
        mesh = imported_obj.data
        points = np.array([vert.co for vert in mesh.vertices])
        return imported_obj, points
    except Exception as e:
        print(f"Error: Failed to import PLY file: {e}")
        return None, None

def create_launch_platform(name="LaunchPlatform", num_layers=10, points_per_layer=100, radius=5.0):
    # コレクションを作成
    collection = bpy.data.collections.new(name)
    bpy.context.scene.collection.children.link(collection)
    # 球表面を層にして配置
    verticesArray = []
    for layer in range(1, num_layers + 1):
        mesh = bpy.data.meshes.new(f"{name}_Layer{layer}")
        vertices = []
        layer_radius = (layer / num_layers) * radius  # 内側から外側へ層状に配置
        for i in range(points_per_layer):
            phi = (1 + np.sqrt(5)) / 2
            theta = 2 * math.pi * (i / phi)  # 層ごとに均等配置
            z = 1 - (2 * i) / (points_per_layer - 1)
            r = np.sqrt(1 - z**2) 
            x = r * math.cos(theta) * layer_radius
            y = r * math.sin(theta) * layer_radius
            z = z * layer_radius
            vertices.append((x, y, z))
        mesh.from_pydata(vertices, [], [])
        obj = bpy.data.objects.new(f"{name}_Layer{layer}", mesh)
        collection.objects.link(obj)
        verticesArray.append(vertices)
        points_per_layer+=15

    return collection, np.array(verticesArray, dtype=object)

def segment_launch_platform(launch_layers, downsampled_point_cloud, mesh_obj):
    # 花火用のオブジェクト（小さな球体）を作成
    normal_firework = create_firework_object("Firework_Normal", (0, 1, 1))
    segmented_firework = create_firework_object("Firework_Segmented", (1, 0, 0),0.2)
    # 内外のコレクションを作成
    inside_collection = bpy.data.collections.new("InsideLaunchPlatforms")
    outside_collection = bpy.data.collections.new("OutsideLaunchPlatforms")
    bpy.context.scene.collection.children.link(inside_collection)
    bpy.context.scene.collection.children.link(outside_collection)
    
    # Ray-Casting による内外判定
    mesh_eval = mesh_obj.evaluated_get(bpy.context.evaluated_depsgraph_get())
    mesh_eval.to_mesh()
    
    inside_layers = []
    outside_layers = []
    layer = 0.0
    for launch_points in launch_layers:
        layer += 1.0
        layer_radius = (layer / 10.0) * 5.0
        inside_points = []
        outside_points = []
        for lp in launch_points:
            origin = Vector((0.0,0.0,0.0))
            direction = Vector(lp) # X方向にレイを飛ばす
            hitcount = 0
            hit_flg = 0 
            Distance = (origin - Vector(lp)).length
            rayDist = 0
            for i in range(10):
                hit, location, normal, index = mesh_eval.ray_cast(origin, direction)
                if hit:
                    hit_flg = 1
                    hitcount += 1
                    rayDist += (origin - location).length
                    if rayDist >= Distance:
                        hitcount -= 1
                        break
                    origin = location * 1.0001
            if hit_flg == 0:
                outside_points.append(lp)
            elif hitcount % 2 == 0:
                inside_points.append(lp)
            else:
                outside_points.append(lp)
        # ダウンサンプリングした結果を追加
        for  dp in downsampled_point_cloud:
            dist = np.linalg.norm(dp)
            if (dist < layer_radius + 0.25) and (dist > layer_radius - 0.25):
                inside_points.append(dp)
        
        # 各レイヤーごとにオブジェクトを作成し、対応するコレクションに追加
        if inside_points:
            inside_obj = create_segmented_launch_platform(f"Inside_Layer{layer}", np.array(inside_points))
            inside_collection.objects.link(inside_obj)
            inside_layers.append(inside_obj)
            add_firework_particle_system(inside_obj, "Fireworks_Inside", segmented_firework, velocity_scale=layer)
        if outside_points:
            outside_obj = create_segmented_launch_platform(f"Outside_Layer{layer}", np.array(outside_points))
            outside_collection.objects.link(outside_obj)
            outside_layers.append(outside_obj)
            add_firework_particle_system(outside_obj, "Fireworks_Outside", normal_firework, velocity_scale=layer)
    
    return inside_collection, outside_collection, inside_layers, outside_layers

def create_segmented_launch_platform(name, points):
    # セグメント化された発射台のオブジェクトを作成
    mesh = bpy.data.meshes.new(name)
    mesh.from_pydata(points.tolist(), [], [])
    obj = bpy.data.objects.new(name, mesh)
    return obj

def create_firework_object(name, color, size=0.1):
    # 花火用のオブジェクト（小さな球体）を作成
    bpy.ops.mesh.primitive_uv_sphere_add(radius=size)
    sphere = bpy.context.object
    sphere.name = name
    
    # マテリアルを作成
    mat = bpy.data.materials.new(name=name + "_Material")
    mat.use_nodes = True
    # 既存のノードを削除
    for node in mat.node_tree.nodes:
        mat.node_tree.nodes.remove(node)
    emission = mat.node_tree.nodes.new(type='ShaderNodeEmission')
    output = mat.node_tree.nodes.get("Material Output")
    if output is None:
        output = mat.node_tree.nodes.new(type="ShaderNodeOutputMaterial")
    mat.node_tree.links.new(emission.outputs[0], output.inputs[0])
    emission.inputs[0].default_value = (*color, 1)  # RGB + Alpha
    emission.inputs[1].default_value = 5.0
    if output and emission:
        emission.inputs[0].default_value = (*color, 1)  # RGB + Alpha
        emission.inputs[1].default_value = 5.0
    sphere.data.materials.append(mat)
    
    return sphere

def add_firework_particle_system(obj, particle_name, firework_obj, velocity_scale=2.0):
    # パーティクルシステムを追加
    ps = obj.modifiers.new(name=particle_name, type='PARTICLE_SYSTEM')
    particle_settings = bpy.data.particles.new(name=particle_name)
    ps.particle_system.settings = particle_settings
    
    # パーティクル設定
    particle_settings.count = len(obj.data.vertices) * 2
    particle_settings.frame_start = 1
    particle_settings.frame_end = 3
    particle_settings.lifetime = 180
    particle_settings.lifetime_random = 0.35
    
    particle_settings.emit_from = 'VERT'
    particle_settings.physics_type = 'NEWTON'
    particle_settings.render_type = 'OBJECT'
    particle_settings.instance_object = firework_obj
    particle_settings.particle_size = 2.0
    
    # 発射速度を発射台の距離に応じて変更
    particle_settings.normal_factor = velocity_scale * 7
    particle_settings.factor_random = 0.05
    particle_settings.drag_factor = 0.01
    return particle_settings

def downsample_point_cloud_fps(points, num_samples):
    """
    FPS による点群のダウンサンプリング後、Blender のシーンに追加。
    """
    """
    Farthest Point Sampling (FPS) による点群のダウンサンプリング。
    :param points: 入力点群 (numpy.ndarray)
    :param num_samples: サンプリングする点の数
    :return: ダウンサンプリングされた点群 (numpy.ndarray)
    """
    num_points = points.shape[0]
    sampled_indices = np.zeros(num_samples, dtype=int)
    distances = np.full(num_points, np.inf)
    
    # 最初の点をランダムに選択
    sampled_indices[0] = np.random.randint(num_points)
    
    for i in range(1, num_samples):
        last_sampled = points[sampled_indices[i - 1]]
        dist_to_last = np.linalg.norm(points - last_sampled, axis=1)
        distances = np.minimum(distances, dist_to_last)
        sampled_indices[i] = np.argmax(distances)  # 最も遠い点を選択
    
    downsampled_points = points[sampled_indices]
    
    # Blender のオブジェクトとして登録
    segmented_firework = create_firework_object("Firework_Segmented", (1, 0, 0),0.2)
    mesh = bpy.data.meshes.new("DownsampledPointCloud")
    mesh.from_pydata(downsampled_points.tolist(), [], [])
    obj = bpy.data.objects.new("DownsampledPointCloud", mesh)
    bpy.context.collection.objects.link(obj)
#    average_distance = compute_average_distance_from_origin(downsampled_points)
#    add_firework_particle_system(obj, "Fireworks_Inside", segmented_firework, velocity_scale=average_distance)
    
    return downsampled_points

def compute_average_distance_from_origin(points):
    """
    点群データの原点からの平均距離を計算。
    :param points: 入力点群 (numpy.ndarray)
    :return: 平均距離 (float)
    """
    distances = np.linalg.norm(points, axis=1)
    return np.mean(distances)

# def main():
#     # シーンをリセット
#     bpy.ops.object.select_all(action='SELECT')
#     bpy.ops.object.delete()
    
#     # 点群データをロード
#     file_path = "path\\to\\plyFile"
#     point_cloud_obj, point_cloud = import_ply(file_path)
#     if point_cloud is None:
#         return  # エラー時に終了
#         # 点群データをダウンサンプリング
#     downsampled_point_cloud = downsample_point_cloud_fps(point_cloud, num_samples=1000)

#     # 球状の発射台を作成
#     launch_obj, launch_points = create_launch_platform()
    
#     # 発射台をセグメンテーション
#     inside_collection, outside_collection, inside_layers, outside_layers = segment_launch_platform(launch_points, downsampled_point_cloud, point_cloud_obj)
    
# main()
def register():
    bpy.utils.register_class(FIREWORKS_OT_ImportPointCloud)
    bpy.utils.register_class(FIREWORKS_PT_Panel)

def unregister():
    bpy.utils.unregister_class(FIREWORKS_OT_ImportPointCloud)
    bpy.utils.unregister_class(FIREWORKS_PT_Panel)

if __name__ == "__main__":
    register()