# Install necessary packages (only run once)
# pip install trimesh numpy scikit-learn

import trimesh
import numpy as np
from sklearn.cluster import KMeans
import xml.etree.ElementTree as ET
import sys
import os

def fit_error(points, primitive):
    return np.mean(trimesh.proximity.ProximityQuery(primitive).signed_distance(points) ** 2)

def process_link(link, mesh_path, num_clusters=3):
    mesh = trimesh.load(mesh_path)
    points, _ = trimesh.sample.sample_surface(mesh, 10000)
    kmeans = KMeans(n_clusters=num_clusters, random_state=42, n_init='auto')
    labels = kmeans.fit_predict(points)
    collisions = []
    for i in range(num_clusters):
        cluster_points = points[labels == i]
        if len(cluster_points) < 10:
            continue
        hull = trimesh.points.PointCloud(cluster_points).convex_hull
        box = hull.bounding_box_oriented
        box_err = fit_error(cluster_points, box)
        sphere = hull.bounding_sphere
        sphere_err = fit_error(cluster_points, sphere)
        try:
            cylinder = hull.bounding_cylinder
            cylinder_err = fit_error(cluster_points, cylinder)
        except Exception:
            cylinder = None
            cylinder_err = np.inf
        errors = {'box': box_err, 'sphere': sphere_err, 'cylinder': cylinder_err}
        best_type = min(errors, key=errors.get)
        if best_type == 'box':
            prim = box.primitive
            size = prim.extents
            transform = prim.transform
            xml = f'<geometry><box size="{size[0]:.4f} {size[1]:.4f} {size[2]:.4f}"/></geometry>'
        elif best_type == 'sphere':
            prim = sphere.primitive
            transform = np.eye(4)
            transform[:3, 3] = prim.center
            xml = f'<geometry><sphere radius="{prim.radius:.4f}"/></geometry>'
        else:
            prim = cylinder.primitive
            transform = prim.transform
            xml = f'<geometry><cylinder radius="{prim.radius:.4f}" length="{prim.height:.4f}"/></geometry>'
        xyz = transform[:3, 3]
        rpy = trimesh.transformations.euler_from_matrix(transform[:3, :3])
        collision = ET.Element('collision')
        origin = ET.SubElement(collision, 'origin')
        origin.set('xyz', f'{xyz[0]:.4f} {xyz[1]:.4f} {xyz[2]:.4f}')
        origin.set('rpy', f'{rpy[0]:.4f} {rpy[1]:.4f} {rpy[2]:.4f}')
        geometry = ET.fromstring(xml)
        collision.append(geometry)
        collisions.append(collision)
    return collisions

def main():
    if len(sys.argv) != 3:
        print('Usage: python collision_trimesh.py input.urdf output.urdf')
        sys.exit(1)
    input_urdf = sys.argv[1]
    output_urdf = sys.argv[2]
    tree = ET.parse(input_urdf)
    root = tree.getroot()
    for link in root.findall('link'):
        coll = link.find('collision')
        if coll is not None:
            geom = coll.find('geometry')
            mesh = geom.find('mesh') if geom is not None else None
            if mesh is not None:
                mesh_file = mesh.attrib['filename']
                # Handle package://unitree_g1_dex3_stack/robots/g1_description prefix for mesh paths
                if mesh_file.startswith('package://unitree_g1_dex3_stack/robots/g1_description/'):
                    mesh_rel_path = mesh_file.replace('package://unitree_g1_dex3_stack/robots/g1_description/', '')
                    mesh_path = os.path.abspath(os.path.join(os.path.dirname(input_urdf), mesh_rel_path))
                elif mesh_file.startswith('package://'):
                    mesh_rel_path = mesh_file.split('/', 3)[-1]
                    mesh_path = os.path.abspath(os.path.join(os.path.dirname(input_urdf), mesh_rel_path))
                else:
                    mesh_path = os.path.abspath(os.path.join(os.path.dirname(input_urdf), mesh_file))
                if not os.path.exists(mesh_path):
                    print(f"Mesh file {mesh_path} does not exist, skipping.")
                    continue
                # Remove old collision
                link.remove(coll)
                # Add new collisions
                for new_coll in process_link(link, mesh_path):
                    link.append(new_coll)
    tree.write(output_urdf)
    print(f"Saved new URDF with primitives: {output_urdf}")

if __name__ == '__main__':
    main()
