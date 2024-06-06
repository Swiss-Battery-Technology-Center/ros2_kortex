import xml.etree.ElementTree as ET
import numpy as np
import transforms3d as tf3d
import argparse


def extract_transformation_matrices(filename):
    tree = ET.parse(filename)
    root = tree.getroot()
    transforms = root.find('ArchitecturalCalibration')
    
    def create_homogeneous_matrix(transform):
        matrix = np.eye(4)
        matrix[0, 0] = float(transform.get('m00'))
        matrix[0, 1] = float(transform.get('m01'))
        matrix[0, 2] = float(transform.get('m02'))
        matrix[0, 3] = float(transform.get('m03'))
        matrix[1, 0] = float(transform.get('m10'))
        matrix[1, 1] = float(transform.get('m11'))
        matrix[1, 2] = float(transform.get('m12'))
        matrix[1, 3] = float(transform.get('m13'))
        matrix[2, 0] = float(transform.get('m20'))
        matrix[2, 1] = float(transform.get('m21'))
        matrix[2, 2] = float(transform.get('m22'))
        matrix[2, 3] = float(transform.get('m23'))
        return matrix
    
    matrices = []
    for transform in transforms.findall('Transform'):
        matrix = create_homogeneous_matrix(transform)
        matrices.append(matrix)
    
    return matrices



def adjust_urdf(urdf_filename, calib_filename, output_filename):
    # Extract transformation matrices from the calibration file
    matrices = extract_transformation_matrices(calib_filename)
    print(matrices)

    # Parse the URDF file
    tree = ET.parse(urdf_filename)
    root = tree.getroot()
    ns = {'xacro': 'http://ros.org/wiki/xacro'}
    

    # Find all joints and transform their origins
    for i, joint in enumerate(root.findall('.//xacro:macro//joint', ns)):
        origin = joint.find('origin')
        if origin is not None:
            try:
                joint_number = int(joint.get('name').split('_')[1])
            except:
                continue
            
            joint_number = int(joint.get('name').split('_')[1])
            print(f"Joint number: {joint_number}")
                
            xyz = [float(val) for val in origin.get('xyz').split()]
            rpy = [float(val) for val in origin.get('rpy').split()]
            
            print(f"Joint {joint.get('name')}:")
            print(f"Original xyz: {xyz}")
            print(f"Original rpy: {rpy}")
            
            matrix = np.eye(4)
            matrix[:3, :3] = tf3d.euler.euler2mat(rpy[0], rpy[1], rpy[2], 'szyx')
            matrix[:3, 3] = xyz

            new_matrix = matrix @ matrices[joint_number-1]
            
            new_xyz = new_matrix[:3, 3]
            new_rpy = tf3d.euler.mat2euler(new_matrix[:3, :3], 'szyx')
            print(f"Transformed xyz: {new_xyz}")
            print(f"Transformed rpy: {new_rpy}")
            
            # Update the xyz attribute with the transformed position
            origin.set('xyz', ' '.join([str(val) for val in new_xyz]))
            origin.set('rpy', ' '.join([str(val) for val in new_rpy]))
    
    # Write the modified URDF to a new file
    ET.register_namespace('xacro', 'http://ros.org/wiki/xacro')
    tree.write(output_filename, xml_declaration=True, encoding='utf-8')
    
    
    # Reopen the file and adjust some formatting
    with open(output_filename, "r") as f:
        lines = f.readlines()

    with open(output_filename, "w") as f:
        for line in lines:
            if 'params=' in line:
                line = line.replace('     ', '\n\t\t')

            if '<xacro:kortex_ros2_control' in line:
                line = line.replace(' ', '\n', 1)
                line = line.replace('" ', '"\n\t\t\t')

            f.write(line)
            



# Use argparser for input files and output file
parser = argparse.ArgumentParser(description='Adjust URDF file with calibration data')
parser.add_argument('--input_urdf', type=str, help='URDF file to adjust')
parser.add_argument('--calib', type=str, help='Calibration file')
parser.add_argument('--output_urdf', default='output.xacro', type=str, help='Output file')

args = parser.parse_args()
adjust_urdf(args.input_urdf, args.calib, args.output_urdf)

