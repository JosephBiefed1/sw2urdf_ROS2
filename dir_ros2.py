import os
import sys
import shutil
import argparse
import subprocess
import xml.etree.ElementTree as ET
from pathlib import Path

try:
    import tkinter as tk
    from tkinter import filedialog
    TKINTER_AVAILABLE = True
except ImportError:
    TKINTER_AVAILABLE = False


class ROS2PackageSetupError(Exception):
    pass


class DependencyCheckError(ROS2PackageSetupError):
    pass


def check_file_exists(file_path: str, file_description: str = "File") -> None:
    if not os.path.isfile(file_path):
        raise DependencyCheckError(f"{file_description} does not exist: {file_path}")


def check_directory_exists(dir_path: str, dir_description: str = "Directory") -> None:
    if not os.path.isdir(dir_path):
        raise DependencyCheckError(f"{dir_description} does not exist: {dir_path}")


def check_command_available(command: str) -> bool:
    try:
        subprocess.run(
            [command, "--version"],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            check=False,
            timeout=5
        )
        return True
    except (subprocess.TimeoutExpired, FileNotFoundError):
        return False


def write_file_content(file_path: str, content: str, encoding: str = "utf-8") -> None:
    parent_dir = os.path.dirname(file_path)
    if parent_dir:
        os.makedirs(parent_dir, exist_ok=True)
    with open(file_path, 'w', encoding=encoding) as f:
        f.write(content)


def read_file_content(file_path: str, encoding: str = "utf-8") -> str:
    with open(file_path, 'r', encoding=encoding) as f:
        return f.read()


def insert_content_at_line(source_file: str, target_file: str, line_number: int) -> None:
    source_content = read_file_content(source_file)
    target_lines = read_file_content(target_file).splitlines(keepends=True)

    insert_index = min(line_number - 1, len(target_lines))
    target_lines.insert(insert_index, source_content + '\n')

    write_file_content(target_file, ''.join(target_lines))
    print(f"Content successfully inserted into {target_file} at line {line_number}")


def replace_first_line(file_path: str, new_first_line: str) -> None:
    lines = read_file_content(file_path).splitlines(keepends=True)
    if lines:
        lines[0] = new_first_line + '\n'
    write_file_content(file_path, ''.join(lines))


def delete_directory(dir_path: str) -> bool:
    if os.path.isdir(dir_path):
        try:
            shutil.rmtree(dir_path)
            print(f"Directory deleted: {dir_path}")
            return True
        except Exception as e:
            print(f"Failed to delete directory: {e}")
            return False
    return False


def delete_file(file_path: str) -> bool:
    if os.path.isfile(file_path):
        try:
            os.remove(file_path)
            print(f"File deleted: {file_path}")
            return True
        except Exception as e:
            print(f"Failed to delete file: {e}")
            return False
    return False


def ensure_directory(dir_path: str) -> None:
    os.makedirs(dir_path, exist_ok=True)


def convert_urdf_to_sdf(urdf_file: str, output_dir: str) -> str:
    ensure_directory(output_dir)
    sdf_file = os.path.join(output_dir, "model.sdf")
    
    command = f'gz sdf -p "{urdf_file}"'
    try:
        result = subprocess.run(
            command,
            shell=True,
            check=True,
            capture_output=True,
            text=True,
            timeout=30
        )
        with open(sdf_file, 'w', encoding='utf-8') as f:
            f.write(result.stdout)
        print(f"URDF to SDF conversion successful: {sdf_file}")
        return sdf_file
    except subprocess.CalledProcessError as e:
        raise ROS2PackageSetupError(f"Gazebo SDF conversion failed: {e.stderr}")
    except subprocess.TimeoutExpired:
        raise ROS2PackageSetupError("Gazebo SDF conversion timed out")
    except FileNotFoundError:
        raise ROS2PackageSetupError("Gazebo command line tool (gz) not found. Please ensure Gazebo is installed")


def create_model_config(model_dir: str, model_name: str, sdf_file: str) -> str:
    """Create model.config next to model.sdf for a Gazebo model package."""
    try:
        sdf_root = ET.parse(sdf_file).getroot()
    except ET.ParseError as e:
        raise ROS2PackageSetupError(f"Invalid SDF XML in {sdf_file}: {e}") from e

    sdf_version = sdf_root.get("version")
    sdf_model = sdf_root.find("model")
    sdf_model_name = sdf_model.get("name") if sdf_model is not None else None
    if not sdf_version or not sdf_model_name:
        raise ROS2PackageSetupError(
            f"SDF must contain <sdf version=...><model name=...>: {sdf_file}"
        )
    if sdf_model_name != model_name:
        raise ROS2PackageSetupError(
            f"URDF model name '{model_name}' does not match SDF model name "
            f"'{sdf_model_name}'"
        )

    config_root = ET.Element("model")
    ET.SubElement(config_root, "name").text = model_name
    ET.SubElement(config_root, "version").text = "1.0"
    sdf_element = ET.SubElement(config_root, "sdf", {"version": sdf_version})
    sdf_element.text = "model.sdf"
    ET.SubElement(config_root, "description").text = (
        f"Gazebo Sim model generated from the {model_name} URDF."
    )
    ET.indent(config_root, space="  ")

    model_config_path = os.path.join(model_dir, "model.config")
    ET.ElementTree(config_root).write(
        model_config_path, encoding="utf-8", xml_declaration=True
    )
    print(f"model.config created: {model_config_path}")
    return model_config_path


def deploy_gazebo_model(package_name: str, source_urdf_dir: str, 
                         gazebo_models_dir: str) -> None:
    target_dir = os.path.join(gazebo_models_dir, package_name)
    
    if os.path.exists(target_dir):
        print(f"Target directory already exists, removing: {target_dir}")
        shutil.rmtree(target_dir)

    ensure_directory(target_dir)
    print(f"Creating Gazebo model directory: {target_dir}")

    sdf_source = os.path.join(source_urdf_dir, "model.sdf")
    sdf_target = os.path.join(target_dir, "model.sdf")
    if os.path.exists(sdf_source):
        shutil.copy2(sdf_source, sdf_target)
        print(f"SDF file copied")

    meshes_source = os.path.join(os.path.dirname(source_urdf_dir), "meshes")
    meshes_target = os.path.join(target_dir, "meshes")
    if os.path.exists(meshes_source):
        shutil.copytree(meshes_source, meshes_target)
        print(f"Meshes directory copied")

    textures_source = os.path.join(os.path.dirname(source_urdf_dir), "textures")
    textures_target = os.path.join(target_dir, "materials", "textures")
    if os.path.exists(textures_source):
        ensure_directory(textures_target)
        shutil.copytree(textures_source, textures_target, dirs_exist_ok=True)
        print(f"Textures directory copied")

    create_model_config(target_dir, package_name, sdf_target)


def create_display_launch(package_name: str, launch_dir: str) -> None:
    launch_file = os.path.join(launch_dir, "display.launch.py")
    content = f"""import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    package_dir = get_package_share_directory('{package_name}')
    urdf_file = os.path.join(package_dir, 'urdf', '{package_name}.urdf')

    with open(urdf_file, 'r') as file:
        robot_description = file.read()

    return LaunchDescription([
        DeclareLaunchArgument('urdf_file', default_value=urdf_file),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{{'robot_description': robot_description}}]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            parameters=[{{'robot_description': robot_description}}]
        ),
    ])
"""
    write_file_content(launch_file, content)
    print(f"display.launch.py created: {launch_file}")


def create_gazebo_launch(package_name: str, launch_dir: str) -> None:
    launch_file = os.path.join(launch_dir, "gazebo.launch.py")
    content = f"""import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os


def generate_launch_description():
    robot_name_in_model = "{package_name}"
    urdf_tutorial_path = get_package_share_directory('{package_name}')
    default_model_path = os.path.join(
        urdf_tutorial_path, 'urdf', '{package_name}.urdf')

    with open(default_model_path, 'r') as urdf_file:
        robot_description = urdf_file.read()

    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{{'robot_description': robot_description}}]
    )

    launch_gazebo = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory(
            'ros_gz_sim'), '/launch', '/gazebo.launch.py']),
    )

    spawn_entity_node = launch_ros.actions.Node(
        package='ros_gz_sim',
        executable='spawn_entity.py',
        arguments=['-topic', '/robot_description',
                   '-entity', robot_name_in_model])

    return launch.LaunchDescription([
        robot_state_publisher_node,
        launch_gazebo,
        spawn_entity_node
    ])
"""
    write_file_content(launch_file, content)
    print(f"gazebo.launch.py created: {launch_file}")


def create_cmakelists(package_name: str, target_dir: str) -> None:
    cmake_file = os.path.join(target_dir, "CMakeLists.txt")
    content = f"""cmake_minimum_required(VERSION 3.8)
project({package_name})

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(rviz2 REQUIRED)
find_package(ros_gz_sim REQUIRED)

install(FILES model.config model.sdf
    DESTINATION share/${{PROJECT_NAME}})

install(DIRECTORY launch config meshes urdf
    DESTINATION share/${{PROJECT_NAME}})

install(DIRECTORY textures
    DESTINATION share/${{PROJECT_NAME}}
    OPTIONAL)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
"""
    write_file_content(cmake_file, content)
    print(f"CMakeLists.txt created: {cmake_file}")


def create_package_xml(package_name: str, target_dir: str) -> None:
    xml_file = os.path.join(target_dir, "package.xml")
    content = f"""<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>{package_name}</name>
  <version>0.0.0</version>
  <description>TODO: Package description</description>
  <maintainer email="robotsheep@todo.todo">robotsheep</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <depend>rclcpp</depend>
  <depend>robot_state_publisher</depend>
  <depend>rviz2</depend>
  <depend>ros_gz_sim</depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
"""
    write_file_content(xml_file, content)
    print(f"package.xml created: {xml_file}")


def validate_prerequisites() -> None:
    if not check_command_available("gz"):
        raise DependencyCheckError(
            "Gazebo command line tool (gz) is not available. "
            "Please ensure Gazebo is installed and gz is in your system PATH."
        )
    
    script_dir = os.path.dirname(os.path.abspath(__file__))
    insert_urdf = os.path.join(script_dir, "insert_urdf.txt")
    insert_sdf = os.path.join(script_dir, "insert_sdf.txt")
    
    if not os.path.isfile(insert_urdf):
        raise DependencyCheckError(
            f"insert_urdf.txt not found: {insert_urdf}\n"
            "Please ensure this file is in the same directory as the script"
        )
    if not os.path.isfile(insert_sdf):
        raise DependencyCheckError(
            f"insert_sdf.txt not found: {insert_sdf}\n"
            "Please ensure this file is in the same directory as the script"
        )


def select_target_directory() -> str:
    if not TKINTER_AVAILABLE:
        raise ROS2PackageSetupError(
            "tkinter is not available. Please specify target directory via command line argument\n"
            "Usage: python dir_ros2.py /path/to/robot_package"
        )

    root = tk.Tk()
    root.withdraw()
    root.attributes('-topmost', True)

    directory = filedialog.askdirectory(title="Select target ROS2 package directory")

    root.destroy()

    if not directory:
        raise ROS2PackageSetupError("No valid directory selected")
    
    return directory
def get_model_name_from_urdf(urdf_file: str) -> str:
    try:
        root = ET.parse(urdf_file).getroot()
    except ET.ParseError as e:
        raise ROS2PackageSetupError(f"Invalid URDF XML in {urdf_file}: {e}") from e

    model_name = root.get("name")
    if not model_name:
        raise ROS2PackageSetupError(f"Model name not found in URDF file: {urdf_file}")
    return model_name



def get_joints_from_urdf(urdf_file: str) -> list:
    try:
        root = ET.parse(urdf_file).getroot()
    except ET.ParseError as e:
        raise ROS2PackageSetupError(f"Invalid URDF XML in {urdf_file}: {e}") from e

    # A position controller is only useful for movable, named joints.
    return [
        joint.get("name")
        for joint in root.findall("joint")
        if joint.get("name") and joint.get("type") != "fixed"
    ]


def insert_plugin_on_joint_names(urdf_file: str, ) -> str  :
    joints = get_joints_from_urdf(urdf_file)
    model_name = get_model_name_from_urdf(urdf_file)
    str_to_insert = ""
    for joint in joints:
        plugin_template = f"""
        <plugin filename="gz-sim-joint-position-controller-system"
                name="gz::sim::systems::JointPositionController">
        <joint_name>{joint}</joint_name>
        <topic>/{model_name}/{joint}_cmd</topic>
        <p_gain>2.0</p_gain>
        <i_gain>0.05</i_gain>
        <d_gain>0.2</d_gain>
        <cmd_min>-5</cmd_min>
        <cmd_max>5</cmd_max>
        </plugin>"""
        str_to_insert += plugin_template

    return str_to_insert


def validate_target_directory(target_dir: str) -> str:
    if not os.path.isdir(target_dir):
        raise ROS2PackageSetupError(f"Target directory does not exist: {target_dir}")

    package_name = os.path.basename(target_dir)
    urdf_file = os.path.join(target_dir, "urdf", f"{package_name}.urdf")

    if not os.path.isfile(urdf_file):
        raise ROS2PackageSetupError(
            f"URDF file does not exist: {urdf_file}\n"
            "Target directory should contain urdf/<package_name>.urdf file"
        )
    
    return package_name

def write_sdf_file(sdf_file_path: str, content: str) -> None:
    parent_dir = os.path.dirname(sdf_file_path)
    if parent_dir:
        os.makedirs(parent_dir, exist_ok=True)
    with open(sdf_file_path, 'w', encoding='utf-8') as f:
        f.write(content)
    print(f"SDF file written: {sdf_file_path}")


def setup_ros2_package(target_directory: str) -> None:
    package_name = os.path.basename(target_directory)
    print(f"=" * 50)
    print(f"Starting ROS2 package setup: {package_name}")
    print(f"Target directory: {target_directory}")
    print(f"=" * 50)

    launch_dir = os.path.join(target_directory, "launch")
    delete_directory(launch_dir)
    ensure_directory(launch_dir)

    delete_file(os.path.join(target_directory, "CMakeLists.txt"))
    delete_file(os.path.join(target_directory, "package.xml"))

    create_cmakelists(package_name, target_directory)
    create_package_xml(package_name, target_directory)
    create_display_launch(package_name, launch_dir)
    create_gazebo_launch(package_name, launch_dir)

    script_dir = os.path.dirname(os.path.abspath(__file__))
    insert_urdf_file = os.path.join(script_dir, "insert_urdf.txt")
    insert_sdf_file = os.path.join(script_dir, "insert_sdf.txt")

    urdf_file = os.path.join(target_directory, "urdf", f"{package_name}.urdf")
    insert_content_at_line(insert_urdf_file, urdf_file, 7)
    replace_first_line(urdf_file, '<?xml version="1.0" ?>')

    urdf_dir = os.path.join(target_directory, "urdf")
    sdf_file = convert_urdf_to_sdf(urdf_file, urdf_dir)

    insert_content_at_line(insert_sdf_file, sdf_file, 1)

    #mkdir sdf
    str_to_insert = insert_plugin_on_joint_names(urdf_file)
    model_name = get_model_name_from_urdf(urdf_file)

    print(f"Plugin content to insert into SDF:\n{str_to_insert}")
    print("Check your limits of your joints!!!!!-----------------------")
    #insert this str before /model> in sdf file
    with open(sdf_file, 'r', encoding='utf-8') as f:
        sdf_content = f.read()
        sdf_content = sdf_content.replace("</model>", str_to_insert + "</model>")
    write_sdf_file(sdf_file, sdf_content)
    packaged_sdf = os.path.join(target_directory, "model.sdf")
    shutil.copy2(sdf_file, packaged_sdf)
    print(f"SDF file copied to {packaged_sdf}")
    create_model_config(target_directory, model_name, packaged_sdf)


    temp_sdf = os.path.join(urdf_dir, "model.sdf")
    delete_file(temp_sdf)

    print(f"=" * 50)
    print(f"ROS2 package setup complete: {package_name}")
    print(f"=" * 50)


def main():
    parser = argparse.ArgumentParser(
        description="ROS2 robot model conversion and deployment tool",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Example usage:
  python dir_ros2.py                                  # Use GUI to select directory
  python dir_ros2.py /path/to/robot_package           # Specify directory via command line
        """
    )
    parser.add_argument(
        "target_directory",
        nargs="?",
        default="",
        help="Target directory path for the ROS2 package"
    )
    parser.add_argument(
        "--skip-validation",
        action="store_true",
        help="Skip dependency checks"
    )
    
    args = parser.parse_args()

    try:
        if not args.skip_validation:
            validate_prerequisites()
        
        if args.target_directory:
            target_directory = os.path.abspath(args.target_directory)
        else:
            target_directory = select_target_directory()
        
        package_name = validate_target_directory(target_directory)
        setup_ros2_package(target_directory)
        
    except ROS2PackageSetupError as e:
        print(f"Error: {e}", file=sys.stderr)
        sys.exit(1)
    except KeyboardInterrupt:
        print("\nOperation cancelled")
        sys.exit(130)
    except Exception as e:
        print(f"Unknown error: {e}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
