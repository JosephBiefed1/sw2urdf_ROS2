import os
import sys
import shutil
import argparse
import subprocess
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


def check_file_exists(file_path: str, file_description: str = "文件") -> None:
    if not os.path.isfile(file_path):
        raise DependencyCheckError(f"{file_description} 不存在: {file_path}")


def check_directory_exists(dir_path: str, dir_description: str = "目录") -> None:
    if not os.path.isdir(dir_path):
        raise DependencyCheckError(f"{dir_description} 不存在: {dir_path}")


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
    print(f"内容已成功插入到 {target_file} 第 {line_number} 行")


def replace_first_line(file_path: str, new_first_line: str) -> None:
    lines = read_file_content(file_path).splitlines(keepends=True)
    if lines:
        lines[0] = new_first_line + '\n'
    write_file_content(file_path, ''.join(lines))


def delete_directory(dir_path: str) -> bool:
    if os.path.isdir(dir_path):
        try:
            shutil.rmtree(dir_path)
            print(f"已删除目录: {dir_path}")
            return True
        except Exception as e:
            print(f"删除目录失败: {e}")
            return False
    return False


def delete_file(file_path: str) -> bool:
    if os.path.isfile(file_path):
        try:
            os.remove(file_path)
            print(f"已删除文件: {file_path}")
            return True
        except Exception as e:
            print(f"删除文件失败: {e}")
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
        print(f"URDF 转换为 SDF 成功: {sdf_file}")
        return sdf_file
    except subprocess.CalledProcessError as e:
        raise ROS2PackageSetupError(f"Gazebo SDF 转换失败: {e.stderr}")
    except subprocess.TimeoutExpired:
        raise ROS2PackageSetupError("Gazebo SDF 转换超时")
    except FileNotFoundError:
        raise ROS2PackageSetupError("Gazebo 命令行工具 (gz) 未找到，请确保已安装 Gazebo")


def deploy_gazebo_model(package_name: str, source_urdf_dir: str, 
                         gazebo_models_dir: str) -> None:
    target_dir = os.path.join(gazebo_models_dir, package_name)
    
    if os.path.exists(target_dir):
        print(f"目标目录已存在，先删除: {target_dir}")
        shutil.rmtree(target_dir)
    
    ensure_directory(target_dir)
    print(f"创建 Gazebo 模型目录: {target_dir}")

    sdf_source = os.path.join(source_urdf_dir, "model.sdf")
    sdf_target = os.path.join(target_dir, "model.sdf")
    if os.path.exists(sdf_source):
        shutil.copy2(sdf_source, sdf_target)
        print(f"已复制 SDF 文件")

    meshes_source = os.path.join(os.path.dirname(source_urdf_dir), "meshes")
    meshes_target = os.path.join(target_dir, "meshes")
    if os.path.exists(meshes_source):
        shutil.copytree(meshes_source, meshes_target)
        print(f"已复制 meshes 目录")

    textures_source = os.path.join(os.path.dirname(source_urdf_dir), "textures")
    textures_target = os.path.join(target_dir, "materials", "textures")
    if os.path.exists(textures_source):
        ensure_directory(textures_target)
        shutil.copytree(textures_source, textures_target, dirs_exist_ok=True)
        print(f"已复制 textures 目录")

    model_config_path = os.path.join(target_dir, "model.config")
    model_config = f"""<?xml version="1.0"?>
<model>
  <name>{package_name}</name>
  <version>1.0</version>
  <sdf version="1.7">model.sdf</sdf>
  <author>
    <name>todo</name>
    <email>todo@todo.todo</email>
  </author>
  <description>
    sw2urdf ROS2 for gazebo11
  </description>
</model>
"""
    write_file_content(model_config_path, model_config)
    print(f"已创建 model.config")


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
    print(f"已创建 display.launch.py: {launch_file}")


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
            'gazebo_ros'), '/launch', '/gazebo.launch.py']),
    )

    spawn_entity_node = launch_ros.actions.Node(
        package='gazebo_ros',
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
    print(f"已创建 gazebo.launch.py: {launch_file}")


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
find_package(gazebo_ros REQUIRED)

install(DIRECTORY launch config meshes urdf
    DESTINATION share/${{PROJECT_NAME}})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
"""
    write_file_content(cmake_file, content)
    print(f"已创建 CMakeLists.txt: {cmake_file}")


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
  <depend>gazebo_ros</depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
"""
    write_file_content(xml_file, content)
    print(f"已创建 package.xml: {xml_file}")


def validate_prerequisites() -> None:
    if not check_command_available("gz"):
        raise DependencyCheckError(
            "Gazebo 命令行工具 (gz) 不可用。"
            "请确保已安装 Gazebo 并将 gz 添加到系统 PATH。"
        )
    
    script_dir = os.path.dirname(os.path.abspath(__file__))
    insert_urdf = os.path.join(script_dir, "insert_urdf.txt")
    insert_sdf = os.path.join(script_dir, "insert_sdf.txt")
    
    if not os.path.isfile(insert_urdf):
        raise DependencyCheckError(
            f"insert_urdf.txt 不存在: {insert_urdf}\n"
            "请确保该文件与脚本位于同一目录"
        )
    if not os.path.isfile(insert_sdf):
        raise DependencyCheckError(
            f"insert_sdf.txt 不存在: {insert_sdf}\n"
            "请确保该文件与脚本位于同一目录"
        )


def select_target_directory() -> str:
    if not TKINTER_AVAILABLE:
        raise ROS2PackageSetupError(
            "tkinter 不可用，请通过命令行参数指定目标目录\n"
            "使用: python dir_ros2.py /path/to/robot_package"
        )
    
    root = tk.Tk()
    root.withdraw()
    root.attributes('-topmost', True)
    
    directory = filedialog.askdirectory(title="选择目标 ROS2 包目录")
    
    root.destroy()
    
    if not directory:
        raise ROS2PackageSetupError("未选择有效目录")
    
    return directory


def validate_target_directory(target_dir: str) -> str:
    if not os.path.isdir(target_dir):
        raise ROS2PackageSetupError(f"目标目录不存在: {target_dir}")
    
    package_name = os.path.basename(target_dir)
    urdf_file = os.path.join(target_dir, "urdf", f"{package_name}.urdf")
    
    if not os.path.isfile(urdf_file):
        raise ROS2PackageSetupError(
            f"URDF 文件不存在: {urdf_file}\n"
            "目标目录应包含 urdf/<package_name>.urdf 文件"
        )
    
    return package_name


def setup_ros2_package(target_directory: str) -> None:
    package_name = os.path.basename(target_directory)
    print(f"=" * 50)
    print(f"开始配置 ROS2 包: {package_name}")
    print(f"目标目录: {target_directory}")
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

    gazebo_models_dir = os.path.expanduser("~/.gazebo/models")
    deploy_gazebo_model(package_name, urdf_dir, gazebo_models_dir)

    temp_sdf = os.path.join(urdf_dir, "model.sdf")
    delete_file(temp_sdf)

    print(f"=" * 50)
    print(f"ROS2 包配置完成: {package_name}")
    print(f"=" * 50)


def main():
    parser = argparse.ArgumentParser(
        description="ROS2 机器人模型转换与部署工具",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例用法:
  python dir_ros2.py                                  # 使用图形界面选择目录
  python dir_ros2.py /path/to/robot_package           # 通过命令行指定目录
        """
    )
    parser.add_argument(
        "target_directory",
        nargs="?",
        default="",
        help="ROS2 包的目标目录路径"
    )
    parser.add_argument(
        "--skip-validation",
        action="store_true",
        help="跳过依赖项检查"
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
        print(f"错误: {e}", file=sys.stderr)
        sys.exit(1)
    except KeyboardInterrupt:
        print("\n操作已取消")
        sys.exit(130)
    except Exception as e:
        print(f"未知错误: {e}", file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
