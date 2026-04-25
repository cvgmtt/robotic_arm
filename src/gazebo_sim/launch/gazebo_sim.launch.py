import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command, PathJoinSubstitution
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    # 1. PREPARO IL ROBOT
    pkg_robot_description = FindPackageShare('robot_description')
    xacro_file = PathJoinSubstitution([pkg_robot_description, 'urdf', 'ur5_with_gripper.urdf.xacro'])
    robot_description_content = Command(['xacro ', xacro_file])
    
    robot_state_publisher = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    name='robot_state_publisher',
    output='both',
    parameters=[
        {'use_sim_time': True},
        {'robot_description': robot_description_content},
    ]
    )


    # 2. APRO GAZEBO COL TUO MONDO LEGO E RISOLVO I PERCORSI
    pkg_ros_gz_sim = FindPackageShare('ros_gz_sim')
    pkg_mio_gazebo = FindPackageShare('gazebo_sim')
    world_file = os.path.join(pkg_mio_gazebo.find('gazebo_sim'), 'worlds', 'table_scene.sdf')
    
    # --- IL TRUCCO "BLINDATO" PER IGNITION FORTRESS ---
    # Saliamo di un livello (dirname) dalla cartella share per far vedere a Gazebo tutto il pacchetto
    percorso_ur = os.path.dirname(get_package_share_directory('ur_description'))
    percorso_robotiq = os.path.dirname(get_package_share_directory('robotiq_description'))
    percorso_lego = os.path.join(get_package_share_directory('gazebo_sim'), 'lego_models')

    # Unisco i percorsi
    percorsi_modelli = f"{percorso_ur}:{percorso_robotiq}:{percorso_lego}"

    # Uso IGN_ (Ignition) e non GZ_
    imposta_percorsi_gazebo = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=percorsi_modelli
    )
    # --------------------------------------------------

    avvia_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim.find('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_file}'}.items(),
    )

    # 3. passa il file urdf a gazebo
    nodo_spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', '/robot_description', 
            '-name', 'ur5_robotiq',         
            '-x', '0.6', '-y', '-0.8', '-z', '0.7' 
        ],
        output='screen'
    )

    nodo_spawn_camera = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/rgbd_camera/image@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/rgbd_camera/depth_image@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/rgbd_camera/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked'
        ],
        output='screen'
    )

    nodo_static_tf_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        # Argomenti: x, y, z, yaw, pitch, roll, frame_padre, frame_figlio
        arguments=[
            '1.19153', '-0.343359', '1.74695',  # Posizione (X, Y, Z)
            '1.57159', '1.53339', '3.14159',    # Orientamento (Yaw, Pitch, Roll)
            'world', 'static_rgbd_camera/camera_link/rgbd_camera'
        ]
    )


    # 4 Definiamo i nodi che attivano i controllori dei joint
    # Questo carica e attiva il trasmettitore dei joint state di gazebo
    load_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    # Questo carica e attiva i muscoli del braccio
    load_ur_joint_trajectory_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["ur_joint_trajectory_controller"],
    )

    # 4.2 Gestiamo il tempismo (EVENT HANDLERS)
    # Vogliamo che i controller partano SOLO DOPO che lo spawner del robot 
    # (nodo_spawn_robot) ha finito di materializzare il robot in Gazebo.
    
    delayed_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=nodo_spawn_robot, # Il tuo nodo che fa 'create'
            on_exit=[load_joint_state_broadcaster, load_ur_joint_trajectory_controller],
        )
    )

    bridge_config_file = os.path.join(
        get_package_share_directory('gazebo_sim'),
        'config',
        'bridge_topic_mapping.YAML'
    )

    # 2. Creo il nodo che fa da Traduttore
    # nodo_bridge = Node(
    #     package='ros_gz_bridge',
    #     executable='parameter_bridge',
    #     parameters=[{
    #         'config_file': bridge_config_file
    #     }],
    #     output='screen'
    # )

    return LaunchDescription([
        robot_state_publisher,
        imposta_percorsi_gazebo,
        avvia_gazebo,
        nodo_spawn_robot,
        nodo_spawn_camera,
        nodo_static_tf_camera,
        #nodo_bridge
        delayed_controller_spawner
    ])