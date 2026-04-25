import os
from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 1. DEFINISCO I PERCORSI
    # Trovo dove colcon ha installato il mio pacchetto
    mio_pacchetto_path = FindPackageShare('robot_description')
    
    # Costruisco il percorso esatto al mio file xacro (il "Frankenstein")
    xacro_file = PathJoinSubstitution([
        mio_pacchetto_path,
        'urdf',
        'ur5_with_gripper.urdf.xacro'
    ])

    # 2. CONVERTO LO XACRO IN URDF
    # Questo comando dice a ROS: "Esegui il comando 'xacro' su questo file e dammi la stringa XML risultante"
    robot_description_content = Command(
        [FindExecutable(name='xacro'), ' ', xacro_file]
    )
    # Salvo la stringa in un dizionario parametrico
    robot_description = {'robot_description': robot_description_content}

    # 3. IL NODO MATEMATICO (Robot State Publisher)
    # Prende l'URDF, calcola tutta la cinematica (chi è figlio di chi) e la pubblica sulla rete ROS
    nodo_rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # 4. IL NODO DEGLI SLIDER (Joint State Publisher GUI)
    # Genera la finestrella per muovere manualmente i giunti di braccio e pinza
    nodo_jsp_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen'
    )

    # 5. IL NODO GRAFICO (RViz2)
    # Apre l'interfaccia 3D
    nodo_rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen'
    )

    # 6. AZIONE! (Il Regista avvia tutto)
    return LaunchDescription([
        nodo_rsp,
        nodo_jsp_gui,
        nodo_rviz,
    ])
