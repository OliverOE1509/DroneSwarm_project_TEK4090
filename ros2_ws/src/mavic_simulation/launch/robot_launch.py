#!/usr/bin/env python

# Copyright 1996-2023 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Launch Webots Mavic 2 Pro driver."""

import os
import launch
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions.path_join_substitution import PathJoinSubstitution
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.webots_controller import WebotsController
import shutil
import os
from launch_ros.actions import Node, SetUseSimTime
import numpy as np # For random flag position generation

def generate_launch_description():
    # Método para adicionar a quantidade especificada de drones ao arquivo .wbt do mundo que será simulado
    def generate_wbt_file(num_drones):
        # Abre o modelo base do mundo simulado e lê o conteúdo
        with open('install/mavic_simulation/share/mavic_simulation/worlds/mavic_world.wbt') as template_file:
            template_content = template_file.read()

        # Cria drones de acordo com a quantidade desejada
        wbt_content = ''
        y = 0
        for i in range(num_drones):
            # Modify template content for each drone instance
            drone_content = f"Mavic2Pro {{\n"
            drone_content += f"  translation 0 {y} 0.07\n"  # Adjust translation based on 'i'
            drone_content += "  rotation 0 0 1 3.141590777218456\n"
            drone_content += f"  name \"Mavic_2_PRO_{i+1}\"\n"
            drone_content += "  controller \"<extern>\"\n"
            drone_content += "  supervisor TRUE\n"
            drone_content += "  cameraSlot [\n"
            drone_content += "    Camera {\n"
            drone_content += "      width 400\n"
            drone_content += "      height 240\n"
            drone_content += "      near 0.2\n"
            drone_content += "      rotation 0 1 0 1.5708\n"
            drone_content += "    }\n"
            drone_content += "  ]\n"
            drone_content += "}\n\n"

            wbt_content += drone_content
            y += 1
        
        
        # Adiciona modelo base do mundo aos drones criados
        wbt_content = template_content + wbt_content
        
        # Verifica se já existe um arquivo updated_world.wbt (se existir dá erro). Se sim, apaga ele.
        if os.path.exists('install/mavic_simulation/share/mavic_simulation/worlds/updated_world.wbt'):
            os.remove('install/mavic_simulation/share/mavic_simulation/worlds/updated_world.wbt')

        # Destino do novo arquivo de mundo que será usado pela simulação
        destination_dir = 'install/mavic_simulation/share/mavic_simulation/worlds/'
        
        # Save the updated WBT content to a new file
        with open('updated_world.wbt', 'w') as output_file:
            output_file.write(wbt_content)

        # Move novo arquivo de mundo para a pasta onde ele será lido posteriormente
        shutil.move('updated_world.wbt', destination_dir)

    # Configurações do mundo que será simulado
    package_dir = get_package_share_directory('mavic_simulation')
    world = LaunchConfiguration('world')

    webots = WebotsLauncher(
        world=PathJoinSubstitution([package_dir, 'worlds', world]),
        ros2_supervisor=True
    )
    robot_description_path = os.path.join(package_dir, 'resource', 'mavic_webots.urdf')

    # Set the number of drones.
    num_drones = 4
    # Set the frequency of the control loops
    freq_hz = 10.0

    # Chama o método para adicionar os modelos dos drones no arquivo .wbt do mundo
    generate_wbt_file(num_drones)

    # Make flag publisher node
    flag_node = Node(
        package = 'mavic_simulation',
        executable = 'flag_node',
        parameters = [{'NDrones':  num_drones},
                      {'use_sim_time': True},
                      {'loop_freq_hz':freq_hz}]
    )

    # Instancia os drivers para a quantidade de drones desejada
    mavic_drivers = {}
    mavic_controllers = {}
    for i in range(num_drones):
        driver_name = f"mavic_driver_{i + 1}"
        controller_name = f"Mavic_2_PRO_{i + 1}_Controller"
        drone_name = f"Mavic_2_PRO_{i+1}"

        mavic_drivers[driver_name] = WebotsController(
            robot_name=drone_name,
            parameters=[
                {'robot_description': robot_description_path},
                {'use_sim_time': True}
            ],
            respawn=True
        )

        mavic_controllers[controller_name] = Node(
            package="mavic_simulation",
            executable="mavic_controller",
            name=controller_name,
            parameters=[{"NDrones":num_drones},
                        {"MavicID":i+1},
                        {'use_sim_time': True},
                        {'loop_freq_hz':freq_hz}]
        )


    # Cria o launch
    ld = LaunchDescription([
        SetUseSimTime(True),
        DeclareLaunchArgument(
            'world',
            default_value='updated_world.wbt',
            description='Choose one of the world files from `/mavic_simulation/worlds` directory'
        ),
        webots,
        flag_node,
        webots._supervisor,
        # This action will kill all nodes once the Webots simulation has exited
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots,
                on_exit=[
                    launch.actions.EmitEvent(event=launch.events.Shutdown())
                ],
            )
        )
    ])

    # Adiciona os drivers ao launch da simulação
    for mavic in mavic_drivers:
        ld.add_action(mavic_drivers[mavic])
    for ctrlr in mavic_controllers:
        ld.add_action(mavic_controllers[ctrlr])

    return ld