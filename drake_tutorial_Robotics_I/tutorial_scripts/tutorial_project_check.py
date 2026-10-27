"""
Test file for inspecting models
--------------------
"""

import os
from pathlib import Path
import pydot
from pydrake.geometry import StartMeshcat
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydrake.systems.framework import DiagramBuilder
from pydrake.visualization import AddDefaultVisualization, ModelVisualizer
from pydrake.systems.analysis import Simulator  

# ------------------ Settings ------------------
meshcat = StartMeshcat()

# Adjust the path to where the SDF is in your directory
scene_path = os.path.join(
    # "..", "..", "models", "final_projects", "project_01_sorting", "project_01_pick_and_place_sorting_world.sdf"
    # "..", "..", "models", "final_projects", "project_02_drawing", "project_02_drawing_robot_world.sdf"
    # "..", "..", "models", "final_projects", "project_03_stacking", "project_03_stacking.sdf"     
    # "..", "..", "models", "final_projects", "project_04_door_opening","project_04_door_opening.sdf"    
    # "..", "..", "models", "final_projects", "project_05_cooperation", "project_05_cooperation_transportation.sdf" ## ---> Remove or change mobilel robot to omnidirectional
    # "..", "..", "models", "final_projects", "project_06_TAMP", "project_06_TAMP.sdf" 
    "..", "..", "models", "final_projects", "project_07_handover", "project_07_object_handover.sdf"
)

# ------------------ Functions ------------------
def create_sim_scene(sim_time_step):
    """Creates a MultibodyPlant + SceneGraph diagram."""
    meshcat.Delete()
    meshcat.DeleteAddedControls()

    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=sim_time_step)
    parser= Parser(plant)
    parser.package_map().Add(
        "robotics_ii_models",
        str(Path(__file__).resolve().parents[2] / "models")
    )

    parser.AddModelsFromUrl("file://" + os.path.abspath(scene_path))

    plant.Finalize()
    AddDefaultVisualization(builder, meshcat)
    return builder.Build()

def run_simulation(sim_time_step=0.001, sim_time = 10.0):
    """Run full simulation if visualize=False."""
    diagram = create_sim_scene(sim_time_step)
    simulator = Simulator(diagram)  
    simulator.Initialize()
    simulator.set_target_realtime_rate(1.0) 
    meshcat.StartRecording()
    simulator.AdvanceTo(sim_time)
    meshcat.PublishRecording()
    # Save diagram imageu
    svg_data = diagram.GetGraphvizString(max_depth=2)
    graph = pydot.graph_from_dot_data(svg_data)[0]
    graph.write_png("figures/block_diagram_project.png")
    print("Block diagram saved as figures/block_diagram_project.png")

# ------------------ Main ------------------
run_simulation()
