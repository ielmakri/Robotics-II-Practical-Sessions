"""
Test file for inspecting models
--------------------
"""

import os
import pydot
from pydrake.geometry import StartMeshcat
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydrake.systems.framework import DiagramBuilder
from pydrake.visualization import AddDefaultVisualization, ModelVisualizer
from pydrake.systems.analysis import Simulator  

# ------------------ Settings ------------------
visualize = True  # True = only visualize, False = run full simulation
meshcat = StartMeshcat()
# Adjust the path to where the URDF is in your directory
model_path = os.path.join(
    "..", "tutorial_models", "descriptions", "robots", "humanoids", "atlas", "atlas_minimal_contact.urdf"
    # "..", "tutorial_models", "projects", "envtest.sdf"
)

def run_visualizer():
    """Minimal visualization using ModelVisualizer."""
    visualizer = ModelVisualizer(meshcat=meshcat)
    visualizer.parser().AddModelsFromUrl("file://" + os.path.abspath(model_path))
    visualizer.Run()

# ------------- Main ------------------
run_visualizer()

