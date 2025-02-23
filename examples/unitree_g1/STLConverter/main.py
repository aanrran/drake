import bpy
import os

# Set the input and output directories
input_directory = "meshes"
output_directory = "converted_meshes"

# Ensure the output directory exists
if not os.path.exists(output_directory):
    os.makedirs(output_directory)

# Get a list of all STL files in the input directory
stl_files = [f for f in os.listdir(input_directory) if f.lower().endswith('.stl')]

# Iterate over each STL file
for stl_file in stl_files:
    # Construct the full file path
    input_filepath = os.path.join(input_directory, stl_file)

    # Import the STL file with specified axis parameters
    bpy.ops.import_mesh.stl(filepath=input_filepath)

    # Get the imported object
    imported_object = bpy.context.selected_objects[0]

    # Remove duplicate vertices
    bpy.ops.object.mode_set(mode="EDIT")
    bpy.ops.mesh.remove_doubles(threshold=0.002)  # Merge very close vertices
    bpy.ops.object.mode_set(mode="OBJECT")

    # Apply mesh simplification (decimate modifier)
    bpy.context.view_layer.objects.active = imported_object
    modifier = imported_object.modifiers.new(name="Decimate", type='DECIMATE')
    modifier.ratio = 0.5  # Reduce to 50% of original polygons (adjustable)

    bpy.ops.object.modifier_apply(modifier="Decimate")



    # Define the output file path (change the extension to .obj)
    output_filename = os.path.splitext(stl_file)[0] + ".obj"
    output_filepath = os.path.join(output_directory, output_filename)

    # Select the imported object
    bpy.ops.object.select_all(action='DESELECT')
    imported_object.select_set(True)
    bpy.context.view_layer.objects.active = imported_object

    # Export the object to OBJ format with correct axis parameters
    bpy.ops.wm.obj_export(filepath=output_filepath, export_selected_objects=True, forward_axis='Y', up_axis='Z')

    # Delete the imported object to prepare for the next iteration
    bpy.ops.object.delete()
