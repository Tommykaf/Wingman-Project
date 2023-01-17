# Wingman

## Introduction
The purpose of this tool is to take a body and two wings, and attach them to each other, s.t. the wings are movable using an attache lever.

## Usage instructions
One must take a blender file with the body and wings placed in their endgoal positions for the final model, and run the script inside it with the matching input.

### The `.blend` file

The `.blend` file must include:
- The main body
- The two wings
  - Note that we assume that the wings are comprised of two parts each - the main wing and secondary wing
- The handle (Can be placed arbitrarily)

The model should be centered with the y axis as an axis of symmetry.


### The main function
The main function attaches two wings to the main body.
It can either:
- export to stl for printing
- generate and assemble the model in blender


Parameters for main:
- view_vs_export = 'v' for viewing in blender and 'e' for stl export
- main_wing* = the main part of the wing (that attaches to the body directly)
- another_wing* = the secondary part of the wing (that attaches to the main wing)
- start_point_on_first_wing = location the gear connecting the main wing to the body
- end_point_on_first_wing = locatio of the ggear that connects the two parts of the wing
- width_constraint = max diametar constraint on the gears
- angle1 = angle of main_wing
- angle2 = angle of another_wing
- normal = the normal to the wings' planes
- export_path = export_path for the stls 


## Notes
The script optimaizes the gears size and amount in the interval between the given endpoints, and takes into account an internal epsilon value.


Thus, the user's given coordinates can change infinetisemally for correctness/optimality of movement.

In addition, there are a few physical constraints such as diametral pitch on the gears (so it will be printable).

If the constraints cannot be satisfied, the code returns an error and exits
