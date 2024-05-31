## Prerequisites

* Install some packages via `sudo dnf install python3-qt5 python3-pillow-qt python3-pyyaml`
* Get some measurement equipment to measure the field

## Create the Map

* Start the map tool gui by calling `./map_tool_gui.py`
* If necessary, enhance the window size to show all text boxes (should be 22)
* For every text field in the top part (above the button `Create Map-String`)
    * Click into the field (this will mark the corresponding distance in the map in red)
    * Measure the length of this distance
    * Enter it into the text field (in the unit meters)
* Click the button `Create Map-String` (will remove all red elements)
* If elements are in the field, which are not yet shown in the map
    * Change the map string manually, see description given by `./map_tool.py -h`
* Click `Save map.png` (will save the map and open a info window with the origin)
* Copy `map.png` to `cfg/maps`
* Edit `cfg/conf.d/amcl.yaml`
    * Change `map_file` point to the newly created map
    * Adjust `origin_x` and `origin_y` to the values given by the map tool
* Smile

## Notes

* The map string as well as the origin is written to `map_creation.txt`
* Upon saving the map all text field values are stored to `values.yaml` (back to defaults by deleting this file)
* Don't click any upper text field anymore when manually changing the map string (it will delete all your changes)
