# mapgen-toolbox-extension
Map Generalization Toolbox

## MapGen Toolbox Installation instructions

Copy the following files into OpenJUMPs "ext" folder:
- mapgen-toolbox-extension-2.0.jar
- jmat_5.0.jar

restart OpenJUMP.

You should find a new set of functions under the main menu item
Plugins > Generalization > ...

## License
For the license terms under which the software/toolbox can be used see the LICENSE file.


### Version 2.0 - 15 May 2021

This version is the adaptation of Version 1.1 to OpenJUMP 2.0 and JTS 1.18
It includes :
- upload to github
- mavenize
- deactivate LineSimplifyJTS15AlgorithmPlugIn (JTS 1.18 algo available in OpenJUMP Core)
- update imports (from org.vividsolutions.jts to org.locationtech.jts)
- generify most collections
- change most iterators to for loops
- change deprecated method (ex. geometry.clone to geometry.copy)
- clean redundant headers
- fix javado
- make many object attributes final and removed redundant initializations


### Version 1.1 - 11 Sept. 2013


### Version 1.0 - 2006

- minor updates: placed in correct OpenJUMP menu "Plugins" and improved spelling of Function names.
- updated the function "Square Selected Buildings", to check the outputs of the squaring algorithms. 
  For that a new value (allowed change in area) needs to be provided by the user. 
  Produced invalid geometries and squaring results with a large deviation in area from input building are now returned in a separate layer. 
  These problematic buildings should then be generalized manually by the user.
- added Bezier curve based Smoothing Plugin by Michael Michaud and Michael Bedward
- added Orthogonalize Plugin by Larry Becker 
