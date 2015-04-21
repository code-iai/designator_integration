^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package designator_integration
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* package.xml updates
* Added support for human desigs in cpp.
* Added default value parameters for string and double values
* Renamed package designator_integration into designator_integration_cpp.
* Contributors: Georg Bartels, Jan Winkler

0.0.1 (2014-12-10)
------------------
* Fixed CMakeLists.txt
* Mostly cleaned up package.xml and CMakeLists.txt for deployment
* Changed data type to ''void'' instead of ''char'' to not confuse string interface
* Added convenience constructor and changed default designator type to 'OBJECT'
* Add quotation marks around printed strings
* Allow multiple definitions of values with the same key; also, support output for pose and posestamped
* Convenience constructor
* Added automatic conversion mechanisms between strings and floats when asking for the respective value from a key value pair (i.e. a designator)
* Added functions for getting a list of all defined keys in a designator
* Forgot to commit changes to other files
* Onlye send out example designator once
* Made the example node a bit more expressive in terms of sending out designator messages
* Added copying key value pairs and initializing designators from key value pairs
* fixed CMakeLists.txt and package.xml dependencies.
* The property type is now correctly set to `DATA' when data is placed in it.
* Properly integrated the use of the `data' field.
  Communicated designators can now carry the `data' field, which is a vector of bytes on ROS side and a char array on the C++ side. This might be useful for transferring binary data, such as images, textures, meshes, audio, etc.
* Added missing dependencies
* Merge branch 'master' of github.com:code-iai/designator_integration
* Corrected printing out designator key value pairs
* Added function to clear the contents of a designator
* Introduced type `pose' and added a few more convenience functions
* Greatly simplified interface and added a few checks for already existing keys
* Restructured code such that managing child nodes is not done in desigs AND kvps
* Added .gitignore and adapted to the new message format
* Added C++ classes for communicating via designators
* Contributors: Jan Winkler, Wiedemeyer
