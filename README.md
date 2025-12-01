# UIC OpenStreetMaps
This is a project for my Data Structures course at the University of Illinois Chicago (UIC).

### Project Summary
The main features of this project were to:
- Write and test a Graph data structure (`graph.h`), and a Dijkstra's algorithm implementation (`application.h`)
- Locate and use a header-only JSON parser to parse OpenStreetMap (OSM) data that has been converted to JSON format.
- Prompt an LLM to help write a function (`buildGraph()`) that parses the OSM data using the JSON parser.
- Implement my version of Dijkstra's algorithm to provide the shorest path between two points on UIC campus.

## Using The Program
1. Launch the server by running ```osm_server```
2. Open your browser to http://localhost:1251 to see the map once the server has loaded.
- You may move the map by clicking and dragging with your mouse.
3. Select the `Route` tab at the top of the website.
4. Click in either the `Location` or `Destination` sections.
- Once a Location or Destination has been chosen you can left-click on a UIC building or parking lot to save its location in the respective field.
- Choose a Location and a Destination.
5. Click on the `Find Route` button and a blue line representing the shortest path between your chosen locations will appear!
6. To stop the server press `CTRL+C` in the terminal.

# Compiling Files
1. `Make` and `clang++` are reqired.
2. The `make` command will automatically compile all targets. This should create three executable files:
- `osm_main`
- `osm_server`
- `osm_tests`
3. You may also compile and execute the above targets directly. For example, to compile and run the server you can do `make run_server`.
4. `make run_osm` and `osm_main` provides a Command Line Interface to interact with the map data, however it's more difficult to use compared to running the server.
5. `make test_all` and `osm_tests` runs a GoogleTest testing suite on the Graph and Dijkstra's algorithm implementations.
