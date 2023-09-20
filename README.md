# zeroCity

https://github.com/W298/zeroCity/assets/25034289/103bf520-ef6c-4538-b821-18c3b16fcd1e

**Procedural City-Generation** Project.

This project is a sub-project of the [FindingYonghee](https://youtu.be/IbnIGOHuBFU?si=1Ml9hwgt_yT7VyK0) project.

- `FindingYonghee` project has further implemented a Crowd Simulation based on Reinforcement Learning.
- This project only handles City-Generation.

## About the code

- Every City-Generating Code is at [CityGenerator.cs](https://github.com/W298/zeroCity/blob/master/Assets/CityGeneration/Scripts/CityGenerator.cs) file.
- You can download executable file from [Release](https://github.com/W298/zeroCity/releases).

_[Warning] Because this project was developed with paid assets (3D models, toolkits), we intentionally did not upload them to this repository, so even if you download those assets and run the project, it may not work properly. If you want to test this project, please use [released executable file](https://github.com/W298/zeroCity/releases)._

**Dependencies**

- Included

  - [csDelaunay](https://github.com/PouletFrit/csDelaunay)
  - [Weighted Item Randomizer](https://github.com/BlueRaja/Weighted-Item-Randomizer-for-C-Sharp)
  - [MathNet.Numerics](https://www.nuget.org/packages/MathNet.Numerics/)

- Not Included
  - [Dreamteck Splines](https://assetstore.unity.com/packages/tools/utilities/dreamteck-splines-61926)
  - [Module Based City Pack](https://assetstore.unity.com/packages/3d/environments/urban/module-based-city-pack-154302)

> Reference Paper  
> [Citygen: An Interactive System for Procedural City Generation](https://www.citygen.net/files/citygen_gdtw07.pdf)

## Generation Step Description

### Table of Contents

- [Generate Road and City Cell Data](#generate-road-and-city-cell-data)

1. Generate Initial Point & Voronoi Diagram
2. Set Boundary of Voronoi as Primary Road
3. Group Voronoi as City Cell & Set City Cell Boundary as Primary Road
4. Generate Secondary Road & Graph

- [Generate Road Mesh](#generate-road-mesh)

5. Generate Primary Road Spline
6. Generate Secondary Road Spline
7. Reposition Connector

- [Generate Buildings](#generate-buildings)

8. Generate Primary Buildings
9. Generate Secondary Buildings

### Generate Road and City Cell Data

<img width="700" alt="shapes1" src="https://github.com/W298/zeroCity/assets/25034289/97dce9c1-cbd8-4e82-b4d6-6dbb3c6ebc5d">

#### 1. Generate Initial Point & Voronoi Diagram

- Generate initial points with an appropriate distribution using `Poisson disk sampling` and use them to create a Voronoi diagram.

- Afther that, use `Lloyd's Algorithm` to relax Voronoi Diagram with specified iteration count.

#### 2. Set Boundary of Voronoi as Primary Road

- Set the outermost edges of the Voronoi Diagram as the `Primary Road`.

- `Primary Road` is a wide road, from which the `Secondary Road`, a narrower road, branches off.

#### 3. Group Voronoi as City Cell & Set City Cell Boundary as Primary Road

- First, find a bounding rectangle that contains all of the sites in the previously created Voronoi Diagram, and make it 3 times the distance between the centers of each site compared to original Voronoi Diagram, so that each site is larger.

- This will result in two Voronoi Diagrams, grouping all the `Lower Voronoi Diagram` sites inside the sites of the `Upper Voronoi Diagram` (the larger one), which we call `City Cells`.

  - `City Cell` is not a site in the `Upper Voronoi Diagram`, but rather to a set of sites in a grouped `Lower Voronoi Diagram`. In the following steps, the `City Cell` is used as the basic unit.

- After that, All of the outer edges of this `City Cell` are then set as Primary Roads.

#### 4. Generate Secondary Road & Graph

- Recursively create a `Secondary Road` from the Boundary of the `City Cell` created above.

<img width="350" alt="shapes" src="https://github.com/W298/zeroCity/assets/25034289/934fb7a2-0b2d-4f08-a381-3736381658e8">

- Each Loop creates a straight road, with the number and direction of roads to be created from the current vertex determined based on a specified `Probability`.
  - The directions are `Left`, `Forward`, and `Right`, and a `Random Angle Offset` is added to these to create various shapes of roads.

<img width="650" alt="shapes2" src="https://github.com/W298/zeroCity/assets/25034289/8d0de935-fd91-47f2-8f35-09f58483c307">

- Once we know which direction and what size roads will be generated, we need to validate that they are valid. Below, the new vertex is the end point of the road that will be created.

  - Check 01. Is the new vertex inside the site?
    - If not, cancel the road.
  - Check 02. Is the new vertex close enough to other vertices?
    - If the nearest vertex is already connected to more than 4 edges, cancel.
    - If any road intersects the current road, cancel.
    - Otherwise, merge the two vertices.
  - Check 03. Is the new vertex close enough to another edge?
    - If yes, divide the new vertex by the point where it intersects the nearest edge.
  - Check 04. Is the new road intersect with another road?
    - If yes, divide divide the intersecting roads. (new one and old one)
  - If none of the above conditions apply, create the road as is.

- Repeat this procedure until reached at specified `Dimension(Stack)`.

### Generate Road Mesh

<img width="700" alt="shapes3" src="https://github.com/W298/zeroCity/assets/25034289/941c7e1f-f721-4fbf-a458-3ae52abc90be">

#### 5. Generate Primary Road Spline

- Create the actual road mesh based on the road data we have created so far.

- Use `B-Spline` to connect the straight roads into a single spline, and if there is an intersection, place a `Pre-Defined Intersection Prefab` at the location and connect the splines.
  - At this time, if you create a spline using only the previously created points, it will look quite rough, so we create a natural spline by creating `Sub-Point`s using the vectors to the previous and next points based on the created points.
  - Also, if there are vertices that are too close, including sub-points, during the creation process, merge them to maintain the structure.

#### 6. Generate Secondary Road Spline

- Same as above generation.

#### 7. Reposition Connector

- Since the previously created road mesh does not fully match the road vertex data, it is possible that the created roads may go off-site or not be attached, so check and fix this.

### Generate Buildings

<img width="700" alt="shapes4" src="https://github.com/W298/zeroCity/assets/25034289/94e0c7d0-42c4-4f93-999f-e0a5baebaad6">

#### 8. Generate Primary Buildings

- `Primary Building` is a building that is created next to a `Primary Road`.

- It tries to place the building within a specified width area based on the vertex pair of the created `Primary Road Spline`.
  - If there is an overlap or insufficient space, the placement is canceled, and the rotation is set to face the road after placement.

#### 9. Generate Secondary Buildings

- `Secondary Building` is a building that is created next to a `Secondary Road`, placement method is the same as above.

- Additionally, `Parks` are placed at regular intervals, and if space is not available, buildings in that space are deleted and `Park` is placed.
