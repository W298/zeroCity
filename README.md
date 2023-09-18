# zeroCity

## About this repository

**Procedural City-Generation** Project.

This project is a sub-project of the [FindingYonghee](https://youtu.be/IbnIGOHuBFU?si=1Ml9hwgt_yT7VyK0) project.

- `FindingYonghee` project has further implemented a Crowd Simulation based on Reinforcement Learning.
- This project only handles City-Generation.

**About the code**
- Every City-Generating Code is at [CityGenerator.cs](https://github.com/W298/zeroCity/blob/master/Assets/CityGeneration/Scripts/CityGenerator.cs) file.  
- You can download executable file from [Release](https://github.com/W298/zeroCity/releases).

_[Warning] Because this project was developed with paid assets (3D models, toolkits), we intentionally did not upload them to this repository, so even if you download those assets and run the project, it may not work properly. If you want to test this project, please use released executable file._

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

## Generation Step

**1. Generate Initial Point & Voronoi Diagram**

Generate initial points using `Poisson disk sampling` and use them to create a Voronoi diagram. 

**2. Set Boundary of Voronoi as Primary Road**

**3. Group Voronoi as City Cell & Set City Cell Boundary as Primary Road**

**4. Genrate Secondary Road & Graph**

**5. Generate Primary Road Spline**

**6. Generate Secondary Road Spline**

**7. Reposition Connector**

**8. Generate Primary Buildings**

**9. Generate Secondary Buildings** 
