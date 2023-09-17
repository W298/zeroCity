using UnityEditor;
using UnityEngine;

namespace Assets.Editor
{
	[CustomEditor(typeof(CityGenerator))]
	public class CityGeneratorEditor : UnityEditor.Editor
	{
		public override void OnInspectorGUI()
		{
			base.OnInspectorGUI();

			var cityGenInstance = (CityGenerator)target;

			GUILayout.Space(20);
			GUILayout.Label("Step Actions");

			GUILayout.BeginHorizontal();
			if (GUILayout.Button("1. Generate Initial Point"))
			{
				cityGenInstance.GenerateInitialPoint();
			}
			else if (GUILayout.Button("2. Generate Voronoi"))
			{
				cityGenInstance.GenerateVoronoi();
			}

			GUILayout.EndHorizontal();

			GUILayout.BeginHorizontal();
			if (GUILayout.Button("3. Set Boundary As Primary Road"))
			{
				cityGenInstance.SetBoundaryAsPrimaryRoad();
			}
			else if (GUILayout.Button("4. Group Voronoi As City Cell"))
			{
				cityGenInstance.GroupVoronoiAsCityCell();
			}

			GUILayout.EndHorizontal();

			GUILayout.BeginHorizontal();
			if (GUILayout.Button("5. Set City Cell Boundary As Primary Road"))
			{
				cityGenInstance.SetCityCellBoundaryAsPrimaryRoad();
			}
			else if (GUILayout.Button("6. Refine City Cell"))
			{
				cityGenInstance.RefineCityCell();
			}

			GUILayout.EndHorizontal();

			GUILayout.BeginHorizontal();
			if (GUILayout.Button("7-1. Generate Secondary Road Routine"))
			{
				cityGenInstance.GenerateSecondaryRoadRoutine();
			}
			else if (GUILayout.Button("7-2. Generate Secondary Road Immediate"))
			{
				cityGenInstance.GenerateSecondaryRoad();
			}

			GUILayout.EndHorizontal();

			GUILayout.BeginHorizontal();
			if (GUILayout.Button("8. Generate Road Spline"))
			{
				cityGenInstance.GeneratePrimaryRoadSpline();
			}
			else if (GUILayout.Button("9. Generate Buildings"))
			{
				cityGenInstance.GenerateSecondaryBuildings();
			}

			GUILayout.EndHorizontal();

			if (GUILayout.Button("10. Display"))
			{
				cityGenInstance.Display();
			}

			GUILayout.Space(20);

			GUILayout.Label("Actions");
			GUILayout.BeginHorizontal();
			if (GUILayout.Button("Run All Routine"))
			{
				cityGenInstance.RunAll();
			}
			else if (GUILayout.Button("Run All Immediate"))
			{
				cityGenInstance.RunAllImmediate();
			}
			else if (GUILayout.Button("Clear"))
			{
				cityGenInstance.Clear();
			}

			GUILayout.EndHorizontal();
			GUILayout.Space(20);

			GUILayout.Label("Poisson-Disk Prop");
			cityGenInstance.radius = EditorGUILayout.Slider("    Radius", cityGenInstance.radius, 0, 100);
			cityGenInstance.sampleRegionSize.x =
				EditorGUILayout.IntSlider("    Region Scale X", (int)cityGenInstance.sampleRegionSize.x, 1, 2000);
			cityGenInstance.sampleRegionSize.y =
				EditorGUILayout.IntSlider("    Region Scale Y", (int)cityGenInstance.sampleRegionSize.y, 1, 2000);
			cityGenInstance.numSamplesBeforeRejection =
				EditorGUILayout.IntSlider("    Rejection", cityGenInstance.numSamplesBeforeRejection, 1, 100);

			GUILayout.Space(20);

			GUILayout.Label("Voronoi Prop");
			cityGenInstance.isLloydIterationEnabled =
				EditorGUILayout.Toggle("    Lloyd Iteration", cityGenInstance.isLloydIterationEnabled);
			EditorGUI.BeginDisabledGroup(!cityGenInstance.isLloydIterationEnabled);
			cityGenInstance.lloydIteration =
				EditorGUILayout.IntSlider("    Iteration Count", cityGenInstance.lloydIteration, 0, 100);
			EditorGUI.EndDisabledGroup();

			GUILayout.Space(20);

			GUILayout.Label("Secondary-Road Growth Prop");
			cityGenInstance.gridGrowthProp.minLen = EditorGUILayout.Slider("    Min Length",
				cityGenInstance.gridGrowthProp.minLen, 3.0f, 100.0f);
			cityGenInstance.gridGrowthProp.maxLen = EditorGUILayout.Slider("    Max Length",
				cityGenInstance.gridGrowthProp.maxLen, 3.0f, 100.0f);
			cityGenInstance.gridGrowthProp.depth =
				EditorGUILayout.IntSlider("    Depth", cityGenInstance.gridGrowthProp.depth, 0, 30);

			GUILayout.Space(20);

			GUILayout.Label("Secondary-Road Default Prob");
			cityGenInstance.gridGrowthProp.defaultProb.count[0] = EditorGUILayout.IntSlider("    Count 1",
				cityGenInstance.gridGrowthProp.defaultProb.count[0], 0, 100);
			cityGenInstance.gridGrowthProp.defaultProb.count[1] = EditorGUILayout.IntSlider("    Count 2",
				cityGenInstance.gridGrowthProp.defaultProb.count[1], 0, 100);
			cityGenInstance.gridGrowthProp.defaultProb.count[2] = EditorGUILayout.IntSlider("    Count 3",
				cityGenInstance.gridGrowthProp.defaultProb.count[2], 0, 100);

			GUILayout.Space(20);

			cityGenInstance.gridGrowthProp.defaultProb.direction[0] = EditorGUILayout.IntSlider("    Front",
				cityGenInstance.gridGrowthProp.defaultProb.direction[0], 0, 100);
			cityGenInstance.gridGrowthProp.defaultProb.direction[1] = EditorGUILayout.IntSlider("    Left",
				cityGenInstance.gridGrowthProp.defaultProb.direction[1], 0, 100);
			cityGenInstance.gridGrowthProp.defaultProb.direction[2] = EditorGUILayout.IntSlider("    Right",
				cityGenInstance.gridGrowthProp.defaultProb.direction[2], 0, 100);

			GUILayout.Space(20);

			GUILayout.Label("Secondary-Road Boundary Prop");
			cityGenInstance.gridGrowthBoundary.vertexMerge = EditorGUILayout.Slider("    Vertex Merge",
				cityGenInstance.gridGrowthBoundary.vertexMerge, 1.0f, 100.0f);
			cityGenInstance.gridGrowthBoundary.vertexDestroy = EditorGUILayout.Slider("    Vertex Destroy",
				cityGenInstance.gridGrowthBoundary.vertexDestroy, 1.0f, 100.0f);
			cityGenInstance.gridGrowthBoundary.edgeDivide = EditorGUILayout.Slider("    Edge Divide",
				cityGenInstance.gridGrowthBoundary.edgeDivide, 1.0f, 100.0f);
			cityGenInstance.gridGrowthBoundary.intersectMerge = EditorGUILayout.Slider("    Intersect Merge",
				cityGenInstance.gridGrowthBoundary.intersectMerge, 1.0f, 100.0f);

			GUILayout.Space(20);

			GUILayout.Label("Lot Generation Prop");
			cityGenInstance.lotOffset = EditorGUILayout.Slider("    Lot Offset",
				cityGenInstance.lotOffset, 1.0f, 100.0f);
			cityGenInstance.secondaryRoadWidth = EditorGUILayout.Slider("    Secondary Road Width",
				cityGenInstance.secondaryRoadWidth, 1.0f, 100.0f);

			GUILayout.Space(20);
			GUILayout.Label("Cell Size : " + cityGenInstance.cellSize);
		}
	}
}