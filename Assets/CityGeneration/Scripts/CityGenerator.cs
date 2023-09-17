using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using csDelaunay;
using Dreamteck.Splines;
using MathNet.Numerics.Distributions;
using Weighted_Randomizer;
using Edge = csDelaunay.Edge;
using Random = UnityEngine.Random;
using Object = System.Object;

public class RoadGraph<T>
{
	public List<T> vertexList;
	public List<UIndexedEdge> edgeList;

	public RoadGraph()
	{
		vertexList = new List<T>();
		edgeList = new List<UIndexedEdge>();
	}
}

public class Vertex2D
{
	public Vector2 position;
	public List<UIndexedEdge> connectedEdgeList;

	public Vertex2D(Vector2 position)
	{
		this.position = position;
		connectedEdgeList = new List<UIndexedEdge>();
	}
}

public class Vertex3D
{
	public Vector3 position;
	public List<UIndexedEdge> connectedEdgeList;

	public Vertex3D(Vector3 position)
	{
		this.position = position;
		connectedEdgeList = new List<UIndexedEdge>();
	}
}

public class UIndexedEdge
{
	public int v1Index;
	public int v2Index;

	public UIndexedEdge(int v1Index, int v2Index)
	{
		this.v1Index = v1Index;
		this.v2Index = v2Index;
	}

	public override int GetHashCode() => v1Index ^ v2Index;

	public override bool Equals(object obj)
	{
		if (obj == null) return false;
		if (GetType() != obj.GetType()) return false;

		var other = (UIndexedEdge)obj;

		return (v1Index == other.v1Index && v2Index == other.v2Index) ||
		       (v1Index == other.v2Index && v2Index == other.v1Index);
	}

	public static bool operator ==(UIndexedEdge edge1, UIndexedEdge edge2)
	{
		return Object.Equals(edge1, edge2);
	}

	public static bool operator !=(UIndexedEdge edge1, UIndexedEdge edge2)
	{
		return !Object.Equals(edge1, edge2);
	}
}

public class CityCell
{
	public List<Site> siteList;
	public List<Edge> boundaryEdgeList;
}

public class Probability
{
	public int[] count;
	public int[] direction;
}

public struct GridGrowthProp
{
	public float minLen;
	public float maxLen;
	public float[,] growthOffsetRange;
	public int depth;
	public Probability defaultProb;
}

public struct GridGrowthBoundary
{
	public float vertexMerge;
	public float vertexDestroy;
	public float edgeDivide;
	public float intersectMerge;
}

public class IndexedEdge
{
	public int startVertexIndex;
	public int endVertexIndex;
}

public class Vector2Edge
{
	public Vector2 startPoint;
	public Vector2 endPoint;
}

public class ConnectorInfo
{
	public Edge basePrimaryEdgeTemp;
	public UIndexedEdge basePrimaryEdge;

	public IndexedEdge connectorEdge;
	public CityCell connectorCityCell;

	public SplineComputer basePrimaryEdgeSpline;
	public SplineComputer connectorEdgeSpline;

	public ConnectorInfo(Edge basePrimaryEdgeTemp, IndexedEdge connectorEdge, CityCell connectorCityCell)
	{
		this.basePrimaryEdgeTemp = basePrimaryEdgeTemp;
		this.connectorEdge = connectorEdge;
		this.connectorCityCell = connectorCityCell;
	}
}

public class CityGenerator : MonoBehaviour
{
	private class Vec3DIndexPair
	{
		public Vector3 vec3D;
		public int index;

		public Vec3DIndexPair(Vector3 vec3D, int index)
		{
			this.vec3D = vec3D;
			this.index = index;
		}
	}

	private static CityGenerator _instance = null;
	public static CityGenerator Instance => _instance ??= FindObjectOfType<CityGenerator>();

	[SerializeField] private GameObject roadColliderPrefab;
	[SerializeField] private GameObject roadSplinePrefab;
	[SerializeField] private GameObject secondaryRoadSplinePrefab;
	[SerializeField] private GameObject yIntersectionPrefab;
	[SerializeField] private GameObject tIntersectionPrefab;
	[SerializeField] private GameObject debugCubePrefab;
	[SerializeField] private Material planeMaterial;

	private GameObject _roadSplineContainer = null;
	private GameObject _roadIntersectionContainer = null;
	private GameObject _roadColliderContainer = null;
	private GameObject _buildingContainer = null;

	private List<Vector2> _poissonPointList = new();
	private Dictionary<Vector2f, Site> _siteDict = new();
	private List<Edge> _edgeList = new();

	private readonly List<Site> _outerSiteList = new();
	private readonly List<Site> _innerSiteList = new();

	private List<Vector2> _dividerCenterList = new();
	private List<Edge> _dividerEdgeList = new();
	private List<Site> _dividerSiteList = new();
	private List<Vector2> _dividerBorder = new();

	private readonly List<Edge> _primaryRoadList = new();
	private readonly Dictionary<Site, List<Site>> _cityCellDict = new();
	private readonly List<Color> _cityCellColorList = new();
	private readonly List<CityCell> _cityCellList = new();

	private readonly Dictionary<CityCell, List<Vector2>> _secondaryRoadVertexDict = new();
	private readonly Dictionary<CityCell, List<IndexedEdge>> _secondaryRoadEdgeDict = new();
	private IEnumerator _secondaryRoadGenRoutine = null;

	private RoadGraph<Vertex2D> _primaryRoadGraph2D = new();
	private readonly Dictionary<CityCell, RoadGraph<Vertex2D>> _secondaryRoadGraph2DDict = new();

	private readonly List<SplineComputer> _primaryRoadSplineList = new();
	private readonly List<SplineComputer> _secondaryRoadSplineList = new();

	private readonly List<ConnectorInfo> _connectorInfoList = new();
	private readonly List<Tuple<GameObject, Vector3, Vector3>> _primaryBuildingInfoList = new();

	[HideInInspector] public float cellSize;

	[HideInInspector] public float radius = 90;
	[HideInInspector] public Vector2 sampleRegionSize = Vector2.one * 1350;
	[HideInInspector] public int numSamplesBeforeRejection = 30;

	[HideInInspector] public bool isLloydIterationEnabled = true;
	[HideInInspector] public int lloydIteration = 5;

	[HideInInspector] public GridGrowthProp gridGrowthProp = new()
	{
		minLen = 35.5f,
		maxLen = 65.5f,
		growthOffsetRange = new[,] { { -2.5f, 2.5f }, { 0f, 2.5f }, { 0f, 2.5f } },
		depth = 8,
		defaultProb = new Probability() { count = new[] { 30, 30, 40 }, direction = new[] { 30, 35, 35 } },
	};

	[HideInInspector] public GridGrowthBoundary gridGrowthBoundary = new()
	{
		vertexMerge = 14.4f,
		vertexDestroy = 22.5f,
		edgeDivide = 17.52f,
		intersectMerge = 17.52f
	};

	[HideInInspector] public float lotOffset = 10f;
	[HideInInspector] public float secondaryRoadWidth = 15f;

	public RoadGraph<Vertex2D> GetPrimaryRoadGraph2D()
	{
		return _primaryRoadGraph2D;
	}

	public RoadGraph<Vertex3D> GetPrimaryRoadGrpah3D()
	{
		var roadGraph = new RoadGraph<Vertex3D>
		{
			vertexList = _primaryRoadGraph2D.vertexList.Select(v => new Vertex3D(GetVec3DPoint(v.position))).ToList(),
			edgeList = _primaryRoadGraph2D.edgeList
		};

		return roadGraph;
	}

	public RoadGraph<Vertex2D> GetSecondaryRoadGraph2D(CityCell cityCell)
	{
		return _secondaryRoadGraph2DDict[cityCell];
	}

	public Dictionary<CityCell, RoadGraph<Vertex2D>> GetAllSecondaryRoadGraph2D()
	{
		return _secondaryRoadGraph2DDict;
	}

	public RoadGraph<Vertex3D> GetSecondaryRoadGraph3D(CityCell cityCell)
	{
		var roadGraph = new RoadGraph<Vertex3D>()
		{
			vertexList = _secondaryRoadGraph2DDict[cityCell].vertexList
				.Select(v => new Vertex3D(GetVec3DPoint(v.position))).ToList(),
			edgeList = _secondaryRoadGraph2DDict[cityCell].edgeList
		};

		return roadGraph;
	}

	public Dictionary<CityCell, RoadGraph<Vertex3D>> GetAllSecondaryRoadGraph3D()
	{
		var roadGraph3DDict = new Dictionary<CityCell, RoadGraph<Vertex3D>>();
		foreach (var (cityCell, _) in _secondaryRoadGraph2DDict)
		{
			roadGraph3DDict.Add(cityCell, GetSecondaryRoadGraph3D(cityCell));
		}

		return roadGraph3DDict;
	}

	public List<ConnectorInfo> GetConnectorInfoList()
	{
		return _connectorInfoList;
	}

	/*
	 *	Item1 - Building GameObject
	 *	Item2 - Forward Vector of Building
	 *	Item3 - Projected Position to Road
	 */
	public List<Tuple<GameObject, Vector3, Vector3>> GetPrimaryBuildingInfoList()
	{
		return _primaryBuildingInfoList;
	}

	public Vector3 GetVec3DPoint(Vector2 point)
	{
		var planeWidth = transform.localScale.x * transform.localScale.x;
		var divider = sampleRegionSize.x * (transform.localScale.x / 10f) / planeWidth;
		var offset = planeWidth / 2 * (1 / (transform.localScale.x / 10f));

		var point3D = new Vector3(point.x / divider - offset, 0, point.y / divider - offset);
		point3D = Quaternion.Euler(0, 180f, 0) * point3D;

		return point3D;
	}

	public Vector2 GetVec2DPoint(Vector3 point3D)
	{
		var planeWidth = transform.localScale.x * transform.localScale.x;
		var divider = sampleRegionSize.x * (transform.localScale.x / 10f) / planeWidth;
		var offset = planeWidth / 2 * (1 / (transform.localScale.x / 10f));

		var point2D = new Vector3((point3D.x - offset) * divider, 0, (point3D.z - offset) * divider);
		point2D = Quaternion.Euler(0, 180f, 0) * point2D;

		return new Vector2(point2D.x, point2D.z);
	}

	public Vector3 GetVec3DScale(Vector2 scale, float height)
	{
		var planeWidth = transform.localScale.x * transform.localScale.x;
		var divider = sampleRegionSize.x * 1.5f / planeWidth;

		var scale3D = new Vector3(scale.x / divider, height, scale.y / divider);

		return scale3D;
	}

	public Vector2 GetVec2DScale(Vector3 scale3D)
	{
		var planeWidth = transform.localScale.x * transform.localScale.x;
		var divider = sampleRegionSize.x * 1.5f / planeWidth;

		var scale = scale3D * divider;

		return new Vector2(scale.x, scale.z);
	}

	public void GenerateInitialPoint()
	{
		cellSize = ToolBox.PoissonDisk(radius, sampleRegionSize, numSamplesBeforeRejection, ref _poissonPointList);
	}

	public void GenerateVoronoi()
	{
		var voronoi = ToolBox.VoronoiDiagram(sampleRegionSize, isLloydIterationEnabled, lloydIteration,
			ref _poissonPointList);
		_siteDict = voronoi.SitesIndexedByLocation;
		_edgeList = voronoi.Edges;
	}

	public void SetBoundaryAsPrimaryRoad()
	{
		foreach (var kv in _siteDict)
		{
			var hasBoundary = kv.Value.Edges.Any(IsEdgeBoundary);
			if (hasBoundary)
			{
				_outerSiteList.Add(kv.Value);
			}
			else
			{
				_innerSiteList.Add(kv.Value);
			}
		}

		foreach (var outerSite in _outerSiteList)
		{
			foreach (var validEdge in outerSite.Edges.Where(edge => edge.ClippedEnds != null))
			{
				var nearSite = validEdge.LeftSite == outerSite ? validEdge.RightSite : validEdge.LeftSite;
				if (_outerSiteList.Contains(nearSite)) continue;

				_primaryRoadList.Add(validEdge);
			}
		}
	}

	public void GroupVoronoiAsCityCell()
	{
		var minX = float.MaxValue;
		var minY = float.MaxValue;
		var maxX = float.MinValue;
		var maxY = float.MinValue;

		var allPosList = new List<Vector2f>();
		_primaryRoadList.ForEach(r =>
		{
			allPosList.Add(r.ClippedEnds[LR.LEFT]);
			allPosList.Add(r.ClippedEnds[LR.RIGHT]);
		});

		foreach (var pos in allPosList)
		{
			if (pos.x < minX) minX = pos.x;
			else if (pos.x > maxX) maxX = pos.x;

			if (pos.y < minY) minY = pos.y;
			else if (pos.y > maxY) maxY = pos.y;
		}

		var regionSize = new Vector2(maxX - minX, maxY - minY);
		var regionCenter = new Vector2((minX + maxX) / 2, (minY + maxY) / 2);
		var oldCenter = regionSize / 2;
		var offset = regionCenter - oldCenter;

		ToolBox.PoissonDisk(radius * 3, regionSize, 30, ref _dividerCenterList);

		var voronoi = ToolBox.VoronoiDiagram(regionSize, true, 1, ref _dividerCenterList);
		_dividerSiteList = voronoi.SitesIndexedByLocation.Values.ToList();
		_dividerEdgeList = voronoi.Edges;

		for (var i = 0; i < _dividerCenterList.Count; i++)
		{
			_dividerCenterList[i] += new Vector2(offset.x, offset.y);
		}

		foreach (var site in _dividerSiteList)
		{
			site.Coord += new Vector2f(offset.x, offset.y);
		}

		foreach (var edge in _dividerEdgeList.Where(edge => edge.ClippedEnds != null))
		{
			edge.ClippedEnds[LR.LEFT] += new Vector2f(offset.x, offset.y);
			edge.ClippedEnds[LR.RIGHT] += new Vector2f(offset.x, offset.y);
		}

		_dividerBorder = new List<Vector2>()
		{
			new(minX, minY),
			new(maxX, minY),
			new(maxX, minY),
			new(maxX, maxY),
			new(maxX, maxY),
			new(minX, maxY),
			new(minX, maxY),
			new(minX, minY),
		};

		foreach (var divSite in _dividerSiteList)
		{
			_cityCellDict.Add(divSite, new List<Site>());
			foreach (var innerSite in _innerSiteList)
			{
				if (IsPointInSite(new Vector2(innerSite.x, innerSite.y), divSite))
				{
					if (!_cityCellDict[divSite].Contains(innerSite))
					{
						_cityCellDict[divSite].Add(innerSite);
					}
				}
			}
		}
	}

	public void SetCityCellBoundaryAsPrimaryRoad()
	{
		foreach (var (_, siteList) in _cityCellDict)
		{
			foreach (var site in siteList)
			{
				foreach (var siteEdge in site.Edges)
				{
					var nearSite = siteEdge.LeftSite == site ? siteEdge.RightSite : siteEdge.LeftSite;
					if (siteList.Contains(nearSite)) continue;
					if (!_primaryRoadList.Contains(siteEdge)) _primaryRoadList.Add(siteEdge);
				}
			}
		}
	}

	public void RefineCityCell()
	{
		foreach (var (_, siteList) in _cityCellDict)
		{
			var boundaryEdgeList = new List<Edge>();
			foreach (var site in siteList)
			{
				foreach (var siteEdge in site.Edges)
				{
					var nearSite = siteEdge.LeftSite == site ? siteEdge.RightSite : siteEdge.LeftSite;
					if (siteList.Contains(nearSite)) continue;
					if (!boundaryEdgeList.Contains(siteEdge)) boundaryEdgeList.Add(siteEdge);
				}
			}

			_cityCellList.Add(new CityCell()
			{
				siteList = siteList,
				boundaryEdgeList = boundaryEdgeList
			});
		}

		foreach (var cityCell in _cityCellList)
		{
			foreach (var edge in cityCell.boundaryEdgeList)
			{
				// lPoint
				var lPointPos = new Vector2(edge.ClippedEnds[LR.LEFT].x, edge.ClippedEnds[LR.LEFT].y);
				var lPointDupIndex = -1;

				for (var i = 0; i < _primaryRoadGraph2D.vertexList.Count; i++)
				{
					if (Vector2.Distance(_primaryRoadGraph2D.vertexList[i].position, lPointPos) > 0.01f) continue;

					lPointDupIndex = i;
					break;
				}

				if (lPointDupIndex == -1) _primaryRoadGraph2D.vertexList.Add(new Vertex2D(lPointPos));
				var lPointIndex = lPointDupIndex != -1 ? lPointDupIndex : _primaryRoadGraph2D.vertexList.Count - 1;

				// rPoint
				var rPointPos = new Vector2(edge.ClippedEnds[LR.RIGHT].x, edge.ClippedEnds[LR.RIGHT].y);
				var rPointDupIndex = -1;

				for (var i = 0; i < _primaryRoadGraph2D.vertexList.Count; i++)
				{
					if (Vector2.Distance(_primaryRoadGraph2D.vertexList[i].position, rPointPos) > 0.01f) continue;

					rPointDupIndex = i;
					break;
				}

				if (rPointDupIndex == -1) _primaryRoadGraph2D.vertexList.Add(new Vertex2D(rPointPos));
				var rPointIndex = rPointDupIndex != -1 ? rPointDupIndex : _primaryRoadGraph2D.vertexList.Count - 1;

				var e = new UIndexedEdge(lPointIndex, rPointIndex);
				if (!_primaryRoadGraph2D.edgeList.Contains(e))
				{
					_primaryRoadGraph2D.edgeList.Add(e);

					_primaryRoadGraph2D.vertexList[lPointIndex].connectedEdgeList.Add(e);
					_primaryRoadGraph2D.vertexList[rPointIndex].connectedEdgeList.Add(e);
				}
			}
		}
	}

	public IEnumerator GenerateSecondaryRoadRoutine()
	{
		_secondaryRoadGenRoutine = SecondaryRoadGenOuterLoopRoutine();
		yield return _secondaryRoadGenRoutine;
	}

	public void GenerateSecondaryRoad()
	{
		SecondaryRoadGenOuterLoop();
	}

	public void GenerateSecondaryRoadGraph()
	{
		foreach (var cityCell in _cityCellList)
		{
			var roadGraph = new RoadGraph<Vertex2D>();

			roadGraph.vertexList = _secondaryRoadVertexDict[cityCell].Select(vec2 => new Vertex2D(vec2)).ToList();
			roadGraph.edgeList = _secondaryRoadEdgeDict[cityCell]
				.Select(ie => new UIndexedEdge(ie.startVertexIndex, ie.endVertexIndex)).ToList();

			for (var i = 0; i < roadGraph.edgeList.Count; i++)
			{
				for (var j = 0; j < roadGraph.edgeList.Count; j++)
				{
					if (i == j) continue;

					if (roadGraph.edgeList[i] == roadGraph.edgeList[j])
					{
						roadGraph.edgeList[j] = null;
					}
				}
			}

			roadGraph.edgeList = roadGraph.edgeList.Where(e => e != null).ToList();

			for (var i = 0; i < roadGraph.vertexList.Count; i++)
			{
				roadGraph.vertexList[i].connectedEdgeList =
					roadGraph.edgeList.FindAll(e => e.v1Index == i || e.v2Index == i);
			}

			_secondaryRoadGraph2DDict.Add(cityCell, roadGraph);
		}
	}

	public IEnumerator GeneratePrimaryRoadSplineRoutine()
	{
		if (_roadSplineContainer == null)
		{
			_roadSplineContainer = new GameObject("RoadSplineContainer");
			_roadSplineContainer.transform.parent = transform;
		}

		if (_roadIntersectionContainer == null)
		{
			_roadIntersectionContainer = new GameObject("RoadIntersectionContainer");
			_roadIntersectionContainer.transform.parent = transform;
		}

		if (_roadColliderContainer == null)
		{
			_roadColliderContainer = new GameObject("RoadColliderContainer");
			_roadColliderContainer.transform.parent = transform;
		}

		foreach (var connectorInfo in _connectorInfoList)
		{
			var v1 = connectorInfo.basePrimaryEdgeTemp.ClippedEnds[LR.LEFT];
			var v2 = connectorInfo.basePrimaryEdgeTemp.ClippedEnds[LR.RIGHT];

			var v1v = new Vector2(v1.x, v1.y);
			var v2v = new Vector2(v2.x, v2.y);

			UIndexedEdge foundEdge = null;
			foreach (var uIndexedEdge in _primaryRoadGraph2D.edgeList)
			{
				var v1Pos = _primaryRoadGraph2D.vertexList[uIndexedEdge.v1Index].position;
				var v2Pos = _primaryRoadGraph2D.vertexList[uIndexedEdge.v2Index].position;

				if (Vector2.Distance(v1Pos, v1v) <= 0.01f && Vector2.Distance(v2Pos, v2v) <= 0.01f)
				{
					foundEdge = uIndexedEdge;
					break;
				}
			}

			connectorInfo.basePrimaryEdge = foundEdge;
		}

		var unsortedList = new List<List<UIndexedEdge>>();
		foreach (var v2 in _primaryRoadGraph2D.vertexList.Where(v => v.connectedEdgeList.Count == 2))
		{
			var left = v2.connectedEdgeList[0];
			var right = v2.connectedEdgeList[1];

			var leftIndex = -1;
			var rightIndex = -1;

			for (var i = 0; i < unsortedList.Count; i++)
			{
				if (unsortedList[i].Contains(left)) leftIndex = i;
				if (unsortedList[i].Contains(right)) rightIndex = i;

				if (leftIndex != -1 && rightIndex != -1) break;
			}

			if (leftIndex == -1 && rightIndex == -1)
			{
				unsortedList.Add(new List<UIndexedEdge>() { left, right });
			}
			else if (leftIndex == -1 && rightIndex != -1)
			{
				unsortedList[rightIndex].Add(left);
			}
			else if (leftIndex != -1 && rightIndex == -1)
			{
				unsortedList[leftIndex].Add(right);
			}
			else
			{
				unsortedList[leftIndex].AddRange(unsortedList[rightIndex]);
				unsortedList.RemoveAt(rightIndex);
			}
		}

		var sortedList = new List<List<UIndexedEdge>>();
		var splineList = new List<SplineComputer>();

		foreach (var edges in unsortedList)
		{
			var vertexList = new List<int>();
			foreach (var edge in edges)
			{
				if (!vertexList.Contains(edge.v1Index)) vertexList.Add(edge.v1Index);
				if (!vertexList.Contains(edge.v2Index)) vertexList.Add(edge.v2Index);
			}

			var v3 = -1;
			foreach (var edge in edges)
			{
				if (_primaryRoadGraph2D.vertexList[edge.v1Index].connectedEdgeList.Count == 3)
				{
					v3 = edge.v1Index;
					break;
				}

				if (_primaryRoadGraph2D.vertexList[edge.v2Index].connectedEdgeList.Count == 3)
				{
					v3 = edge.v2Index;
					break;
				}
			}

			var searchedList = new List<UIndexedEdge>();
			var cand = _primaryRoadGraph2D.vertexList[v3].connectedEdgeList
				.Where(e => vertexList.Contains(e.v1Index) && vertexList.Contains(e.v2Index) &&
							!searchedList.Contains(e)).ToList();

			while (cand.Count > 0)
			{
				searchedList.Add(new UIndexedEdge(v3, cand[0].v1Index == v3 ? cand[0].v2Index : cand[0].v1Index));

				v3 = cand[0].v1Index == v3 ? cand[0].v2Index : cand[0].v1Index;

				cand = _primaryRoadGraph2D.vertexList[v3].connectedEdgeList
					.Where(e => vertexList.Contains(e.v1Index) && vertexList.Contains(e.v2Index) &&
								!searchedList.Contains(e)).ToList();
			}

			sortedList.Add(searchedList);
		}

		foreach (var sortedEdgeList in sortedList)
		{
			yield return new WaitForSeconds(0.014f);

			var sortedPointList = new List<Vec3DIndexPair>();
			for (var i = 0; i < sortedEdgeList.Count; i++)
			{
				var edge = sortedEdgeList[i];

				if (i == 0)
					sortedPointList.Add(new Vec3DIndexPair(
						GetVec3DPoint(_primaryRoadGraph2D.vertexList[edge.v1Index].position), edge.v1Index));
				sortedPointList.Add(
					new Vec3DIndexPair(GetVec3DPoint(_primaryRoadGraph2D.vertexList[edge.v2Index].position),
						edge.v2Index));
			}

			// Modify Close Vertex Positions
			for (var i = 0; i < sortedPointList.Count - 1; i++)
			{
				if (Vector3.Distance(sortedPointList[i].vec3D, sortedPointList[i + 1].vec3D) > 8f) continue;

				var dir = sortedPointList[i + 1].vec3D - sortedPointList[i].vec3D;
				var center = (sortedPointList[i].vec3D + sortedPointList[i + 1].vec3D) / 2;
				sortedPointList[i].vec3D = center - dir.normalized * 4.2f;
				sortedPointList[i + 1].vec3D = center + dir.normalized * 4.2f;

				_primaryRoadGraph2D.vertexList[sortedPointList[i].index].position =
					GetVec2DPoint(sortedPointList[i].vec3D);
				_primaryRoadGraph2D.vertexList[sortedPointList[i + 1].index].position =
					GetVec2DPoint(sortedPointList[i + 1].vec3D);

				i++;
			}

			var obj = Instantiate(roadSplinePrefab, Vector3.zero, Quaternion.identity);
			obj.transform.parent = _roadSplineContainer.transform;

			var spline = obj.GetComponent<SplineComputer>();
			splineList.Add(spline);

			var cl = _connectorInfoList.FindAll(con => sortedEdgeList.Contains(con.basePrimaryEdge));
			foreach (var connectorInfo in cl)
			{
				connectorInfo.basePrimaryEdgeSpline = spline;
			}

			var cursor = 0;
			for (var i = 0; i < sortedPointList.Count; i++)
			{
				var bPos = sortedPointList[i].vec3D;

				if (i == 0 || i == sortedPointList.Count - 1)
				{
					spline.SetPoint(cursor++, new SplinePoint(bPos));
					continue;
				}

				var b2a = (sortedPointList[i - 1].vec3D - bPos);
				var b2c = (sortedPointList[i + 1].vec3D - bPos);

				var minMag = Mathf.Min(b2a.magnitude, b2c.magnitude);
				if (minMag < 2.5f)
				{
					spline.SetPoint(cursor++, new SplinePoint(bPos));
					continue;
				}

				var mag = Mathf.Clamp(minMag * 0.2f, 2.0f, 8.0f);

				var aPos = bPos + b2a.normalized * mag;
				var cPos = bPos + b2c.normalized * mag;

				if (Vector3.Angle(b2a, b2c) < 100)
				{
					aPos += b2a.normalized;
					cPos += b2c.normalized;

					var proj = Vector3.Project((bPos - aPos).normalized, (cPos - aPos).normalized);
					var dir = proj - (bPos - aPos);

					bPos += dir;

					_primaryRoadGraph2D.vertexList[sortedPointList[i].index].position = GetVec2DPoint(bPos);
				}

				spline.SetPoint(cursor++, new SplinePoint(aPos));
				spline.SetPoint(cursor++, new SplinePoint(bPos));
				spline.SetPoint(cursor++, new SplinePoint(cPos));
			}
		}

		var v3v3List = _primaryRoadGraph2D.edgeList.Where(e =>
			_primaryRoadGraph2D.vertexList[e.v1Index].connectedEdgeList.Count == 3 &&
			_primaryRoadGraph2D.vertexList[e.v2Index].connectedEdgeList.Count == 3).ToList();
		var v3v3SplineList = new List<SplineComputer>();

		foreach (var edge in v3v3List)
		{
			yield return new WaitForSeconds(0.014f);

			var obj = Instantiate(roadSplinePrefab, Vector3.zero, Quaternion.identity);
			obj.transform.parent = _roadSplineContainer.transform;

			var spline = obj.GetComponent<SplineComputer>();

			var c = _connectorInfoList.Find(con => con.basePrimaryEdge == edge);
			if (c != null) c.basePrimaryEdgeSpline = spline;

			var from = GetVec3DPoint(_primaryRoadGraph2D.vertexList[edge.v1Index].position);
			var to = GetVec3DPoint(_primaryRoadGraph2D.vertexList[edge.v2Index].position);

			if (Vector3.Distance(from, to) < 8f)
			{
				var dir = to - from;
				var center = (to + from) / 2;

				from = center - dir.normalized * 4.2f;
				to = center + dir.normalized * 4.2f;

				_primaryRoadGraph2D.vertexList[edge.v1Index].position = GetVec2DPoint(from);
				_primaryRoadGraph2D.vertexList[edge.v2Index].position = GetVec2DPoint(to);
			}

			spline.SetPoint(0, new SplinePoint(from));
			spline.SetPoint(1, new SplinePoint(to));

			v3v3SplineList.Add(spline);
		}

		var v3PointList = _primaryRoadGraph2D.vertexList.Where(v => v.connectedEdgeList.Count == 3).ToList();
		foreach (var v3 in v3PointList)
		{
			yield return new WaitForSeconds(0.014f);

			var dirList = new Vector3[3];
			for (var i = 0; i < v3.connectedEdgeList.Count; i++)
			{
				var id = sortedList.FindIndex(l => l.Contains(v3.connectedEdgeList[i]));

				if (id == -1)
				{
					id = v3v3List.FindIndex(e => e == v3.connectedEdgeList[i]);

					var v3v3HeadPos = v3v3SplineList[id].GetPointPosition(0);
					var v3v3TailPos = v3v3SplineList[id].GetPointPosition(v3v3SplineList[id].pointCount - 1);

					var v3v3IsHead = Vector3.Distance(v3v3HeadPos, GetVec3DPoint(v3.position)) <
									 Vector3.Distance(v3v3TailPos, GetVec3DPoint(v3.position));

					var v3v3Dir = v3v3IsHead
						? v3v3SplineList[id].GetPointPosition(1) - v3v3HeadPos
						: v3v3SplineList[id].GetPointPosition(v3v3SplineList[id].pointCount - 2) - v3v3TailPos;

					dirList[i] = v3v3Dir;

					continue;
				}

				var headPos = splineList[id].GetPointPosition(0);
				var tailPos = splineList[id].GetPointPosition(splineList[id].pointCount - 1);

				var isHead =
					Vector3.Distance(headPos, GetVec3DPoint(v3.position)) <
					Vector3.Distance(tailPos, GetVec3DPoint(v3.position));

				var dir = isHead
					? splineList[id].GetPointPosition(1) - headPos
					: splineList[id].GetPointPosition(splineList[id].pointCount - 2) - tailPos;

				dirList[i] = dir;
			}

			var useTIntersection = false;

			var splineMap = new int[3] { -1, -1, -1 };
			var otherPair = new Tuple<int, int>(-1, -1);
			var straightIndex = -1;

			for (var i = 0; i < dirList.Length; i++)
			{
				var angle = Vector3.Angle(dirList[i], dirList[(i + 1) % 3]);
				if (angle > 130f)
				{
					otherPair = new Tuple<int, int>(i, (i + 1) % 3);
					straightIndex = (i + 2) % 3;
					useTIntersection = true;
					break;
				}
			}

			if (!useTIntersection)
			{
				var minAngle = float.MaxValue;
				for (var i = 0; i < dirList.Length; i++)
				{
					var angle = Vector3.Angle(dirList[i], dirList[(i + 1) % 3]);
					if (angle < minAngle)
					{
						minAngle = angle;
						otherPair = new Tuple<int, int>(i, (i + 1) % 3);
					}
				}

				var tempList = new List<int> { 0, 1, 2 };
				straightIndex = tempList.Find(e => e != otherPair.Item1 && e != otherPair.Item2);
			}

			var intersectionObj = Instantiate(useTIntersection ? tIntersectionPrefab : yIntersectionPrefab,
				GetVec3DPoint(v3.position), Quaternion.identity);
			intersectionObj.transform.parent = _roadIntersectionContainer.transform;

			var intersectionSplineAry = intersectionObj.GetComponentsInChildren<SplineComputer>();

			var sdir = intersectionSplineAry[0].GetPointPosition(0) - intersectionSplineAry[0].GetPointPosition(1);
			intersectionObj.transform.rotation =
				Quaternion.Euler(0, Vector3.SignedAngle(sdir, dirList[straightIndex], Vector3.up), 0);

			var case1Error =
				Vector3.Angle(dirList[otherPair.Item1],
					intersectionSplineAry[1].GetPointPosition(0) - intersectionSplineAry[1].GetPointPosition(1)) +
				Vector3.Angle(dirList[otherPair.Item2],
					intersectionSplineAry[2].GetPointPosition(0) - intersectionSplineAry[2].GetPointPosition(1));

			var case2Error =
				Vector3.Angle(dirList[otherPair.Item2],
					intersectionSplineAry[1].GetPointPosition(0) - intersectionSplineAry[1].GetPointPosition(1)) +
				Vector3.Angle(dirList[otherPair.Item1],
					intersectionSplineAry[2].GetPointPosition(0) - intersectionSplineAry[2].GetPointPosition(1));

			splineMap[straightIndex] = 0;
			splineMap[otherPair.Item1] = case1Error < case2Error ? 1 : 2;
			splineMap[otherPair.Item2] = case1Error < case2Error ? 2 : 1;

			for (var i = 0; i < splineMap.Length; i++)
			{
				DebugExtension.DebugArrow(
					GetVec3DPoint(v3.position),
					dirList[i].normalized * 2f,
					splineMap[i] == 0 ? Color.cyan : splineMap[i] == 1 ? Color.magenta : Color.yellow,
					1000f);
			}

			for (var i = 0; i < 3; i++)
			{
				DebugExtension.DebugArrow(
					GetVec3DPoint(v3.position),
					intersectionSplineAry[i].GetPointPosition(0) - intersectionSplineAry[i].GetPointPosition(1),
					Color.white, 1000f);
			}

			foreach (var splineComputer in intersectionSplineAry)
			{
				splineComputer.RebuildImmediate();
			}

			var sortedConnectedEdgeList = new List<UIndexedEdge> { null, null, null };
			for (var i = 0; i < v3.connectedEdgeList.Count; i++)
			{
				sortedConnectedEdgeList[splineMap[i]] = v3.connectedEdgeList[i];
			}

			for (var i = 0; i < sortedConnectedEdgeList.Count; i++)
			{
				var id = sortedList.FindIndex(l => l.Contains(sortedConnectedEdgeList[i]));

				// vertex in v3v3List
				if (id == -1)
				{
					id = v3v3List.FindIndex(e => e == sortedConnectedEdgeList[i]);

					if (id == -1)
					{
						Debug.LogError("ERROR");
						yield break;
					}

					// TODO : Handle Y road left / right mismatching problem.
					var v3v3HeadPos = v3v3SplineList[id].GetPointPosition(0);
					var v3v3TailPos = v3v3SplineList[id].GetPointPosition(v3v3SplineList[id].pointCount - 1);

					if (Vector3.Distance(v3v3HeadPos, GetVec3DPoint(v3.position)) <
						Vector3.Distance(v3v3TailPos, GetVec3DPoint(v3.position)))
					{
						var np = intersectionSplineAry[i].GetPointPosition(0);
						var anp = intersectionSplineAry[i].GetPointPosition(1);

						v3v3SplineList[id].SetPointPosition(0, np);

						var lastIndex = v3v3SplineList[id].pointCount - 1;
						for (var j = lastIndex; j >= 1; j--)
						{
							if (j == lastIndex)
								v3v3SplineList[id].SetPoint(j + 1,
									new SplinePoint(v3v3SplineList[id].GetPointPosition(j)));
							v3v3SplineList[id].SetPointPosition(j + 1, v3v3SplineList[id].GetPointPosition(j));
						}

						v3v3SplineList[id].SetPointPosition(1, np + (np - anp).normalized * 2f);
					}
					else
					{
						var np = intersectionSplineAry[i].GetPointPosition(0);
						var anp = intersectionSplineAry[i].GetPointPosition(1);

						v3v3SplineList[id].SetPointPosition(v3v3SplineList[id].pointCount - 1,
							np + (np - anp).normalized * 2f);
						v3v3SplineList[id].SetPoint(v3v3SplineList[id].pointCount, new SplinePoint(np));
					}

					continue;
				}

				// vertex in sortedList
				var headPos = splineList[id].GetPointPosition(0);
				var tailPos = splineList[id].GetPointPosition(splineList[id].pointCount - 1);

				if (Vector3.Distance(headPos, GetVec3DPoint(v3.position)) <
					Vector3.Distance(tailPos, GetVec3DPoint(v3.position)))
				{
					var np = intersectionSplineAry[i].GetPointPosition(0);
					var anp = intersectionSplineAry[i].GetPointPosition(1);

					splineList[id].SetPointPosition(0, np);

					var lastIndex = splineList[id].pointCount - 1;
					for (var j = lastIndex; j >= 1; j--)
					{
						if (j == lastIndex)
							splineList[id].SetPoint(j + 1, new SplinePoint(splineList[id].GetPointPosition(j)));
						splineList[id].SetPointPosition(j + 1, splineList[id].GetPointPosition(j));
					}

					splineList[id].SetPointPosition(1, np + (np - anp).normalized * 2f);
				}
				else
				{
					var np = intersectionSplineAry[i].GetPointPosition(0);
					var anp = intersectionSplineAry[i].GetPointPosition(1);

					splineList[id].SetPointPosition(splineList[id].pointCount - 1, np + (np - anp).normalized * 2f);
					splineList[id].SetPoint(splineList[id].pointCount, new SplinePoint(np));
				}
			}

			// Post processing
			foreach (var splineComputer in intersectionSplineAry)
			{
				splineComputer.RebuildImmediate();

				var collection = new SampleCollection();
				splineComputer.GetSamples(collection);
				foreach (var sample in collection.samples)
				{
					var obj = Instantiate(roadColliderPrefab, sample.position, sample.rotation);
					obj.transform.parent = _roadColliderContainer.transform;
				}

				_primaryRoadSplineList.Add(splineComputer);
			}
		}

		// Post processing
		foreach (var splineComputer in splineList)
		{
			splineComputer.RebuildImmediate();

			var collection = new SampleCollection();
			splineComputer.GetSamples(collection);
			foreach (var sample in collection.samples)
			{
				var obj = Instantiate(roadColliderPrefab, sample.position, sample.rotation);
				obj.transform.parent = _roadColliderContainer.transform;
			}

			_primaryRoadSplineList.Add(splineComputer);
		}

		// Post processing
		foreach (var splineComputer in v3v3SplineList)
		{
			splineComputer.RebuildImmediate();

			var collection = new SampleCollection();
			splineComputer.GetSamples(collection);
			foreach (var sample in collection.samples)
			{
				var obj = Instantiate(roadColliderPrefab, sample.position, sample.rotation);
				obj.transform.parent = _roadColliderContainer.transform;
			}

			_primaryRoadSplineList.Add(splineComputer);
		}
	}

	public void GeneratePrimaryRoadSpline()
	{
		if (_roadSplineContainer == null)
		{
			_roadSplineContainer = new GameObject("RoadSplineContainer");
			_roadSplineContainer.transform.parent = transform;
		}

		if (_roadIntersectionContainer == null)
		{
			_roadIntersectionContainer = new GameObject("RoadIntersectionContainer");
			_roadIntersectionContainer.transform.parent = transform;
		}

		if (_roadColliderContainer == null)
		{
			_roadColliderContainer = new GameObject("RoadColliderContainer");
			_roadColliderContainer.transform.parent = transform;
		}

		foreach (var connectorInfo in _connectorInfoList)
		{
			var v1 = connectorInfo.basePrimaryEdgeTemp.ClippedEnds[LR.LEFT];
			var v2 = connectorInfo.basePrimaryEdgeTemp.ClippedEnds[LR.RIGHT];

			var v1v = new Vector2(v1.x, v1.y);
			var v2v = new Vector2(v2.x, v2.y);

			UIndexedEdge foundEdge = null;
			foreach (var uIndexedEdge in _primaryRoadGraph2D.edgeList)
			{
				var v1Pos = _primaryRoadGraph2D.vertexList[uIndexedEdge.v1Index].position;
				var v2Pos = _primaryRoadGraph2D.vertexList[uIndexedEdge.v2Index].position;

				if (Vector2.Distance(v1Pos, v1v) <= 0.01f && Vector2.Distance(v2Pos, v2v) <= 0.01f)
				{
					foundEdge = uIndexedEdge;
					break;
				}
			}

			connectorInfo.basePrimaryEdge = foundEdge;
		}

		var unsortedList = new List<List<UIndexedEdge>>();
		foreach (var v2 in _primaryRoadGraph2D.vertexList.Where(v => v.connectedEdgeList.Count == 2))
		{
			var left = v2.connectedEdgeList[0];
			var right = v2.connectedEdgeList[1];

			var leftIndex = -1;
			var rightIndex = -1;

			for (var i = 0; i < unsortedList.Count; i++)
			{
				if (unsortedList[i].Contains(left)) leftIndex = i;
				if (unsortedList[i].Contains(right)) rightIndex = i;

				if (leftIndex != -1 && rightIndex != -1) break;
			}

			if (leftIndex == -1 && rightIndex == -1)
			{
				unsortedList.Add(new List<UIndexedEdge>() { left, right });
			}
			else if (leftIndex == -1 && rightIndex != -1)
			{
				unsortedList[rightIndex].Add(left);
			}
			else if (leftIndex != -1 && rightIndex == -1)
			{
				unsortedList[leftIndex].Add(right);
			}
			else
			{
				unsortedList[leftIndex].AddRange(unsortedList[rightIndex]);
				unsortedList.RemoveAt(rightIndex);
			}
		}

		var sortedList = new List<List<UIndexedEdge>>();
		var splineList = new List<SplineComputer>();

		foreach (var edges in unsortedList)
		{
			var vertexList = new List<int>();
			foreach (var edge in edges)
			{
				if (!vertexList.Contains(edge.v1Index)) vertexList.Add(edge.v1Index);
				if (!vertexList.Contains(edge.v2Index)) vertexList.Add(edge.v2Index);
			}

			var v3 = -1;
			foreach (var edge in edges)
			{
				if (_primaryRoadGraph2D.vertexList[edge.v1Index].connectedEdgeList.Count == 3)
				{
					v3 = edge.v1Index;
					break;
				}

				if (_primaryRoadGraph2D.vertexList[edge.v2Index].connectedEdgeList.Count == 3)
				{
					v3 = edge.v2Index;
					break;
				}
			}

			var searchedList = new List<UIndexedEdge>();
			var cand = _primaryRoadGraph2D.vertexList[v3].connectedEdgeList
				.Where(e => vertexList.Contains(e.v1Index) && vertexList.Contains(e.v2Index) &&
				            !searchedList.Contains(e)).ToList();

			while (cand.Count > 0)
			{
				searchedList.Add(new UIndexedEdge(v3, cand[0].v1Index == v3 ? cand[0].v2Index : cand[0].v1Index));

				v3 = cand[0].v1Index == v3 ? cand[0].v2Index : cand[0].v1Index;

				cand = _primaryRoadGraph2D.vertexList[v3].connectedEdgeList
					.Where(e => vertexList.Contains(e.v1Index) && vertexList.Contains(e.v2Index) &&
					            !searchedList.Contains(e)).ToList();
			}

			sortedList.Add(searchedList);
		}

		foreach (var sortedEdgeList in sortedList)
		{
			var sortedPointList = new List<Vec3DIndexPair>();
			for (var i = 0; i < sortedEdgeList.Count; i++)
			{
				var edge = sortedEdgeList[i];

				if (i == 0)
					sortedPointList.Add(new Vec3DIndexPair(
						GetVec3DPoint(_primaryRoadGraph2D.vertexList[edge.v1Index].position), edge.v1Index));
				sortedPointList.Add(
					new Vec3DIndexPair(GetVec3DPoint(_primaryRoadGraph2D.vertexList[edge.v2Index].position),
						edge.v2Index));
			}

			// Modify Close Vertex Positions
			for (var i = 0; i < sortedPointList.Count - 1; i++)
			{
				if (Vector3.Distance(sortedPointList[i].vec3D, sortedPointList[i + 1].vec3D) > 8f) continue;

				var dir = sortedPointList[i + 1].vec3D - sortedPointList[i].vec3D;
				var center = (sortedPointList[i].vec3D + sortedPointList[i + 1].vec3D) / 2;
				sortedPointList[i].vec3D = center - dir.normalized * 4.2f;
				sortedPointList[i + 1].vec3D = center + dir.normalized * 4.2f;

				_primaryRoadGraph2D.vertexList[sortedPointList[i].index].position =
					GetVec2DPoint(sortedPointList[i].vec3D);
				_primaryRoadGraph2D.vertexList[sortedPointList[i + 1].index].position =
					GetVec2DPoint(sortedPointList[i + 1].vec3D);

				i++;
			}

			var obj = Instantiate(roadSplinePrefab, Vector3.zero, Quaternion.identity);
			obj.transform.parent = _roadSplineContainer.transform;

			var spline = obj.GetComponent<SplineComputer>();
			splineList.Add(spline);

			var cl = _connectorInfoList.FindAll(con => sortedEdgeList.Contains(con.basePrimaryEdge));
			foreach (var connectorInfo in cl)
			{
				connectorInfo.basePrimaryEdgeSpline = spline;
			}

			var cursor = 0;
			for (var i = 0; i < sortedPointList.Count; i++)
			{
				var bPos = sortedPointList[i].vec3D;

				if (i == 0 || i == sortedPointList.Count - 1)
				{
					spline.SetPoint(cursor++, new SplinePoint(bPos));
					continue;
				}

				var b2a = (sortedPointList[i - 1].vec3D - bPos);
				var b2c = (sortedPointList[i + 1].vec3D - bPos);

				var minMag = Mathf.Min(b2a.magnitude, b2c.magnitude);
				if (minMag < 5f)
				{
					spline.SetPoint(cursor++, new SplinePoint(bPos));
					continue;
				}

				var mag = Mathf.Clamp(minMag * 0.2f, 2.5f, 8.0f);

				var aPos = bPos + b2a.normalized * mag;
				var cPos = bPos + b2c.normalized * mag;

				if (Vector3.Angle(b2a, b2c) < 100)
				{
					aPos += b2a.normalized;
					cPos += b2c.normalized;

					var proj = Vector3.Project((bPos - aPos).normalized, (cPos - aPos).normalized);
					var dir = proj - (bPos - aPos);

					bPos += dir;

					_primaryRoadGraph2D.vertexList[sortedPointList[i].index].position = GetVec2DPoint(bPos);
				}

				if (Vector3.Distance(spline.GetPointPosition(cursor - 1), aPos) > 4.5f)
				{
					_primaryRoadGraph2D.vertexList.Add(new Vertex2D(GetVec2DPoint(aPos)));
					_primaryRoadGraph2D.vertexList.Add(new Vertex2D(GetVec2DPoint(cPos)));
				}

				spline.SetPoint(cursor++, new SplinePoint(aPos));
				spline.SetPoint(cursor++, new SplinePoint(bPos));
				spline.SetPoint(cursor++, new SplinePoint(cPos));
			}
		}

		var v3v3List = _primaryRoadGraph2D.edgeList.Where(e =>
			_primaryRoadGraph2D.vertexList[e.v1Index].connectedEdgeList.Count == 3 &&
			_primaryRoadGraph2D.vertexList[e.v2Index].connectedEdgeList.Count == 3).ToList();
		var v3v3SplineList = new List<SplineComputer>();

		foreach (var edge in v3v3List)
		{
			var obj = Instantiate(roadSplinePrefab, Vector3.zero, Quaternion.identity);
			obj.transform.parent = _roadSplineContainer.transform;

			var spline = obj.GetComponent<SplineComputer>();

			var c = _connectorInfoList.Find(con => con.basePrimaryEdge == edge);
			if (c != null) c.basePrimaryEdgeSpline = spline;

			var from = GetVec3DPoint(_primaryRoadGraph2D.vertexList[edge.v1Index].position);
			var to = GetVec3DPoint(_primaryRoadGraph2D.vertexList[edge.v2Index].position);

			if (Vector3.Distance(from, to) < 8f)
			{
				var dir = to - from;
				var center = (to + from) / 2;

				from = center - dir.normalized * 4.2f;
				to = center + dir.normalized * 4.2f;

				_primaryRoadGraph2D.vertexList[edge.v1Index].position = GetVec2DPoint(from);
				_primaryRoadGraph2D.vertexList[edge.v2Index].position = GetVec2DPoint(to);
			}

			spline.SetPoint(0, new SplinePoint(from));
			spline.SetPoint(1, new SplinePoint(to));

			v3v3SplineList.Add(spline);
		}

		var v3PointList = _primaryRoadGraph2D.vertexList.Where(v => v.connectedEdgeList.Count == 3).ToList();
		foreach (var v3 in v3PointList)
		{
			var dirList = new Vector3[3];
			for (var i = 0; i < v3.connectedEdgeList.Count; i++)
			{
				var id = sortedList.FindIndex(l => l.Contains(v3.connectedEdgeList[i]));

				if (id == -1)
				{
					id = v3v3List.FindIndex(e => e == v3.connectedEdgeList[i]);

					var v3v3HeadPos = v3v3SplineList[id].GetPointPosition(0);
					var v3v3TailPos = v3v3SplineList[id].GetPointPosition(v3v3SplineList[id].pointCount - 1);

					var v3v3IsHead = Vector3.Distance(v3v3HeadPos, GetVec3DPoint(v3.position)) <
					                 Vector3.Distance(v3v3TailPos, GetVec3DPoint(v3.position));

					var v3v3Dir = v3v3IsHead
						? v3v3SplineList[id].GetPointPosition(1) - v3v3HeadPos
						: v3v3SplineList[id].GetPointPosition(v3v3SplineList[id].pointCount - 2) - v3v3TailPos;

					dirList[i] = v3v3Dir;

					continue;
				}

				var headPos = splineList[id].GetPointPosition(0);
				var tailPos = splineList[id].GetPointPosition(splineList[id].pointCount - 1);

				var isHead =
					Vector3.Distance(headPos, GetVec3DPoint(v3.position)) <
					Vector3.Distance(tailPos, GetVec3DPoint(v3.position));

				var dir = isHead
					? splineList[id].GetPointPosition(1) - headPos
					: splineList[id].GetPointPosition(splineList[id].pointCount - 2) - tailPos;

				dirList[i] = dir;
			}

			var useTIntersection = false;

			var splineMap = new int[3] { -1, -1, -1 };
			var otherPair = new Tuple<int, int>(-1, -1);
			var straightIndex = -1;

			for (var i = 0; i < dirList.Length; i++)
			{
				var angle = Vector3.Angle(dirList[i], dirList[(i + 1) % 3]);
				if (angle > 130f)
				{
					otherPair = new Tuple<int, int>(i, (i + 1) % 3);
					straightIndex = (i + 2) % 3;
					useTIntersection = true;
					break;
				}
			}

			if (!useTIntersection)
			{
				var minAngle = float.MaxValue;
				for (var i = 0; i < dirList.Length; i++)
				{
					var angle = Vector3.Angle(dirList[i], dirList[(i + 1) % 3]);
					if (angle < minAngle)
					{
						minAngle = angle;
						otherPair = new Tuple<int, int>(i, (i + 1) % 3);
					}
				}

				var tempList = new List<int> { 0, 1, 2 };
				straightIndex = tempList.Find(e => e != otherPair.Item1 && e != otherPair.Item2);
			}

			var intersectionObj = Instantiate(useTIntersection ? tIntersectionPrefab : yIntersectionPrefab,
				GetVec3DPoint(v3.position), Quaternion.identity);
			intersectionObj.transform.parent = _roadIntersectionContainer.transform;

			var intersectionSplineAry = intersectionObj.GetComponentsInChildren<SplineComputer>();

			var sdir = intersectionSplineAry[0].GetPointPosition(0) - intersectionSplineAry[0].GetPointPosition(1);
			intersectionObj.transform.rotation =
				Quaternion.Euler(0, Vector3.SignedAngle(sdir, dirList[straightIndex], Vector3.up), 0);

			var case1Error =
				Vector3.Angle(dirList[otherPair.Item1],
					intersectionSplineAry[1].GetPointPosition(0) - intersectionSplineAry[1].GetPointPosition(1)) +
				Vector3.Angle(dirList[otherPair.Item2],
					intersectionSplineAry[2].GetPointPosition(0) - intersectionSplineAry[2].GetPointPosition(1));

			var case2Error =
				Vector3.Angle(dirList[otherPair.Item2],
					intersectionSplineAry[1].GetPointPosition(0) - intersectionSplineAry[1].GetPointPosition(1)) +
				Vector3.Angle(dirList[otherPair.Item1],
					intersectionSplineAry[2].GetPointPosition(0) - intersectionSplineAry[2].GetPointPosition(1));

			splineMap[straightIndex] = 0;
			splineMap[otherPair.Item1] = case1Error < case2Error ? 1 : 2;
			splineMap[otherPair.Item2] = case1Error < case2Error ? 2 : 1;

			for (var i = 0; i < splineMap.Length; i++)
			{
				DebugExtension.DebugArrow(
					GetVec3DPoint(v3.position),
					dirList[i].normalized * 2f,
					splineMap[i] == 0 ? Color.cyan : splineMap[i] == 1 ? Color.magenta : Color.yellow,
					1000f);
			}

			for (var i = 0; i < 3; i++)
			{
				DebugExtension.DebugArrow(
					GetVec3DPoint(v3.position),
					intersectionSplineAry[i].GetPointPosition(0) - intersectionSplineAry[i].GetPointPosition(1),
					Color.white, 1000f);
			}

			foreach (var splineComputer in intersectionSplineAry)
			{
				splineComputer.RebuildImmediate();
			}

			var sortedConnectedEdgeList = new List<UIndexedEdge> { null, null, null };
			for (var i = 0; i < v3.connectedEdgeList.Count; i++)
			{
				sortedConnectedEdgeList[splineMap[i]] = v3.connectedEdgeList[i];
			}

			for (var i = 0; i < sortedConnectedEdgeList.Count; i++)
			{
				var id = sortedList.FindIndex(l => l.Contains(sortedConnectedEdgeList[i]));

				// vertex in v3v3List
				if (id == -1)
				{
					id = v3v3List.FindIndex(e => e == sortedConnectedEdgeList[i]);

					if (id == -1)
					{
						Debug.LogError("ERROR");
						return;
					}

					// TODO : Handle Y road left / right mismatching problem.
					var v3v3HeadPos = v3v3SplineList[id].GetPointPosition(0);
					var v3v3TailPos = v3v3SplineList[id].GetPointPosition(v3v3SplineList[id].pointCount - 1);

					if (Vector3.Distance(v3v3HeadPos, GetVec3DPoint(v3.position)) <
					    Vector3.Distance(v3v3TailPos, GetVec3DPoint(v3.position)))
					{
						var np = intersectionSplineAry[i].GetPointPosition(0);
						var anp = intersectionSplineAry[i].GetPointPosition(1);

						v3v3SplineList[id].SetPointPosition(0, np);

						var lastIndex = v3v3SplineList[id].pointCount - 1;
						for (var j = lastIndex; j >= 1; j--)
						{
							if (j == lastIndex)
								v3v3SplineList[id].SetPoint(j + 1,
									new SplinePoint(v3v3SplineList[id].GetPointPosition(j)));
							v3v3SplineList[id].SetPointPosition(j + 1, v3v3SplineList[id].GetPointPosition(j));
						}

						v3v3SplineList[id].SetPointPosition(1, np + (np - anp).normalized * 2f);
					}
					else
					{
						var np = intersectionSplineAry[i].GetPointPosition(0);
						var anp = intersectionSplineAry[i].GetPointPosition(1);

						v3v3SplineList[id].SetPointPosition(v3v3SplineList[id].pointCount - 1,
							np + (np - anp).normalized * 2f);
						v3v3SplineList[id].SetPoint(v3v3SplineList[id].pointCount, new SplinePoint(np));
					}

					continue;
				}

				// vertex in sortedList
				var headPos = splineList[id].GetPointPosition(0);
				var tailPos = splineList[id].GetPointPosition(splineList[id].pointCount - 1);

				if (Vector3.Distance(headPos, GetVec3DPoint(v3.position)) <
				    Vector3.Distance(tailPos, GetVec3DPoint(v3.position)))
				{
					var np = intersectionSplineAry[i].GetPointPosition(0);
					var anp = intersectionSplineAry[i].GetPointPosition(1);

					splineList[id].SetPointPosition(0, np);

					var lastIndex = splineList[id].pointCount - 1;
					for (var j = lastIndex; j >= 1; j--)
					{
						if (j == lastIndex)
							splineList[id].SetPoint(j + 1, new SplinePoint(splineList[id].GetPointPosition(j)));
						splineList[id].SetPointPosition(j + 1, splineList[id].GetPointPosition(j));
					}

					splineList[id].SetPointPosition(1, np + (np - anp).normalized * 2f);
				}
				else
				{
					var np = intersectionSplineAry[i].GetPointPosition(0);
					var anp = intersectionSplineAry[i].GetPointPosition(1);

					splineList[id].SetPointPosition(splineList[id].pointCount - 1, np + (np - anp).normalized * 2f);
					splineList[id].SetPoint(splineList[id].pointCount, new SplinePoint(np));
				}
			}

			// Post processing
			foreach (var splineComputer in intersectionSplineAry)
			{
				splineComputer.RebuildImmediate();

				var collection = new SampleCollection();
				splineComputer.GetSamples(collection);
				foreach (var sample in collection.samples)
				{
					var obj = Instantiate(roadColliderPrefab, sample.position, sample.rotation);
					obj.transform.parent = _roadColliderContainer.transform;
				}

				_primaryRoadSplineList.Add(splineComputer);
			}
		}

		// Post processing
		foreach (var splineComputer in splineList)
		{
			splineComputer.RebuildImmediate();

			var collection = new SampleCollection();
			splineComputer.GetSamples(collection);
			foreach (var sample in collection.samples)
			{
				var obj = Instantiate(roadColliderPrefab, sample.position, sample.rotation);
				obj.transform.parent = _roadColliderContainer.transform;
			}

			_primaryRoadSplineList.Add(splineComputer);
		}

		// Post processing
		foreach (var splineComputer in v3v3SplineList)
		{
			splineComputer.RebuildImmediate();

			var collection = new SampleCollection();
			splineComputer.GetSamples(collection);
			foreach (var sample in collection.samples)
			{
				var obj = Instantiate(roadColliderPrefab, sample.position, sample.rotation);
				obj.transform.parent = _roadColliderContainer.transform;
			}

			_primaryRoadSplineList.Add(splineComputer);
		}
	}

	public IEnumerator GenerateSecondaryRoadSplineRoutine()
	{
		if (_roadSplineContainer == null)
		{
			_roadSplineContainer = new GameObject("RoadSplineContainer");
			_roadSplineContainer.transform.parent = transform;
		}

		if (_roadIntersectionContainer == null)
		{
			_roadIntersectionContainer = new GameObject("RoadIntersectionContainer");
			_roadIntersectionContainer.transform.parent = transform;
		}

		if (_roadColliderContainer == null)
		{
			_roadColliderContainer = new GameObject("RoadColliderContainer");
			_roadColliderContainer.transform.parent = transform;
		}

		foreach (var cityCell in _cityCellList)
		{
			var roadGraph = _secondaryRoadGraph2DDict[cityCell];

			// Merge Edges
			var unsortedList = new List<List<UIndexedEdge>>();
			foreach (var v2 in roadGraph.vertexList.Where(v => v.connectedEdgeList.Count == 2))
			{
				var left = v2.connectedEdgeList[0];
				var right = v2.connectedEdgeList[1];

				var leftIndex = -1;
				var rightIndex = -1;

				for (var i = 0; i < unsortedList.Count; i++)
				{
					if (unsortedList[i].Contains(left)) leftIndex = i;
					if (unsortedList[i].Contains(right)) rightIndex = i;

					if (leftIndex != -1 && rightIndex != -1) break;
				}

				if (leftIndex == -1 && rightIndex == -1)
				{
					unsortedList.Add(new List<UIndexedEdge>() { left, right });
				}
				else if (leftIndex == -1 && rightIndex != -1)
				{
					unsortedList[rightIndex].Add(left);
				}
				else if (leftIndex != -1 && rightIndex == -1)
				{
					unsortedList[leftIndex].Add(right);
				}
				else
				{
					unsortedList[leftIndex].AddRange(unsortedList[rightIndex]);
					unsortedList.RemoveAt(rightIndex);
				}
			}

			// Sort Merged Edges
			var sortedList = new List<List<UIndexedEdge>>();
			foreach (var edges in unsortedList)
			{
				var vertexList = new List<int>();
				foreach (var edge in edges)
				{
					if (!vertexList.Contains(edge.v1Index)) vertexList.Add(edge.v1Index);
					if (!vertexList.Contains(edge.v2Index)) vertexList.Add(edge.v2Index);
				}

				var v3 = -1;
				foreach (var edge in edges)
				{
					if (roadGraph.vertexList[edge.v1Index].connectedEdgeList.Count >= 3 ||
						roadGraph.vertexList[edge.v1Index].connectedEdgeList.Count == 1)
					{
						v3 = edge.v1Index;
						break;
					}

					if (roadGraph.vertexList[edge.v2Index].connectedEdgeList.Count >= 3 ||
						roadGraph.vertexList[edge.v2Index].connectedEdgeList.Count == 1)
					{
						v3 = edge.v2Index;
						break;
					}
				}

				var searchedList = new List<UIndexedEdge>();
				var cand = roadGraph.vertexList[v3].connectedEdgeList
					.Where(e => vertexList.Contains(e.v1Index) && vertexList.Contains(e.v2Index) &&
								!searchedList.Contains(e) &&
								(roadGraph.vertexList[e.v1Index].connectedEdgeList.Count == 2 ||
								 roadGraph.vertexList[e.v2Index].connectedEdgeList.Count == 2)).ToList();

				while (cand.Count > 0)
				{
					searchedList.Add(new UIndexedEdge(v3, cand[0].v1Index == v3 ? cand[0].v2Index : cand[0].v1Index));

					v3 = cand[0].v1Index == v3 ? cand[0].v2Index : cand[0].v1Index;

					cand = roadGraph.vertexList[v3].connectedEdgeList
						.Where(e => vertexList.Contains(e.v1Index) && vertexList.Contains(e.v2Index) &&
									!searchedList.Contains(e) &&
									(roadGraph.vertexList[e.v1Index].connectedEdgeList.Count == 2 ||
									 roadGraph.vertexList[e.v2Index].connectedEdgeList.Count == 2)).ToList();
				}

				sortedList.Add(searchedList);
			}

			// Spawn Spline
			var splineList = new List<SplineComputer>();
			foreach (var sortedEdgeList in sortedList)
			{
				yield return new WaitForSeconds(0.014f);

				var sortedPointList = new List<Vec3DIndexPair>();
				for (var i = 0; i < sortedEdgeList.Count; i++)
				{
					var edge = sortedEdgeList[i];

					if (i == 0)
						sortedPointList.Add(new Vec3DIndexPair(
							GetVec3DPoint(roadGraph.vertexList[edge.v1Index].position), edge.v1Index));
					sortedPointList.Add(
						new Vec3DIndexPair(GetVec3DPoint(roadGraph.vertexList[edge.v2Index].position),
							edge.v2Index));
				}

				var obj = Instantiate(secondaryRoadSplinePrefab, Vector3.zero, Quaternion.identity);
				obj.transform.parent = _roadSplineContainer.transform;

				var spline = obj.GetComponent<SplineComputer>();
				splineList.Add(spline);

				var cl = _connectorInfoList.FindAll(con =>
					sortedEdgeList.Contains(new UIndexedEdge(con.connectorEdge.startVertexIndex,
						con.connectorEdge.endVertexIndex)) && con.connectorCityCell == cityCell);
				foreach (var connectorInfo in cl)
				{
					connectorInfo.connectorEdgeSpline = spline;
				}

				var cursor = 0;
				for (var i = 0; i < sortedPointList.Count; i++)
				{
					var bPos = sortedPointList[i].vec3D;

					if (i == 0 || i == sortedPointList.Count - 1)
					{
						spline.SetPoint(cursor++, new SplinePoint(bPos));
						continue;
					}

					var b2a = (sortedPointList[i - 1].vec3D - bPos);
					var b2c = (sortedPointList[i + 1].vec3D - bPos);

					var minMag = Mathf.Min(b2a.magnitude, b2c.magnitude);
					if (minMag < 2.5f)
					{
						spline.SetPoint(cursor++, new SplinePoint(bPos));
						continue;
					}

					var mag = Mathf.Clamp(minMag * 0.2f, 2.0f, 8.0f);

					var aPos = bPos + b2a.normalized * mag;
					var cPos = bPos + b2c.normalized * mag;

					spline.SetPoint(cursor++, new SplinePoint(aPos));
					spline.SetPoint(cursor++, new SplinePoint(bPos));
					spline.SetPoint(cursor++, new SplinePoint(cPos));
				}
			}

			var v3v3List = roadGraph.edgeList.Where(e =>
				roadGraph.vertexList[e.v1Index].connectedEdgeList.Count != 2 &&
				roadGraph.vertexList[e.v2Index].connectedEdgeList.Count != 2).ToList();
			var v3v3SplineList = new List<SplineComputer>();

			foreach (var edge in v3v3List)
			{
				yield return new WaitForSeconds(0.014f);

				var obj = Instantiate(secondaryRoadSplinePrefab, Vector3.zero, Quaternion.identity);
				obj.transform.parent = _roadSplineContainer.transform;

				var spline = obj.GetComponent<SplineComputer>();

				var c = _connectorInfoList.Find(con =>
					edge == new UIndexedEdge(con.connectorEdge.startVertexIndex, con.connectorEdge.endVertexIndex) &&
					con.connectorCityCell == cityCell);
				if (c != null) c.connectorEdgeSpline = spline;

				var from = GetVec3DPoint(roadGraph.vertexList[edge.v1Index].position);
				var to = GetVec3DPoint(roadGraph.vertexList[edge.v2Index].position);

				spline.SetPoint(0, new SplinePoint(from));
				spline.SetPoint(1, new SplinePoint(to));

				v3v3SplineList.Add(spline);
			}

			foreach (var v3 in roadGraph.vertexList.Where(v => v.connectedEdgeList.Count == 3))
			{
				var dirList = new Vector3[3];
				for (var i = 0; i < v3.connectedEdgeList.Count; i++)
				{
					var id = sortedList.FindIndex(l => l.Contains(v3.connectedEdgeList[i]));

					if (id == -1)
					{
						id = v3v3List.FindIndex(e => e == v3.connectedEdgeList[i]);

						var v3v3HeadPos = v3v3SplineList[id].GetPointPosition(0);
						var v3v3TailPos = v3v3SplineList[id].GetPointPosition(v3v3SplineList[id].pointCount - 1);

						var v3v3IsHead = Vector3.Distance(v3v3HeadPos, GetVec3DPoint(v3.position)) <
										 Vector3.Distance(v3v3TailPos, GetVec3DPoint(v3.position));

						var v3v3Dir = v3v3IsHead
							? v3v3SplineList[id].GetPointPosition(1) - v3v3HeadPos
							: v3v3SplineList[id].GetPointPosition(v3v3SplineList[id].pointCount - 2) - v3v3TailPos;

						dirList[i] = v3v3Dir;

						continue;
					}

					var headPos = splineList[id].GetPointPosition(0);
					var tailPos = splineList[id].GetPointPosition(splineList[id].pointCount - 1);

					var isHead =
						Vector3.Distance(headPos, GetVec3DPoint(v3.position)) <
						Vector3.Distance(tailPos, GetVec3DPoint(v3.position));

					var dir = isHead
						? splineList[id].GetPointPosition(1) - headPos
						: splineList[id].GetPointPosition(splineList[id].pointCount - 2) - tailPos;

					dirList[i] = dir;
				}

				var otherPair = new Tuple<int, int>(-1, -1);
				var straightIndex = -1;

				var angleList = new float[3];
				for (var i = 0; i < dirList.Length; i++)
				{
					var angle = Vector3.Angle(dirList[i], dirList[(i + 1) % 3]);
					angleList[i] = angle;
				}

				var maxAngle = 0f;
				var maxAngleIndex = -1;

				for (var i = 0; i < angleList.Length; i++)
				{
					if (angleList[i] > maxAngle)
					{
						maxAngle = angleList[i];
						maxAngleIndex = i;
					}
				}

				otherPair = new Tuple<int, int>(maxAngleIndex, (maxAngleIndex + 1) % 3);
				straightIndex = (maxAngleIndex + 2) % 3;

				var to1 = dirList[otherPair.Item1];
				var to2 = -to1.normalized * 1.5f;

				for (var i = 0; i < v3.connectedEdgeList.Count; i++)
				{
					var id = sortedList.FindIndex(l => l.Contains(v3.connectedEdgeList[i]));

					if (id == -1)
					{
						id = v3v3List.FindIndex(e => e == v3.connectedEdgeList[i]);

						var v3v3HeadPos = v3v3SplineList[id].GetPointPosition(0);
						var v3v3TailPos = v3v3SplineList[id].GetPointPosition(v3v3SplineList[id].pointCount - 1);

						var v3v3IsHead = Vector3.Distance(v3v3HeadPos, GetVec3DPoint(v3.position)) <
										 Vector3.Distance(v3v3TailPos, GetVec3DPoint(v3.position));

						v3v3SplineList[id].RebuildImmediate();
						var v3v3Per = 0.5f / v3v3SplineList[id].CalculateLength();
						var v3v3Mesh = v3v3SplineList[id].gameObject.GetComponent<SplineMesh>();

						if (v3v3IsHead)
						{
							for (var j = 1; j < 4; j++)
							{
								v3v3Mesh.GetChannel(j).clipFrom = v3v3Mesh.clipFrom + v3v3Per;
							}
						}
						else
						{
							for (var j = 1; j < 4; j++)
							{
								v3v3Mesh.GetChannel(j).clipTo = v3v3Mesh.clipTo - v3v3Per;
							}
						}

						v3v3SplineList[id].SetPointPosition(v3v3IsHead ? 0 : v3v3SplineList[id].pointCount - 1,
							(v3v3IsHead ? v3v3HeadPos : v3v3TailPos) + new Vector3(0, 0.0001f, 0) * i);
					}
					else
					{
						var hp = splineList[id].GetPointPosition(0);
						var tp = splineList[id].GetPointPosition(splineList[id].pointCount - 1);

						var H =
							Vector3.Distance(hp, GetVec3DPoint(v3.position)) <
							Vector3.Distance(tp, GetVec3DPoint(v3.position));

						splineList[id].RebuildImmediate();
						var per = 0.5f / splineList[id].CalculateLength();
						var mesh = splineList[id].gameObject.GetComponent<SplineMesh>();

						if (H)
						{
							for (var j = 1; j < 4; j++)
							{
								mesh.GetChannel(j).clipFrom = mesh.clipFrom + per;
							}
						}
						else
						{
							for (var j = 1; j < 4; j++)
							{
								mesh.GetChannel(j).clipTo = mesh.clipTo - per;
							}
						}

						splineList[id].SetPointPosition(H ? 0 : splineList[id].pointCount - 1,
							(H ? hp : tp) + new Vector3(0, 0.0001f, 0) * i);
					}

					id = sortedList.FindIndex(l => l.Contains(v3.connectedEdgeList[i]));
					if (i == otherPair.Item2)
					{
						if (id == -1)
						{
							id = v3v3List.FindIndex(e => e == v3.connectedEdgeList[i]);

							var v3v3HeadPos = v3v3SplineList[id].GetPointPosition(0);
							var v3v3TailPos = v3v3SplineList[id].GetPointPosition(v3v3SplineList[id].pointCount - 1);

							var v3v3IsHead = Vector3.Distance(v3v3HeadPos, GetVec3DPoint(v3.position)) <
											 Vector3.Distance(v3v3TailPos, GetVec3DPoint(v3.position));

							if (v3v3IsHead)
							{
								var lastIndex = v3v3SplineList[id].pointCount - 1;
								for (var j = lastIndex; j >= 1; j--)
								{
									if (j == lastIndex)
										v3v3SplineList[id].SetPoint(j + 1,
											new SplinePoint(v3v3SplineList[id].GetPointPosition(j)));
									v3v3SplineList[id].SetPointPosition(j + 1, v3v3SplineList[id].GetPointPosition(j));
								}

								v3v3SplineList[id].SetPointPosition(1, v3v3HeadPos + to2);
							}
							else
							{
								v3v3SplineList[id].SetPointPosition(v3v3SplineList[id].pointCount - 1,
									v3v3TailPos + to2);
								v3v3SplineList[id].SetPoint(v3v3SplineList[id].pointCount,
									new SplinePoint(v3v3TailPos));
							}

							continue;
						}

						var headPos = splineList[id].GetPointPosition(0);
						var tailPos = splineList[id].GetPointPosition(splineList[id].pointCount - 1);

						var isHead =
							Vector3.Distance(headPos, GetVec3DPoint(v3.position)) <
							Vector3.Distance(tailPos, GetVec3DPoint(v3.position));

						if (isHead)
						{
							var lastIndex = splineList[id].pointCount - 1;
							for (var j = lastIndex; j >= 1; j--)
							{
								if (j == lastIndex)
									splineList[id].SetPoint(j + 1, new SplinePoint(splineList[id].GetPointPosition(j)));
								splineList[id].SetPointPosition(j + 1, splineList[id].GetPointPosition(j));
							}

							splineList[id].SetPointPosition(1, headPos + to2);
						}
						else
						{
							splineList[id].SetPointPosition(splineList[id].pointCount - 1, tailPos + to2);
							splineList[id].SetPoint(splineList[id].pointCount, new SplinePoint(tailPos));
						}
					}
				}
			}

			foreach (var v4 in roadGraph.vertexList.Where(v => v.connectedEdgeList.Count == 4))
			{
				for (var i = 0; i < v4.connectedEdgeList.Count; i++)
				{
					var id = sortedList.FindIndex(l => l.Contains(v4.connectedEdgeList[i]));

					if (id == -1)
					{
						id = v3v3List.FindIndex(e => e == v4.connectedEdgeList[i]);

						var v3v3HeadPos = v3v3SplineList[id].GetPointPosition(0);
						var v3v3TailPos = v3v3SplineList[id].GetPointPosition(v3v3SplineList[id].pointCount - 1);

						var v3v3IsHead = Vector3.Distance(v3v3HeadPos, GetVec3DPoint(v4.position)) <
										 Vector3.Distance(v3v3TailPos, GetVec3DPoint(v4.position));

						v3v3SplineList[id].RebuildImmediate();
						var v3v3Per = 0.5f / v3v3SplineList[id].CalculateLength();
						var v3v3Mesh = v3v3SplineList[id].gameObject.GetComponent<SplineMesh>();

						if (v3v3IsHead)
						{
							for (var j = 1; j < 4; j++)
							{
								v3v3Mesh.GetChannel(j).clipFrom = v3v3Mesh.clipFrom + v3v3Per;
							}
						}
						else
						{
							for (var j = 1; j < 4; j++)
							{
								v3v3Mesh.GetChannel(j).clipTo = v3v3Mesh.clipTo - v3v3Per;
							}
						}

						v3v3SplineList[id].SetPointPosition(v3v3IsHead ? 0 : v3v3SplineList[id].pointCount - 1,
							(v3v3IsHead ? v3v3HeadPos : v3v3TailPos) + new Vector3(0, 0.0001f, 0) * i);
					}
					else
					{
						var hp = splineList[id].GetPointPosition(0);
						var tp = splineList[id].GetPointPosition(splineList[id].pointCount - 1);

						var H =
							Vector3.Distance(hp, GetVec3DPoint(v4.position)) <
							Vector3.Distance(tp, GetVec3DPoint(v4.position));

						splineList[id].RebuildImmediate();
						var per = 0.5f / splineList[id].CalculateLength();
						var mesh = splineList[id].gameObject.GetComponent<SplineMesh>();

						if (H)
						{
							for (var j = 1; j < 4; j++)
							{
								mesh.GetChannel(j).clipFrom = mesh.clipFrom + per;
							}
						}
						else
						{
							for (var j = 1; j < 4; j++)
							{
								mesh.GetChannel(j).clipTo = mesh.clipTo - per;
							}
						}

						splineList[id].SetPointPosition(H ? 0 : splineList[id].pointCount - 1,
							(H ? hp : tp) + new Vector3(0, 0.0001f, 0) * i);
					}
				}
			}

			// Post Processing
			foreach (var spline in splineList)
			{
				var mesh = spline.gameObject.GetComponent<SplineMesh>();

				var countTemp = new int[mesh.GetChannelCount()];
				for (var i = 0; i < countTemp.Length; i++)
				{
					countTemp[i] = mesh.GetChannel(i).count;
				}

				for (var i = 0; i < countTemp.Length; i++)
				{
					mesh.GetChannel(i).autoCount = false;
				}

				for (var i = 0; i < countTemp.Length; i++)
				{
					mesh.GetChannel(i).count = countTemp[i] * 3;
				}

				spline.RebuildImmediate();

				var collection = new SampleCollection();
				spline.GetSamples(collection);
				foreach (var sample in collection.samples)
				{
					var obj = Instantiate(roadColliderPrefab, sample.position, sample.rotation);
					obj.transform.localScale =
						new Vector3(0.8f, obj.transform.localScale.y, obj.transform.localScale.z);
					obj.transform.parent = _roadColliderContainer.transform;
				}

				_secondaryRoadSplineList.Add(spline);
			}

			// Post Processing
			foreach (var spline in v3v3SplineList)
			{
				spline.RebuildImmediate();

				var collection = new SampleCollection();
				spline.GetSamples(collection);
				foreach (var sample in collection.samples)
				{
					var obj = Instantiate(roadColliderPrefab, sample.position, sample.rotation);
					obj.transform.localScale =
						new Vector3(0.8f, obj.transform.localScale.y, obj.transform.localScale.z);
					obj.transform.parent = _roadColliderContainer.transform;
				}

				_secondaryRoadSplineList.Add(spline);
			}
		}
	}

	public void GenerateSecondaryRoadSpline()
	{
		if (_roadSplineContainer == null)
		{
			_roadSplineContainer = new GameObject("RoadSplineContainer");
			_roadSplineContainer.transform.parent = transform;
		}

		if (_roadIntersectionContainer == null)
		{
			_roadIntersectionContainer = new GameObject("RoadIntersectionContainer");
			_roadIntersectionContainer.transform.parent = transform;
		}

		if (_roadColliderContainer == null)
		{
			_roadColliderContainer = new GameObject("RoadColliderContainer");
			_roadColliderContainer.transform.parent = transform;
		}

		foreach (var cityCell in _cityCellList)
		{
			var roadGraph = _secondaryRoadGraph2DDict[cityCell];

			// Merge Edges
			var unsortedList = new List<List<UIndexedEdge>>();
			foreach (var v2 in roadGraph.vertexList.Where(v => v.connectedEdgeList.Count == 2))
			{
				var left = v2.connectedEdgeList[0];
				var right = v2.connectedEdgeList[1];

				var leftIndex = -1;
				var rightIndex = -1;

				for (var i = 0; i < unsortedList.Count; i++)
				{
					if (unsortedList[i].Contains(left)) leftIndex = i;
					if (unsortedList[i].Contains(right)) rightIndex = i;

					if (leftIndex != -1 && rightIndex != -1) break;
				}

				if (leftIndex == -1 && rightIndex == -1)
				{
					unsortedList.Add(new List<UIndexedEdge>() { left, right });
				}
				else if (leftIndex == -1 && rightIndex != -1)
				{
					unsortedList[rightIndex].Add(left);
				}
				else if (leftIndex != -1 && rightIndex == -1)
				{
					unsortedList[leftIndex].Add(right);
				}
				else
				{
					unsortedList[leftIndex].AddRange(unsortedList[rightIndex]);
					unsortedList.RemoveAt(rightIndex);
				}
			}

			// Sort Merged Edges
			var sortedList = new List<List<UIndexedEdge>>();
			foreach (var edges in unsortedList)
			{
				var vertexList = new List<int>();
				foreach (var edge in edges)
				{
					if (!vertexList.Contains(edge.v1Index)) vertexList.Add(edge.v1Index);
					if (!vertexList.Contains(edge.v2Index)) vertexList.Add(edge.v2Index);
				}

				var v3 = -1;
				foreach (var edge in edges)
				{
					if (roadGraph.vertexList[edge.v1Index].connectedEdgeList.Count >= 3 ||
					    roadGraph.vertexList[edge.v1Index].connectedEdgeList.Count == 1)
					{
						v3 = edge.v1Index;
						break;
					}

					if (roadGraph.vertexList[edge.v2Index].connectedEdgeList.Count >= 3 ||
					    roadGraph.vertexList[edge.v2Index].connectedEdgeList.Count == 1)
					{
						v3 = edge.v2Index;
						break;
					}
				}

				var searchedList = new List<UIndexedEdge>();
				var cand = roadGraph.vertexList[v3].connectedEdgeList
					.Where(e => vertexList.Contains(e.v1Index) && vertexList.Contains(e.v2Index) &&
					            !searchedList.Contains(e) &&
					            (roadGraph.vertexList[e.v1Index].connectedEdgeList.Count == 2 ||
					             roadGraph.vertexList[e.v2Index].connectedEdgeList.Count == 2)).ToList();

				while (cand.Count > 0)
				{
					searchedList.Add(new UIndexedEdge(v3, cand[0].v1Index == v3 ? cand[0].v2Index : cand[0].v1Index));

					v3 = cand[0].v1Index == v3 ? cand[0].v2Index : cand[0].v1Index;

					cand = roadGraph.vertexList[v3].connectedEdgeList
						.Where(e => vertexList.Contains(e.v1Index) && vertexList.Contains(e.v2Index) &&
						            !searchedList.Contains(e) &&
						            (roadGraph.vertexList[e.v1Index].connectedEdgeList.Count == 2 ||
						             roadGraph.vertexList[e.v2Index].connectedEdgeList.Count == 2)).ToList();
				}

				sortedList.Add(searchedList);
			}

			// Spawn Spline
			var splineList = new List<SplineComputer>();
			foreach (var sortedEdgeList in sortedList)
			{
				var sortedPointList = new List<Vec3DIndexPair>();
				for (var i = 0; i < sortedEdgeList.Count; i++)
				{
					var edge = sortedEdgeList[i];

					if (i == 0)
						sortedPointList.Add(new Vec3DIndexPair(
							GetVec3DPoint(roadGraph.vertexList[edge.v1Index].position), edge.v1Index));
					sortedPointList.Add(
						new Vec3DIndexPair(GetVec3DPoint(roadGraph.vertexList[edge.v2Index].position),
							edge.v2Index));
				}

				var obj = Instantiate(secondaryRoadSplinePrefab, Vector3.zero, Quaternion.identity);
				obj.transform.parent = _roadSplineContainer.transform;

				var spline = obj.GetComponent<SplineComputer>();
				splineList.Add(spline);

				var cl = _connectorInfoList.FindAll(con =>
					sortedEdgeList.Contains(new UIndexedEdge(con.connectorEdge.startVertexIndex,
						con.connectorEdge.endVertexIndex)) && con.connectorCityCell == cityCell);
				foreach (var connectorInfo in cl)
				{
					connectorInfo.connectorEdgeSpline = spline;
				}

				var cursor = 0;
				for (var i = 0; i < sortedPointList.Count; i++)
				{
					var bPos = sortedPointList[i].vec3D;

					if (i == 0 || i == sortedPointList.Count - 1)
					{
						spline.SetPoint(cursor++, new SplinePoint(bPos));
						continue;
					}

					var b2a = (sortedPointList[i - 1].vec3D - bPos);
					var b2c = (sortedPointList[i + 1].vec3D - bPos);

					var minMag = Mathf.Min(b2a.magnitude, b2c.magnitude);
					if (minMag < 2.5f)
					{
						spline.SetPoint(cursor++, new SplinePoint(bPos));
						continue;
					}

					var mag = Mathf.Clamp(minMag * 0.2f, 2.0f, 8.0f);

					var aPos = bPos + b2a.normalized * mag;
					var cPos = bPos + b2c.normalized * mag;

					spline.SetPoint(cursor++, new SplinePoint(aPos));
					spline.SetPoint(cursor++, new SplinePoint(bPos));
					spline.SetPoint(cursor++, new SplinePoint(cPos));
				}
			}

			var v3v3List = roadGraph.edgeList.Where(e =>
				roadGraph.vertexList[e.v1Index].connectedEdgeList.Count != 2 &&
				roadGraph.vertexList[e.v2Index].connectedEdgeList.Count != 2).ToList();
			var v3v3SplineList = new List<SplineComputer>();

			foreach (var edge in v3v3List)
			{
				var obj = Instantiate(secondaryRoadSplinePrefab, Vector3.zero, Quaternion.identity);
				obj.transform.parent = _roadSplineContainer.transform;

				var spline = obj.GetComponent<SplineComputer>();

				var c = _connectorInfoList.Find(con =>
					edge == new UIndexedEdge(con.connectorEdge.startVertexIndex, con.connectorEdge.endVertexIndex) &&
					con.connectorCityCell == cityCell);
				if (c != null) c.connectorEdgeSpline = spline;

				var from = GetVec3DPoint(roadGraph.vertexList[edge.v1Index].position);
				var to = GetVec3DPoint(roadGraph.vertexList[edge.v2Index].position);

				spline.SetPoint(0, new SplinePoint(from));
				spline.SetPoint(1, new SplinePoint(to));

				v3v3SplineList.Add(spline);
			}

			foreach (var v3 in roadGraph.vertexList.Where(v => v.connectedEdgeList.Count == 3))
			{
				var dirList = new Vector3[3];
				for (var i = 0; i < v3.connectedEdgeList.Count; i++)
				{
					var id = sortedList.FindIndex(l => l.Contains(v3.connectedEdgeList[i]));

					if (id == -1)
					{
						id = v3v3List.FindIndex(e => e == v3.connectedEdgeList[i]);

						var v3v3HeadPos = v3v3SplineList[id].GetPointPosition(0);
						var v3v3TailPos = v3v3SplineList[id].GetPointPosition(v3v3SplineList[id].pointCount - 1);

						var v3v3IsHead = Vector3.Distance(v3v3HeadPos, GetVec3DPoint(v3.position)) <
						                 Vector3.Distance(v3v3TailPos, GetVec3DPoint(v3.position));

						var v3v3Dir = v3v3IsHead
							? v3v3SplineList[id].GetPointPosition(1) - v3v3HeadPos
							: v3v3SplineList[id].GetPointPosition(v3v3SplineList[id].pointCount - 2) - v3v3TailPos;

						dirList[i] = v3v3Dir;

						continue;
					}

					var headPos = splineList[id].GetPointPosition(0);
					var tailPos = splineList[id].GetPointPosition(splineList[id].pointCount - 1);

					var isHead =
						Vector3.Distance(headPos, GetVec3DPoint(v3.position)) <
						Vector3.Distance(tailPos, GetVec3DPoint(v3.position));

					var dir = isHead
						? splineList[id].GetPointPosition(1) - headPos
						: splineList[id].GetPointPosition(splineList[id].pointCount - 2) - tailPos;

					dirList[i] = dir;
				}

				var otherPair = new Tuple<int, int>(-1, -1);
				var straightIndex = -1;

				var angleList = new float[3];
				for (var i = 0; i < dirList.Length; i++)
				{
					var angle = Vector3.Angle(dirList[i], dirList[(i + 1) % 3]);
					angleList[i] = angle;
				}

				var maxAngle = 0f;
				var maxAngleIndex = -1;

				for (var i = 0; i < angleList.Length; i++)
				{
					if (angleList[i] > maxAngle)
					{
						maxAngle = angleList[i];
						maxAngleIndex = i;
					}
				}

				otherPair = new Tuple<int, int>(maxAngleIndex, (maxAngleIndex + 1) % 3);
				straightIndex = (maxAngleIndex + 2) % 3;

				var to1 = dirList[otherPair.Item1];
				var to2 = -to1.normalized * 1.5f;

				for (var i = 0; i < v3.connectedEdgeList.Count; i++)
				{
					var id = sortedList.FindIndex(l => l.Contains(v3.connectedEdgeList[i]));

					if (id == -1)
					{
						id = v3v3List.FindIndex(e => e == v3.connectedEdgeList[i]);

						var v3v3HeadPos = v3v3SplineList[id].GetPointPosition(0);
						var v3v3TailPos = v3v3SplineList[id].GetPointPosition(v3v3SplineList[id].pointCount - 1);

						var v3v3IsHead = Vector3.Distance(v3v3HeadPos, GetVec3DPoint(v3.position)) <
						                 Vector3.Distance(v3v3TailPos, GetVec3DPoint(v3.position));

						v3v3SplineList[id].RebuildImmediate();
						var v3v3Per = 0.5f / v3v3SplineList[id].CalculateLength();
						var v3v3Mesh = v3v3SplineList[id].gameObject.GetComponent<SplineMesh>();

						if (v3v3IsHead)
						{
							for (var j = 1; j < 4; j++)
							{
								v3v3Mesh.GetChannel(j).clipFrom = v3v3Mesh.clipFrom + v3v3Per;
							}
						}
						else
						{
							for (var j = 1; j < 4; j++)
							{
								v3v3Mesh.GetChannel(j).clipTo = v3v3Mesh.clipTo - v3v3Per;
							}
						}

						v3v3SplineList[id].SetPointPosition(v3v3IsHead ? 0 : v3v3SplineList[id].pointCount - 1,
							(v3v3IsHead ? v3v3HeadPos : v3v3TailPos) + new Vector3(0, 0.0001f, 0) * i);
					}
					else
					{
						var hp = splineList[id].GetPointPosition(0);
						var tp = splineList[id].GetPointPosition(splineList[id].pointCount - 1);

						var H =
							Vector3.Distance(hp, GetVec3DPoint(v3.position)) <
							Vector3.Distance(tp, GetVec3DPoint(v3.position));

						splineList[id].RebuildImmediate();
						var per = 0.5f / splineList[id].CalculateLength();
						var mesh = splineList[id].gameObject.GetComponent<SplineMesh>();

						if (H)
						{
							for (var j = 1; j < 4; j++)
							{
								mesh.GetChannel(j).clipFrom = mesh.clipFrom + per;
							}
						}
						else
						{
							for (var j = 1; j < 4; j++)
							{
								mesh.GetChannel(j).clipTo = mesh.clipTo - per;
							}
						}

						splineList[id].SetPointPosition(H ? 0 : splineList[id].pointCount - 1,
							(H ? hp : tp) + new Vector3(0, 0.0001f, 0) * i);
					}

					id = sortedList.FindIndex(l => l.Contains(v3.connectedEdgeList[i]));
					if (i == otherPair.Item2)
					{
						if (id == -1)
						{
							id = v3v3List.FindIndex(e => e == v3.connectedEdgeList[i]);

							var v3v3HeadPos = v3v3SplineList[id].GetPointPosition(0);
							var v3v3TailPos = v3v3SplineList[id].GetPointPosition(v3v3SplineList[id].pointCount - 1);

							var v3v3IsHead = Vector3.Distance(v3v3HeadPos, GetVec3DPoint(v3.position)) <
							                 Vector3.Distance(v3v3TailPos, GetVec3DPoint(v3.position));

							if (v3v3IsHead)
							{
								var lastIndex = v3v3SplineList[id].pointCount - 1;
								for (var j = lastIndex; j >= 1; j--)
								{
									if (j == lastIndex)
										v3v3SplineList[id].SetPoint(j + 1,
											new SplinePoint(v3v3SplineList[id].GetPointPosition(j)));
									v3v3SplineList[id].SetPointPosition(j + 1, v3v3SplineList[id].GetPointPosition(j));
								}

								v3v3SplineList[id].SetPointPosition(1, v3v3HeadPos + to2);
							}
							else
							{
								v3v3SplineList[id].SetPointPosition(v3v3SplineList[id].pointCount - 1,
									v3v3TailPos + to2);
								v3v3SplineList[id].SetPoint(v3v3SplineList[id].pointCount,
									new SplinePoint(v3v3TailPos));
							}

							continue;
						}

						var headPos = splineList[id].GetPointPosition(0);
						var tailPos = splineList[id].GetPointPosition(splineList[id].pointCount - 1);

						var isHead =
							Vector3.Distance(headPos, GetVec3DPoint(v3.position)) <
							Vector3.Distance(tailPos, GetVec3DPoint(v3.position));

						if (isHead)
						{
							var lastIndex = splineList[id].pointCount - 1;
							for (var j = lastIndex; j >= 1; j--)
							{
								if (j == lastIndex)
									splineList[id].SetPoint(j + 1, new SplinePoint(splineList[id].GetPointPosition(j)));
								splineList[id].SetPointPosition(j + 1, splineList[id].GetPointPosition(j));
							}

							splineList[id].SetPointPosition(1, headPos + to2);
						}
						else
						{
							splineList[id].SetPointPosition(splineList[id].pointCount - 1, tailPos + to2);
							splineList[id].SetPoint(splineList[id].pointCount, new SplinePoint(tailPos));
						}
					}
				}
			}

			foreach (var v4 in roadGraph.vertexList.Where(v => v.connectedEdgeList.Count == 4))
			{
				for (var i = 0; i < v4.connectedEdgeList.Count; i++)
				{
					var id = sortedList.FindIndex(l => l.Contains(v4.connectedEdgeList[i]));

					if (id == -1)
					{
						id = v3v3List.FindIndex(e => e == v4.connectedEdgeList[i]);

						var v3v3HeadPos = v3v3SplineList[id].GetPointPosition(0);
						var v3v3TailPos = v3v3SplineList[id].GetPointPosition(v3v3SplineList[id].pointCount - 1);

						var v3v3IsHead = Vector3.Distance(v3v3HeadPos, GetVec3DPoint(v4.position)) <
						                 Vector3.Distance(v3v3TailPos, GetVec3DPoint(v4.position));

						v3v3SplineList[id].RebuildImmediate();
						var v3v3Per = 0.5f / v3v3SplineList[id].CalculateLength();
						var v3v3Mesh = v3v3SplineList[id].gameObject.GetComponent<SplineMesh>();

						if (v3v3IsHead)
						{
							for (var j = 1; j < 4; j++)
							{
								v3v3Mesh.GetChannel(j).clipFrom = v3v3Mesh.clipFrom + v3v3Per;
							}
						}
						else
						{
							for (var j = 1; j < 4; j++)
							{
								v3v3Mesh.GetChannel(j).clipTo = v3v3Mesh.clipTo - v3v3Per;
							}
						}

						v3v3SplineList[id].SetPointPosition(v3v3IsHead ? 0 : v3v3SplineList[id].pointCount - 1,
							(v3v3IsHead ? v3v3HeadPos : v3v3TailPos) + new Vector3(0, 0.0001f, 0) * i);
					}
					else
					{
						var hp = splineList[id].GetPointPosition(0);
						var tp = splineList[id].GetPointPosition(splineList[id].pointCount - 1);

						var H =
							Vector3.Distance(hp, GetVec3DPoint(v4.position)) <
							Vector3.Distance(tp, GetVec3DPoint(v4.position));

						splineList[id].RebuildImmediate();
						var per = 0.5f / splineList[id].CalculateLength();
						var mesh = splineList[id].gameObject.GetComponent<SplineMesh>();

						if (H)
						{
							for (var j = 1; j < 4; j++)
							{
								mesh.GetChannel(j).clipFrom = mesh.clipFrom + per;
							}
						}
						else
						{
							for (var j = 1; j < 4; j++)
							{
								mesh.GetChannel(j).clipTo = mesh.clipTo - per;
							}
						}

						splineList[id].SetPointPosition(H ? 0 : splineList[id].pointCount - 1,
							(H ? hp : tp) + new Vector3(0, 0.0001f, 0) * i);
					}
				}
			}

			// Post Processing
			foreach (var spline in splineList)
			{
				var mesh = spline.gameObject.GetComponent<SplineMesh>();

				var countTemp = new int[mesh.GetChannelCount()];
				for (var i = 0; i < countTemp.Length; i++)
				{
					countTemp[i] = mesh.GetChannel(i).count;
				}

				for (var i = 0; i < countTemp.Length; i++)
				{
					mesh.GetChannel(i).autoCount = false;
				}

				for (var i = 0; i < countTemp.Length; i++)
				{
					mesh.GetChannel(i).count = countTemp[i] * 3;
				}

				spline.RebuildImmediate();

				var collection = new SampleCollection();
				spline.GetSamples(collection);
				foreach (var sample in collection.samples)
				{
					var obj = Instantiate(roadColliderPrefab, sample.position, sample.rotation);
					obj.transform.localScale =
						new Vector3(0.8f, obj.transform.localScale.y, obj.transform.localScale.z);
					obj.transform.parent = _roadColliderContainer.transform;
				}

				_secondaryRoadSplineList.Add(spline);
			}

			// Post Processing
			foreach (var spline in v3v3SplineList)
			{
				spline.RebuildImmediate();

				var collection = new SampleCollection();
				spline.GetSamples(collection);
				foreach (var sample in collection.samples)
				{
					var obj = Instantiate(roadColliderPrefab, sample.position, sample.rotation);
					obj.transform.localScale =
						new Vector3(0.8f, obj.transform.localScale.y, obj.transform.localScale.z);
					obj.transform.parent = _roadColliderContainer.transform;
				}

				_secondaryRoadSplineList.Add(spline);
			}
		}
	}

	public IEnumerator GeneratePrimaryBuildingsRoutine()
	{
		if (_buildingContainer == null)
		{
			_buildingContainer = new GameObject("BuildingContainer");
			_buildingContainer.transform.parent = transform;
		}

		foreach (var spline in _primaryRoadSplineList)
		{
			for (var i = 0; i < spline.pointCount - 1; i++)
			{
				yield return new WaitForSeconds(0.014f);

				var a = spline.GetPointPosition(i);
				var b = spline.GetPointPosition(i + 1);

				var mag = (b - a).magnitude;
				var forward = (b - a).normalized;
				var right = Quaternion.Euler(0, 90, 0) * forward;
				var angle = Vector3.SignedAngle(right, Vector3.right, Vector3.up);
				var center = (a + b) / 2;

				var transformMat = Matrix4x4.Rotate(Quaternion.Euler(0, angle, 0)) * Matrix4x4.Translate(-center);

				var vertPadding = 3f;
				var minY = -(mag / 2 + vertPadding);
				var maxY = mag / 2 + vertPadding;

				for (var c = 0; c < 50; c++)
				{
					var randomPos = new Vector3(Random.Range(-1f, 1f) < 0 ? -0.8f : 0.8f, 0,
						Random.Range(minY, maxY));
					randomPos = transformMat.inverse * new Vector4(randomPos.x, randomPos.y, randomPos.z, 1);

					string[] categoryList =
						{ "Apartment", "Modern", "Multiple", "Office", "Residential", "Universal" };
					IWeightedRandomizer<string> categorySelect = new DynamicWeightedRandomizer<string>();
					foreach (var cat in categoryList)
					{
						categorySelect.Add(cat, 1);
					}

					var prefabs = Resources.LoadAll("BuildingPrefabs/" + categorySelect.NextWithRemoval() + "/");

					IWeightedRandomizer<int> buildingSelect = new DynamicWeightedRandomizer<int>();
					for (var bi = 0; bi < prefabs.Length; bi++)
					{
						buildingSelect.Add(bi, 1);
					}

					var selectedBuildingPrefab = prefabs[buildingSelect.NextWithRemoval()] as GameObject;

					var colSize = selectedBuildingPrefab.GetComponent<BoxCollider>().size;
					var locSize = selectedBuildingPrefab.transform.localScale;
					var realScale = new Vector3(locSize.x * colSize.x, locSize.y * colSize.y,
						locSize.z * colSize.z);

					var result = new SplineSample();
					spline.Project(randomPos, ref result);
					var dir = result.position - randomPos;
					var rot = Quaternion.Euler(0, Vector3.SignedAngle(Vector3.forward, dir, Vector3.up) - 90, 0);

					var diff = dir.magnitude - (realScale.x / 2 + 2f);
					randomPos += dir.normalized * diff;

					var colList = Physics.OverlapBox(randomPos, realScale * 0.5f, rot,
							LayerMask.GetMask("Building"))
						.ToList();
					if (colList.Count > 0) continue;

					DebugExtension.DebugArrow(randomPos, dir.normalized, Color.magenta, 1000f);

					var obj = Instantiate(selectedBuildingPrefab, randomPos, rot);
					obj.transform.parent = _buildingContainer.transform;
				}
			}
		}
	}

	public void GeneratePrimaryBuildings()
	{
		if (_buildingContainer == null)
		{
			_buildingContainer = new GameObject("BuildingContainer");
			_buildingContainer.transform.parent = transform;
		}

		foreach (var spline in _primaryRoadSplineList)
		{
			for (var i = 0; i < spline.pointCount - 1; i++)
			{
				var a = spline.GetPointPosition(i);
				var b = spline.GetPointPosition(i + 1);

				var mag = (b - a).magnitude;
				var forward = (b - a).normalized;
				var right = Quaternion.Euler(0, 90, 0) * forward;
				var angle = Vector3.SignedAngle(right, Vector3.right, Vector3.up);
				var center = (a + b) / 2;

				var transformMat = Matrix4x4.Rotate(Quaternion.Euler(0, angle, 0)) * Matrix4x4.Translate(-center);

				var vertPadding = 3f;
				var minY = -(mag / 2 + vertPadding);
				var maxY = mag / 2 + vertPadding;

				for (var c = 0; c < 50; c++)
				{
					var randomPos = new Vector3(Random.Range(-1f, 1f) < 0 ? -0.8f : 0.8f, 0,
						Random.Range(minY, maxY));
					randomPos = transformMat.inverse * new Vector4(randomPos.x, randomPos.y, randomPos.z, 1);

					string[] categoryList =
						{ "Apartment", "Modern", "Multiple", "Office", "Residential", "Universal" };
					IWeightedRandomizer<string> categorySelect = new DynamicWeightedRandomizer<string>();
					foreach (var cat in categoryList)
					{
						categorySelect.Add(cat, 1);
					}

					var prefabs = Resources.LoadAll("BuildingPrefabs/" + categorySelect.NextWithRemoval() + "/");

					IWeightedRandomizer<int> buildingSelect = new DynamicWeightedRandomizer<int>();
					for (var bi = 0; bi < prefabs.Length; bi++)
					{
						buildingSelect.Add(bi, 1);
					}

					var selectedBuildingPrefab = prefabs[buildingSelect.NextWithRemoval()] as GameObject;

					var colSize = selectedBuildingPrefab.GetComponent<BoxCollider>().size;
					var locSize = selectedBuildingPrefab.transform.localScale;
					var realScale = new Vector3(locSize.x * colSize.x, locSize.y * colSize.y,
						locSize.z * colSize.z);

					var result = new SplineSample();
					spline.Project(randomPos, ref result);
					var dir = result.position - randomPos;
					var rot = Quaternion.Euler(0, Vector3.SignedAngle(Vector3.forward, dir, Vector3.up) - 90, 0);

					var diff = dir.magnitude - (realScale.x / 2 + 2f);
					randomPos += dir.normalized * diff;

					var colList = Physics.OverlapBox(randomPos, realScale * 0.5f, rot,
							LayerMask.GetMask("Building"))
						.ToList();
					if (colList.Count > 0) continue;

					DebugExtension.DebugArrow(randomPos, dir.normalized, Color.magenta, 1000f);

					var obj = Instantiate(selectedBuildingPrefab, randomPos, rot);
					obj.transform.parent = _buildingContainer.transform;

					_primaryBuildingInfoList.Add(new Tuple<GameObject, Vector3, Vector3>(obj, dir.normalized, result.position));
				}
			}
		}
	}

	public IEnumerator GenerateSecondaryBuildingsRoutine()
	{
		if (_buildingContainer == null)
		{
			_buildingContainer = new GameObject("BuildingContainer");
			_buildingContainer.transform.parent = transform;
		}

		foreach (var spline in _secondaryRoadSplineList)
		{
			for (var i = 0; i < spline.pointCount - 1; i++)
			{
				yield return new WaitForSeconds(0.014f);

				var a = spline.GetPointPosition(i);
				var b = spline.GetPointPosition(i + 1);

				var mag = (b - a).magnitude;
				var forward = (b - a).normalized;
				var right = Quaternion.Euler(0, 90, 0) * forward;
				var angle = Vector3.SignedAngle(right, Vector3.right, Vector3.up);
				var center = (a + b) / 2;

				var transformMat = Matrix4x4.Rotate(Quaternion.Euler(0, angle, 0)) * Matrix4x4.Translate(-center);

				var vertPadding = 3f;
				var minY = -(mag / 2 + vertPadding);
				var maxY = mag / 2 + vertPadding;

				var clearedCategoryList = new List<string>();
				var usedBuildingList = new List<Tuple<string, int>>();

				if (mag > 3f)
				{
					var pos = new Vector3(-0.8f, 0, 0);
					var pos2 = new Vector3(0.8f, 0, 0);

					pos = transformMat.inverse * new Vector4(pos.x, pos.y, pos.z, 1);
					pos2 = transformMat.inverse * new Vector4(pos2.x, pos2.y, pos2.z, 1);

					var parkPrefab = Resources.Load("Park") as GameObject;

					var colSize = parkPrefab.GetComponent<BoxCollider>().size;
					var locSize = parkPrefab.transform.localScale;
					var realScale = new Vector3(locSize.x * colSize.x, locSize.y * colSize.y,
						locSize.z * colSize.z);

					// try pos
					var result = new SplineSample();
					spline.Project(pos, ref result);
					var dir = result.position - pos;
					var rot = Quaternion.Euler(0, Vector3.SignedAngle(Vector3.forward, dir, Vector3.up) - 90, 0);

					var diff = dir.magnitude - (realScale.x / 2 + 0.6f);
					pos += dir.normalized * diff;

					var checkList = Physics.OverlapSphere(pos2, 20f, LayerMask.GetMask("Building")).ToList();
					if (checkList.Exists(p => p.gameObject.tag == "Park")) goto GEN_BUILDING;

					var colList = Physics.OverlapBox(pos, realScale * 0.33f, rot,
							LayerMask.GetMask("Building"))
						.ToList();
					if (colList.Count > 0)
					{
						// try pos2
						var result2 = new SplineSample();
						spline.Project(pos2, ref result2);
						var dir2 = result2.position - pos2;
						var rot2 = Quaternion.Euler(0, Vector3.SignedAngle(Vector3.forward, dir2, Vector3.up) - 90, 0);

						var diff2 = dir2.magnitude - (realScale.x / 2 + 0.3f);
						pos2 += dir2.normalized * diff2;

						var checkList2 = Physics.OverlapSphere(pos2, 20f, LayerMask.GetMask("Building")).ToList();
						if (checkList2.Exists(p => p.gameObject.tag == "Park")) goto GEN_BUILDING;

						var colList2 = Physics.OverlapBox(pos2, realScale * 0.33f, rot2,
								LayerMask.GetMask("Building"))
							.ToList();
						if (colList2.Count > 0)
						{
							foreach (var col in colList2)
							{
								if (col.gameObject.tag == "Road") goto GEN_BUILDING;
								Destroy(col.gameObject);
							}
						}

						DebugExtension.DebugArrow(pos2, dir2.normalized, Color.magenta, 1000f);

						var obj2 = Instantiate(parkPrefab, pos2 + new Vector3(0, 0.08f, 0), rot2);
						obj2.transform.parent = _buildingContainer.transform;

						continue;
					}

					DebugExtension.DebugArrow(pos, dir.normalized, Color.magenta, 1000f);

					var obj = Instantiate(parkPrefab, pos + new Vector3(0, 0.08f, 0), rot);
					obj.transform.parent = _buildingContainer.transform;

					continue;
				}

				GEN_BUILDING:
				for (var c = 0; c < 50; c++)
				{
					var randomPos = new Vector3(Random.Range(-1f, 1f) < 0 ? -0.8f : 0.8f, 0,
						Random.Range(minY, maxY));
					randomPos = transformMat.inverse * new Vector4(randomPos.x, randomPos.y, randomPos.z, 1);

					string[] categoryList =
						{ "Apartment", "Modern", "Multiple", "Office", "Residential", "Universal" };
					IWeightedRandomizer<string> categorySelect = new DynamicWeightedRandomizer<string>();
					foreach (var cat in categoryList)
					{
						if (clearedCategoryList.Contains(cat)) continue;
						categorySelect.Add(cat, 1);
					}

					if (categorySelect.Count <= 0) continue;

					var selectedCategory = categorySelect.NextWithRemoval();
					var prefabs = Resources.LoadAll("BuildingPrefabs/" + selectedCategory + "/");

					IWeightedRandomizer<int> buildingSelect = new DynamicWeightedRandomizer<int>();
					for (var bi = 0; bi < prefabs.Length; bi++)
					{
						if (usedBuildingList.Exists(b => b.Item1 == selectedCategory && b.Item2 == bi)) continue;
						buildingSelect.Add(bi, 1);
					}

					var selectedBuildingIndex = buildingSelect.NextWithRemoval();
					usedBuildingList.Add(new Tuple<string, int>(selectedCategory, selectedBuildingIndex));
					if (usedBuildingList.Count(b => b.Item1 == selectedCategory) == prefabs.Length)
					{
						clearedCategoryList.Add(selectedCategory);
					}

					var selectedBuildingPrefab = prefabs[selectedBuildingIndex] as GameObject;

					var colSize = selectedBuildingPrefab.GetComponent<BoxCollider>().size;
					var locSize = selectedBuildingPrefab.transform.localScale;
					var realScale = new Vector3(locSize.x * colSize.x, locSize.y * colSize.y,
						locSize.z * colSize.z);

					var result = new SplineSample();
					spline.Project(randomPos, ref result);
					var dir = result.position - randomPos;
					var rot = Quaternion.Euler(0, Vector3.SignedAngle(Vector3.forward, dir, Vector3.up) - 90, 0);

					var diff = dir.magnitude - (realScale.x / 2 + 1f);
					randomPos += dir.normalized * diff;

					var colList = Physics.OverlapBox(randomPos, realScale * 0.33f, rot,
							LayerMask.GetMask("Building"))
						.ToList();
					if (colList.Count > 0) continue;

					DebugExtension.DebugArrow(randomPos, dir.normalized, Color.magenta, 1000f);

					var obj = Instantiate(selectedBuildingPrefab, randomPos, rot);
					obj.transform.parent = _buildingContainer.transform;
				}
			}
		}
	}

	public void GenerateSecondaryBuildings()
	{
		if (_buildingContainer == null)
		{
			_buildingContainer = new GameObject("BuildingContainer");
			_buildingContainer.transform.parent = transform;
		}

		foreach (var spline in _secondaryRoadSplineList)
		{
			for (var i = 0; i < spline.pointCount - 1; i++)
			{
				var a = spline.GetPointPosition(i);
				var b = spline.GetPointPosition(i + 1);

				var mag = (b - a).magnitude;
				var forward = (b - a).normalized;
				var right = Quaternion.Euler(0, 90, 0) * forward;
				var angle = Vector3.SignedAngle(right, Vector3.right, Vector3.up);
				var center = (a + b) / 2;

				var transformMat = Matrix4x4.Rotate(Quaternion.Euler(0, angle, 0)) * Matrix4x4.Translate(-center);

				var vertPadding = 3f;
				var minY = -(mag / 2 + vertPadding);
				var maxY = mag / 2 + vertPadding;

				var clearedCategoryList = new List<string>();
				var usedBuildingList = new List<Tuple<string, int>>();

				if (mag > 3f)
				{
					var pos = new Vector3(-0.8f, 0, 0);
					var pos2 = new Vector3(0.8f, 0, 0);

					pos = transformMat.inverse * new Vector4(pos.x, pos.y, pos.z, 1);
					pos2 = transformMat.inverse * new Vector4(pos2.x, pos2.y, pos2.z, 1);

					var parkPrefab = Resources.Load("Park") as GameObject;

					var colSize = parkPrefab.GetComponent<BoxCollider>().size;
					var locSize = parkPrefab.transform.localScale;
					var realScale = new Vector3(locSize.x * colSize.x, locSize.y * colSize.y,
						locSize.z * colSize.z);

					// try pos
					var result = new SplineSample();
					spline.Project(pos, ref result);
					var dir = result.position - pos;
					var rot = Quaternion.Euler(0, Vector3.SignedAngle(Vector3.forward, dir, Vector3.up) - 90, 0);

					var diff = dir.magnitude - (realScale.x / 2 + 0.6f);
					pos += dir.normalized * diff;

					var checkList = Physics.OverlapSphere(pos2, 20f, LayerMask.GetMask("Building")).ToList();
					if (checkList.Exists(p => p.gameObject.tag == "Park")) goto GEN_BUILDING;

					var colList = Physics.OverlapBox(pos, realScale * 0.33f, rot,
							LayerMask.GetMask("Building"))
						.ToList();
					if (colList.Count > 0)
					{
						// try pos2
						var result2 = new SplineSample();
						spline.Project(pos2, ref result2);
						var dir2 = result2.position - pos2;
						var rot2 = Quaternion.Euler(0, Vector3.SignedAngle(Vector3.forward, dir2, Vector3.up) - 90, 0);

						var diff2 = dir2.magnitude - (realScale.x / 2 + 0.3f);
						pos2 += dir2.normalized * diff2;

						var checkList2 = Physics.OverlapSphere(pos2, 20f, LayerMask.GetMask("Building")).ToList();
						if (checkList2.Exists(p => p.gameObject.tag == "Park")) goto GEN_BUILDING;

						var colList2 = Physics.OverlapBox(pos2, realScale * 0.33f, rot2,
								LayerMask.GetMask("Building"))
							.ToList();
						if (colList2.Count > 0)
						{
							foreach (var col in colList2)
							{
								if (col.gameObject.tag == "Road") goto GEN_BUILDING;
								_primaryBuildingInfoList.RemoveAll(t => t.Item1 == col.gameObject);
								Destroy(col.gameObject);
							}
						}

						DebugExtension.DebugArrow(pos2, dir2.normalized, Color.magenta, 1000f);

						var obj2 = Instantiate(parkPrefab, pos2 + new Vector3(0, 0.08f, 0), rot2);
						obj2.transform.parent = _buildingContainer.transform;

						continue;
					}

					DebugExtension.DebugArrow(pos, dir.normalized, Color.magenta, 1000f);

					var obj = Instantiate(parkPrefab, pos + new Vector3(0, 0.08f, 0), rot);
					obj.transform.parent = _buildingContainer.transform;

					continue;
				}

				GEN_BUILDING:
				for (var c = 0; c < 50; c++)
				{
					var randomPos = new Vector3(Random.Range(-1f, 1f) < 0 ? -0.8f : 0.8f, 0,
						Random.Range(minY, maxY));
					randomPos = transformMat.inverse * new Vector4(randomPos.x, randomPos.y, randomPos.z, 1);

					string[] categoryList =
						{ "Apartment", "Modern", "Multiple", "Office", "Residential", "Universal" };
					IWeightedRandomizer<string> categorySelect = new DynamicWeightedRandomizer<string>();
					foreach (var cat in categoryList)
					{
						if (clearedCategoryList.Contains(cat)) continue;
						categorySelect.Add(cat, 1);
					}

					if (categorySelect.Count <= 0) continue;

					var selectedCategory = categorySelect.NextWithRemoval();
					var prefabs = Resources.LoadAll("BuildingPrefabs/" + selectedCategory + "/");

					IWeightedRandomizer<int> buildingSelect = new DynamicWeightedRandomizer<int>();
					for (var bi = 0; bi < prefabs.Length; bi++)
					{
						if (usedBuildingList.Exists(b => b.Item1 == selectedCategory && b.Item2 == bi)) continue;
						buildingSelect.Add(bi, 1);
					}

					var selectedBuildingIndex = buildingSelect.NextWithRemoval();
					usedBuildingList.Add(new Tuple<string, int>(selectedCategory, selectedBuildingIndex));
					if (usedBuildingList.Count(b => b.Item1 == selectedCategory) == prefabs.Length)
					{
						clearedCategoryList.Add(selectedCategory);
					}

					var selectedBuildingPrefab = prefabs[selectedBuildingIndex] as GameObject;

					var colSize = selectedBuildingPrefab.GetComponent<BoxCollider>().size;
					var locSize = selectedBuildingPrefab.transform.localScale;
					var realScale = new Vector3(locSize.x * colSize.x, locSize.y * colSize.y,
						locSize.z * colSize.z);

					var result = new SplineSample();
					spline.Project(randomPos, ref result);
					var dir = result.position - randomPos;
					var rot = Quaternion.Euler(0, Vector3.SignedAngle(Vector3.forward, dir, Vector3.up) - 90, 0);

					var diff = dir.magnitude - (realScale.x / 2 + 1f);
					randomPos += dir.normalized * diff;

					var colList = Physics.OverlapBox(randomPos, realScale * 0.33f, rot,
							LayerMask.GetMask("Building"))
						.ToList();
					if (colList.Count > 0) continue;

					DebugExtension.DebugArrow(randomPos, dir.normalized, Color.magenta, 1000f);

					var obj = Instantiate(selectedBuildingPrefab, randomPos, rot);
					obj.transform.parent = _buildingContainer.transform;
				}
			}
		}
	}

	public void RepositionConnector()
	{
		foreach (var connectorInfo in _connectorInfoList)
		{
			GetSecondaryRoadVertex(connectorInfo.connectorCityCell, connectorInfo.connectorEdge.startVertexIndex,
				out var from);
			GetSecondaryRoadVertex(connectorInfo.connectorCityCell, connectorInfo.connectorEdge.endVertexIndex,
				out var to);

			var from3D = GetVec3DPoint(from);
			var to3D = GetVec3DPoint(to);

			if (connectorInfo.connectorEdgeSpline == null || connectorInfo.basePrimaryEdgeSpline == null) continue;

			var sample = connectorInfo.basePrimaryEdgeSpline.Project(from3D);
			var isHead = Vector3.Distance(connectorInfo.connectorEdgeSpline.GetPointPosition(0), from3D) <
			             Vector3.Distance(
				             connectorInfo.connectorEdgeSpline.GetPointPosition(connectorInfo.connectorEdgeSpline
					             .pointCount - 1), from3D);

			DebugExtension.DebugWireSphere(sample.position, Color.red, 0.3f, 1000f);

			connectorInfo.connectorEdgeSpline.SetPointPosition(
				isHead ? 0 : connectorInfo.connectorEdgeSpline.pointCount - 1,
				sample.position + new Vector3(0, -0.001f, 0));

			var mesh = connectorInfo.connectorEdgeSpline.gameObject.GetComponent<SplineMesh>();

			connectorInfo.connectorEdgeSpline.RebuildImmediate();
			var per = 1f / connectorInfo.connectorEdgeSpline.CalculateLength();

			if (isHead)
			{
				mesh.GetChannel(1).clipFrom = per;
				mesh.GetChannel(2).clipFrom = per;
				mesh.GetChannel(3).clipFrom = per;
			}
			else
			{
				mesh.GetChannel(1).clipTo = 1 - per;
				mesh.GetChannel(2).clipTo = 1 - per;
				mesh.GetChannel(3).clipTo = 1 - per;
			}

			DebugExtension.DebugArrow(
				connectorInfo.connectorEdgeSpline.GetPointPosition(isHead
					? 0
					: connectorInfo.connectorEdgeSpline.pointCount - 1),
				(connectorInfo.connectorEdgeSpline.GetPointPosition(isHead
					 ? 1
					 : connectorInfo.connectorEdgeSpline.pointCount - 2) -
				 connectorInfo.connectorEdgeSpline.GetPointPosition(isHead
					 ? 0
					 : connectorInfo.connectorEdgeSpline.pointCount - 1)).normalized, Color.red, 1000f);
		}
	}

	public void Display2DDraw()
	{
		var tx = new Texture2D((int)sampleRegionSize.x, (int)sampleRegionSize.y);

		DrawSite(ref tx);
		DrawDivider(ref tx);
		DrawEdges(ref tx);

		tx.Apply();

		GetComponent<MeshRenderer>().material.mainTexture = tx;
	}

	public void Display()
	{
		GetComponent<MeshRenderer>().material = planeMaterial;

		foreach (var (_, roadGraph) in _secondaryRoadGraph2DDict)
		{
			foreach (var vert in roadGraph.vertexList)
			{
				var pos3D = GetVec3DPoint(vert.position);
				DebugExtension.DebugWireSphere(pos3D,
					vert.connectedEdgeList.Count == 0 ? Color.black :
					vert.connectedEdgeList.Count == 1 ? Color.yellow :
					vert.connectedEdgeList.Count == 2 ? Color.green :
					vert.connectedEdgeList.Count == 3 ? Color.blue : Color.white, 0.3f, 1000f);

				foreach (var uIndexedEdge in vert.connectedEdgeList)
				{
					var a = GetVec3DPoint(roadGraph.vertexList[uIndexedEdge.v1Index].position);
					var b = GetVec3DPoint(roadGraph.vertexList[uIndexedEdge.v2Index].position);
					DebugExtension.DebugArrow(pos3D + new Vector3(0, 0.2f, 0),
						Vector3.Distance(a, pos3D) <= 0.01f ? (b - a).normalized : (a - b).normalized,
						vert.connectedEdgeList.Count == 0 ? Color.black :
						vert.connectedEdgeList.Count == 1 ? Color.yellow :
						vert.connectedEdgeList.Count == 2 ? Color.green :
						vert.connectedEdgeList.Count == 3 ? Color.blue : Color.white, 1000f);
				}
			}
		}

		foreach (var (_, _, item3) in _primaryBuildingInfoList)
		{
			DebugExtension.DebugPoint(item3, Color.white, 0.4f, 1000f);
		}
	}

	public void Clear()
	{
		_poissonPointList.Clear();
		_siteDict.Clear();
		_edgeList.Clear();

		_outerSiteList.Clear();
		_innerSiteList.Clear();

		_dividerCenterList.Clear();
		_dividerEdgeList.Clear();
		_dividerSiteList.Clear();
		_dividerBorder.Clear();

		_primaryRoadList.Clear();
		_cityCellDict.Clear();
		_cityCellList.Clear();

		_secondaryRoadVertexDict.Clear();
		_secondaryRoadEdgeDict.Clear();

		if (_secondaryRoadGenRoutine != null) StopCoroutine(_secondaryRoadGenRoutine);
		_secondaryRoadGenRoutine = null;

		_primaryRoadGraph2D = new RoadGraph<Vertex2D>();
		_secondaryRoadGraph2DDict.Clear();

		_primaryRoadSplineList.Clear();
		_secondaryRoadSplineList.Clear();

		_connectorInfoList.Clear();

		cellSize = 0.0f;

		GetComponent<MeshRenderer>().material.mainTexture = default;

		if (_roadSplineContainer != null)
		{
			foreach (var child in _roadSplineContainer.transform)
			{
				DestroyImmediate(child as GameObject);
			}

			DestroyImmediate(_roadSplineContainer);
			_roadSplineContainer = null;
		}

		if (_roadIntersectionContainer != null)
		{
			foreach (var child in _roadIntersectionContainer.transform)
			{
				DestroyImmediate(child as GameObject);
			}

			DestroyImmediate(_roadIntersectionContainer);
			_roadIntersectionContainer = null;
		}

		if (_roadColliderContainer != null)
		{
			foreach (var child in _roadColliderContainer.transform)
			{
				DestroyImmediate(child as GameObject);
			}

			DestroyImmediate(_roadColliderContainer);
			_roadColliderContainer = null;
		}

		if (_buildingContainer != null)
		{
			foreach (var child in _buildingContainer.transform)
			{
				DestroyImmediate(child as GameObject);
			}

			DestroyImmediate(_buildingContainer);
			_buildingContainer = null;
		}
	}

	public void RunAll()
	{
		StartCoroutine(RunAllRoutine());
	}

	public void Step01()
	{
		GenerateInitialPoint();
		GenerateVoronoi();
		Display2DDraw();
	}

	public void Step02()
	{
		SetBoundaryAsPrimaryRoad();
		Display2DDraw();
	}

	public void Step03()
	{
		GroupVoronoiAsCityCell();
		SetCityCellBoundaryAsPrimaryRoad();
		RefineCityCell();
		Display2DDraw();
	}

	public void Step04()
	{
		GenerateSecondaryRoad();
		GenerateSecondaryRoadGraph();
		Display2DDraw();
	}
	
	public void Step05()
	{
		GeneratePrimaryRoadSpline();
	}
	
	public void Step06()
	{
		GenerateSecondaryRoadSpline();
	}

	public void Step07()
	{
		RepositionConnector();
	}

	public void Step08()
	{
		GeneratePrimaryBuildings();
	}

	public void Step09()
	{
		GenerateSecondaryBuildings();
	}

	public void Step10()
	{
		Display();
	}

	public IEnumerator RunAllRoutine()
	{
		Clear();

		// 1. Generate Voronoi
		yield return new WaitForSeconds(1f);
		GenerateInitialPoint();
		GenerateVoronoi();
		Display2DDraw();

		// 2. Set Boundary As Primary Road
		yield return new WaitForSeconds(1f);
		SetBoundaryAsPrimaryRoad();
		Display2DDraw();

		// 3. Group Voronoi As City Cell & Set City Cell Boundary As Primary Road
		yield return new WaitForSeconds(1f);
		GroupVoronoiAsCityCell();
		SetCityCellBoundaryAsPrimaryRoad();
		RefineCityCell();
		Display2DDraw();

		// 4. Generate Secondary Road
		yield return new WaitForSeconds(1f);
		yield return GenerateSecondaryRoadRoutine();
		GenerateSecondaryRoadGraph();

		// 5. Generate Primary Road Spline
		yield return new WaitForSeconds(1f);
		yield return GeneratePrimaryRoadSplineRoutine();

		// 6. Generate Secondary Road Spline
		yield return new WaitForSeconds(1f);
		yield return GenerateSecondaryRoadSplineRoutine();

		// 7. Reposition Connector
		yield return new WaitForSeconds(1f);
		RepositionConnector();

		// 8. Generate Primary Buildings
		yield return new WaitForSeconds(1f);
		yield return GeneratePrimaryBuildingsRoutine();

		// 9. Generate Secondary Buildings
		yield return new WaitForSeconds(1f);
		yield return GenerateSecondaryBuildingsRoutine();

		// 10. The End.
		yield return new WaitForSeconds(1f);
		Display();
	}

	public void RunAllImmediate()
	{
		Clear();
		GenerateInitialPoint();
		GenerateVoronoi();
		SetBoundaryAsPrimaryRoad();
		GroupVoronoiAsCityCell();
		SetCityCellBoundaryAsPrimaryRoad();
		RefineCityCell();
		GenerateSecondaryRoad();
		GenerateSecondaryRoadGraph();
		GeneratePrimaryRoadSpline();
		GenerateSecondaryRoadSpline();
		RepositionConnector();
		GeneratePrimaryBuildings();
		GenerateSecondaryBuildings();
		Display();
	}

	private IEnumerator SecondaryRoadGenOuterLoopRoutine()
	{
		InitSecondaryRoadDict(_cityCellList);

		foreach (var cityCell in _cityCellList)
		{
			cityCell.boundaryEdgeList.Sort((e1, e2) =>
			{
				var e1Sq = Vector2f.DistanceSquare(e1.ClippedEnds[LR.LEFT], e1.ClippedEnds[LR.RIGHT]);
				var e2Sq = Vector2f.DistanceSquare(e2.ClippedEnds[LR.LEFT], e2.ClippedEnds[LR.RIGHT]);
				return e1Sq.CompareTo(e2Sq);
			});
		}

		foreach (var cityCell in _cityCellList)
		{
			for (var order = 1; order <= cityCell.siteList.Count; order++)
			{
				var (initialVertexIndex, prevVertexIndex) = SelectVertexFromBoundary(cityCell, order);
				yield return SecondaryRoadGenInnerLoopRoutine(initialVertexIndex, prevVertexIndex, cityCell, 0,
					new Probability() { count = new[] { 0, 1, 0 }, direction = new[] { 0, 1, 1 } }, order);
			}
		}
	}

	private void SecondaryRoadGenOuterLoop()
	{
		InitSecondaryRoadDict(_cityCellList);

		foreach (var cityCell in _cityCellList)
		{
			cityCell.boundaryEdgeList.Sort((e1, e2) =>
			{
				var e1Sq = Vector2f.DistanceSquare(e1.ClippedEnds[LR.LEFT], e1.ClippedEnds[LR.RIGHT]);
				var e2Sq = Vector2f.DistanceSquare(e2.ClippedEnds[LR.LEFT], e2.ClippedEnds[LR.RIGHT]);
				return e1Sq.CompareTo(e2Sq);
			});
		}

		foreach (var cityCell in _cityCellList)
		{
			for (var order = 1; order <= cityCell.siteList.Count; order++)
			{
				var (initialVertexIndex, prevVertexIndex) = SelectVertexFromBoundary(cityCell, order);
				SecondaryRoadGenInnerLoop(initialVertexIndex, prevVertexIndex, cityCell, 0,
					new Probability() { count = new[] { 0, 1, 0 }, direction = new[] { 0, 1, 1 } }, order);
			}
		}
	}

	private IEnumerator SecondaryRoadGenInnerLoopRoutine(int fromVertexIndex, int prevVertexIndex, CityCell cityCell,
		int depth,
		Probability prob = null, int order = -1)
	{
		Display2DDraw();
		yield return new WaitForSeconds(0.014f);

		if (depth > gridGrowthProp.depth) yield break;

		GetSecondaryRoadVertex(cityCell, fromVertexIndex, out var fromVertexPosition);
		GetSecondaryRoadVertex(cityCell, prevVertexIndex, out var prevVertexPosition);

		var suggestionVertexPositionList =
			GridGrowth(fromVertexPosition, prevVertexPosition, prob ?? gridGrowthProp.defaultProb);

		var resultList = suggestionVertexPositionList.Select(newVertexPosition =>
			IsVertexValid(prevVertexIndex, newVertexPosition, cityCell)).ToList();

		var validResultCount = resultList.Count(r => r.action != VertexSnapAction.DESTROY);
		if (validResultCount == 0)
		{
			yield return SecondaryRoadGenInnerLoopRoutine(fromVertexIndex, prevVertexIndex, cityCell, depth + 1, prob, order);
			yield break;
		}

		var nextLoopVertexList = new List<int>();
		foreach (var result in resultList)
		{
			switch (result.action)
			{
				case VertexSnapAction.NEW:
					{
						var newVertexIndex = AddSecondaryRoadVertex(cityCell, result.vector2Value1);
						var newEdge = AddSecondaryRoadEdge(cityCell, fromVertexIndex, newVertexIndex);
						if (order != -1)
							_connectorInfoList.Add(new ConnectorInfo(cityCell.boundaryEdgeList[^order], newEdge, cityCell));
						nextLoopVertexList.Add(newVertexIndex);
						break;
					}
				case VertexSnapAction.DIVIDE_TARGET_EDGE:
					{
						var newDivideVertexIndex = AddSecondaryRoadVertex(cityCell, result.vector2Value1);

						var targetEdgeStartVertexIndex = result.edgeValue.startVertexIndex;
						var targetEdgeEndVertexIndex = result.edgeValue.endVertexIndex;

						DestroySecondaryRoadEdge(cityCell, result.edgeValue);

						AddSecondaryRoadEdge(cityCell, targetEdgeStartVertexIndex, newDivideVertexIndex);
						AddSecondaryRoadEdge(cityCell, newDivideVertexIndex, targetEdgeEndVertexIndex);
						var newEdge = AddSecondaryRoadEdge(cityCell, fromVertexIndex, newDivideVertexIndex);
						if (order != -1)
							_connectorInfoList.Add(new ConnectorInfo(cityCell.boundaryEdgeList[^order], newEdge, cityCell));
						break;
					}
				case VertexSnapAction.DIVIDE_SOURCE_EDGE:
					{
						var intersectVertexIndex = result.intValue;
						var sourceEdgeEndVertexIndex = AddSecondaryRoadVertex(cityCell, result.vector2Value1);

						var newEdge = AddSecondaryRoadEdge(cityCell, fromVertexIndex, intersectVertexIndex);
						if (order != -1)
							_connectorInfoList.Add(new ConnectorInfo(cityCell.boundaryEdgeList[^order], newEdge, cityCell));
						AddSecondaryRoadEdge(cityCell, intersectVertexIndex, sourceEdgeEndVertexIndex);
						break;
					}
				case VertexSnapAction.DIVIDE_SOURCE_EDGE_NEW:
					{
						var intersectVertexIndex = AddSecondaryRoadVertex(cityCell, result.vector2Value1);
						var sourceEdgeEndVertexIndex = AddSecondaryRoadVertex(cityCell, result.vector2Value2);

						var newEdge = AddSecondaryRoadEdge(cityCell, fromVertexIndex, intersectVertexIndex);
						if (order != -1)
							_connectorInfoList.Add(new ConnectorInfo(cityCell.boundaryEdgeList[^order], newEdge, cityCell));
						AddSecondaryRoadEdge(cityCell, intersectVertexIndex, sourceEdgeEndVertexIndex);

						var targetEdgeStartVertexIndex = result.edgeValue.startVertexIndex;
						var targetEdgeEndVertexIndex = result.edgeValue.endVertexIndex;

						DestroySecondaryRoadEdge(cityCell, result.edgeValue);

						AddSecondaryRoadEdge(cityCell, targetEdgeStartVertexIndex, intersectVertexIndex);
						AddSecondaryRoadEdge(cityCell, intersectVertexIndex, targetEdgeEndVertexIndex);
						break;
					}
				case VertexSnapAction.MERGE_VERTEX:
					{
						var newEdge = AddSecondaryRoadEdge(cityCell, fromVertexIndex, result.intValue);
						if (order != -1)
							_connectorInfoList.Add(new ConnectorInfo(cityCell.boundaryEdgeList[^order], newEdge, cityCell));
						break;
					}
				case VertexSnapAction.DESTROY:
					break;
				default:
					throw new ArgumentOutOfRangeException();
			}
		}

		foreach (var newVertexIndex in nextLoopVertexList)
		{
			yield return SecondaryRoadGenInnerLoopRoutine(newVertexIndex, fromVertexIndex, cityCell, depth + 1);
		}
	}

	private void SecondaryRoadGenInnerLoop(int fromVertexIndex, int prevVertexIndex, CityCell cityCell,
		int depth,
		Probability prob = null, int order = -1)
	{
		if (depth > gridGrowthProp.depth) return;

		GetSecondaryRoadVertex(cityCell, fromVertexIndex, out var fromVertexPosition);
		GetSecondaryRoadVertex(cityCell, prevVertexIndex, out var prevVertexPosition);

		var suggestionVertexPositionList =
			GridGrowth(fromVertexPosition, prevVertexPosition, prob ?? gridGrowthProp.defaultProb);

		var resultList = suggestionVertexPositionList.Select(newVertexPosition =>
			IsVertexValid(prevVertexIndex, newVertexPosition, cityCell)).ToList();

		var validResultCount = resultList.Count(r => r.action != VertexSnapAction.DESTROY);
		if (validResultCount == 0)
		{
			SecondaryRoadGenInnerLoop(fromVertexIndex, prevVertexIndex, cityCell, depth + 1, prob, order);
			return;
		}

		var nextLoopVertexList = new List<int>();
		foreach (var result in resultList)
		{
			switch (result.action)
			{
				case VertexSnapAction.NEW:
				{
					var newVertexIndex = AddSecondaryRoadVertex(cityCell, result.vector2Value1);
					var newEdge = AddSecondaryRoadEdge(cityCell, fromVertexIndex, newVertexIndex);
					if (order != -1)
						_connectorInfoList.Add(new ConnectorInfo(cityCell.boundaryEdgeList[^order], newEdge, cityCell));
					nextLoopVertexList.Add(newVertexIndex);
					break;
				}
				case VertexSnapAction.DIVIDE_TARGET_EDGE:
				{
					var newDivideVertexIndex = AddSecondaryRoadVertex(cityCell, result.vector2Value1);

					var targetEdgeStartVertexIndex = result.edgeValue.startVertexIndex;
					var targetEdgeEndVertexIndex = result.edgeValue.endVertexIndex;

					DestroySecondaryRoadEdge(cityCell, result.edgeValue);

					AddSecondaryRoadEdge(cityCell, targetEdgeStartVertexIndex, newDivideVertexIndex);
					AddSecondaryRoadEdge(cityCell, newDivideVertexIndex, targetEdgeEndVertexIndex);
					var newEdge = AddSecondaryRoadEdge(cityCell, fromVertexIndex, newDivideVertexIndex);
					if (order != -1)
						_connectorInfoList.Add(new ConnectorInfo(cityCell.boundaryEdgeList[^order], newEdge, cityCell));
					break;
				}
				case VertexSnapAction.DIVIDE_SOURCE_EDGE:
				{
					var intersectVertexIndex = result.intValue;
					var sourceEdgeEndVertexIndex = AddSecondaryRoadVertex(cityCell, result.vector2Value1);

					var newEdge = AddSecondaryRoadEdge(cityCell, fromVertexIndex, intersectVertexIndex);
					if (order != -1)
						_connectorInfoList.Add(new ConnectorInfo(cityCell.boundaryEdgeList[^order], newEdge, cityCell));
					AddSecondaryRoadEdge(cityCell, intersectVertexIndex, sourceEdgeEndVertexIndex);
					break;
				}
				case VertexSnapAction.DIVIDE_SOURCE_EDGE_NEW:
				{
					var intersectVertexIndex = AddSecondaryRoadVertex(cityCell, result.vector2Value1);
					var sourceEdgeEndVertexIndex = AddSecondaryRoadVertex(cityCell, result.vector2Value2);

					var newEdge = AddSecondaryRoadEdge(cityCell, fromVertexIndex, intersectVertexIndex);
					if (order != -1)
						_connectorInfoList.Add(new ConnectorInfo(cityCell.boundaryEdgeList[^order], newEdge, cityCell));
					AddSecondaryRoadEdge(cityCell, intersectVertexIndex, sourceEdgeEndVertexIndex);

					var targetEdgeStartVertexIndex = result.edgeValue.startVertexIndex;
					var targetEdgeEndVertexIndex = result.edgeValue.endVertexIndex;

					DestroySecondaryRoadEdge(cityCell, result.edgeValue);

					AddSecondaryRoadEdge(cityCell, targetEdgeStartVertexIndex, intersectVertexIndex);
					AddSecondaryRoadEdge(cityCell, intersectVertexIndex, targetEdgeEndVertexIndex);
					break;
				}
				case VertexSnapAction.MERGE_VERTEX:
				{
					var newEdge = AddSecondaryRoadEdge(cityCell, fromVertexIndex, result.intValue);
					if (order != -1)
						_connectorInfoList.Add(new ConnectorInfo(cityCell.boundaryEdgeList[^order], newEdge, cityCell));
					break;
				}
				case VertexSnapAction.DESTROY:
					break;
				default:
					throw new ArgumentOutOfRangeException();
			}
		}

		foreach (var newVertexIndex in nextLoopVertexList)
		{
			SecondaryRoadGenInnerLoop(newVertexIndex, fromVertexIndex, cityCell, depth + 1);
		}
	}

	private Tuple<int, int> SelectVertexFromBoundary(CityCell cityCell, int order = 1)
	{
		var edge = cityCell.boundaryEdgeList[^order];

		var a = edge.ClippedEnds[LR.LEFT];
		var b = edge.ClippedEnds[LR.RIGHT];

		var initialVertexIndex = AddSecondaryRoadVertex(cityCell, new Vector2((a.x + b.x) / 2, (a.y + b.y) / 2));
		var prevVertexIndex = AddSecondaryRoadVertex(cityCell, new Vector2(a.x, a.y));

		return new Tuple<int, int>(initialVertexIndex, prevVertexIndex);
	}

	private List<Vector2> GridGrowth(Vector2 fromVertexPosition, Vector2 prevVertexPosition, Probability prob)
	{
		var suggestedVertexPositionList = new List<Vector2>();

		var prevDirection = (fromVertexPosition - prevVertexPosition).normalized;
		var rightVec3 = Quaternion.Euler(0, 0, -90f) * prevDirection;
		var right = new Vector2(rightVec3.x, rightVec3.y);

		IWeightedRandomizer<int> countRandom = new DynamicWeightedRandomizer<int>();
		for (var i = 0; i < prob.count.Length; i++)
		{
			countRandom.Add(i + 1, prob.count[i]);
		}

		var growthCount = countRandom.NextWithRemoval();
		var dirList = new List<int>();

		IWeightedRandomizer<int> dirRandom = new DynamicWeightedRandomizer<int>();
		for (var i = 0; i < prob.direction.Length; i++)
		{
			dirRandom.Add(i + 1, prob.direction[i]);
		}

		for (var i = 0; i < growthCount; i++)
		{
			dirList.Add(dirRandom.NextWithRemoval());
		}

		foreach (var i in dirList)
		{
			var distribution =
				new SkewedGeneralizedT((gridGrowthProp.minLen + gridGrowthProp.maxLen) / 2, 5, 0.2, 5, 5);

			var len = (float)distribution.Sample();
			var rotatedDir = Vector3.zero;

			var offsetRangeStart = gridGrowthProp.growthOffsetRange[i - 1, 0];
			var offsetRangeEnd = gridGrowthProp.growthOffsetRange[i - 1, 1];
			var offset = Random.Range(offsetRangeStart, offsetRangeEnd);

			rotatedDir = i switch
			{
				1 =>
					// Forward
					Quaternion.Euler(0, 0, offset) * prevDirection,
				2 =>
					// Turn L
					Quaternion.Euler(0, 0, -offset) * -right,
				3 =>
					// Turn R
					Quaternion.Euler(0, 0, offset) * right,
				_ => rotatedDir
			};

			var dir = new Vector2(rotatedDir.x, rotatedDir.y);
			suggestedVertexPositionList.Add(fromVertexPosition + dir * len);
		}

		return suggestedVertexPositionList;
	}

	private enum VertexSnapAction
	{
		NEW,
		DIVIDE_TARGET_EDGE,
		DIVIDE_SOURCE_EDGE,
		DIVIDE_SOURCE_EDGE_NEW,
		MERGE_VERTEX,
		DESTROY
	}

	private struct VertexValidResult
	{
		public VertexSnapAction action;
		public Vector2 vector2Value1;
		public Vector2 vector2Value2;
		public IndexedEdge edgeValue;
		public int intValue;
	}

	private VertexValidResult IsVertexValid(int prevVertexIndex, Vector2 newVertexPosition, CityCell cityCell)
	{
		GetSecondaryRoadVertex(cityCell, prevVertexIndex, out var prevVertexPosition);

		/* +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-
         * check 01. is new vertex locate in site?
         * +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+- */
		// if not, destroy new vertex.
		if (!IsPointInCityCell(newVertexPosition, cityCell))
		{
			return new VertexValidResult() { action = VertexSnapAction.DESTROY };
		}

		/* +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-
		 * check 02. is new vertex close to another vertex?
		 * +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+- */
		var closestVertexIndex = -1;
		var closestVertexDistance = float.MaxValue;
		foreach (var indexedEdge in _secondaryRoadEdgeDict[cityCell])
		{
			GetSecondaryRoadVertex(cityCell, indexedEdge.startVertexIndex, out var start);
			GetSecondaryRoadVertex(cityCell, indexedEdge.endVertexIndex, out var end);

			var vertexDist = Vector2.Distance(start, newVertexPosition);
			if (vertexDist < closestVertexDistance)
			{
				closestVertexDistance = vertexDist;
				closestVertexIndex = indexedEdge.startVertexIndex;
			}

			vertexDist = Vector2.Distance(end, newVertexPosition);
			if (vertexDist < closestVertexDistance)
			{
				closestVertexDistance = vertexDist;
				closestVertexIndex = indexedEdge.endVertexIndex;
			}
		}

		// if new vertex is close enough to another vertex, merge it.
		if (0 <= closestVertexDistance && closestVertexDistance < gridGrowthBoundary.vertexMerge)
		{
			// if closest vertex is already connected with 4 or more edges, destroy it.
			var closestVertexConnectedEdgeCount = _secondaryRoadEdgeDict[cityCell].Count(e =>
				e.startVertexIndex == closestVertexIndex || e.endVertexIndex == closestVertexIndex);
			if (closestVertexConnectedEdgeCount >= 4)
			{
				return new VertexValidResult() { action = VertexSnapAction.DESTROY };
			}

			// if edge (prev, closest) is intersect with other, destroy it.
			if (CheckIntersectCount(cityCell, prevVertexIndex, closestVertexIndex) > 0)
			{
				Debug.LogWarning("Try to Merge, but Destroyed by Intersection");
				return new VertexValidResult() { action = VertexSnapAction.DESTROY };
			}

			return new VertexValidResult() { action = VertexSnapAction.MERGE_VERTEX, intValue = closestVertexIndex };
		}
		// if new vertex is not close enough to another vertex, destroy it.
		else if (closestVertexDistance < gridGrowthBoundary.vertexDestroy)
		{
			return new VertexValidResult() { action = VertexSnapAction.DESTROY };
		}

		/* +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-
		 * check 03. is new vertex close to another edge?
		 * +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+- */
		IndexedEdge closestIndexedEdge = null;
		var minEdgeProjDist = float.MaxValue;
		foreach (var indexedEdge in _secondaryRoadEdgeDict[cityCell])
		{
			GetSecondaryRoadVertex(cityCell, indexedEdge.startVertexIndex, out var start);
			GetSecondaryRoadVertex(cityCell, indexedEdge.endVertexIndex, out var end);

			var edgeProjDist = ToolBox.GetDistance(start, end, newVertexPosition);
			if (edgeProjDist < minEdgeProjDist)
			{
				minEdgeProjDist = edgeProjDist;
				closestIndexedEdge = indexedEdge;
			}
		}

		if (0 <= minEdgeProjDist && minEdgeProjDist <= gridGrowthBoundary.edgeDivide)
		{
			// if close vertex exist, merge that vertex and new vertex
			if (0 <= closestVertexDistance && closestVertexDistance < gridGrowthBoundary.vertexMerge)
			{
				// if closest vertex is already connected with 4 or more edges, destroy it.
				var closestVertexConnectedEdgeCount = _secondaryRoadEdgeDict[cityCell].Count(e =>
					e.startVertexIndex == closestVertexIndex || e.endVertexIndex == closestVertexIndex);
				if (closestVertexConnectedEdgeCount >= 4)
				{
					return new VertexValidResult() { action = VertexSnapAction.DESTROY };
				}

				// if edge (prev, closest) is intersect with other, destroy it.
				if (CheckIntersectCount(cityCell, prevVertexIndex, closestVertexIndex) > 0)
				{
					Debug.LogWarning("Try to Merge, but Destroyed by Intersection");
					return new VertexValidResult() { action = VertexSnapAction.DESTROY };
				}

				return new VertexValidResult()
					{ action = VertexSnapAction.MERGE_VERTEX, intValue = closestVertexIndex };
			}

			// if not, divide target edge.
			GetSecondaryRoadVertex(cityCell, closestIndexedEdge.startVertexIndex, out var edgeStart);
			GetSecondaryRoadVertex(cityCell, closestIndexedEdge.endVertexIndex, out var edgeEnd);

			var (_, proj) = ToolBox.GetProjection(edgeStart, edgeEnd, newVertexPosition);
			var divideVertexPosition = edgeStart + proj;

			// if edge (prev, divide) is intersect with other, destroy it.
			if (CheckIntersectCount(cityCell, prevVertexIndex, divideVertexPosition, out var _, out var _) > 0)
			{
				Debug.LogWarning("Try to Merge, but Destroyed by Intersection");
				return new VertexValidResult() { action = VertexSnapAction.DESTROY };
			}

			return new VertexValidResult()
			{
				action = VertexSnapAction.DIVIDE_TARGET_EDGE,
				edgeValue = closestIndexedEdge,
				vector2Value1 = divideVertexPosition
			};
		}

		/* +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-
		 * check 04. intersect?
		 * +-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+- */
		CheckIntersectCount(cityCell, prevVertexIndex, newVertexPosition, out var intersectEdgeList,
			out var intersectPointList);

		// if number of intersected edges is bigger than 1, destroy.
		if (intersectEdgeList.Count >= 1)
		{
			return new VertexValidResult() { action = VertexSnapAction.DESTROY };
		}

		return new VertexValidResult() { action = VertexSnapAction.NEW, vector2Value1 = newVertexPosition };
	}

	private int CheckIntersectCount(CityCell cityCell, int fromIndex, int toIndex)
	{
		GetSecondaryRoadVertex(cityCell, fromIndex, out var prevVertexPosition);
		GetSecondaryRoadVertex(cityCell, toIndex, out var closestVertexPosition);

		var intersectEdgeList = new List<IndexedEdge>();
		foreach (var indexedEdge in _secondaryRoadEdgeDict[cityCell])
		{
			if (indexedEdge.startVertexIndex == fromIndex ||
			    indexedEdge.endVertexIndex == fromIndex ||
			    indexedEdge.startVertexIndex == toIndex ||
			    indexedEdge.endVertexIndex == toIndex) continue;

			GetSecondaryRoadVertex(cityCell, indexedEdge.startVertexIndex, out var start);
			GetSecondaryRoadVertex(cityCell, indexedEdge.endVertexIndex, out var end);

			var intersectResult = ToolBox.GetIntersectPoint(prevVertexPosition, closestVertexPosition, start, end);

			if (intersectResult.Item1)
			{
				intersectEdgeList.Add(indexedEdge);
			}
		}

		return intersectEdgeList.Count;
	}

	private int CheckIntersectCount(CityCell cityCell, int fromIndex, Vector2 toVec2,
		out List<IndexedEdge> intersectEdgeList, out List<Vector2> intersectPointList)
	{
		intersectEdgeList = new List<IndexedEdge>();
		intersectPointList = new List<Vector2>();

		GetSecondaryRoadVertex(cityCell, fromIndex, out var prevVertexPosition);

		foreach (var indexedEdge in _secondaryRoadEdgeDict[cityCell])
		{
			if (indexedEdge.startVertexIndex == fromIndex ||
			    indexedEdge.endVertexIndex == fromIndex) continue;

			GetSecondaryRoadVertex(cityCell, indexedEdge.startVertexIndex, out var start);
			GetSecondaryRoadVertex(cityCell, indexedEdge.endVertexIndex, out var end);

			var intersectResult = ToolBox.GetIntersectPoint(prevVertexPosition, toVec2, start, end);

			if (intersectResult.Item1)
			{
				intersectEdgeList.Add(indexedEdge);
				intersectPointList.Add(intersectResult.Item2);
			}
		}

		return intersectEdgeList.Count;
	}

	private void InitSecondaryRoadDict(List<CityCell> cityCellList)
	{
		foreach (var cityCell in cityCellList)
		{
			if (!_secondaryRoadVertexDict.ContainsKey(cityCell))
				_secondaryRoadVertexDict.Add(cityCell, new List<Vector2>());
			if (!_secondaryRoadEdgeDict.ContainsKey(cityCell))
				_secondaryRoadEdgeDict.Add(cityCell, new List<IndexedEdge>());
		}
	}

	private int AddSecondaryRoadVertex(CityCell cityCell, Vector2 vertex)
	{
		_secondaryRoadVertexDict[cityCell].Add(vertex);
		return _secondaryRoadVertexDict[cityCell].Count - 1;
	}

	private bool GetSecondaryRoadVertex(CityCell cityCell, int vertexIndex, out Vector2 position)
	{
		if (!_secondaryRoadVertexDict.ContainsKey(cityCell) ||
		    _secondaryRoadVertexDict[cityCell].Count <= vertexIndex)
		{
			position = Vector2.zero;
			Debug.LogError("Out of Index [Secondary Road Vertex]");
			return false;
		}

		position = _secondaryRoadVertexDict[cityCell][vertexIndex];
		return true;
	}

	private bool ModifySecondaryRoadVertex(CityCell cityCell, int vertexIndex, Vector2 modifiedValue)
	{
		if (!_secondaryRoadVertexDict.ContainsKey(cityCell) ||
		    _secondaryRoadVertexDict[cityCell].Count <= vertexIndex) return false;

		_secondaryRoadVertexDict[cityCell][vertexIndex] = modifiedValue;
		return true;
	}

	private IndexedEdge AddSecondaryRoadEdge(CityCell cityCell, int startVertexIndex, int endVertexIndex)
	{
		var edge = new IndexedEdge()
			{ startVertexIndex = startVertexIndex, endVertexIndex = endVertexIndex };
		_secondaryRoadEdgeDict[cityCell].Add(edge);

		return edge;
	}

	private void DestroySecondaryRoadEdge(CityCell cityCell, IndexedEdge edge)
	{
		_secondaryRoadEdgeDict[cityCell].Remove(edge);
	}

	private void DrawSite(ref Texture2D tx)
	{
		if (_cityCellColorList.Count == 0)
		{
			for (var i = 0; i < _cityCellDict.Keys.Count; i++)
			{
				_cityCellColorList.Add(Random.ColorHSV(0f, 1f, 1f, 1f, 0.5f, 1f));
			}
		}
		
		var c = 0;
		foreach (var (_, siteList) in _cityCellDict)
		{
			foreach (var site in siteList)
			{
				for (var i = -4; i <= 4; i++)
				{
					for (var j = -4; j <= 4; j++)
					{
						tx.SetPixel((int)(site.x + i), (int)(site.y + j), _cityCellColorList[c]);
					}
				}
			}

			c++;
		}
	}

	private void DrawDivider(ref Texture2D tx)
	{
		for (var i = 0; i < _dividerBorder.Count / 2; i++)
		{
			DrawLineThick(new Vector2f(_dividerBorder[2 * i].x, _dividerBorder[2 * i].y),
				new Vector2f(_dividerBorder[2 * i + 1].x, _dividerBorder[2 * i + 1].y), tx, Color.yellow);
		}

		foreach (var edge in _dividerEdgeList.Where(edge => edge.ClippedEnds != null))
		{
			DrawLineThick(edge.ClippedEnds[LR.LEFT], edge.ClippedEnds[LR.RIGHT], tx, Color.yellow);
		}
	}

	private void DrawEdges(ref Texture2D tx)
	{
		foreach (var edge in _edgeList.Where(edge => edge.ClippedEnds != null))
		{
			DrawLineThick(edge.ClippedEnds[LR.LEFT], edge.ClippedEnds[LR.RIGHT], tx,
				_primaryRoadList.Contains(edge) ? Color.red : Color.grey);
		}

		foreach (var (cityCell, list) in _secondaryRoadEdgeDict)
		{
			foreach (var indexedEdge in list)
			{
				GetSecondaryRoadVertex(cityCell, indexedEdge.startVertexIndex, out var start);
				GetSecondaryRoadVertex(cityCell, indexedEdge.endVertexIndex, out var end);

				DrawLineThick(start, end, tx, Color.blue);
			}
		}
	}

	private bool IsEdgeBoundary(Edge edge)
	{
		return edge.ClippedEnds == null ||
		       edge.ClippedEnds[LR.LEFT].x is 0 ||
		       edge.ClippedEnds[LR.LEFT].x == sampleRegionSize.x ||
		       edge.ClippedEnds[LR.LEFT].y == 0 ||
		       edge.ClippedEnds[LR.LEFT].y == sampleRegionSize.y ||
		       edge.ClippedEnds[LR.RIGHT].x == 0 ||
		       edge.ClippedEnds[LR.RIGHT].x == sampleRegionSize.x ||
		       edge.ClippedEnds[LR.RIGHT].y == 0 ||
		       edge.ClippedEnds[LR.RIGHT].y == sampleRegionSize.y;
	}

	private void DrawLineThin(Vector2f p0, Vector2f p1, Texture2D tx, Color c, int offset = 0)
	{
		var x0 = (int)p0.x;
		var y0 = (int)p0.y;
		var x1 = (int)p1.x;
		var y1 = (int)p1.y;

		var dx = Mathf.Abs(x1 - x0);
		var dy = Mathf.Abs(y1 - y0);
		var sx = x0 < x1 ? 1 : -1;
		var sy = y0 < y1 ? 1 : -1;
		var err = dx - dy;

		while (true)
		{
			tx.SetPixel(x0 + offset, y0 + offset, c);

			if (x0 == x1 && y0 == y1) break;

			var e2 = 2 * err;
			if (e2 > -dy)
			{
				err -= dy;
				x0 += sx;
			}

			if (e2 >= dx) continue;

			err += dx;
			y0 += sy;
		}
	}

	private void DrawLineThick(Vector2f p0, Vector2f p1, Texture2D tx, Color c, int offset = 0)
	{
		var dir = (new Vector2(p1.x, p1.y) - new Vector2(p0.x, p0.y));
		var len = dir.magnitude;
		dir.Normalize();

		var rightVec3 = Quaternion.Euler(0, 0, -90) * dir;
		var right = new Vector2(rightVec3.x, rightVec3.y);

		var lt = new Vector2(p0.x, p0.y) - right * 1;
		var rt = lt + dir * len;
		var rb = rt + right * 2;
		var lb = rb - dir * len;

		DrawLineThin(new Vector2f(lt.x, lt.y), new Vector2f(rt.x, rt.y), tx, c, offset);
		DrawLineThin(new Vector2f(rt.x, rt.y), new Vector2f(rb.x, rb.y), tx, c, offset);
		DrawLineThin(new Vector2f(rb.x, rb.y), new Vector2f(lb.x, lb.y), tx, c, offset);
		DrawLineThin(new Vector2f(lb.x, lb.y), new Vector2f(lt.x, lt.y), tx, c, offset);
		DrawLineThin(p0, p1, tx, c, offset);
	}

	private void DrawLineThick(Vector2 p0, Vector2 p1, Texture2D tx, Color c, int offset = 0)
	{
		DrawLineThick(new Vector2f(p0.x, p0.y), new Vector2f(p1.x, p1.y), tx, c, offset);
	}

	private bool IsPointInSite(Vector2 point, Site site)
	{
		return site.Edges.Where(edge => edge.ClippedEnds != null).All(edge =>
		{
			var a = edge.ClippedEnds[LR.LEFT];
			var b = edge.ClippedEnds[LR.RIGHT];
			return !ToolBox.GetIntersectPoint(site.x, site.y, point.x, point.y, a.x, a.y, b.x, b.y).Item1;
		});
	}

	private bool IsPointInCityCell(Vector2 point, CityCell cityCell)
	{
		var cityCellCenter = Vector2.zero;
		foreach (var site in cityCell.siteList)
		{
			cityCellCenter += new Vector2(site.x, site.y);
		}

		cityCellCenter /= cityCell.siteList.Count;

		var altered = new List<Tuple<Vector2, Vector2>>();

		foreach (var edge in cityCell.boundaryEdgeList)
		{
			var a = new Vector2(edge.ClippedEnds[LR.LEFT].x, edge.ClippedEnds[LR.LEFT].y);
			var b = new Vector2(edge.ClippedEnds[LR.RIGHT].x, edge.ClippedEnds[LR.RIGHT].y);

			var dir = b - a;
			var dir3D = new Vector3(dir.x, 0, dir.y);
			var r = Quaternion.Euler(0, -90, 0) * dir3D;
			var r2D = new Vector2(r.x, r.z);

			var center = (a + b) / 2;

			if (!cityCell.siteList.Any(site => IsPointInSite(center + r2D.normalized * 20f, site)))
			{
				r = Quaternion.Euler(0, 90, 0) * dir3D;
				r2D = new Vector2(r.x, r.z);
			}

			center += r2D.normalized * 20f;
			a = center + dir.normalized * (-dir.magnitude * 0.5f * 1.5f);
			b = center + dir.normalized * (dir.magnitude * 0.5f * 1.5f);

			altered.Add(new Tuple<Vector2, Vector2>(a, b));
		}

		return altered.All(t => { return !ToolBox.GetIntersectPoint(cityCellCenter, point, t.Item1, t.Item2).Item1; });
	}
}