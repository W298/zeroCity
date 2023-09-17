using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using csDelaunay;
using Random = UnityEngine.Random;

class WeightedEdge : IComparable<WeightedEdge>
{
	public int src, dest, weight;

	public int CompareTo(WeightedEdge other)
	{
		return this.weight - other.weight;
	}
}

public static class ToolBox
{
	public static float PoissonDisk(float radius, Vector2 sampleRegionSize, int numSamplesBeforeRejection,
		ref List<Vector2> pointList)
	{
		var cellSize = radius / Mathf.Sqrt(2);
		var grid = new int[Mathf.CeilToInt(sampleRegionSize.x / cellSize),
			Mathf.CeilToInt(sampleRegionSize.y / cellSize)];
		var spawnPoints = new List<Vector2> { sampleRegionSize / 2 };

		while (spawnPoints.Count > 0)
		{
			var spawnIndex = Random.Range(0, spawnPoints.Count);
			var spawnCentre = spawnPoints[spawnIndex];
			var candidateAccepted = false;

			for (var i = 0; i < numSamplesBeforeRejection; i++)
			{
				var angle = Random.value * Mathf.PI * 2;
				var dir = new Vector2(Mathf.Sin(angle), Mathf.Cos(angle));
				var candidate = spawnCentre + dir * Random.Range(radius, 2 * radius);

				if (!IsCandidateValid(candidate, grid, radius, sampleRegionSize, cellSize, ref pointList)) continue;

				pointList.Add(candidate);
				spawnPoints.Add(candidate);

				grid[(int)(candidate.x / cellSize), (int)(candidate.y / cellSize)] = pointList.Count;
				candidateAccepted = true;

				break;
			}

			if (!candidateAccepted) spawnPoints.RemoveAt(spawnIndex);
		}

		return cellSize;
	}

	public static Voronoi VoronoiDiagram(Vector2 sampleRegionSize, bool isLloydIterationEnabled, int lloydIteration,
		ref List<Vector2> pointList)
	{
		var pointsF = pointList.Select(vector2 => new Vector2f(vector2.x, vector2.y)).ToList();

		var bounds = new Rectf(0, 0, sampleRegionSize.x, sampleRegionSize.y);
		var voronoi = new Voronoi(pointsF, bounds);

		if (isLloydIterationEnabled) voronoi.LloydRelaxation(lloydIteration);

		return voronoi;
	}

	public static Tuple<bool, Vector2> GetIntersectPoint(Vector2 e1Start, Vector2 e1End, Vector2 e2Start, Vector2 e2End)
	{
		return GetIntersectPoint(e1Start.x, e1Start.y, e1End.x, e1End.y, e2Start.x, e2Start.y, e2End.x, e2End.y);
	}

	public static Tuple<bool, Vector2> GetIntersectPoint(float x1, float y1, float x2, float y2, float x3, float y3, float x4,
		float y4)
	{
		var denominator = ((y4 - y3) * (x2 - x1)) - ((x4 - x3) * (y2 - y1));
		var ua = ((x4 - x3) * (y1 - y3)) - ((y4 - y3) * (x1 - x3));
		ua /= denominator;
		
		var ub = ((x2 - x1) * (y1 - y3)) - ((y2 - y1) * (x1 - x3));
		ub /= denominator;
		
		if (ua >= 0 && ua <= 1 && ub >= 0 && ub <= 1)
		{
			var intersectionX = x1 + (ua * (x2 - x1));
			var intersectionY = y1 + (ua * (y2 - y1));
			return new Tuple<bool, Vector2>(true, new Vector2(intersectionX, intersectionY));
		}

		return new Tuple<bool, Vector2>(false, Vector2.zero);
	}

	public static float GetDistance(Edge edge, Vector2 point)
	{
		return GetDistance(new Vector2(edge.ClippedEnds[LR.LEFT].x, edge.ClippedEnds[LR.LEFT].y),
			new Vector2(edge.ClippedEnds[LR.RIGHT].x, edge.ClippedEnds[LR.RIGHT].y), point);
	}

	public static float GetDistance(Vector2 edgeStart, Vector2 edgeEnd, Vector2 point)
	{
		var lr = edgeEnd - edgeStart;
		var lp = point - edgeStart;
		var scalar = Math.Max(0.0f, Mathf.Min(Vector2.Dot(lr, lr), Vector2.Dot(lr, lp))) / Vector2.Dot(lr, lr);
		var proj = scalar * lr;
		var normal = lp - proj;

		return normal.magnitude;
	}

	public static Tuple<bool, Vector2> GetProjection(Edge edge, Vector2 point)
	{
		return GetProjection(new Vector2(edge.ClippedEnds[LR.LEFT].x, edge.ClippedEnds[LR.LEFT].y),
			new Vector2(edge.ClippedEnds[LR.RIGHT].x, edge.ClippedEnds[LR.RIGHT].y), point);
	}

	public static Tuple<bool, Vector2> GetProjection(Vector2 edgeStart, Vector2 edgeEnd, Vector2 point)
	{
		var lr = edgeEnd - edgeStart;
		var lp = point - edgeStart;
		var scalar = Vector2.Dot(lp, lr) / Vector2.Dot(lr, lr);
		var proj = scalar * lr;

		return new Tuple<bool, Vector2>(0 <= scalar && scalar <= lr.magnitude, proj);
	}

	private static bool IsCandidateValid(Vector2 candidate, int[,] grid, float radius, Vector2 sampleRegionSize,
		float cellSize, ref List<Vector2> pointList)
	{
		if (!(candidate.x >= 0) || !(candidate.x < sampleRegionSize.x) ||
		    !(candidate.y >= 0) || !(candidate.y < sampleRegionSize.y))
			return false;

		var cellX = (int)(candidate.x / cellSize);
		var cellY = (int)(candidate.y / cellSize);
		var searchStartX = Mathf.Max(0, cellX - 2);
		var searchEndX = Mathf.Min(cellX + 2, grid.GetLength(0) - 1);
		var searchStartY = Mathf.Max(0, cellY - 2);
		var searchEndY = Mathf.Min(cellY + 2, grid.GetLength(1) - 1);

		for (var x = searchStartX; x <= searchEndX; x++)
		{
			for (var y = searchStartY; y <= searchEndY; y++)
			{
				var pointIndex = grid[x, y] - 1;
				if (pointIndex == -1) continue;

				var sqrDst = (candidate - pointList[pointIndex]).sqrMagnitude;
				if (sqrDst < radius * radius) return false;
			}
		}

		return true;
	}
}