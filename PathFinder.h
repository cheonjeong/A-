#pragma once

#include <functional>
#include <unordered_map>
#include <vector>

class Player;

struct Node
{
	int row;
	int col;
	int weight;
	
	float actualCostFromStart;	// g
	float heuristic;			// h

	Node* parent;

	bool inOpenSet;
	bool inCloseSet;

	Node()
		: weight(0), actualCostFromStart(0), heuristic(0.0f),
		parent(nullptr),
		inOpenSet(false), inCloseSet(false) { }
};

class PathFinder
{
public:
	void Setup(int row, int col, int cellWidth, int cellHeight);
	void SetObstacle(int row, int col);

	void Find(int sx, int sy, int ex, int ey);

	void Play();
	void Stop();

	void UpdatePlayer(Player* player);

private:
	float ComputeHeuristic(int cx, int cy, int ex, int ey);

private:
	int row_;
	int col_;
	int cellWidth_;
	int cellHeight_;
	std::vector<std::vector<Node>> nodes_;
	std::unordered_map<Node*, Node*> path_;
	Node* current_;
	Node* start_;
	bool finish_;
};