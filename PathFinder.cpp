#include "stdafx.h"
#include "PathFinder.h"
#include "Player.h"
#include "Transform.h"

#include <array>
#include <queue>

void PathFinder::Setup(int row, int col, int cellWidth, int cellHeight)
{
	row_ = row;
	col_ = col;
	cellWidth_ = cellWidth;
	cellHeight_ = cellHeight;

	nodes_.resize(col, std::vector<Node>(row));
	start_ = nullptr;
	finish_ = true;
}

void PathFinder::SetObstacle(int row, int col)
{
	nodes_[row][col].weight = 1000;
}

void PathFinder::Find(int sx, int sy, int ex, int ey)
{
	static std::array<int, 8> dirX = { -1, -1, -1, 0, 1, 1, 1, 0 };
	static std::array<int, 8> dirY = { -1, 0, 1, 1, 1, 0, -1, -1 };

	std::vector<Node*> openSet;

	Node* current = &nodes_[sy][sx];
	Node* goal = &nodes_[ey][ex];

	current->row = sy;
	current->col = sx;
	current->inCloseSet = true;

	goal->row = ey;
	goal->col = ex;

	do
	{
		for (size_t i = 0; i < 8; ++i)
		{
			int x = dirX[i];
			int y = dirY[i];

			if (current->row + y < 0 || current->row + y >= nodes_.size() ||
				current->col + x < 0 || current->col + x >= nodes_.size())
				continue;

			// 이웃 노드 초기화
			Node* neighbor = &nodes_[current->row + y][current->col + x];
			neighbor->row = current->row + y;
			neighbor->col = current->col + x;

			// 이웃 노드가 이미 평가 안됐으면
			if (!neighbor->inCloseSet)
			{
				// 아직 평가 리스트에 들어오지 않았으면
				if (!neighbor->inOpenSet)
				{
					// 경로 설정
					path_[neighbor] = neighbor;
					path_[neighbor]->parent = current;

					neighbor->heuristic = ComputeHeuristic(current->col, current->row, goal->col, goal->row);
					neighbor->actualCostFromStart = current->actualCostFromStart + neighbor->weight + sqrtf(x * x + y * y);
					neighbor->inOpenSet = true;
					openSet.push_back(neighbor);
				}
				else
				{
					float newG = current->actualCostFromStart + neighbor->weight + sqrtf(x * x + y * y);
					if (newG < neighbor->actualCostFromStart)
					{
						path_[neighbor]->parent = current;
						neighbor->actualCostFromStart = newG;
					}
				}
			}
		}

		if (openSet.empty())
			break;

		auto it = std::min_element(openSet.begin(), openSet.end(),
			[](const Node* lhs, const Node* rhs) {
			return lhs->heuristic + lhs->actualCostFromStart < rhs->heuristic + rhs->actualCostFromStart;
		});

		current = *it;
		openSet.erase(it);
		current->inOpenSet = false;
		current->inCloseSet = true;
	} while (current != goal);

	if (current == goal)
		start_ = goal;
}

void PathFinder::Play()
{
	current_ = start_;
	finish_ = false;
}

void PathFinder::Stop()
{
	finish_ = true;
}

void PathFinder::UpdatePlayer(Player* player)
{
	if (finish_)
		return;

	if (current_)
	{
		D3DXVECTOR3 position = player->GetTransform()->GetPosition();
		D3DXVECTOR3 destination(
			(-(cellWidth_ * col_) + 1) + (current_->col * cellWidth_ * 2.0f),
			position.y,
			(-(cellHeight_ * row_) + 1) + (current_->row * cellHeight_ * 2.0f));

		D3DXVECTOR3 direction = destination - position;
		float distance = D3DXVec3Length(&direction);

		// player->LookAt(direction / distance);
		player->Move(direction / distance * 0.01f);

		if (distance <= 0.15f)
			current_ = current_->parent;
	}
	else
		finish_ = true;
}

float PathFinder::ComputeHeuristic(int cx, int cy, int ex, int ey)
{
	return std::sqrtf((cx - ex) * (cx - ex) + (cy - ey) * (cy - ey));
}