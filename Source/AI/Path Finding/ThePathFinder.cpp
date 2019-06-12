#include <stdafx.h>

#include <Defines.h>
#include "Ai/Path Finding/ThePathFinder.h"

namespace tse
{
	ThePathFinder::ThePathFinder(const std::vector<GeneralNode*>& nodelist, const std::vector<GeneralEdge*>& edgeList)
	{
		startNode = new GeneralNode();
		endNode = new GeneralNode();
		goalNode = new DijkNode();
		for (GeneralNode* n : nodelist)
		{
			//Generate nodes for AStar
			StarNode* starNode = new StarNode();
			starNode->SetPosition(n->position);
			starNode->SetType(n->type);
			starNodes.push_back(starNode);

			//Generate nodes for Dijkstra
			DijkNode* dijkNode = new DijkNode();
			dijkNode->SetPosition(n->position);
			dijkNode->SetType(n->type);
			dijkNodes.push_back(dijkNode);
		}
		for (GeneralEdge* e : edgeList)
		{
			//Generate edges for AStar
			starNodes[e->fromIndex]->addChild(starNodes[e->toIndex], e->distance);
			starNodes[e->toIndex]->addChild(starNodes[e->fromIndex], e->distance);

			//Generate edges for Dijkstra
			dijkNodes[e->fromIndex]->addChild(dijkNodes[e->toIndex], e->distance);
			dijkNodes[e->toIndex]->addChild(dijkNodes[e->fromIndex], e->distance);
		}
	}

	ThePathFinder::~ThePathFinder()
	{
	}

	std::vector<GeneralNode*> ThePathFinder::CalculatePathToGoal(TSVector3 startLocation, TSVector3 endLocation, btCollisionWorld * world, bool smoothPath)
	{
		if (starNodes.empty()) {
			return std::vector<GeneralNode*>();
		}
		float endDist = FLT_MAX;
		float startDist = FLT_MAX;

		int startIndex = 0;
		int endIndex = 0;
		for (size_t i = 0; i < starNodes.size(); ++i)
		{
			btCollisionWorld::ClosestRayResultCallback hit;
			float nodeDist;

			//Closest node to start
			nodeDist = starNodes[i]->GetPosition().GetVecBt().distance(startLocation.GetVecBt());
			world->rayTest(startLocation.GetVecBt(), starNodes[i]->GetPosition().GetVecBt(), hit);
			if (nodeDist < startDist && !hit.hasHit()) {
				startIndex = (int)i;
				startDist = nodeDist;
			}

			//Closest node to end
			nodeDist = starNodes[i]->GetPosition().GetVecBt().distance(endLocation.GetVecBt());
			world->rayTest(endLocation.GetVecBt(), starNodes[i]->GetPosition().GetVecBt(), hit);
			if (nodeDist < endDist && !hit.hasHit()) {
				endIndex = (int)i;
				endDist = nodeDist;
			}
		}
		//Calculate path
		PathFinder<StarNode> path;
		std::vector<StarNode*> starSolution;
		path.setStart(*starNodes[startIndex]);
		path.setGoal(*starNodes[endIndex]);

		bool isFound = path.findPath<AStar>(starSolution);
		if (!isFound)
		{
			//Failed to find path!
			return std::vector<GeneralNode*>();
		}

		//Convert to general node list
		//And add current location and goal location
		std::vector<GeneralNode*> solution;

		startNode->position = startLocation;
		solution.push_back(startNode);

		for (StarNode*n : starSolution)
		{
			GeneralNode* node = new GeneralNode();
			node->position = n->GetPosition();
			node->type = n->GetType();
			solution.push_back(node);
		}
		endNode->position = endLocation;
		solution.push_back(endNode);

		if (smoothPath) {
			solution = SmoothPath(solution, world);
		}
		return solution;
	}

	std::vector<GeneralNode*> ThePathFinder::CalculatePathToType(TSVector3 startLocation, InterestType type, btCollisionWorld * world, bool smoothPath)
	{
		if (dijkNodes.empty()) {
			return std::vector<GeneralNode*>();
		}

		float startDist = FLT_MAX;
		int startIndex = 0;
		for (size_t i = 0; i < dijkNodes.size(); ++i)
		{
			//FHitResult hit;
			float nodeDist;

			//Closest node to start
			nodeDist = dijkNodes[i]->GetPosition().GetVecBt().distance(startLocation.GetVecBt());
			btCollisionWorld::ClosestRayResultCallback hit;
			world->rayTest(startLocation.GetVecBt(), starNodes[i]->GetPosition().GetVecBt(), hit);
			if (nodeDist < startDist && !hit.hasHit()) {
				startIndex = (int)i;
				startDist = nodeDist;
			}
		}
		goalNode->SetType(type);

		PathFinder<DijkNode> path;
		std::vector<DijkNode*> dijkSolution;
		path.setStart(*dijkNodes[startIndex]);
		path.setGoal(*goalNode);

		bool isFound = path.findPath<Dijkstra>(dijkSolution);
		if (!isFound)
		{
			//Failed to find path!
			return std::vector<GeneralNode*>();
		}

		//Convert to general node list
		//And add current location
		std::vector<GeneralNode*> solution;

		startNode->position = startLocation;
		solution.push_back(startNode);

		for (DijkNode*n : dijkSolution)
		{
			GeneralNode* node = new GeneralNode();
			node->position = n->GetPosition();
			node->type = n->GetType();
			solution.push_back(node);
		}

		if (smoothPath) {
			solution = SmoothPath(solution, world);
		}
		return solution;
	}

	std::vector<GeneralNode*> ThePathFinder::SmoothPath(std::vector<GeneralNode*> existingPath, btCollisionWorld * world)
	{
		if (existingPath.size() < 2) {
			return existingPath;
		}

		tse::tsuint64 idx1 = 0;
		tse::tsuint64 idx2;
		//FHitResult hit;
		while (idx1 < existingPath.size())
		{
			idx2 = idx1 + 2;
			while (idx2 < existingPath.size())
			{
				btCollisionWorld::ClosestRayResultCallback hit;
				world->rayTest(existingPath[idx1]->position.GetVecBt(), existingPath[idx2]->position.GetVecBt(), hit);
				if (!hit.hasHit()) {
					existingPath.erase(existingPath.begin() + (idx1 + 1), existingPath.begin() + (idx2));
					idx2 = idx1 + 2;
				}
				else {
					++idx2;
				}
			}
			++idx1;
		}
		return existingPath;
	}
}