// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "Ai/NavigationUtil.h"
#include "PathFinder.h"

#include <bulletPhysics\btBulletCollisionCommon.h>

namespace tse
{
	class ThePathFinder
	{
	public:
		ThePathFinder(const std::vector<GeneralNode*>& nodeList, const std::vector<GeneralEdge*>& edgeList);
		~ThePathFinder();

		//needed UWorld for raycasting
		std::vector<GeneralNode*> CalculatePathToGoal(TSVector3 startLocation, TSVector3 endLocation, btCollisionWorld* world, bool smoothPath = false);
		std::vector<GeneralNode*> CalculatePathToType(TSVector3 startLocation, InterestType type, btCollisionWorld* world, bool smoothPath = false);
		std::vector<GeneralNode*> SmoothPath(std::vector<GeneralNode*> existingPath, btCollisionWorld* world);

		bool IsWalking() { return isWalking; }

	protected:
		std::vector<StarNode*> starNodes;
		std::vector<DijkNode*> dijkNodes;
		bool isWalking;

		GeneralNode* startNode;
		GeneralNode* endNode;
		DijkNode* goalNode;
	};
}
