// Fill out your copyright notice in the Description page of Project Settings.

#pragma once

#include "Ai\NavigationUtil.h"
#include <vector>
#include <fstream>
#include <string>

#include <bulletPhysics\btBulletCollisionCommon.h>

namespace tse
{
	class NavigationGenerator
	{
	public:
		NavigationGenerator(TSVector3 startPos, btBoxShape boundBox, int gridSize = 100, float maxWalkableSlope = 45.f, int agentWidth = 0, int agentHeight = 0);
		~NavigationGenerator();

		//Needed UWorld for raycasting
		//Generates grid at once
		void GenerateAtOnce(btCollisionWorld* world);
		//Allows generation of grid over time
		void StartGeneration();
		void GenerationStep(btCollisionWorld* world);
		void AddNodeOfIntrest(TSVector3 position, InterestType type, btCollisionWorld* world);

		void SetStartPosition(TSVector3 startPos);
		void SetBounds(btBoxShape bounds);
		void SetGridSize(int gridSize);
		void SetWalkableSlope(float walkableSlope);
		void SetAgentWidth(int agentWidth);
		void SetAgentHeight(int agentHeight);

		const std::vector<GeneralNode*>& GetNodes() { return nodelist; }
		const std::vector<GeneralEdge*>& GetEdges() { return edgeList; }
		const std::vector<GeneralNode*>& GetFrontier() { return frontier; }
		const bool IsGenerating() { return isGenerating; }
		bool JustFinished();

		int ExportToFile(tsString fileName);
		int ImportFromFile(tsString fileName);

	private:
		TSVector3 startPos;
		btBoxShape boundBox = btBoxShape(btVector3());
		btSphereShape* agentSphere;

		int gridSize;
		tsfloat agentWidth;
		tsfloat agentHeight;
		float maxWalkHeight;
		bool isGenerating;
		bool isJustFinished;

		std::string nodeHeader = "Nodes:";
		std::string edgeHeader = "Edges:";
		char* delim = ";";

		std::vector<GeneralNode*> nodelist;
		std::vector<GeneralEdge*> edgeList;

		std::vector<GeneralNode*> frontier;

		void AddEdgesToNeighbours(GeneralNode* node, btCollisionWorld* world);
		bool IsNodeInRange(TSVector3 positionToCheck, std::vector<GeneralNode*>& listToCheck);
	};
}
