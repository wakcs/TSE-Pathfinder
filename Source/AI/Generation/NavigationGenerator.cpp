#include <stdafx.h>

#include "Ai/Generation/NavigationGenerator.h"


namespace tse
{
	NavigationGenerator::NavigationGenerator(TSVector3 startPos, btBoxShape boundBox, int gridSize, float maxWalkableSlope, int agentWidth, int agentHeight)
	{
		SetStartPosition(startPos);
		SetBounds(boundBox);
		SetGridSize(gridSize);
		SetWalkableSlope(maxWalkableSlope);
		SetAgentWidth(agentWidth);
		SetAgentHeight(agentHeight);
		isGenerating = false;
	}

	NavigationGenerator::~NavigationGenerator()
	{
	}

	void NavigationGenerator::GenerateAtOnce(btCollisionWorld * world)
	{
		StartGeneration();
		while (isGenerating)
		{
			GenerationStep(world);
		}
	}

	void NavigationGenerator::StartGeneration()
	{
		nodelist.clear();
		edgeList.clear();

		//init start node
		frontier.clear();
		GeneralNode* startFrontier = new GeneralNode();
		startFrontier->position = startPos;
		frontier.push_back(startFrontier);
		isGenerating = true;
	}

	void NavigationGenerator::GenerationStep(btCollisionWorld * world)
	{
		if (!isGenerating)
		{
			isGenerating = false;
			return;
		}
		if (frontier.empty()) {
			isGenerating = false;
			isJustFinished = true;
			return;
		}
		for (int dir = 0; dir <= 4; ++dir)
		{
			//only checks and adds new frontier if its a new direction
			if (dir != frontier[0]->comesFrom) {
				GeneralNode* newFrontier = new GeneralNode();
				newFrontier->position = frontier[0]->position;
				newFrontier->comesFrom = (NavDirection)dir;
				//TODO: Check if axes are correct with the TSE axes
				switch (dir)
				{
				case NAV_NORTH:
					newFrontier->position.SetX(newFrontier->position.GetX() + gridSize);
					break;
				case NAV_EAST:
					newFrontier->position.SetZ(newFrontier->position.GetZ() + gridSize);
					break;
				case NAV_SOUTH:
					newFrontier->position.SetX(newFrontier->position.GetX() - gridSize);
					break;
				case NAV_WEST:
					newFrontier->position.SetZ(newFrontier->position.GetZ() - gridSize);
					break;
				}

				//Check if node collides with anything
				btVector3 vMaxHeight = newFrontier->position.GetVecBt();
				vMaxHeight.setY(vMaxHeight.y() + maxWalkHeight);
				btVector3 vMinHeight = newFrontier->position.GetVecBt();
				vMinHeight.setY(vMinHeight.y() - maxWalkHeight);

				//set starting ray.Z as high as possible (within walkslope limit)
				btCollisionWorld::ClosestRayResultCallback hit(newFrontier->position.GetVecBt(), vMaxHeight);
				world->rayTest(newFrontier->position.GetVecBt(), vMaxHeight, hit);
				if (hit.hasHit()) {
					vMaxHeight.setY(hit.m_hitPointWorld.y());
				}

				//set frontier.Z as low as possible (within walkslope limit)
				hit.m_rayFromWorld = vMaxHeight;
				hit.m_rayToWorld = vMinHeight;
				world->rayTest(vMaxHeight, vMinHeight, hit);
				if (hit.hasHit()) {
					newFrontier->position.SetY(hit.m_hitPointWorld.y() + 1);
				}
				else {
					continue;
				}

				if (boundBox.isInside(newFrontier->position.GetVecBt(), 0)) {
					//Check if new position isn't already in range in frontier or in nodelist
					if (!IsNodeInRange(newFrontier->position, frontier) && !IsNodeInRange(newFrontier->position, nodelist)) {
						//Check if anything around the node colides with it
						btVector3 sweepOffset = newFrontier->position.GetVecBt();
						sweepOffset.setY(sweepOffset.y() + agentHeight);
						btCollisionWorld::ClosestConvexResultCallback sweepHit(newFrontier->position.GetVecBt(), sweepOffset);
						world->convexSweepTest(agentSphere, btTransform(btQuaternion(0, 0, 0), newFrontier->position.GetVecBt()), btTransform(btQuaternion(0, 0, 0), sweepOffset), sweepHit);
						if (!sweepHit.hasHit()) {
							frontier.push_back(newFrontier);
						}
					}
				}
			}
		}
		AddEdgesToNeighbours(frontier[0], world);

		//Adds first frontier to nodelist
		nodelist.push_back(frontier[0]);
		frontier.erase(frontier.begin());
	}

	void NavigationGenerator::AddNodeOfIntrest(TSVector3 position, InterestType type, btCollisionWorld * world)
	{
		GeneralNode* node = new GeneralNode();
		node->position = position;
		node->type = type;
		AddEdgesToNeighbours(node, world);
		nodelist.push_back(node);
	}

	void NavigationGenerator::SetStartPosition(TSVector3 startPos)
	{
		this->startPos = startPos;
	}

	void NavigationGenerator::SetBounds(btBoxShape bounds)
	{
		this->boundBox = bounds;
	}

	void NavigationGenerator::SetGridSize(int gridSize)
	{
		this->gridSize = gridSize;
	}

	void NavigationGenerator::SetWalkableSlope(float walkableSlope)
	{
		if (walkableSlope != 0) {
			maxWalkHeight = (gridSize / sinf(walkableSlope))*sin(90 - walkableSlope);
		}
		else {
			maxWalkHeight = 0.01f;
		}
	}

	void NavigationGenerator::SetAgentWidth(int agentWidth)
	{
		this->agentWidth = agentWidth;
		agentSphere = new btSphereShape(btScalar(agentWidth));
	}

	void NavigationGenerator::SetAgentHeight(int agentHeight)
	{
		this->agentHeight = agentHeight;
	}

	bool NavigationGenerator::JustFinished()
	{
		if (isJustFinished) {
			isJustFinished = false;
			return true;
		}
		return false;
	}

	int NavigationGenerator::ExportToFile(tsString fileName)
	{
		if (nodelist.empty() || edgeList.empty()) {
			//No grid to save
			return 2;
		}

		std::ofstream exportFile;
		exportFile.open(fileName.c_str(), std::ios::trunc);
		if (exportFile.is_open()) {
			exportFile << nodeHeader << "\n";
			for (GeneralNode* n : nodelist)
			{
				exportFile << n->position.GetX() << delim << n->position.GetY() << delim << n->position.GetZ() << delim << n->type << "\n";
			}
			exportFile << edgeHeader << "\n";
			for (GeneralEdge* e : edgeList)
			{
				exportFile << e->fromIndex << delim << e->toIndex << delim << e->distance << "\n";
			}
			exportFile.close();
			return 0;
		}
		else {
			return 1;
		}
	}

	int NavigationGenerator::ImportFromFile(tsString fileName)
	{
		std::ifstream importFile;
		GeneralNode* newNode;
		GeneralEdge* newEdge;
		importFile.open(fileName);
		int readingType = 0;
		if (importFile.is_open()) {
			nodelist.clear();
			edgeList.clear();
			while (!importFile.eof())
			{
				char lineTxt[128];
				importFile.getline(lineTxt, 128, '\n');
				if (lineTxt == nodeHeader) {
					readingType = 1;
				}
				else if (lineTxt == edgeHeader) {
					readingType = 2;
				}
				else if (!std::string(lineTxt).empty()) {
					char* remain;
					char* val;
					switch (readingType)
					{
					case 1:
						newNode = new GeneralNode();
						val = strtok_s(lineTxt, delim, &remain);
						newNode->position.SetX(std::stof(val));
						val = strtok_s(NULL, delim, &remain);
						newNode->position.SetY(std::stof(val));
						val = strtok_s(NULL, delim, &remain);
						newNode->position.SetZ(std::stof(val));
						val = strtok_s(NULL, delim, &remain);
						newNode->type = (InterestType)std::stoi(val);
						nodelist.push_back(newNode);
						break;
					case 2:
						newEdge = new GeneralEdge();
						val = strtok_s(lineTxt, delim, &remain);
						newEdge->fromIndex = std::stoi(val);
						val = strtok_s(NULL, delim, &remain);
						newEdge->toIndex = std::stoi(val);
						val = strtok_s(NULL, delim, &remain);
						newEdge->distance = std::stof(val);
						edgeList.push_back(newEdge);
						break;
					}
				}
			}
			importFile.close();
			return 0;
		}
		else {
			return 1;
		}
	}

	void NavigationGenerator::AddEdgesToNeighbours(GeneralNode * node, btCollisionWorld * world)
	{
		for (size_t n = 0; n < nodelist.size(); ++n)
		{
			//Checks if node is neighboring frontier if so,
			//checks if they can see each other.
			btCollisionWorld::ClosestRayResultCallback hit(node->position.GetVecBt(), nodelist[n]->position.GetVecBt());
			world->rayTest(node->position.GetVecBt(), nodelist[n]->position.GetVecBt(), hit);

			btVector3 dist = btVector3(node->position.GetVecBt() - nodelist[n]->position.GetVecBt()).absolute();
			if (dist.x() <= gridSize &&
				dist.z() <= gridSize &&
				dist.y() <= (maxWalkHeight + agentWidth)
				&& !hit.hasHit()) {
				GeneralEdge* newEdge = new GeneralEdge();
				newEdge->fromIndex = (int)n;
				newEdge->toIndex = (int)nodelist.size();
				newEdge->distance = roundf(node->position.GetVecBt().distance(nodelist[n]->position.GetVecBt()) * 100) / 100;
				edgeList.push_back(newEdge);
			}
		}
	}

	bool NavigationGenerator::IsNodeInRange(TSVector3 positionToCheck, std::vector<GeneralNode*>& listToCheck)
	{
		for (GeneralNode* n : listToCheck) {
			if (positionToCheck.GetX() == n->position.GetX() &&
				positionToCheck.GetZ() == n->position.GetZ() &&
				fabs(positionToCheck.GetY() - n->position.GetY()) <= agentHeight) {
				return true;
			}
		}
		return false;
	}
}