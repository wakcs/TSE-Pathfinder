#pragma once
#include "Wrappers\TSVector3.h"
#include "Ai\Path Finding\AStar.h"
#include "Ai\Path Finding\Dijkstra.h"

namespace tse
{
	enum InterestType
	{
		INTEREST_DEFAULT = 0,
		INTEREST_AMMO,
		INTEREST_HEALTH
	};

	enum NavDirection
	{
		NAV_NOWHERE = -1,
		NAV_NORTH,
		NAV_EAST,
		NAV_SOUTH,
		NAV_WEST
	};

	struct GeneralNode
	{
		TSVector3 position;
		NavDirection comesFrom = NAV_NOWHERE;
		InterestType type = INTEREST_DEFAULT;
	};

	struct GeneralEdge
	{
		int fromIndex;
		int toIndex;
		float distance;
	};

	class StarNode : public AStarNode
	{
	private:
		float m_z;
		InterestType type;

	public:
		StarNode() {};
		~StarNode() {};

		float distanceTo(AStarNode* node) const {
			return float(TSVector3(m_x - node->getX(), m_y - node->getY(), 0).Magnitude());
		}

		float distanceToVec(TSVector3 vector) {
			return float(TSVector3(GetPosition() -vector).Magnitude());
		}

		TSVector3 GetPosition() { return TSVector3(m_x, m_y, m_z); }
		InterestType GetType() { return type; }

		void SetPosition(float x, float y, float z) {
			AStarNode::setPosition(x, y);
			m_z = z;
		}
		void SetPosition(TSVector3 vec) {
			AStarNode::setPosition(vec.GetX(), vec.GetY());
			m_z = vec.GetZ();
		}
		void SetType(InterestType type) { this->type = type; }
	};

	class DijkNode : public DijkstraNode
	{
	private:
		TSVector3 position;

	public:
		DijkNode() {};
		~DijkNode() {};

		TSVector3 GetPosition() { return position; }
		InterestType GetType() { return (InterestType)type; }

		void SetPosition(TSVector3 position) { this->position = position; }
		void SetType(InterestType newType) { type = newType; }
	};
}