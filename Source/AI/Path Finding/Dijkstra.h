#pragma once

#include "PathAlgorithm.h"
#include "Node.h"
#include <vector>

class DijkstraNode : public Node
{
	static const float infinity;

public:
	DijkstraNode() :
		distance(infinity),
		closed(false),
		type(0)
	{}

	~DijkstraNode()
	{}

	void setClosed(bool value)
	{
		closed = value;
	}

	void setDistance(float value)
	{
		distance = value;
	}

	inline bool isClosed() const
	{
		return closed;
	}

	inline float getDistance() const
	{
		return distance;
	}

	void release()
	{
		distance = infinity;
		closed = false;
		m_parent = nullptr;
	}

	int getType() const {
		return type;
	}

	void setType(int newType) {
		type = newType;
	}

protected:
	bool closed;
	float distance;
	int type;
};

struct CompareDNodes
{
	bool operator() (const DijkstraNode* n1, const DijkstraNode* n2)
	{
		return n1->getDistance() < n2->getDistance();
	}
};

class Dijkstra : public PathAlgorithm<DijkstraNode>
{
public:

	static Dijkstra& getInstance()
	{
		static Dijkstra instance;
		return instance;
	}

	bool getPath(DijkstraNode* start, DijkstraNode* goal, std::vector<DijkstraNode*>& path);
	void clear();

private:

	Dijkstra();
	~Dijkstra();

	void pushOpen(DijkstraNode* node);
	void popOpen(DijkstraNode* node);
	void releaseNodes();

	std::vector<DijkstraNode*> open, closed;
};