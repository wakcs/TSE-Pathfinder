#pragma once

#include <Debug\Inspector.h>
#include "Ai\Generation\NavigationGenerator.h"
#include <vector>

namespace tse
{
	class NavGenInspector :	public Inspectable
	{
	public:
		NavGenInspector(NavigationGenerator& navGen, tsString levelPath);
		~NavGenInspector();

		void Initialize();
		virtual void Inspect();
		bool HasGridChanged();

	protected:
		NavigationGenerator& m_navGen;
		tsString m_navpath;

		float m_origin[3]{ 0,0,0 };
		float m_boxExtends[3]{ 500,200,500 };
		float m_maxWalkableSlope = 20;
		int m_gridSize = 100;
		float m_agentExtends[2]{ 40,100 };
		float m_drawScale = 1;
		bool m_drawDebug = false;
		bool m_ShowWindow = false;
		bool m_gridChanged = false;
		char m_fileName[255];
	};
}

