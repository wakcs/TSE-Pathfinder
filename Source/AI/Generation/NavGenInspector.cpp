#include "stdafx.h"
#include <Ai\Generation\NavGenInspector.h>
#include <Debug\DebugLogService.h>

namespace tse
{
	NavGenInspector::NavGenInspector(NavigationGenerator & navGen, tsString levelPath)
		: m_navGen(navGen)
	{
		m_navpath = levelPath + L".nav";
	}

	NavGenInspector::~NavGenInspector()
	{
	}

	void NavGenInspector::Initialize()
	{
		int result = m_navGen.ImportFromFile(m_navpath);
		switch (result)
		{
		case 0:
			LWRITE(LogChannel::INFO, TSTEXT("Nav-Grid import succesfully!"));
			break;
		case 1:
			LWRITE(LogChannel::EXCEPTION, TSTEXT("Can't import Nav-Grid from file: "), m_navpath.c_str());
			LWRITE(LogChannel::WARNING, TSTEXT("Enemy components won't have pathfinding!"));
			break;
		}
	}

	void NavGenInspector::Inspect()
	{
		if (ImGui::Button("Navigation Generator")) {
			m_ShowWindow = !m_ShowWindow;
			if (m_ShowWindow) {
				ImGui::SetNextWindowSize(ImVec2(550, 250));
			}
		}

		if (m_ShowWindow) {
			ImGui::Begin("Navigation Generator", &m_ShowWindow);

			ImGui::InputFloat3("Origin position", m_origin);
			ImGui::InputFloat3("Bounds extends", m_boxExtends);
			ImGui::InputInt("Grid Size", &m_gridSize);
			ImGui::SliderFloat("Max Walkable Angle", &m_maxWalkableSlope, 0, 89);
			ImGui::InputFloat2("Agent Width & Height", m_agentExtends, 3);
			ImGui::Checkbox("Draw debug lines", &m_drawDebug);
			ImGui::SliderFloat("Draw Scale", &m_drawScale, 0.1f, 10);

			if (ImGui::Button("Start Generation")) {
				m_navGen.SetStartPosition(TSVector3(m_origin[0], m_origin[1], m_origin[2]));
				m_navGen.SetBounds(btBoxShape(btVector3(m_boxExtends[0], m_boxExtends[1], m_boxExtends[2])));
				m_navGen.SetGridSize(m_gridSize);
				m_navGen.SetWalkableSlope(m_maxWalkableSlope);
				m_navGen.SetAgentWidth(m_agentExtends[0]);
				m_navGen.SetAgentHeight(m_agentExtends[1]);
				m_navGen.StartGeneration();
			}
			if (ImGui::Button("Save grid")) {
				int result = m_navGen.ExportToFile(m_navpath);
				switch (result)
				{
				case 0:
					LWRITE(LogChannel::INFO, TSTEXT("Nav-Grid export succesfully!"));
					m_gridChanged = true;
					break;
				case 1:
					LWRITE(LogChannel::EXCEPTION, TSTEXT("Failed to open file to write to!"));
					break;
				case 2:
					LWRITE(LogChannel::WARNING, TSTEXT("Cant export empty grid!"));
					break;
				}
			}
			if (ImGui::Button("Load grid")) {
				int result = m_navGen.ImportFromFile(m_navpath);
				switch (result)
				{
				case 0:
					LWRITE(LogChannel::INFO, TSTEXT("Nav-Grid import succesfully!"));
					m_gridChanged = true;
					break;
				case 1:
					LWRITE(LogChannel::EXCEPTION, TSTEXT("Can't import Navgrid from file: "), m_navpath);
					break;
				}
			}
			ImGui::End();
		}

		if (m_drawDebug) {
			ImGuiWindowFlags flags =
				ImGuiWindowFlags_NoTitleBar |
				ImGuiWindowFlags_NoResize |
				ImGuiWindowFlags_NoMove |
				ImGuiWindowFlags_NoScrollbar |
				ImGuiWindowFlags_NoSavedSettings |
				ImGuiWindowFlags_NoCollapse |
				ImGuiWindowFlags_NoInputs |
				ImGuiWindowFlags_NoBringToFrontOnFocus;
			TSVector2 windowSize = TSVector2(1600, 900);
			TSVector3 debugCenter = TSVector3(windowSize.GetX() / 2, windowSize.GetY() / 2, 0);
			ImGui::Begin("DebugDraw", (bool*)true, windowSize.GetVecIm(), 0, flags);
			ImDrawList* drawList = ImGui::GetWindowDrawList();
			if (m_navGen.IsGenerating()) {
				for (GeneralNode* n : m_navGen.GetFrontier())
				{
					TSVector2 nodePos = TSVector2(debugCenter.GetX() + (n->position.GetX() * m_drawScale), debugCenter.GetY() + (n->position.GetZ() * m_drawScale));
					drawList->AddCircle(nodePos.GetVecIm(), m_agentExtends[0] * m_drawScale, ImColor(0, 0, 1), 7, 2);
				}
			}
			else {
				for (GeneralNode* n : m_navGen.GetNodes())
				{
					TSVector2 nodePos = TSVector2(debugCenter.GetX() + (n->position.GetX() * m_drawScale), debugCenter.GetY() + (n->position.GetZ() * m_drawScale));
					drawList->AddCircle(nodePos.GetVecIm(), m_agentExtends[0] * m_drawScale, ImColor(0, 1, 0), 7, 2);
				}
				for (GeneralEdge* e : m_navGen.GetEdges())
				{
					TSVector2 vFrom = TSVector2(debugCenter.GetX() + (m_navGen.GetNodes()[e->fromIndex]->position.GetX() * m_drawScale), debugCenter.GetY() + (m_navGen.GetNodes()[e->fromIndex]->position.GetZ() * m_drawScale));
					TSVector2 vTo = TSVector2(debugCenter.GetX() + (m_navGen.GetNodes()[e->toIndex]->position.GetX() * m_drawScale), debugCenter.GetY() + (m_navGen.GetNodes()[e->toIndex]->position.GetZ() * m_drawScale));
					drawList->AddLine(vFrom.GetVecIm(), vTo.GetVecIm(), ImColor(1, 0, 0));
				}
			}
			ImGui::End();
		}
	}
	bool NavGenInspector::HasGridChanged()
	{
		if (m_gridChanged) {
			m_gridChanged = false;
			return true;
		}
		return false;
	}
}
