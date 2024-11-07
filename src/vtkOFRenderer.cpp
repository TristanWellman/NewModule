/*Created by Tristan Wellman 2024*/

#include <filesystem>
#include <functional>

#include "vtkOFRenderer.hpp"

using namespace Aftr;

std::vector<std::string> getDirectoryListing(std::string filePath) {
	std::vector<std::string> dirs;
	if(!std::filesystem::is_directory(filePath)) return dirs;
	for (auto& curDir : std::filesystem::recursive_directory_iterator(filePath)) {
		if (curDir.is_directory()) dirs.push_back(curDir.path().string());
	}
	return dirs;
}

std::string splitStr(std::string & str, char delim) {
	int i;
	for (i = 0; i < str.length(); i++) {
		if (str.at(i) == delim) {
			// return the first half of split string and set original to rest of string
			std::string ret = str.substr(0, i);
			str = str.substr(i + 1, str.length());
			return ret;
		}
	}
	return "";
}

std::vector<std::string> vtkOFRenderer::getOpenFoamTimeStamps(std::vector<std::string> dirs) {
	std::vector<std::string> ret;
	for (std::string i : dirs) {
		while(splitStr(i, '/')!="");
		if(splitStr(i, '\\') != "") continue;
		try {
			if(std::stoi(i)||i=="0") ret.push_back(i);
		} catch (const std::invalid_argument &e) { continue; }
	}
	return ret;
}

vtkOFRenderer::vtkOFRenderer(std::string openFoamPath) : filePath(openFoamPath) {
	
	//parser = (vtkParser*)malloc(sizeof(vtkParser*));

	std::vector<std::string> listing = getDirectoryListing(openFoamPath);

	VTKASSERT(!listing.empty(),
		"ERROR:: Failed to open OPENFOAM test case folder: %s", openFoamPath.c_str());

	timeStamps = getOpenFoamTimeStamps(listing);

	VTKASSERT(!timeStamps.empty(),
		"ERROR:: Failed to retrieve OpenFoam case Time Stamps!");
	
	if (openFoamPath.at(openFoamPath.length() - 1) != '/') openFoamPath += '/';
	for (int i = 0; i < timeStamps.size();i++) {
		std::string fullPath = openFoamPath +
			"postProcessing/streamlines/" + timeStamps.at(i) + "/tracks.vtk";
		tracksFiles.push_back(fullPath);
		std::cout << fullPath << std::endl;
	}
	isReady = false; // will be ready after parser is ran
}

void vtkOFRenderer::parseThread(int index) {
	
	vtkParser* parser = &threadParsers.at(index);
	
	parser->setVtkFile(tracksFiles.at(index));
	//parser->printVTKFILE();
	parser->init();
	parser->parseOpenFoam();
	tracksFileData.push_back(parser->getOpenFoamData());
	//parser->dumpOFOAMPolyDataset();
	parser->freeVtkData();

	threadStates.at(index) = 1;
}

int vtkOFRenderer::parseTracksFiles() {

	threadStates.resize(tracksFiles.size(), 0);
	threadParsers.resize(tracksFiles.size());
	threads.resize(tracksFiles.size());

	int i, finished=0;
	for (i = 0; i < tracksFiles.size(); i++) {
		threads.at(i) = std::thread(std::mem_fn(&vtkOFRenderer::parseThread), this, i);
		VTKLOG("INFO:: Started parser thread for: {}", tracksFiles.at(i));
	}
	
	while (!finished) {
		int totalFinished = 0;
		for (i = 0; i < threadStates.size();i++) {
			if (threadStates.at(i)) totalFinished++;
		}
		if (totalFinished == threadStates.size() - 1) finished = 1;
	}
	for (i = 0; i < tracksFiles.size(); i++) {
		threads.at(i).join();
		VTKLOG("INFO:: Finished parser thread for: {}", tracksFiles.at(i));
	}

	isReady = true;
	currentSelectedTimeStamp = timeStamps.at(0).c_str();
#if PRELOAD_TIMESTAMPS
	VTKLOG("INFO:: Preloading OpenFOAM timestamps is enables");
#endif
	return 0;
}

void vtkOFRenderer::updateVtkTrackModel(WorldContainer* wl) {
	static const char* pastTS = currentSelectedTimeStamp;
	openFoamVtkFileData* pptr = new openFoamVtkFileData;
	int i = 0;

	if (currentSelectedTimeStamp != pastTS) {
		for (i = 0; i < WOIDS.size(); i++) {
			WO* tmp = wl->getWOByID(WOIDS.at(i));
			wl->eraseViaWOptr(tmp);
#if !PRELOAD_TIMESTAMPS
			delete tmp;
#endif
		}
		WOIDS.clear();
		int i = 0;
		for (i = 0; i < timeStamps.size(); i++) {
			if (timeStamps.at(i).c_str() == currentSelectedTimeStamp) break;
		}
		pptr = &tracksFileData.at(i);
		std::string point(ManagerEnvironmentConfiguration::getSMM() + "/models/planetSunR10.wrl");
#if !PRELOAD_TIMESTAMPS
		for (i = 0; i < pptr->points.polyData.size(); i += RENDER_RESOLUTION) {
			WO* wo = WO::New(point, Vector(POINT_SIZE, POINT_SIZE, POINT_SIZE), MESH_SHADING_TYPE::mstFLAT);
			wo->setPosition(Vector(
				pptr->points.polyData.at(i).at(0) * POSMUL,
				pptr->points.polyData.at(i).at(1) * POSMUL,
				pptr->points.polyData.at(i).at(2) * POSMUL));
			wo->renderOrderType = RENDER_ORDER_TYPE::roOPAQUE;
			std::string id = "point";
			wo->setLabel(id);
			wl->push_back(wo);
			WOIDS.push_back(wo->getID());

		}
#else 
		for (int j = 0; j < preLoadedWOs.at(i).size(); j+=RENDER_RESOLUTION) {
			WO* tmp = preLoadedWOs.at(i).at(j);
			wl->push_back(tmp);
			WOIDS.push_back(tmp->getID());
		}
#endif
	}
	pastTS = currentSelectedTimeStamp;
}

WO *vtkOFRenderer::renderTimeStampTrack(WorldContainer *worldList) {
	//WO* wo = WO::New();
	
	VTKASSERT(currentSelectedTimeStamp != NULL, 
		"ERROR:: Uninitialized vtk timestamps!");

	static const char* pastTS;
	openFoamVtkFileData* pptr = new openFoamVtkFileData;

	int tloc;
	int i = 0,j;
	for (i = 0; i < timeStamps.size(); i++) {
		if (timeStamps.at(i).c_str() == currentSelectedTimeStamp) break;
	}
	pptr = &tracksFileData.at(i);
	tloc = i;
#if PRELOAD_TIMESTAMPS
		preLoadedWOs = std::vector<std::vector<WO*> >{};
		preLoadedWOs.resize(timeStamps.size());
		for (i = 0; i < preLoadedWOs.size(); i++) {
			pptr = &tracksFileData.at(i);
			preLoadedWOs.at(i).resize(pptr->points.polyData.size());
		}
		pptr = &tracksFileData.at(tloc);
#endif

	std::string point(ManagerEnvironmentConfiguration::getSMM() + "/models/planetSunR10.wrl");
	for (i = 0; i < pptr->points.polyData.size();i+=RENDER_RESOLUTION) {
#if !PRELOAD_TIMESTAMPS
		WO* wo = WO::New(point, Vector(POINT_SIZE, POINT_SIZE, POINT_SIZE), MESH_SHADING_TYPE::mstFLAT);
		wo->setPosition(Vector(
			pptr->points.polyData.at(i).at(0)*POSMUL, 
			pptr->points.polyData.at(i).at(1)*POSMUL, 
			pptr->points.polyData.at(i).at(2)*POSMUL));
		wo->renderOrderType = RENDER_ORDER_TYPE::roOPAQUE;
		std::string id = "point";
		wo->setLabel(id);
		worldList->push_back(wo);
		WOIDS.push_back(wo->getID());
#else 
		preLoadedWOs.at(tloc).at(i) = WO::New(point, Vector(POINT_SIZE, POINT_SIZE, POINT_SIZE), MESH_SHADING_TYPE::mstFLAT);
		preLoadedWOs.at(tloc).at(i)->setPosition(Vector(
			pptr->points.polyData.at(i).at(0) * POSMUL,
			pptr->points.polyData.at(i).at(1) * POSMUL,
			pptr->points.polyData.at(i).at(2) * POSMUL));
		preLoadedWOs.at(tloc).at(i)->renderOrderType = RENDER_ORDER_TYPE::roOPAQUE;
		std::string id = "point";
		preLoadedWOs.at(tloc).at(i)->setLabel(id);
		worldList->push_back(preLoadedWOs.at(tloc).at(i));
		WOIDS.push_back(preLoadedWOs.at(tloc).at(i)->getID());
#endif
	}

// load up rest of object into memory
#if PRELOAD_TIMESTAMPS
	for (i=1; i < timeStamps.size();i++) {
		pptr = &tracksFileData.at(i);
		for (j = 0;j< pptr->points.polyData.size(); j += RENDER_RESOLUTION) {
			preLoadedWOs.at(i).at(j) = WO::New(point, Vector(POINT_SIZE, POINT_SIZE, POINT_SIZE), MESH_SHADING_TYPE::mstFLAT);
			preLoadedWOs.at(i).at(j)->setPosition(Vector(
				pptr->points.polyData.at(j).at(0) * POSMUL,
				pptr->points.polyData.at(j).at(1) * POSMUL,
				pptr->points.polyData.at(j).at(2) * POSMUL));
			preLoadedWOs.at(i).at(j)->renderOrderType = RENDER_ORDER_TYPE::roOPAQUE;
			std::string id = "point";
			preLoadedWOs.at(i).at(j)->setLabel(id);
			//preLoadedWOs.at(i).push_back(wo);
		}
	}
#endif

	pastTS = currentSelectedTimeStamp;

	return nullptr;
}


/*This must be ran in already initialized WOImGui istance*/
void vtkOFRenderer::renderImGuivtkSettings() {


	static int curTime = 0;
	static const char* curItem = timeStamps.at(0).c_str();

	ImGui::SetNextWindowSize(ImVec2(400, 200));
	if (ImGui::Begin("Vtk View", NULL)) {

		ImGui::Text("Select a timestamp to view");
		if (ImGui::BeginCombo("TimeStamps", currentSelectedTimeStamp)) {
			for (int n = 0; n < timeStamps.size(); n++)
			{
				bool is_selected = (currentSelectedTimeStamp == timeStamps.at(n).c_str());
				if (ImGui::Selectable(timeStamps.at(n).c_str(), is_selected))
					currentSelectedTimeStamp = timeStamps.at(n).c_str();
					if (is_selected)
						ImGui::SetItemDefaultFocus(); 
			}
			ImGui::EndCombo();
		}
	}
	ImGui::End();

	//currentSelectedTimeStamp = curItem;

}