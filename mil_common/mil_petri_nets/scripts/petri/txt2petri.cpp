#include <iostream>
#include <fstream>
#include <string>
#include <vector>
using namespace std;

void insertHeader(ofstream& outFile){
	outFile << "<?xml version=\"1.0\"?>\n";
	outFile << "<pnml>\n";
	outFile << "\t<net>\n";
	outFile << "\t\t<token id=\"Default\" enabled=\"true\" red=\"0\" green=\"0\" blue=\"0\"/>\n";
}

void insertPlaces(ofstream& outFile, vector<string>& places, vector<string>& placesTokens, vector<int>& placesX, vector<int>& placesY){
	for(unsigned int i = 0; i < places.size(); i++){
		outFile << "\t\t<place id=\"" + places[i] + "\">\n";
		
		outFile << "\t\t\t<graphics>\n";
		outFile << "\t\t\t\t<position x=\"" + to_string(placesX[i]) + "\" y=\"" + to_string(placesY[i]) + "\"/>\n";
		outFile << "\t\t\t</graphics>\n";
		
		outFile << "\t\t\t<name>\n";
		outFile << "\t\t\t\t<value>" + places[i] + "</value>\n";
		outFile << "\t\t\t</name>\n";

		outFile << "\t\t\t<initialMarking>\n";
		outFile << "\t\t\t\t<value>" + placesTokens[i] + "</value>\n";
		outFile << "\t\t\t</initialMarking>\n";

		outFile << "\t\t</place>\n\n";
	}
}

void insertTransitions(ofstream& outFile, vector<string>& transitions, vector<int>& transitionsX, vector<int>& transitionsY){
	for(unsigned int i = 0; i < transitions.size(); i++){
		outFile << "\t\t<transition id=\"" + transitions[i] + "\">\n";

		outFile << "\t\t\t<graphics>\n";
                outFile << "\t\t\t\t<position x=\"" + to_string(transitionsX[i]) + "\" y=\"" + to_string(transitionsY[i]) + "\"/>\n";
                outFile << "\t\t\t</graphics>\n";

		outFile << "\t\t\t<name>\n";
                outFile << "\t\t\t\t<value>" + transitions[i] + "</value>\n";
                outFile << "\t\t\t</name>\n";

		outFile << "\t\t</transition>\n\n";
	}
}

void insertArcs(ofstream& outFile, vector<string>& arcSources, vector<string>& arcTargets, vector<string>& arcValues){
	for(unsigned int i = 0; i < arcSources.size(); i++){
		outFile << "\t\t<arc source=\"" + arcSources[i] + "\" target=\"" + arcTargets[i] + "\">\n";
		
		outFile << "\t\t\t<inscription>\n";
                outFile << "\t\t\t\t<value>" + arcValues[i] + "</value>\n";
                outFile << "\t\t\t</inscription>\n";

		outFile << "\t\t\t<arcpath id=\"000\" x=\"0\" y=\"0\" curvePoint=\"true\"/>\n";
		outFile << "\t\t\t<arcpath id=\"001\" x=\"0\" y=\"0\" curvePoint=\"true\"/>\n";

		outFile << "\t\t</arc>\n\n";
	}
}

void insertTail(ofstream& outFile){
	outFile<< "\t</net>\n";
	outFile<< "</pnml>";
}

int main(){
	vector<string> places; //vector of places
	vector<string> placesTokens; //vector of number of tokens corresponding to each place
	vector<int> placesX; //Autogenerated X coordinate for each place
	vector<int> placesY; //Autogenerated Y coordinate for each place

	vector<string> transitions; //vector of all transition objects
	vector<int> transitionsX; //Autogenerated X coordinate for each transitions
	vector<int> transitionsY; //Autogenerated Y coordinate for each transitions

	vector<string> arcSources; //vector of initial locations of each arc
	vector<string> arcTargets; //vector of final locations corresponding to each initial
	vector<string> arcValues; //vector of firing values for each arc
	int delimiter;

	//variables used to help parsing logic
	string tempStart;
	string tempMid;
	string tempFinish;
	int group = 0;


	//Parse txt file into vectors
	string line;
	ifstream inputFile("petrinet.txt");

	if(inputFile.is_open()){
		while(getline(inputFile, line)){
			if(line.compare("---") == 0){
				group++;
			}else if(group == 0){//parses places

				delimiter = line.find("-");
				tempStart = line.substr(0,delimiter);
				places.push_back(tempStart);
				line.erase(0,delimiter+1);
				
				tempFinish = line;
				placesTokens.push_back(tempFinish);

				//autogenerated coordinates based on idea of what could work the best
				placesX.push_back(100 + (placesX.size()/4)*250);
				placesY.push_back(100 + (placesY.size()%4)*150);

			}else if (group == 1){//parses transitions
				transitions.push_back(line);

				//autogenerated coordinates based on idea of what could work the best
				if(transitionsX.size() % 7 < 4){
					transitionsX.push_back(225 + (transitionsX.size()/7)*500);
                                	transitionsY.push_back(100 + (transitionsY.size()%7)*150);
				} else {
					transitionsX.push_back(475 + (transitionsX.size()/7)*500);
                                        transitionsY.push_back(200 + (transitionsY.size()%7 - 4)*150);
				}
			}else if (group == 2){//parses arcs
				delimiter = line.find("-");
				tempStart = line.substr(0,delimiter);
				arcSources.push_back(tempStart);
				line.erase(0,delimiter+1);

				delimiter = line.find("-");
				tempMid = line.substr(0,delimiter);
				arcTargets.push_back(tempMid);
				line.erase(0,delimiter+1);

				tempFinish = line;
				arcValues.push_back(tempFinish);
			}
		}
		inputFile.close();
	}else{
		cout << "Unable to open file" << endl;
	}

	//Now convert vectors into PNML File
	ofstream outFile;
	outFile.open("petrinet.xml", std::ofstream::out | std::ofstream::trunc);
	insertHeader(outFile);
	insertPlaces(outFile, places, placesTokens, placesX, placesY);
	insertTransitions(outFile, transitions, transitionsX, transitionsY);
	insertArcs(outFile, arcSources, arcTargets, arcValues);
	insertTail(outFile);
	outFile.close();


	return 0;
}
