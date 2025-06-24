#ifndef _LOAD_DATA_H
#define _LOAD_DATA_H

#include <iostream>
#include <fstream>
#include <map>
#include "PoseGraphOptimization.h"

using namespace std;

class LoadData {
    public:
        LoadData(ifstream& file, string outputFilename, const int max_iterations = 30);
        ~LoadData();

        PoseGraphCeres optimizer;
        string outputFilename_;
        int poseCount = 0;
        int constraintCount = 0;
        
        void runCeresOptimizer();
        void writeOutputToFile() const;
        void loadData();

    private:
        // Initialize ordered_map and vector
        vector<shared_ptr<Constraints>> constraints_;
        map<int, shared_ptr<Poses>> poses_;
        string poseName = "VERTEX_SE3:QUAT";
        string constraintName = "EDGE_SE3:QUAT";
        ifstream& file_;

};


#endif /*_LOAD_DATA_H*/