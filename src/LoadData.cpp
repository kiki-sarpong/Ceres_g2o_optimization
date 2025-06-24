#include "LoadData.h"
#include <glog/logging.h>



LoadData::LoadData(ifstream& file, string outputFilename, const int max_iterations) : 
    file_(file), outputFilename_(outputFilename){

        // Set optimizer to assigned max_iterations
        optimizer = PoseGraphCeres(max_iterations);
    }

LoadData::~LoadData(){
    LOG(INFO) << "Number of total poses and constraints. Poses-> " << poseCount
         << " , constraints->  " << constraintCount;
}

void LoadData::loadData(){
    LOG(INFO) << "Loading data to be optimized...";

    while (!file_.eof()) {
        string name;
        file_ >> name;
        if (name == poseName) {
            //  SE3 vertex
            shared_ptr<Poses> pses = make_shared<Poses>();
            int index = 0;
            file_ >> index;
            pses->id = index; // Here, the id is the index
            pses->read(file_);
            poseCount++;
            poses_[index] = pses;
        } else if (name == constraintName) {
            int index1, index2;     // The two associated vertices
            file_ >> index1 >> index2;

            // SE3-SE3 edge  || For every constraint/edge assign the two nodes that make the edge
            shared_ptr<Constraints> ctnt = make_shared<Constraints>(index1, index2);
            ctnt->id = constraintCount++;   // Here, the id is the count
            ctnt->read(file_);
            constraints_.push_back(ctnt);
        }
        if (!file_.good()) break;
    }

    LOG(INFO) << "Data Loading complete.";
}

void LoadData::runCeresOptimizer(){
    LOG(INFO) << "Starting ceres optimizer ....";
    optimizer.runOptimization(poses_, constraints_);
    LOG(INFO) << "Ceres optimizer complete!";
}

void LoadData::writeOutputToFile() const {
    LOG(INFO) << "Writing data output to file: ../dataset/" << outputFilename_;
    // Write Poses and constraints to file
    ofstream output_file("optimized_result.g2o");

    for (auto& [i, p]:poses_) {  // poses = map(int, poses*)
        output_file << poseName << " ";
        p->write(output_file);
    }
    for (auto c:constraints_) {
        output_file << constraintName << " ";
        c->write(output_file);
    }
    output_file.close();
    LOG(INFO) << "Write complete.";
}



