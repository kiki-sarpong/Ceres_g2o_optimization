#include <glog/logging.h>
#include <chrono>
#include "LoadData.h"


using namespace std;

int main(int argc, char** argv){

    if (argc != 2){
        cerr << "Missing the .g2o file\n";
        return -1;
    }

    ifstream file(argv[1]);
    if (!file){
        cerr << "File: " << argv[1] << " doesn't exist or isn't loading correctly. \n";
        cerr << "Check file path!\n";
        return -1;
    }

    // Start the timer
    auto start = std::chrono::high_resolution_clock::now();

    google::InitGoogleLogging(argv[0]);
    google::SetStderrLogging(google::INFO); // Only INFO and higher levels will show in stderr

    int maxOptimizerIterations = 30; // Optimizer iterations for ceres optimizer
    string outputFile = "optimized_result.g2o";
    LoadData optimizationData(file, outputFile, maxOptimizerIterations);
    // Load data and run optimization
    optimizationData.loadData();
    optimizationData.runCeresOptimizer();
    optimizationData.writeOutputToFile();

    auto end = std::chrono::high_resolution_clock::now(); // End timer
    std::chrono::duration<double> duration = end - start;  

    LOG(INFO) << "Code execution took : " << duration.count() << " seconds";


    return 0;
}