#include <iostream>
#include <unistd.h>

#include "src/config.h"
#include "src/types.h"
#include "src/Params/params.h"
#include "src/Models/model_interface.h"
#include "src/Models/dynamic_bicycle.h"

using std::string;

string getFullPath(const string &relativePath) {
    string cwd = get_current_dir_name();
    return cwd + "/" + relativePath;
}

int main() {
    // Path to parameters
    const string model_params_file = getFullPath("src/Params/model.json");

    // Create model parameters
    ModelParams modelParams = ModelParams(model_params_file);
    // Instantiate a model which implements IModel (we need to use pointer form, otherwise
    // we'll get "abstract class cannot be instantiated" as we have virtual functions.
    IModel *model = new DynamicBicycleModel(modelParams);

    // Initialise state
    State x0 = State::Zero();
    x0(IndexMap.X) = -2.0;
    x0(IndexMap.Y) = 3.0;
    // Initialise input
    Input u0 = Input::Zero();
    u0(IndexMap.d_accel_D) = 0.2;

    // Test model for Ts = 0.02
    std::cout << "Next state is" << std::endl;
    std::cout << model->predictRK4(x0, u0, 0.02) << std::endl;

    return 0;
}
