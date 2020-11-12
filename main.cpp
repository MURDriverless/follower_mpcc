#include <iostream>
#include <unistd.h>

#include "src/config.h"
#include "src/types.h"
#include "src/Params/params.h"
#include "src/Models/model_interface.h"
#include "src/Models/dynamic_bicycle.h"

// Test binary search
#include "src/Utils/binary_search.h"
// Test cubic spline
#include "src/Splines/cubic_spline.h"

using namespace std;

string getFullPath(const string &relativePath) {
    string cwd = get_current_dir_name();
    return cwd + "/" + relativePath;
}

int main() {
    /*
     * Test model
     * */
//    // Path to parameters
//    const string model_params_file = getFullPath("src/Params/model.json");
//
//    // Create model parameters
//    ModelParams modelParams = ModelParams(model_params_file);
//    // Instantiate a model which implements IModel (we need to use pointer form, otherwise
//    // we'll get "abstract class cannot be instantiated" as we have virtual functions.
//    IModel *model = new DynamicBicycleModel(modelParams);
//
//    // Initialise state
//    State x0 = State::Zero();
//    x0(IndexMap.X) = -2.0;
//    x0(IndexMap.Y) = 3.0;
//    // Initialise input
//    Input u0 = Input::Zero();
//    u0(IndexMap.d_accel_D) = 0.2;
//
//    // Test model for Ts = 0.02
//    cout << "Next state is" << endl;
//    cout << model->predictRK4(x0, u0, 0.02) << endl;


    /*
     * Test binary search
     * */
//    VectorXd arr(6);
//    arr(0) = 0.0;
//    arr(1) = 2.0;
//    arr(2) = 4.0;
//    arr(3) = 6.0;
//    arr(4) = 8.0;
//    arr(5) = 10.0;
//    double x = 9.9999999999999;
//    int target_ind = utils::binary_search_left(arr, x);
//    cout << "Arr = [" << arr.transpose() << " ], x = " << x << endl;
//    cout << "Lo index = " << target_ind << ", value = " << arr(target_ind) << endl;


    /*
     * Test cubic spline
     * */
    VectorXd t_arr(6);
    VectorXd ft_arr(6);
    for (int i = 0; i < t_arr.size(); i++) {
        t_arr(i) = (double) i;
        ft_arr(i) = (double) i;
    }
    CubicSpline spline = CubicSpline(t_arr, ft_arr);
    double target_t = 10.0;
    double target_val = spline.getPosition(target_t);
    cout << "At t = " << target_t << ", the value is " << target_val << endl;


    return 0;
}
