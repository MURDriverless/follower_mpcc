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
// Test cubic spline 2d
#include "src/Splines/cubic_spline2d.h"
// Test cost
#include "src/Cost/cost.h"
// Test bounds
#include "src/Constraints/bounds.h"

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
//    VectorXd t_arr(6);
//    VectorXd ft_arr(6);
//    for (int i = 0; i < t_arr.size(); i++) {
//        t_arr(i) = (double) i;
//        ft_arr(i) = (double) i;
//    }
//    CubicSpline spline = CubicSpline(t_arr, ft_arr);
//    double target_t = 10.0;
//    double target_val = spline.getPosition(target_t);
//    cout << "At t = " << target_t << ", the value is " << target_val << endl;


    /*
     * Test cubic spline 2D
     * */
//    VectorXd x_data(20);
//    VectorXd y_data(20);
//    for (int i = 0; i < x_data.size(); i++) {
//        x_data(i) = i;
//        y_data(i) = i * sin(i *3.142 / 180);
//    }
//    CubicSpline2D spline2D = CubicSpline2D(x_data, y_data);
//    cout << spline2D.getPosition(15.0) << endl;

    /*
     * Test cost
     * */
//    // Path to parameters
//    const string cost_params_file = getFullPath("src/Params/cost.json");
//    const string model_params_file = getFullPath("src/Params/model.json");
//
//    // Create parameters
//    CostParams costParams = CostParams(cost_params_file);
//    ModelParams modelParams = ModelParams(model_params_file);
//
//    // Mock some path
//    VectorXd x_data(20);
//    VectorXd y_data(20);
//    for (int i = 0; i < x_data.size(); i++) {
//        x_data(i) = i;
//        y_data(i) = i * sin(i *3.142 / 180);
//    }
//    CubicSpline2D path = CubicSpline2D(x_data, y_data);
//
//    // Mock some initial state
//    State x0 = State::Zero();
//    x0(IndexMap.X) = x_data(0);
//    x0(IndexMap.Y) = y_data(0);
//
//    // Create cost object
//    Cost cost = Cost(costParams, modelParams);
//    CostMatrix costMatrix = cost.getCost(path, x0);
//
//    // Print resulting cost matrix
//    cout << costMatrix.Q << endl;
//    cout << costMatrix.R << endl;
//    cout << costMatrix.S << endl;
//    cout << costMatrix.q << endl;
//    cout << costMatrix.r << endl;
//    cout << costMatrix.Z << endl;
//    cout << costMatrix.z << endl;


    /*
     * Test bounds
     * */
    const string bounds_params_file = getFullPath("src/Params/bounds.json");
    Bounds bounds = Bounds(bounds_params_file);
    cout << bounds.getLowerStateBounds() << endl;
    cout << bounds.getUpperStateBounds() << endl;
    cout << bounds.getLowerInputBounds() << endl;
    cout << bounds.getUpperInputBounds() << endl;
    cout << bounds.getLowerSoftBounds() << endl;
    cout << bounds.getUpperSoftBounds() << endl;

    return 0;
}
