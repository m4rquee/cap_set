#include <gurobi_c++.h>
#include <iostream>
#include <random>

#define n 5

using namespace std;

void coordinates(int val, int *val_vet) {
    for (int i = 0; i < n; i++) val_vet[i] = 0;
    int i = 0;
    while (val > 0) {
        val_vet[n - ++i] = val % 3;
        val /= 3;
    }
}

bool coolinear(int i, int j, int k) {
    int i_vet[n], j_vet[n], k_vet[n];
    coordinates(i, i_vet);
    coordinates(j, j_vet);
    coordinates(k, k_vet);
    for (int l = 0; l < n; l++) if ((i_vet[l] + j_vet[l] + k_vet[l]) % 3 != 0) return 0;
    return 1;
}

class CapSetCB : public GRBCallback {
public:
    int num_vars;
    GRBVar *X;
    bool *solution;

    CapSetCB(int _num_vars, GRBVar *x) : num_vars(_num_vars), X(x) {
        solution = (bool *)malloc(num_vars * sizeof(bool));
    }
    
protected:
    void callback() override {
        if (where == GRB_CB_MIPSOL) { // all variable are integer

        } else
            return; // this code do not take advantage of the other options

        for (int i = 0; i < num_vars; i++) // get the candidate solution
            solution[i] = getSolution(X[i]) > 0.5;

        // Amongst the selected points, there must be no three in line:
        for (int i = 0; i < num_vars; i++)
            for (int j = i + 1; j < num_vars; j++)
                for (int k = j + 1; k < num_vars; k++)
                    if (solution[i] && solution[j] && solution[k])
                        if (coolinear(i, j, k)) { // if there is a coolinear triple in the cadidate solution
                            addLazy(X[i] + X[j] + X[k] <= 2);
                            break;
                        }
    }
};

int main() {
    int aux_vet[n];
    int num_vars = pow(3.0, n);
    GRBVar *X;
    X = (GRBVar *)malloc(num_vars * sizeof(GRBVar));

    try {
        GRBEnv env = GRBEnv();
        GRBModel model = GRBModel(env);
        model.set(GRB_StringAttr_ModelName, "Cap set solver");
        model.set(GRB_IntAttr_ModelSense, GRB_MAXIMIZE);
        model.set(GRB_DoubleParam_Cutoff, pow(2.0, n));
        // if (n > 500) model.set(GRB_IntParam_Threads, 1);
        model.set(GRB_IntParam_Cuts, GRB_CUTS_AGGRESSIVE);
        model.set(GRB_IntParam_Presolve, GRB_PRESOLVE_AGGRESSIVE);
        // model.set(GRB_DoubleParam_Heuristics, 0.25);
        model.set(GRB_IntParam_MIPFocus, 1);
        model.set(GRB_IntParam_LazyConstraints, 1);
        model.set(GRB_DoubleParam_TimeLimit, 120);

        // Create the binary selection variables:
        for (int i = 0; i < num_vars; i++)
            X[i] = model.addVar(0.0, 1.0, 1.0, GRB_BINARY);
        model.update(); // run update to use model inserted variables

        /* GRBLinExpr bound = 0;
        for (int i = 0; i < num_vars; i++)
            bound += X[i];
        model.addConstr(bound <= 50); */

        // Setup callback:
        CapSetCB cb(num_vars, X);
        model.setCallback(&cb);

        model.update(); // run update before optimize
        model.optimize();

        if (model.get(GRB_IntAttr_SolCount) == 0) // if the solver could not obtain a solution
            throw GRBException("Could not obtain a solution!", -1);

        std::cout << "Selected points:" << std::endl;
        for (int i = 0; i < num_vars; i++)
            if (X[i].get(GRB_DoubleAttr_X) > 0.5) {
                coordinates(i, aux_vet);
                for (int i = 0; i < n; i++)
                std::cout << aux_vet[i];
                std::cout << std::endl;
            }            
        std::cout << "Number of points " << model.getObjective().getValue() << std::endl;
    } catch (const GRBException &ex) {
        printf("Exception...\n");
        std::cout << ex.getMessage();
        exit(ex.getErrorCode());
    }
    free(X);
    return 0;
}
