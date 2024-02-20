from sys import argv
from ortools.sat.python import cp_model


def main():
    n = int(argv[1])
    n_pts = int(argv[2])
    model = cp_model.CpModel()

    # Creates the variables.
    p = [[model.NewIntVar(0, 2, f'p{i}_{j}') for j in range(n)] for i in range(n_pts)]

    # Creates the constraints.

    # No two points are equal:
    for i in range(n_pts):
        for j in range(i + 1, n_pts):
            eq_constrs = []
            for k in range(n):
                constr = model.NewBoolVar(f'c{i}{j}{k}')
                model.Add(p[i][k] != p[j][k]).OnlyEnforceIf(constr)
                model.Add(p[i][k] == p[j][k]).OnlyEnforceIf(constr.Not())
                eq_constrs.append(constr)
            model.AddBoolOr(eq_constrs)

    # No three points are colinear:
    for i in range(n_pts):
        for j in range(i + 1, n_pts):
            for k in range(j + 1, n_pts):
                mod_vars = []
                for l in range(n):  # target = (p[i][l] + p[j][l] + p[k][l]) % 3
                    target = model.NewIntVar(0, 2, f't{i}{j}{k}{l}')
                    aux_sum = model.NewIntVar(0, 6, f's{i}{j}{k}{l}')
                    model.Add(aux_sum == p[i][l] + p[j][l] + p[k][l])
                    model.AddModuloEquality(target, aux_sum, 3)
                    mod_vars.append(target)
                model.Add(sum(mod_vars) > 0)

    # Creates a solver and solves the model.
    solver = cp_model.CpSolver()
    solver.parameters.num_search_workers = 4
    status = solver.Solve(model)

    if status == cp_model.OPTIMAL or status == cp_model.FEASIBLE:
        print('Found a feasible solution.')
        for i, pt in enumerate(p):
            print(f'p{i} = (', end='')
            for k in range(n - 1):
                print(solver.Value(pt[k]), end=', ')
            print(f'{solver.Value(pt[-1])})')
    else:
        print('No solution found.')

    # Statistics.
    print('\nStatistics')
    print(f'  status   : {solver.StatusName(status)}')
    print(f'  conflicts: {solver.NumConflicts()}')
    print(f'  branches : {solver.NumBranches()}')
    print(f'  wall time: {solver.WallTime()} s')


if __name__ == "__main__":
    main()
